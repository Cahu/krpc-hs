{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE LambdaCase                 #-}
{-# LANGUAGE OverloadedStrings          #-}
{-# LANGUAGE RecordWildCards            #-}

module KRPCHS.Internal.Requests
( -- * RPC Client & primitives
  RPCClient
, withRPCClient
, sendRequest
  -- * Stream client & stream primitives
, StreamClient
, withStreamClient
, KRPCStream(..)
, KRPCStreamReq(..)
, KRPCStreamMsg(..)
, emptyKRPCStreamMsg
, makeStream
, requestStream
, extractStreamMessage
, extractStreamResponse
, getStreamMessage
  -- * Monad API
, RPCContext(..)
, MonadRPC(..)
  -- * Message manipulation
, KRPCResponseExtractable(..)
, makeArgument
, processResponse
, makeRequest
) where

import Control.Monad.Catch  (MonadThrow(..),MonadCatch,MonadMask)
import Control.Monad.Reader
import Control.Exception
import Control.Concurrent

import Network.Socket
import Network.Socket.ByteString

import KRPCHS.Internal.ProtocolError
import KRPCHS.Internal.NetworkUtils
import KRPCHS.Internal.SerializeUtils

import qualified PB.KRPC.Argument        as KArg
import qualified PB.KRPC.Request         as KReq
import qualified PB.KRPC.Response        as KRes
import qualified PB.KRPC.StreamMessage   as KStreamMsg
import qualified PB.KRPC.StreamResponse  as KStreamRes

import Data.Int
import Data.Word
import Data.Maybe
import qualified Data.Text as T
import qualified Data.Map  as M
import qualified Data.Foldable
import qualified Data.Set      as Set
import qualified Data.Sequence as Seq

import qualified Data.ByteString       as BS
import qualified Data.ByteString.Lazy  as BL
import qualified Text.ProtocolBuffers  as P

----------------------------------------------------------------
-- RPC Client & primitives
----------------------------------------------------------------

-- | Monad from which one can obtain RPC client
class Monad m => MonadRPC m where
  askClient :: m RPCClient


-- | Connection to KRPC.
--
--   Note that to avoid possibility of message fragmentation network
--   IO is done from separate thread.
data RPCClient = RPCClient
  { rpcChan  :: Chan ChanRequest
  , clientId :: BS.ByteString
  }

type ChanRequest = (KReq.Request, MVar (Either SomeException KRes.Response))

-- | Connect to RPC server
withRPCClient
  :: String                     -- ^ Name to send to kRPC
  -> HostName                   -- ^ Hostname to connect to
  -> ServiceName                -- ^ Port to connect to
  -> (RPCClient -> IO a)        -- ^ Action to perform
  -> IO a
withRPCClient name host port action = withSocket host port $ \sock -> do
  -- Perform RPC handshake
  cid <- rpcHandshake sock name
  -- Fork off worker process for network IO (It's done inside bracket
  -- to ensure that we don't leak p
  ch <- newChan
  bracket (forkIOWithUnmask ($ networkIOWorker sock ch)) killThread $ \_ ->
    action RPCClient { rpcChan  = ch
                     , clientId = cid
                     }

-- | Send request to RPC server and receive reply
sendRequest :: (MonadRPC m, MonadIO m) => KReq.Request -> m KRes.Response
sendRequest req = do
    ch <- rpcChan <$> askClient
    liftIO $ do
      var <- newEmptyMVar
      writeChan ch (req,var)
      takeMVar var >>= \case
        Left  e -> throwIO e
        Right r -> return r

-- Worker process for network IO. To send request to RPC server we
-- push message to Chan alongside with empty MVar. Worker perform
-- query and puts query result in the MVar. If network exception
-- happens during IO it's sent back to parent thread
networkIOWorker :: Socket -> Chan ChanRequest -> IO ()
networkIOWorker sock ch = forever $ do
    (req,var) <- readChan ch
    rsp <- try $ do
      sendMsg sock req
      recvResponse sock
    -- We need to special-case ThreadKilled exception to make thread
    -- killable
    case rsp of
      Left e | Just ThreadKilled <- fromException e
             -> throwIO ThreadKilled
      _      -> putMVar var rsp

-- Get response from RPC server
recvResponse :: (P.Wire a, P.ReflectDescriptor a) => Socket -> IO a
recvResponse sock = do
    msg <- recvMsg sock
    either (throwM) (return) (messageGet (BL.fromStrict msg))

-- Perform IO operation using socket. It's closed after action whether
-- upon normal termination of because of exception
withSocket :: HostName -> ServiceName -> (Socket -> IO a) -> IO a
withSocket host port action
  = bracket initS fini body
  where
    initS = do
      -- FIXME: do something more sensible!
      addr:_ <- getAddrInfo Nothing (Just host) (Just port)
      sock   <- socket AF_INET Stream defaultProtocol
      return (sock,addr)
    fini (sock,_)    = close sock
    body (sock,addr) = do
      connect sock (addrAddress addr)
      action sock

-- Perform kRPC handshake. Returns client identifier
rpcHandshake :: Socket -> String -> IO BS.ByteString
rpcHandshake sock name = do
    sendAll sock helloMsg
    sendAll sock (connNameMsg name)
    recvId sock


----------------------------------------------------------------
-- Streaming client
----------------------------------------------------------------

-- | Client for stream request
data StreamClient = StreamClient { streamSocket :: Socket }

newtype KRPCStream a = KRPCStream { streamId :: Int }
    deriving (Show)

newtype KRPCStreamReq a = KRPCStreamReq { streamReq :: KReq.Request }
    deriving (Show)

newtype KRPCStreamMsg = KRPCStreamMsg { streamMsg :: M.Map Int KRes.Response }
    deriving (Show)

emptyKRPCStreamMsg :: KRPCStreamMsg
emptyKRPCStreamMsg = KRPCStreamMsg M.empty


-- | Perform IO action using stream client. Client will be shut down
--   after action is completed either normally of abnormally.
withStreamClient
  :: RPCClient                  -- ^ Connect client
  -> HostName                   -- ^ Host name
  -> ServiceName                -- ^ Port number
  -> (StreamClient -> IO a)     -- ^ Action to perform
  -> IO a
withStreamClient RPCClient{..} host port func =
    withSocket host port $ \sock -> do
        streamHandshake sock clientId
        func (StreamClient sock)

makeStream :: KReq.Request -> KRPCStreamReq a
makeStream r = KRPCStreamReq $
    makeRequest "KRPC" "AddStream" [KArg.Argument (Just 0) (Just $ messagePut r)]

requestStream :: (MonadRPC m, MonadIO m, MonadThrow m) => KRPCStreamReq a -> m (KRPCStream a)
requestStream KRPCStreamReq{..} = do
    res <- sendRequest streamReq
    sid <- processResponse res
    return (KRPCStream sid)

getStreamMessage :: MonadIO m => StreamClient -> m KRPCStreamMsg
getStreamMessage StreamClient{..} = unpackStreamMsg <$> liftIO (recvResponse streamSocket)
  where
    unpackStreamMsg res = KRPCStreamMsg $ M.fromList (extractStreamMessage res)

extractStreamResponse :: KStreamRes.StreamResponse -> Maybe (Int, KRes.Response)
extractStreamResponse streamRes = do
    sid <- KStreamRes.id       streamRes
    res <- KStreamRes.response streamRes
    return (fromIntegral sid, res)

extractStreamMessage :: KStreamMsg.StreamMessage -> [(Int, KRes.Response)]
extractStreamMessage msg = mapMaybe extractStreamResponse responseList
    where responseList = Data.Foldable.toList (KStreamMsg.responses msg)

-- Perform handshake with stream server
streamHandshake :: Socket -> BS.ByteString -> IO ()
streamHandshake sock clientId = do
    sendAll sock helloStreamMsg
    sendAll sock clientId
    res <- recvN sock 2
    case res of
        "OK" -> return ()
        _    -> fail "Could not handshake with stream server"



----------------------------------------------------------------
-- Monad API
----------------------------------------------------------------

-- | Reader monad which uses RPCClient as context.
newtype RPCContext a = RPCContext { runRPCContext :: ReaderT RPCClient IO a }
    deriving (Functor, Applicative, Monad, MonadIO, MonadReader RPCClient, MonadThrow, MonadCatch, MonadMask)

instance MonadRPC RPCContext where
  askClient = ask



----------------------------------------------------------------
-- Message Manipulation
----------------------------------------------------------------

class (PbSerializable a) => KRPCResponseExtractable a where
    extract :: KRes.Response -> Either ProtocolError a
    extract r = do
        checkError r
        maybe (Left ResponseEmpty) (decodePb) (KRes.return_value r)

instance KRPCResponseExtractable Bool
instance KRPCResponseExtractable Float
instance KRPCResponseExtractable Double
instance KRPCResponseExtractable Int
instance KRPCResponseExtractable Int32
instance KRPCResponseExtractable Int64
instance KRPCResponseExtractable Word32
instance KRPCResponseExtractable Word64
instance KRPCResponseExtractable T.Text
instance (PbSerializable a, PbSerializable b)                                     => KRPCResponseExtractable (a, b)
instance (PbSerializable a, PbSerializable b, PbSerializable c)                   => KRPCResponseExtractable (a, b, c)
instance (PbSerializable a, PbSerializable b, PbSerializable c, PbSerializable d) => KRPCResponseExtractable (a, b, c, d)


instance KRPCResponseExtractable () where
    extract = checkError


instance (PbSerializable a) => KRPCResponseExtractable [a] where
    extract r = do
        checkError r
        case (KRes.return_value r) of
            Nothing    -> Right []
            Just bytes -> decodePb bytes


instance (Ord a, PbSerializable a) => KRPCResponseExtractable (Set.Set a) where
    extract r = do
        checkError r
        case (KRes.return_value r) of
            Nothing    -> Right Set.empty
            Just bytes -> decodePb bytes


instance (Ord k, PbSerializable k, PbSerializable v) => KRPCResponseExtractable (M.Map k v) where
    extract r = do
        checkError r
        case (KRes.return_value r) of
            Nothing    -> Right M.empty
            Just bytes -> decodePb bytes


checkError :: KRes.Response -> Either ProtocolError ()
checkError r = case (KRes.has_error r) of
    Just True -> Left (maybe (DecodeFailure "unknown reason") (KRPCError . P.toString) (KRes.error r))
    _         -> return ()


processResponse :: (MonadThrow m, KRPCResponseExtractable a) => KRes.Response -> m a
processResponse res = either (throwM) (return) (extract res)

makeArgument :: (PbSerializable a) => P.Word32 -> a -> KArg.Argument
makeArgument position arg = KArg.Argument (Just position) (Just $ encodePb arg)

makeRequest :: String -> String -> [KArg.Argument] -> KReq.Request
makeRequest serviceName procName params =
    KReq.Request
    { service   = Just $ P.fromString serviceName
    , procedure = Just $ P.fromString procName
    , arguments = Seq.fromList params }
