{-# LANGUAGE RecordWildCards #-}

module KRPCHS
( RPCClient
, StreamClient
, RPCContext
, MonadRPC

, KRPCStream
, KRPCStreamReq
, KRPCStreamMsg
, KRPCResponseExtractable
, emptyKRPCStreamMsg
, getStreamMessage
, getStreamMessageIO
, messageResultsCount
, messageHasResultFor
, addStream
, removeStream
, withStream
, getStreamResult

, withRPCClient
, withStreamClient
, runRPCProg

, KRPC.Status(..)
, KRPC.Service(..)
, KRPC.Services(..)
--, getStatus
--, getServices

, ProtocolError(..)
) where



import KRPCHS.Internal.Requests
import KRPCHS.Internal.ProtocolError
import KRPCHS.Internal.NetworkUtils

import qualified PB.KRPC.Status   as KRPC
import qualified PB.KRPC.Service  as KRPC
import qualified PB.KRPC.Services as KRPC

import Network.Socket
import Control.Monad.Catch
import Control.Monad.Reader

import qualified Data.Map as M
import qualified Data.ByteString.Char8 as BS
import Network.Socket.ByteString


getSocket :: HostName -> ServiceName -> IO Socket
getSocket host service = do
    -- establish connection
    addr <- getAddrInfo Nothing (Just host) (Just service)
    sock <- socket AF_INET Stream defaultProtocol
    connect sock (addrAddress $ head addr)
    return sock


rpcHandshake :: Socket -> String -> IO BS.ByteString
rpcHandshake sock name = do
    sendAll sock helloMsg
    sendAll sock (connNameMsg name)
    recvId sock


streamHandshake :: Socket -> BS.ByteString -> IO ()
streamHandshake sock clientId = do
    sendAll sock helloStreamMsg
    sendAll sock clientId
    res <- recvN sock 2
    case BS.unpack res of
        "OK" -> return ()
        _    -> fail "Could not handshake with stream server"


withRPCClient :: String -> HostName -> ServiceName -> (RPCClient -> IO a) -> IO a
withRPCClient name host port func = do
    sock <- getSocket host port
    finally
        (do clientId <- rpcHandshake sock name
            func (RPCClient sock clientId)
        )
        (close sock)


withStreamClient :: RPCClient -> HostName -> ServiceName -> (StreamClient -> IO a) -> IO a
withStreamClient RPCClient{..} host port func = do
    sock <- getSocket host port
    finally
        (do streamHandshake sock clientId
            func (StreamClient sock))
        (close sock)


runRPCProg :: RPCClient -> RPCContext a -> IO a
runRPCProg client ctx = runReaderT (runRPCContext ctx) client


{-
getStatus :: RPCContext KRPC.Status
getStatus = do
    resp <- sendRequest (makeRequest "KRPC" "GetStatus" [])
    processResponse resp


getServices :: RPCContext KRPC.Services
getServices = do
    resp <- sendRequest (makeRequest "KRPC" "GetServices" [])
    processResponse resp
-}


getStreamMessage :: MonadIO m => StreamClient -> m KRPCStreamMsg
getStreamMessage StreamClient{..} = unpackStreamMsg <$> liftIO (recvResponse streamSocket)
  where
    unpackStreamMsg res = KRPCStreamMsg $ M.fromList (extractStreamMessage res)

-- Deprecated
getStreamMessageIO :: MonadIO m => StreamClient -> m KRPCStreamMsg
getStreamMessageIO = getStreamMessage


messageResultsCount :: KRPCStreamMsg -> Int
messageResultsCount = M.size . streamMsg


messageHasResultFor :: KRPCStream a -> KRPCStreamMsg -> Bool
messageHasResultFor KRPCStream{..} KRPCStreamMsg{..} =
    M.member streamId streamMsg


getStreamResult :: (MonadRPC m, MonadThrow m, KRPCResponseExtractable a) => KRPCStream a -> KRPCStreamMsg -> m a
getStreamResult KRPCStream{..} KRPCStreamMsg{..} =
    maybe (throwM NoSuchStream)
          (processResponse)
          (M.lookup streamId streamMsg)


addStream :: (MonadRPC m, MonadThrow m, MonadIO m, KRPCResponseExtractable a) => KRPCStreamReq a -> m (KRPCStream a)
addStream = requestStream


removeStream :: (MonadRPC m, MonadThrow m, MonadIO m) => KRPCStream a -> m ()
removeStream KRPCStream{..} = do
    resp <- sendRequest (makeRequest "KRPC" "RemoveStream" [ makeArgument 0 streamId ])
    processResponse resp


withStream :: (MonadMask m, MonadRPC m, MonadThrow m, MonadIO m, KRPCResponseExtractable a) => KRPCStreamReq a -> (KRPCStream a -> m b) -> m b
withStream r f = mask $ \restore -> do
    s <- addStream r
    restore (f s) `finally` (removeStream s)
