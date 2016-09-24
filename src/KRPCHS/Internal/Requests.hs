{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}

module KRPCHS.Internal.Requests
( RPCClient(..)
, StreamClient(..)
, RPCContext(..)

, KRPCStream(..)
, KRPCStreamReq(..)
, KRPCStreamMsg(..)
, emptyKRPCStreamMsg

, KRPCResponseExtractable
, extract

, makeArgument
, processResult
, processResponse
, makeRequest
, sendRequest
, makeStream
, requestStream
, extractStreamMessage
, extractStreamResponse
) where


import Network.Socket
import Control.Monad.Catch
import Control.Monad.Reader

import KRPCHS.Internal.ProtocolError
import KRPCHS.Internal.NetworkUtils
import KRPCHS.Internal.SerializeUtils

import qualified PB.KRPC.Argument        as KArg
import qualified PB.KRPC.Request         as KReq
import qualified PB.KRPC.Response        as KRes
import qualified PB.KRPC.ProcedureCall   as KPReq
import qualified PB.KRPC.ProcedureResult as KPRes
import qualified PB.KRPC.StreamUpdate    as KStreamMsg
import qualified PB.KRPC.StreamResult    as KStreamRes

import Data.Int
import Data.Word
import Data.Maybe
import qualified Data.Text as T
import qualified Data.Map  as M
import qualified Data.Foldable
import qualified Data.Sequence as Seq
import           Data.Sequence (ViewL((:<)))

import qualified Data.ByteString.Lazy  as BS
import qualified Text.ProtocolBuffers  as P


data RPCClient    = RPCClient    { rpcSocket    :: Socket, clientId :: BS.ByteString }
data StreamClient = StreamClient { streamSocket :: Socket }


newtype RPCContext a = RPCContext { runRPCContext :: ReaderT RPCClient IO a }
    deriving (Functor, Applicative, Monad, MonadIO, MonadReader RPCClient, MonadThrow, MonadCatch, MonadMask)

newtype KRPCStream a = KRPCStream { streamId :: Int }
    deriving (Show)

newtype KRPCStreamReq a = KRPCStreamReq { streamReq :: KReq.Request }
    deriving (Show)

newtype KRPCStreamMsg = KRPCStreamMsg { streamMsg :: M.Map Int KPRes.ProcedureResult }
    deriving (Show)

emptyKRPCStreamMsg :: KRPCStreamMsg
emptyKRPCStreamMsg = KRPCStreamMsg M.empty


class (PbSerializable a) => KRPCResponseExtractable a where
    extract :: (PbSerializable a) => KPRes.ProcedureResult -> Either ProtocolError a
    extract r = do
        checkError r
        maybe (Left ResponseEmpty) (decodePb) (KPRes.value r)

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
        case (KPRes.value r) of
            Nothing    -> Right []
            Just bytes -> decodePb bytes

instance (Ord k, PbSerializable k, PbSerializable v) => KRPCResponseExtractable (M.Map k v) where
    extract r = do
        checkError r
        case (KPRes.value r) of
            Nothing    -> Right M.empty
            Just bytes -> decodePb bytes


checkError :: KPRes.ProcedureResult -> Either ProtocolError ()
checkError r = case (KPRes.error r) of
    Just err -> Left $ KRPCError (P.toString err)
    Nothing  -> return ()


processResult :: (KRPCResponseExtractable a) => KPRes.ProcedureResult -> RPCContext a
processResult res = either (throwM) (return) (extract res)


processResponse :: (KRPCResponseExtractable a) => KRes.Response -> RPCContext a
processResponse r = case (KRes.error r) of
    Just err -> throwM $ KRPCError (P.toString err)
    Nothing  -> let (result :< _) = Seq.viewl (KRes.results r) in  processResult result


sendRequest :: KReq.Request -> RPCContext KRes.Response
sendRequest r = do
    sock <- asks rpcSocket
    liftIO $ sendMsg sock r
    msg  <- liftIO $ recvMsg sock
    either (throwM) (return) msg


makeArgument :: (PbSerializable a) => P.Word32 -> a -> KArg.Argument
makeArgument position arg = KArg.Argument (Just position) (Just $ encodePb arg)


makeCallRequest :: String -> String -> [KArg.Argument] -> KPReq.ProcedureCall
makeCallRequest serviceName procName params =
    KPReq.ProcedureCall
    { service   = Just $ P.fromString serviceName
    , procedure = Just $ P.fromString procName
    , arguments = Seq.fromList params }


makeRequest :: String -> String -> [KArg.Argument] -> KReq.Request
makeRequest serviceName procName params = KReq.Request . Seq.singleton $ makeCallRequest serviceName procName params


makeStream :: KReq.Request -> KRPCStreamReq a
makeStream r = KRPCStreamReq $
    makeRequest "KRPC" "AddStream" [KArg.Argument (Just 0) (Just $ messagePut r)]


requestStream :: KRPCResponseExtractable a => KRPCStreamReq a -> RPCContext (KRPCStream a)
requestStream KRPCStreamReq{..} = do
    res <- sendRequest streamReq
    sid <- processResponse res
    return (KRPCStream sid)


extractStreamResponse :: KStreamRes.StreamResult -> Maybe (Int, KPRes.ProcedureResult)
extractStreamResponse streamRes = do
    sid <- KStreamRes.id     streamRes
    res <- KStreamRes.result streamRes
    return (fromIntegral sid, res)


extractStreamMessage :: KStreamMsg.StreamUpdate -> [(Int, KPRes.ProcedureResult)]
extractStreamMessage msg = mapMaybe extractStreamResponse responseList
    where responseList = Data.Foldable.toList (KStreamMsg.results msg)

