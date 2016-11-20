{-# LANGUAGE GeneralizedNewtypeDeriving #-}

module KRPCHS.Internal.Requests
( RPCClient(..)
, StreamClient(..)
, RPCContext(..)

, KRPCStream(..)
, KRPCStreamReq(..)
, KRPCStreamMsg()
, emptyKRPCStreamMsg
, getStreamMessage
, getStreamMessageIO

-- re-export from Requests.Stream
, messageHasResultFor
, messageResultsCount
, getStreamResult

, processResponse
, makeRequest
, sendRequest
, requestStream
, requestRemoveStream
) where


import Network.Socket
import Control.Monad.Catch
import Control.Monad.Reader

import KRPCHS.Internal.ProtocolError
import KRPCHS.Internal.SerializeUtils
import KRPCHS.Internal.NetworkUtils
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Batch
import KRPCHS.Internal.Requests.Stream

import qualified PB.KRPC.Argument        as KArg
import qualified PB.KRPC.Response        as KRes
import qualified PB.KRPC.Stream          as KStr
import qualified PB.KRPC.ProcedureResult as KPRes

import Data.Maybe
import Data.Sequence (ViewL((:<)))
import qualified Data.Sequence as Seq

import qualified Data.ByteString.Lazy  as BS
import qualified Text.ProtocolBuffers  as P


-- | An RPC client bound to a KRPC server.
data RPCClient = RPCClient { rpcSocket :: Socket, clientId :: BS.ByteString }

-- | A stream client bound to a KRPC Stream server.
data StreamClient = StreamClient { streamSocket :: Socket }


-- | The context for RPC calls.
newtype RPCContext a = RPCContext { runRPCContext :: ReaderT RPCClient IO a }
    deriving (Functor, Applicative, Monad, MonadIO, MonadReader RPCClient, MonadThrow, MonadCatch, MonadMask)


processProcedureResult :: (KRPCResponseExtractable a) => KPRes.ProcedureResult -> RPCContext a
processProcedureResult res = either (throwM) (return) (extract res)


-- TODO: make a unpackBatchReply in Requests.Batch to replace this function
processResponse :: (KRPCResponseExtractable a) => KRPCCallBatchReply -> RPCContext a
processResponse (KRPCCallBatchReply r) = case (KRes.error r) of
    Just err -> throwM $ KRPCError (P.toString err)
    Nothing  -> let (result :< _) = Seq.viewl (KRes.results r) in processProcedureResult result


sendRequest :: KRPCCallBatch -> RPCContext KRPCCallBatchReply
sendRequest (KRPCCallBatch req) = do
    sock <- asks rpcSocket
    liftIO $ sendMsg sock req
    msg  <- liftIO $ recvMsg sock
    either (throwM) (return . KRPCCallBatchReply) msg


makeRequest :: String -> String -> [KArg.Argument] -> KRPCCallBatch
makeRequest serviceName procName params = makeBatch [req]
    where req = makeCallRequest serviceName procName params


requestStream :: KRPCResponseExtractable a => KRPCStreamReq a -> RPCContext (KRPCStream a)
requestStream strReq = do
    res <- sendRequest (makeBatch [streamCall strReq])
    str <- processResponse res
    let sid = fromJust (KStr.id str)
    return (KRPCStream sid)


requestRemoveStream :: KRPCStream a -> RPCContext ()
requestRemoveStream str = do
    resp <- sendRequest (makeRequest "KRPC" "RemoveStream" [ makeArgument 0 (streamId str) ])
    processResponse resp


-- | @'getStreamMessage' client@ extracts the next 'KRPCStreamMsg' received by
-- the provided 'StreamClient'.
getStreamMessage :: StreamClient -> RPCContext KRPCStreamMsg
getStreamMessage client = getStreamMessageIO client >>= either throwM return


-- | A generalized version of 'getStreamMessage' to be used in any 'MonadIO'.
getStreamMessageIO :: MonadIO m => StreamClient -> m (Either ProtocolError KRPCStreamMsg)
getStreamMessageIO (StreamClient s) = (fmap unpackStreamMsg) <$> liftIO (recvMsg s)


-- | @'getStreamResult' s m@ extracts the result of a 'KRPCStream' @s@ from the
-- 'KRPCStreamMsg' @m@. If no such result exist, an exception is thrown. You
-- can check whether it is safe to call this function with 'messageHasResultFor'.
getStreamResult :: (KRPCResponseExtractable a) => KRPCStream a -> KRPCStreamMsg -> RPCContext a
getStreamResult (KRPCStream i) msg =
    maybe (throwM NoSuchStream)
          (processProcedureResult)
          (messageLookupResult i msg)
