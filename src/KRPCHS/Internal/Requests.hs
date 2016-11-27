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

-- Streams
, messageHasResultFor
, messageResultsCount
, getStreamResult

-- re-export from Requests.Batch
, emptyBatch
, rpcCall
, buildBatch
, batchAddCall
, batchGetResult

, performBatchRequest
, simpleRequest
, simpleCallRequest
, requestAddStream
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

import Data.Maybe (fromJust)
import qualified Data.ByteString.Lazy  as BS


-- | An RPC client bound to a KRPC server.
data RPCClient = RPCClient { rpcSocket :: Socket, clientId :: BS.ByteString }

-- | A stream client bound to a KRPC Stream server.
data StreamClient = StreamClient { streamSocket :: Socket }


-- | The context for RPC calls.
newtype RPCContext a = RPCContext { runRPCContext :: ReaderT RPCClient IO a }
    deriving (Functor, Applicative, Monad, MonadIO, MonadReader RPCClient, MonadThrow, MonadCatch, MonadMask)


processProcedureResult :: (KRPCResponseExtractable a) => KPRes.ProcedureResult -> RPCContext a
processProcedureResult res = either (throwM) (return) (extract res)


-- | Performs a batch request and returns the server's reply as a
-- 'KRPCCallBatchReply'. The reply may be exploited with handles created when
-- building the batch using 'batchGetResult'.
performBatchRequest :: KRPCCallBatch -> RPCContext KRPCCallBatchReply
performBatchRequest (KRPCCallBatch req) = do
    sock <- asks rpcSocket
    liftIO $ sendMsg sock req
    msg <- liftIO $ recvMsg sock
    case msg of
        Left err -> throwM err
        Right m  -> case (KRes.error m) of
            Just err' -> throwM $ KRPCError (unpackUtf8String err')
            Nothing   -> return (unpackBatchReply m)


-- | Retrieve the result of a procedure call from the reply of a batch request.
batchGetResult :: (KRPCResponseExtractable a) => KRPCCall a -> KRPCCallBatchReply -> RPCContext a
batchGetResult c r = case batchLookupResult c r of
    Nothing  -> throwM NotInBatch
    Just res -> processProcedureResult res


simpleRequest :: (KRPCResponseExtractable a) => KRPCCallReq a -> RPCContext a
simpleRequest c = performBatchRequest b >>= batchGetResult hdl
  where (hdl, b) = batchAddCall c emptyBatch


simpleCallRequest :: (KRPCResponseExtractable a) => String -> String -> [KArg.Argument] -> RPCContext a
simpleCallRequest service procedure args = simpleRequest (makeCallReq service procedure args)


requestAddStream :: KRPCResponseExtractable a => KRPCStreamReq a -> RPCContext (KRPCStream a)
requestAddStream strReq = do
    res <- simpleRequest (streamCall strReq)
    let sid = fromJust (KStr.id res)
    return (KRPCStream sid)


requestRemoveStream :: KRPCStream a -> RPCContext ()
requestRemoveStream str = simpleCallRequest "KRPC" "RemoveStream" args
  where args = [ makeArgument 0 (streamId str) ]


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
