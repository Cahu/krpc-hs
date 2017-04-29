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

import qualified PB.KRPC.Status   as KRPC
import qualified PB.KRPC.Service  as KRPC
import qualified PB.KRPC.Services as KRPC

import Control.Monad.Catch
import Control.Monad.Reader

import qualified Data.Map as M



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


-- Deprecated
getStreamMessageIO :: MonadIO m => StreamClient -> m KRPCStreamMsg
getStreamMessageIO = getStreamMessage
{-# DEPRECATED getStreamMessageIO "use 'getStreamMessage' instead" #-}

messageResultsCount :: KRPCStreamMsg -> Int
messageResultsCount = M.size . streamMsg


messageHasResultFor :: KRPCStream a -> KRPCStreamMsg -> Bool
messageHasResultFor KRPCStream{..} KRPCStreamMsg{..} =
    M.member streamId streamMsg


getStreamResult :: (MonadThrow m, KRPCResponseExtractable a) => KRPCStream a -> KRPCStreamMsg -> m a
getStreamResult KRPCStream{..} KRPCStreamMsg{..} =
    maybe (throwM NoSuchStream)
          (processResponse)
          (M.lookup streamId streamMsg)


addStream :: (MonadRPC m, MonadThrow m, MonadIO m) => KRPCStreamReq a -> m (KRPCStream a)
addStream = requestStream


removeStream :: (MonadRPC m, MonadThrow m, MonadIO m) => KRPCStream a -> m ()
removeStream KRPCStream{..} = do
    resp <- sendRequest (makeRequest "KRPC" "RemoveStream" [ makeArgument 0 streamId ])
    processResponse resp


withStream :: (MonadMask m, MonadRPC m, MonadIO m) => KRPCStreamReq a -> (KRPCStream a -> m b) -> m b
withStream r f = mask $ \restore -> do
    s <- addStream r
    restore (f s) `finally` (removeStream s)
