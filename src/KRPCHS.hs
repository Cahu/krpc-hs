{-# LANGUAGE RecordWildCards #-}

module KRPCHS
( RPCClient
, StreamClient
, RPCContext

, KRPCStream
, KRPCStreamReq
, KRPCStreamMsg
, KRPCResponseExtractable
, emptyKRPCStreamMsg
, getStreamMessage
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
import KRPCHS.Internal.SerializeUtils

import qualified PB.KRPC.Status                    as KRPC
import qualified PB.KRPC.Service                   as KRPC
import qualified PB.KRPC.Services                  as KRPC
import qualified PB.KRPC.ConnectionResponse        as KRPC
import qualified PB.KRPC.ConnectionResponse.Status as KRPC.Status

import Network.Socket
import Control.Monad.Catch
import Control.Monad.Reader

import Data.Maybe

import qualified Data.Map as M
import qualified Data.ByteString.Lazy.Char8 as BS
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
    sendMsg sock (connectRpcMsg name)
    resp <- recvMsg sock
    either (fail . show) (extractId) resp
  where
    extractId resp@KRPC.ConnectionResponse{..} = do
        print resp
        case status of
            Just KRPC.Status.OK -> return $ fromJust client_identifier
            Just err            -> fail   $ show err ++ " - details: '" ++ show (unpackUtf8String <$> message) ++ "'"
            Nothing             -> fail   $ "Could not make sense of the server's response"


streamHandshake :: Socket -> BS.ByteString -> IO ()
streamHandshake sock clientId = do
    sendAll sock helloStreamMsg
    sendMsg sock (connectStreamMsg clientId)
    resp <- recvMsg sock
    either (fail . show) (checkResponse) resp
  where
    checkResponse KRPC.ConnectionResponse{..} = case status of
        Just KRPC.Status.OK -> return ()
        err                 -> fail (show err)


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


getStreamMessage :: StreamClient -> RPCContext KRPCStreamMsg
getStreamMessage StreamClient{..} = do
    res <- liftIO $ recvMsg streamSocket
    either (throwM) (return . unpackStreamMsg) res
  where
    unpackStreamMsg res = KRPCStreamMsg $ M.fromList (extractStreamMessage res)


messageResultsCount :: KRPCStreamMsg -> Int
messageResultsCount = M.size . streamMsg


messageHasResultFor :: KRPCStream a -> KRPCStreamMsg -> Bool
messageHasResultFor KRPCStream{..} KRPCStreamMsg{..} =
    M.member streamId streamMsg


getStreamResult :: (KRPCResponseExtractable a) => KRPCStream a -> KRPCStreamMsg -> RPCContext a
getStreamResult KRPCStream{..} KRPCStreamMsg{..} =
    maybe (throwM NoSuchStream)
          (processResult)
          (M.lookup streamId streamMsg)


addStream :: (KRPCResponseExtractable a) => KRPCStreamReq a -> RPCContext (KRPCStream a)
addStream = requestStream


removeStream :: KRPCStream a -> RPCContext ()
removeStream KRPCStream{..} = do
    resp <- sendRequest (makeRequest "KRPC" "RemoveStream" [ makeArgument 0 streamId ])
    processResponse resp


withStream :: KRPCResponseExtractable a => KRPCStreamReq a -> (KRPCStream a -> RPCContext b) -> RPCContext b
withStream r f = mask $ \restore -> do
    s <- addStream r
    restore (f s) `finally` (removeStream s)
