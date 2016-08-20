{-# LANGUAGE RecordWildCards #-}

module KRPCHS
( RPCClient
, StreamClient
, RPCContext

, KRPCStream
, KRPCStreamReq
, KRPCStreamMsg
, emptyKRPCStreamMsg
, getStreamMessage
, messageResultsCount
, messageHasResultFor
, addStream
, removeStream
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
import Control.Monad.Trans
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
    clientId <- recvId sock
    return clientId


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
        (sClose sock)


withStreamClient :: RPCClient -> HostName -> ServiceName -> (StreamClient -> IO a) -> IO a
withStreamClient RPCClient{..} host port func = do
    sock <- getSocket host port
    finally
        (do streamHandshake sock clientId
            func (StreamClient sock))
        (sClose sock)


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
    res <- liftIO $ recvResponse streamSocket
    return $ KRPCStreamMsg $ M.fromList (extractStreamMessage res)


messageResultsCount :: KRPCStreamMsg -> Int
messageResultsCount = M.size . streamMsg


messageHasResultFor :: KRPCStream a -> KRPCStreamMsg -> Bool
messageHasResultFor KRPCStream{..} KRPCStreamMsg{..} =
    M.member streamId streamMsg


getStreamResult :: (KRPCResponseExtractable a) => KRPCStream a -> KRPCStreamMsg -> RPCContext a
getStreamResult KRPCStream{..} KRPCStreamMsg{..} =
    maybe (throwM NoSuchStream)
          (processResponse)
          (M.lookup streamId streamMsg)


addStream :: (KRPCResponseExtractable a) => KRPCStreamReq a -> RPCContext (KRPCStream a)
addStream = requestStream


removeStream :: KRPCStream a -> RPCContext ()
removeStream KRPCStream{..} = do
    resp <- sendRequest (makeRequest "KRPC" "RemoveStream" [ makeArgument 0 streamId ])
    processResponse resp
