{-# LANGUAGE RecordWildCards #-}

module KRPCHS
( RPCClient
, StreamClient
, RPCContext
, StreamContext

, KRPCStream
, streamId
, getStreamMessage
, getStreamResult

, withRPCClient
, withStreamClient
, runRPCProg
, runStreamProg

, KRPC.Status(..)
, KRPC.Services(..)
, getStatus
, getServices
, removeStream
) where



import KRPCHS.Requests
import KRPCHS.NetworkUtils

import qualified PB.KRPC.Status   as KRPC
import qualified PB.KRPC.Services as KRPC

import Network.Socket
import Control.Exception

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


getStatus :: RPCContext (Either String KRPC.Status)
getStatus = do
    resp <- sendRequest (makeRequest "KRPC" "GetStatus" [])
    return $ extractMessage resp


getServices :: RPCContext (Either String KRPC.Services)
getServices = do
    resp <- sendRequest (makeRequest "KRPC" "GetServices" [])
    return $ extractMessage resp


removeStream :: KRPCStream a -> RPCContext (Either String Bool)
removeStream KRPCStream{..} = do
    resp <- sendRequest (makeRequest "KRPC" "RemoveStream" [ makeArgument 0 streamId ])
    return $ extractNothing resp
