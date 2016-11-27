{-# LANGUAGE RecordWildCards #-}

module KRPCHS
(
-- * Clients
  RPCClient
, withRPCClient
, StreamClient
, withStreamClient

-- * RPC context
, RPCContext
, runRPCProg

-- * Streams
, KRPCStream
, KRPCStreamReq
, KRPCStreamMsg
-- ** Creating streams
, addStream
, removeStream
, withStream
-- ** Retrieving results
-- | To retrieve stream results, you first need to call 'getStreamMessage' to
-- extract the next 'KRPCStreamMsg' from the 'StreamClient'. You can then use
-- 'KRPCStream' handles with 'getStreamResult' to extract the corresponding
-- value from the message.
, getStreamMessage
, getStreamMessageIO
, messageHasResultFor
, getStreamResult
, messageResultsCount
, emptyKRPCStreamMsg

-- * Classes
-- | When building abstractions, you may need to know about these classes.
, KRPCResponseExtractable

-- * Exceptions
, ProtocolError(..)
) where



import KRPCHS.Internal.Requests
import KRPCHS.Internal.ProtocolError
import KRPCHS.Internal.NetworkUtils
import KRPCHS.Internal.SerializeUtils

import qualified PB.KRPC.ConnectionResponse        as KRPC
import qualified PB.KRPC.ConnectionResponse.Status as KRPC.Status

import Network.Socket
import Control.Monad.Catch
import Control.Monad.Reader

import Data.Maybe
import qualified Data.ByteString.Lazy.Char8 as BS


getSocket :: HostName -> ServiceName -> IO Socket
getSocket host service = do
    -- establish connection
    addr <- getAddrInfo Nothing (Just host) (Just service)
    sock <- socket AF_INET Stream defaultProtocol
    connect sock (addrAddress $ head addr)
    return sock


rpcHandshake :: Socket -> String -> IO BS.ByteString
rpcHandshake sock name = do
    sendMsg sock (connectRpcMsg name)
    resp <- recvMsg sock
    either (fail . show) (extractId) resp
  where
    extractId KRPC.ConnectionResponse{..} = case status of
        Just KRPC.Status.OK -> return $ fromJust client_identifier
        Just err            -> fail   $ show err ++ " - details: '" ++ show (unpackUtf8String <$> message) ++ "'"
        Nothing             -> fail   $ "Could not make sense of the server's response"


streamHandshake :: Socket -> BS.ByteString -> IO ()
streamHandshake sock clientId = do
    sendMsg sock (connectStreamMsg clientId)
    resp <- recvMsg sock
    either (fail . show) (checkResponse) resp
  where
    checkResponse KRPC.ConnectionResponse{..} = case status of
        Just KRPC.Status.OK -> return ()
        err                 -> fail (show err)


-- | @'withRPCClient' name host port func@ opens a connection to the KRPC server and
-- pass the resulting 'RPCClient' handle to the specified function @func@.
-- When @func@ returns (or if it throws an exception), the connection with the
-- KRPC server is automatically closed.
withRPCClient :: String -> HostName -> ServiceName -> (RPCClient -> IO a) -> IO a
withRPCClient name host port func = do
    sock <- getSocket host port
    finally
        (do clientId <- rpcHandshake sock name
            func (RPCClient sock clientId)
        )
        (close sock)


-- | @'withStreamClient' rpcClient host port func@ opens a connection to the
-- KRPC streams server and pass the resulting 'StreamClient' handle to the
-- specified function @func@. When @func@ returns (or if it throws an
-- exception), the connection with the Stream server is automatically closed.
withStreamClient :: RPCClient -> HostName -> ServiceName -> (StreamClient -> IO a) -> IO a
withStreamClient RPCClient{..} host port func = do
    sock <- getSocket host port
    finally
        (do streamHandshake sock clientId
            func (StreamClient sock))
        (close sock)


-- | @'runRPCProg' client ctx@ runs the 'RPCContext' @ctx@ using the provided
-- 'RPCClient' @client@.
runRPCProg :: RPCClient -> RPCContext a -> IO a
runRPCProg client ctx = runReaderT (runRPCContext ctx) client


-- | Requests the creation of a new 'KRPCStream' using the provided 'KRPCStreamReq'.
addStream :: (KRPCResponseExtractable a) => KRPCStreamReq a -> RPCContext (KRPCStream a)
addStream = requestAddStream


-- | Requests the removal of a stream.
removeStream :: KRPCStream a -> RPCContext ()
removeStream = requestRemoveStream


-- | @'withStream' r f@ requests the creation of a new stream using the
-- 'KRPCStreamReq' @r@ and pass the resulting 'KRPCStream' to @f@. When @f@
-- returns or throws, a request to remove the stream will be sent to the KRPC
-- server automatically.
withStream :: KRPCResponseExtractable a => KRPCStreamReq a -> (KRPCStream a -> RPCContext b) -> RPCContext b
withStream r f = mask $ \restore -> do
    s <- addStream r
    restore (f s) `finally` (removeStream s)
