{-# LANGUAGE ScopedTypeVariables #-}

module KRPCHS.Internal.NetworkUtils
( helloMsg
, helloStreamMsg
, connectRpcMsg
, connectStreamMsg
, recvMsgRaw
, recvMsg
, sendMsg
) where


import KRPCHS.Internal.ProtocolError
import KRPCHS.Internal.SerializeUtils

import Control.Monad

import Network.Socket hiding (send, recv, sendTo, recvFrom)
import Network.Socket.ByteString

import qualified Data.ByteString        as BS
import qualified Data.ByteString.Lazy   as BL
import qualified Data.ByteString.Char8  as BC

import qualified Text.ProtocolBuffers       as P
import qualified PB.KRPC.ConnectionRequest  as KRPC
--import qualified PB.KRPC.ConnectionResponse as KRPC

import Data.Bits
import Data.Word


helloMsg :: BC.ByteString
helloMsg = BC.pack "KRPC-RPC"
--helloMsg = BC.pack "HELLO-RPC\x0\x0\x0"

helloStreamMsg :: BC.ByteString
helloStreamMsg = BC.pack "HELLO-STREAM"


connectRpcMsg :: String -> KRPC.ConnectionRequest
connectRpcMsg name = KRPC.ConnectionRequest
    { KRPC.client_name       = Just (packUtf8String name)
    , KRPC.client_identifier = Nothing }


connectStreamMsg :: BL.ByteString -> KRPC.ConnectionRequest
connectStreamMsg clientid = KRPC.ConnectionRequest
    { KRPC.client_name       = Nothing
    , KRPC.client_identifier = Just clientid }


recvN :: Socket -> Int -> IO BS.ByteString
recvN sock n =
    let
        recvN' numBytesRead bsList | numBytesRead == n =
            return $ BS.concat $ reverse bsList

        recvN' numBytesRead bsList = do
            bytes <- recv sock (n - numBytesRead)
            when (BS.length bytes == 0) $ fail "Read error"
            recvN' (numBytesRead + BS.length bytes) (bytes : bsList)
    in
        recvN' 0 []


recvStopBitEntity :: Socket -> IO BS.ByteString
recvStopBitEntity sock = aux BS.empty
  where
    aux entity = do
        b <- recv sock 1
        let entity' = (BS.append entity b)
        if (testBit (BS.head b) 7) then aux entity' else return entity'


recvMsgRaw :: Socket -> IO BS.ByteString
recvMsgRaw sock = do
    (sz :: Word64) <- recvSize
    recvN sock (fromIntegral sz)
  where
    recvSize = do
        entity <- recvStopBitEntity sock
        either (fail "Malformed message") (return) (decodePb $ BL.fromStrict entity)


recvMsg :: (P.ReflectDescriptor msg, P.Wire msg) => Socket -> IO (Either ProtocolError msg)
recvMsg sock = messageGet . BL.fromStrict <$> recvMsgRaw sock


sendMsg :: (P.ReflectDescriptor msg, P.Wire msg) => Socket -> msg -> IO ()
sendMsg sock msg = sendAll sock (BL.toStrict $ P.messageWithLengthPut msg)
