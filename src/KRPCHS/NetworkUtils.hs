module KRPCHS.NetworkUtils
( helloMsg
, helloStreamMsg
, connNameMsg
, recvN
, recvId
, recvMsg
, sendMsg
) where


import KRPCHS.SerializeUtils

import Control.Monad

import Network.Socket hiding (send, recv, sendTo, recvFrom)
import Network.Socket.ByteString

import qualified Data.ByteString        as BS
import qualified Data.ByteString.Lazy   as BL
import qualified Data.ByteString.Char8  as BC

import Data.Bits
import Data.Word
import Text.ProtocolBuffers


helloMsg :: BC.ByteString
helloMsg = BC.pack $ "HELLO-RPC\x0\x0\x0"

helloStreamMsg :: BC.ByteString
helloStreamMsg = BC.pack $ "HELLO-STREAM"


connNameMsg :: String -> BC.ByteString
connNameMsg name = BC.pack $ take 32 $ name ++ padding
    where padding = repeat '\x0'


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


recvId :: Socket -> IO BS.ByteString
recvId sock = recvN sock 16


recvMsg :: Socket -> IO BS.ByteString
recvMsg sock = do
    sz <- recvSize BS.empty
    case sz of
        Left err  -> fail err
        Right sz' -> recvN sock (fromIntegral sz')
    where
        recvSize :: BS.ByteString -> IO (Either String Word64)
        recvSize sz
            | BC.length sz > 10 = fail "Malformed message"
            | otherwise = do
                b <- recv sock 1
                let sz'  = BS.append sz b
                    more = testBit (BS.head b) 7
                if more then recvSize sz'
                        else return (decodePb $ BL.fromStrict sz')


sendMsg :: (ReflectDescriptor msg, Wire msg) => Socket -> msg -> IO ()
sendMsg sock msg = sendAll sock (BL.toStrict $ messageWithLengthPut msg)
