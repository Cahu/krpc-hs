{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}


{- Implementation notes:
 -
 - 1/ KRPC uses protocol buffers v3 but the library we use only support v2.
 - Fortunatelly, KRPC doesn't use v3 features so we can adapt the .proto file
 - by adding some annotations ('optional') to fields. However, this has
 - implications. For instance, we are unable to distinguish an empty list from
 - the absence of a list.
 -
 - 2/ Two types of exception are thrown from an RPCContext: IO exceptions and
 - 'ExceptT' exceptions.
 -  * IO exceptions are thrown when an IO error or a protocol issue (malformed
 -  message, decoding failure, ...) is - encountered.
 -  * ExceptT - exceptions are meant to be handled within the RPCContext and
 -  are thrown when the program running in the RPCContext can recover from them
 -  (missing stream response, ...).
 -}

module KRPCHS.Internal.Requests
( RPCClient(..)
, StreamClient(..)
, RPCContext(..)

, KRPCStream(..)
, KRPCStreamMsg(..)

, makeArgument
, makeArgumentTuple3
, makeArgumentTuple4
, processResponse
, makeRequest
, sendRequest
, recvResponse
, makeStream
, extractMessage
, extractStreamMessage
, extractStreamResponse
, extractValue
, extractList
, extractMap
, extractTuple3
, extractTuple4
, extractNothing
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
import qualified PB.KRPC.List            as KList
import qualified PB.KRPC.Tuple           as KTuple
import qualified PB.KRPC.Dictionary      as KDict
import qualified PB.KRPC.DictionaryEntry as KDictE
import qualified PB.KRPC.StreamMessage   as KStreamMsg
import qualified PB.KRPC.StreamResponse  as KStreamRes

import Data.Maybe
import qualified Data.Map              as M
import qualified Data.Sequence         as Seq
import qualified Data.Foldable

import qualified Data.ByteString       as BS
import qualified Data.ByteString.Lazy  as BL
import qualified Text.ProtocolBuffers  as P


data RPCClient    = RPCClient    { rpcSocket    :: Socket, clientId :: BS.ByteString }
data StreamClient = StreamClient { streamSocket :: Socket }


newtype RPCContext a = RPCContext { runRPCContext :: ReaderT RPCClient IO a }
    deriving (Functor, Applicative, Monad, MonadIO, MonadReader RPCClient, MonadThrow, MonadCatch, MonadMask)


data KRPCStream a = KRPCStream
    { streamId        :: Int
    , streamExtractor :: KRes.Response -> Either ProtocolError a }


data KRPCStreamMsg = KRPCStreamMsg
    { streamMsg :: M.Map Int KRes.Response }



checkError :: KRes.Response -> Either ProtocolError ()
checkError r = case (KRes.has_error r) of
    Just True -> Left (maybe (DecodeFailure "unknown reason") (KRPCError . P.toString) (KRes.error r))
    _         -> return ()


messageGet :: (P.Wire a, P.ReflectDescriptor a) => BL.ByteString -> Either ProtocolError a
messageGet bytes = either (Left . DecodeFailure) (return . fst) (P.messageGet bytes)


messagePut :: (P.Wire a, P.ReflectDescriptor a) => a -> BL.ByteString
messagePut = P.messagePut


extractMessage :: (P.Wire a, P.ReflectDescriptor a) => KRes.Response -> Either ProtocolError a
extractMessage r = do
    checkError r
    maybe (Left ResponseEmpty) (messageGet) (KRes.return_value r)


extractValue :: (PbSerializable a) => KRes.Response -> Either ProtocolError a
extractValue r = do
    checkError r
    maybe (Left ResponseEmpty) (decodePb) (KRes.return_value r)


extractTuple3 :: (PbSerializable a, PbSerializable b, PbSerializable c)
              => KRes.Response 
              -> Either ProtocolError (a, b, c)
extractTuple3 r = do
    s <- extractMessage r
    let (a:b:c:_) = Data.Foldable.toList $ KTuple.items s
    a' <- decodePb a
    b' <- decodePb b
    c' <- decodePb c
    return (a', b', c')


extractTuple4 :: (PbSerializable a, PbSerializable b, PbSerializable c, PbSerializable d)
              => KRes.Response 
              -> Either ProtocolError (a, b, c, d)
extractTuple4 r = do
    s <- extractMessage r
    let (a:b:c:d:_) = Data.Foldable.toList $ KTuple.items s
    a' <- decodePb a
    b' <- decodePb b
    c' <- decodePb c
    d' <- decodePb d
    return (a', b', c', d')


extractMap :: (Ord a, PbSerializable a, PbSerializable b)
           => KRes.Response
           -> Either ProtocolError (M.Map a b)
extractMap r = case (extractMessage r) of
    Right s             -> M.fromList <$> mapM extractKeyVal (Data.Foldable.toList $ KDict.entries s)
    Left  ResponseEmpty -> Right M.empty
    Left  err           -> Left  err
    where
        extractKeyVal e = do
            k <- maybe (Left $ DecodeFailure "No key") (decodePb) (KDictE.key   e)
            v <- maybe (Left $ DecodeFailure "No val") (decodePb) (KDictE.value e)
            return (k, v)


extractList :: (PbSerializable a) => KRes.Response -> Either ProtocolError [a]
extractList r = case extractMessage r of
    Right s             -> mapM decodePb $ Data.Foldable.toList $ KList.items s
    Left  ResponseEmpty -> Right []
    Left  err           -> Left  err


-- Dummy extractor that only checks if the response has no error
extractNothing :: KRes.Response -> Either ProtocolError Bool 
extractNothing r = checkError r >> return True


processResponse ::  (KRes.Response -> Either ProtocolError a) -> KRes.Response -> RPCContext a
processResponse extractor res = either (throwM) (return) (extractor res)


sendRequest :: KReq.Request -> RPCContext KRes.Response
sendRequest r = do
    sock <- asks rpcSocket
    liftIO $ sendMsg sock r
    liftIO $ recvResponse sock


recvResponse :: (P.Wire a, P.ReflectDescriptor a) => Socket -> IO a
recvResponse sock = do
    msg <- recvMsg sock
    either (throwM) (return) (messageGet (BL.fromStrict msg))


makeRequest :: String -> String -> [KArg.Argument] -> KReq.Request
makeRequest serviceName procName params =
    KReq.Request
    { service   = Just $ P.fromString serviceName
    , procedure = Just $ P.fromString procName
    , arguments = Seq.fromList params }


makeStream :: KReq.Request -> KReq.Request
makeStream r = makeRequest "KRPC" "AddStream" [KArg.Argument (Just 0) (Just $ messagePut r)]


extractStreamResponse :: KStreamRes.StreamResponse -> Maybe (Int, KRes.Response)
extractStreamResponse streamRes = do
    sid <- KStreamRes.id       streamRes
    res <- KStreamRes.response streamRes
    return $ (fromIntegral sid, res)


extractStreamMessage :: KStreamMsg.StreamMessage -> [(Int, KRes.Response)]
extractStreamMessage msg = catMaybes $ map extractStreamResponse responseList
    where responseList = Data.Foldable.toList (KStreamMsg.responses msg)


makeArgument :: (PbSerializable a) => P.Word32 -> a -> KArg.Argument
makeArgument position arg = KArg.Argument (Just position) (Just $ encodePb arg)


makeArgumentTuple3 :: (PbSerializable a, PbSerializable b, PbSerializable c)
                   => P.Word32
                   -> (a, b, c)
                   -> KArg.Argument
makeArgumentTuple3 position (a, b, c) =
    let
        a'    = encodePb a
        b'    = encodePb b
        c'    = encodePb c
        tuple = KTuple.Tuple $ Seq.fromList [a', b', c']
    in
        KArg.Argument (Just position) (Just $ P.messagePut tuple)


makeArgumentTuple4 :: (PbSerializable a, PbSerializable b, PbSerializable c, PbSerializable d)
                   => P.Word32
                   -> (a, b, c, d)
                   -> KArg.Argument
makeArgumentTuple4 position (a, b, c, d) =
    let
        a'    = encodePb a
        b'    = encodePb b
        c'    = encodePb c
        d'    = encodePb d
        tuple = KTuple.Tuple $ Seq.fromList [a', b', c', d']
    in
        KArg.Argument (Just position) (Just $ P.messagePut tuple)
