{-# LANGUAGE RecordWildCards #-}


{- Implementation notes:
 -
 - KRPC uses protocol buffers v3 but the library we use only support v2.
 - Fortunatelly, KRPC doesn't use v3 features so we can adapt the .proto file
 - by adding some annotations ('optional') to fields. However, this has
 - implications. For instance, we are unable to distinguish an empty list from
 - the absence of a list.
 -}

module KRPCHS.Requests
( RPCClient(..)
, StreamClient(..)
, RPCContext
, StreamContext
, runRPCProg
, runStreamProg

, KRPCStream(..)
, KRPCStreamMsg(..)
, getStreamMessage
, getStreamResult

, makeArgument
, makeArgumentTuple3
, makeArgumentTuple4
, processResponse
, makeRequest
, sendRequest
, makeStream
, extractMessage
, extractValue
, extractList
, extractMap
, extractTuple3
, extractTuple4
, extractNothing
) where


import Network.Socket
import Control.Monad.Reader

import KRPCHS.NetworkUtils
import KRPCHS.SerializeUtils

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


type RPCContext    = ReaderT RPCClient    IO
type StreamContext = ReaderT StreamClient IO

data KRPCStream a = KRPCStream
    { streamId        :: Int
    , streamExtractor :: KRes.Response -> Either String a }


data KRPCStreamMsg = KRPCStreamMsg
    { streamMsg :: M.Map Int KRes.Response }


runRPCProg :: RPCClient -> RPCContext a -> IO a
runRPCProg = flip runReaderT


runStreamProg :: StreamClient -> StreamContext a -> IO a
runStreamProg = flip runReaderT


checkError :: KRes.Response -> Either String ()
checkError r = case (KRes.has_error r) of
    Just True -> Left (maybe "Could not extract message" (P.toString) (KRes.error r))
    _         -> return ()


messageGet :: (P.Wire a, P.ReflectDescriptor a) => BL.ByteString -> Either String a
messageGet bytes = either (Left) (return . fst) (P.messageGet bytes)


messagePut :: (P.Wire a, P.ReflectDescriptor a) => a -> BL.ByteString
messagePut = P.messagePut


extractMessage :: (P.Wire a, P.ReflectDescriptor a) => KRes.Response -> Either String a
extractMessage r = do
    checkError r
    maybe (Left "Response with no return value") (messageGet) (KRes.return_value r)


extractValue :: (PbSerializable a) => KRes.Response -> Either String a
extractValue r = do
    checkError r
    maybe (Left "Response with no return value") (decodePb) (KRes.return_value r)


extractTuple3 :: (PbSerializable a, PbSerializable b, PbSerializable c)
              => KRes.Response 
              -> Either String (a, b, c)
extractTuple3 r = do
    s <- extractMessage r
    let (a:b:c:_) = Data.Foldable.toList $ KTuple.items s
    a' <- decodePb a
    b' <- decodePb b
    c' <- decodePb c
    return (a', b', c')


extractTuple4 :: (PbSerializable a, PbSerializable b, PbSerializable c, PbSerializable d)
              => KRes.Response 
              -> Either String (a, b, c, d)
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
           -> Either String (M.Map a b)
extractMap r = case (extractMessage r) of
    -- see notes above for an explanation of why we ignore the error of 'Left' and return empty instead
    Left  _ -> return M.empty
    Right s -> M.fromList <$> mapM extractKeyVal (Data.Foldable.toList $ KDict.entries s)
    where
        extractKeyVal e = do
            k <- maybe (Left "No key") (decodePb) (KDictE.key   e)
            v <- maybe (Left "No val") (decodePb) (KDictE.value e)
            return (k, v)


extractList :: (PbSerializable a) => KRes.Response -> Either String [a]
extractList r = case extractMessage r of
    -- see notes above for an explanation of why we ignore the error of 'Left' and return empty instead
    Left  _ -> return []
    Right s -> mapM decodePb $ Data.Foldable.toList $ KList.items s


-- Dummy extractor that only checks if the response has no error
extractNothing :: KRes.Response -> Either String Bool 
extractNothing r = checkError r >> return True


processResponse ::  (KRes.Response -> Either String a) -> KRes.Response -> RPCContext a
processResponse extractor res = either (fail) (return) (extractor res)


sendRequest :: KReq.Request -> RPCContext KRes.Response
sendRequest r = do
    sock <- asks rpcSocket
    liftIO $ sendMsg sock r
    liftIO $ recvResponse sock


recvResponse :: (P.Wire a, P.ReflectDescriptor a) => Socket -> IO a
recvResponse sock = do
    msg <- recvMsg sock
    either (fail) (return) (messageGet (BL.fromStrict msg))


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

getStreamMessage :: StreamContext KRPCStreamMsg
getStreamMessage = do
    sock <- asks streamSocket
    res  <- liftIO $ recvResponse sock
    return $ KRPCStreamMsg $ M.fromList (extractStreamMessage res)


getStreamResult :: KRPCStream a -> KRPCStreamMsg -> StreamContext (Either String a)
getStreamResult KRPCStream{..} KRPCStreamMsg{..} = return $
    maybe (Left "No result for this stream") (streamExtractor) (M.lookup streamId streamMsg)


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
