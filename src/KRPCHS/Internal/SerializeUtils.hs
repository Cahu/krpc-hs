{-# LANGUAGE ScopedTypeVariables #-}

module KRPCHS.Internal.SerializeUtils
( PbEncodable(..)
, PbDecodable(..)
, messagePut
, messageGet

, B.Utf8
, packUtf8String
, unpackUtf8String

, KRPCResponseExtractable(..)
) where


import Data.Int
import Data.Word
import qualified Data.Text as T
import qualified Data.ByteString.Lazy as BL

import qualified Data.Foldable
import qualified Data.Sequence as Seq
import qualified Data.Set      as Set
import qualified Data.Map      as M

import KRPCHS.Internal.ProtocolError

import qualified Text.ProtocolBuffers             as P
import qualified Text.ProtocolBuffers.Get         as G
import qualified Text.ProtocolBuffers.Basic       as B
import qualified Text.ProtocolBuffers.WireMessage as W

import qualified PB.KRPC.Stream          as KStr
import qualified PB.KRPC.ProcedureResult as KPRes
import qualified PB.KRPC.Set             as KSet
import qualified PB.KRPC.List            as KList
import qualified PB.KRPC.Tuple           as KTuple
import qualified PB.KRPC.Dictionary      as KDict
import qualified PB.KRPC.DictionaryEntry as KDictE

{- This is a wrapper around Text.ProtocolBuffers' internal (de-)serialization
 - functions. As far as I know, there is no better way to extract ints and
 - floats from 'bytes' types in messages

   // 0 is reserved for errors.
   // Order is weird for historical reasons.
   TYPE_DOUBLE         = 1;
   TYPE_FLOAT          = 2;
   TYPE_INT64          = 3;   // Not ZigZag encoded.  Negative numbers
                              // take 10 bytes.  Use TYPE_SINT64 if negative
                              // values are likely.
   TYPE_UINT64         = 4;
   TYPE_INT32          = 5;   // Not ZigZag encoded.  Negative numbers
                              // take 10 bytes.  Use TYPE_SINT32 if negative
                              // values are likely.
   TYPE_FIXED64        = 6;
   TYPE_FIXED32        = 7;
   TYPE_BOOL           = 8;
   TYPE_STRING         = 9;
   TYPE_GROUP          = 10;  // Tag-delimited aggregate.
   TYPE_MESSAGE        = 11;  // Length-delimited aggregate.

   // New in version 2.
   TYPE_BYTES          = 12;
   TYPE_UINT32         = 13;
   TYPE_ENUM           = 14;
   TYPE_SFIXED32       = 15;
   TYPE_SFIXED64       = 16;
   TYPE_SINT32         = 17;  // Uses ZigZag encoding.
   TYPE_SINT64         = 18;  // Uses ZigZag encoding.-
 - -}



messageGet :: (P.Wire a, P.ReflectDescriptor a) => B.ByteString -> Either ProtocolError a
messageGet bytes = either (Left . DecodeFailure) (return . fst) (P.messageGet bytes)


messagePut :: (P.Wire a, P.ReflectDescriptor a) => a -> B.ByteString
messagePut = P.messagePut


packUtf8String :: String -> B.Utf8
packUtf8String = P.fromString


unpackUtf8String :: B.Utf8 -> String
unpackUtf8String = P.toString


decodePb_ :: (W.Wire a) => B.FieldType -> B.ByteString -> Either ProtocolError a
decodePb_ pbType bytes =
    case W.runGet (W.wireGet pbType) bytes of
        G.Finished _ _ v -> Right v
        G.Failed   _ s   -> Left $ DecodeFailure s
        _                -> Left $ DecodeFailure "partial"

class PbDecodable a where
    decodePb :: B.ByteString -> Either ProtocolError a

class PbEncodable a where
    encodePb :: a -> B.ByteString


instance PbDecodable () where decodePb _ = Right ()
instance PbEncodable () where encodePb _ = BL.empty


instance PbDecodable KStr.Stream where decodePb = messageGet
instance PbEncodable KStr.Stream where encodePb = messagePut


instance PbDecodable Double where
    decodePb bytes = realToFrac <$> (decodePb_ 1 bytes :: Either ProtocolError B.Double)

instance PbEncodable Double where
    encodePb f = W.runPut (W.wirePut 1 (realToFrac f :: B.Double))


instance PbDecodable Float where
    decodePb bytes = realToFrac <$> (decodePb_ 2 bytes :: Either ProtocolError B.Float)

instance PbEncodable Float where
    encodePb f = W.runPut (W.wirePut 2 (realToFrac f :: B.Float))


instance PbDecodable Int where
    decodePb bytes = fromIntegral <$> (decodePb_ 4 bytes :: Either ProtocolError B.Word64)

instance PbEncodable Int where
    encodePb i = W.runPut (W.wirePut 4 (fromIntegral i :: B.Word64))


instance PbDecodable Int32 where
    decodePb bytes = fromIntegral <$> (decodePb_ 5 bytes :: Either ProtocolError B.Int32)

instance PbEncodable Int32 where
    encodePb i = W.runPut (W.wirePut 5 (fromIntegral i :: B.Int32))


instance PbDecodable Int64 where
    decodePb bytes = fromIntegral <$> (decodePb_ 3 bytes :: Either ProtocolError B.Int64)

instance PbEncodable Int64 where
    encodePb i = W.runPut (W.wirePut 3 (fromIntegral i :: B.Int64))


instance PbDecodable Word32 where
    decodePb bytes = fromIntegral <$> (decodePb_ 13 bytes :: Either ProtocolError B.Word32)

instance PbEncodable Word32 where
    encodePb i = W.runPut (W.wirePut 13 (fromIntegral i :: B.Word32))


instance PbDecodable Word64 where
    decodePb bytes = fromIntegral <$> (decodePb_ 4 bytes :: Either ProtocolError B.Word64)

instance PbEncodable Word64 where
    encodePb i = W.runPut (W.wirePut 4 (fromIntegral i :: B.Word64))


instance PbDecodable Bool where
    decodePb = decodePb_ 8

instance PbEncodable Bool where
    encodePb b = W.runPut (W.wirePut 8 b)


instance PbDecodable T.Text where
    decodePb bytes = (T.pack . unpackUtf8String) <$> (decodePb_ 9 bytes :: Either ProtocolError B.Utf8)

instance PbEncodable T.Text where
    encodePb t = W.runPut (W.wirePut 9 (packUtf8String $ T.unpack t))


instance (PbEncodable a) => PbEncodable [a] where
    encodePb = messagePut . KList.List . Seq.fromList . map encodePb

instance (PbDecodable a) => PbDecodable [a] where
    decodePb bytes = do
        l <- messageGet bytes
        mapM decodePb $ Data.Foldable.toList (KList.items l)


instance (PbEncodable a) => PbEncodable (Set.Set a) where
    encodePb = messagePut . KSet.Set . Seq.fromList . map encodePb . Set.toList

instance (Ord a, PbDecodable a) => PbDecodable (Set.Set a) where
    decodePb bytes = do
        m <- messageGet bytes
        Set.fromList <$> mapM decodePb (Data.Foldable.toList (KSet.items m))


instance (Ord k, PbDecodable k, PbDecodable v) => PbDecodable (M.Map k v) where
    decodePb bytes = do
        m <- messageGet bytes
        M.fromList <$> mapM extractKeyVal (Data.Foldable.toList $ KDict.entries m)
        where
            extractKeyVal e = do
                k <- maybe (Left $ DecodeFailure "No key") (decodePb) (KDictE.key   e)
                v <- maybe (Left $ DecodeFailure "No val") (decodePb) (KDictE.value e)
                return (k, v)


instance (PbEncodable a, PbEncodable b) => PbEncodable (a, b) where
    encodePb (a, b) =
        let a' = encodePb a
            b' = encodePb b
            tuple = KTuple.Tuple $ Seq.fromList [a', b']
        in
            messagePut tuple

instance (PbDecodable a, PbDecodable b) => PbDecodable (a, b) where
    decodePb bytes = do
        s <- messageGet bytes
        let (a:b:_) = Data.Foldable.toList $ KTuple.items s
        a' <- decodePb a
        b' <- decodePb b
        return (a', b')


instance (PbEncodable a, PbEncodable b, PbEncodable c) => PbEncodable (a, b, c) where
    encodePb (a, b, c) =
        let a' = encodePb a
            b' = encodePb b
            c' = encodePb c
            tuple = KTuple.Tuple $ Seq.fromList [a', b', c']
        in
            messagePut tuple

instance (PbDecodable a, PbDecodable b, PbDecodable c) => PbDecodable (a, b, c) where
    decodePb bytes = do
        s <- messageGet bytes
        let (a:b:c:_) = Data.Foldable.toList $ KTuple.items s
        a' <- decodePb a
        b' <- decodePb b
        c' <- decodePb c
        return (a', b', c')


instance (PbEncodable a, PbEncodable b, PbEncodable c, PbEncodable d) => PbEncodable (a, b, c, d) where
    encodePb (a, b, c, d) = 
        let a' = encodePb a
            b' = encodePb b
            c' = encodePb c
            d' = encodePb d
            tuple = KTuple.Tuple $ Seq.fromList [a', b', c', d']
        in
            messagePut tuple

instance (PbDecodable a, PbDecodable b, PbDecodable c, PbDecodable d) => PbDecodable (a, b, c, d) where
    decodePb bytes = do
        s <- messageGet bytes
        let (a:b:c:d:_) = Data.Foldable.toList $ KTuple.items s
        a' <- decodePb a
        b' <- decodePb b
        c' <- decodePb c
        d' <- decodePb d
        return (a', b', c', d')


checkProcedureResultError :: KPRes.ProcedureResult -> Either ProtocolError ()
checkProcedureResultError r = case (KPRes.error r) of
    Just err -> Left $ KRPCError (show err)
    Nothing  -> return ()


-- | Class of things that can be deserialized from the body of a
-- ProcedureResult message.
class (PbDecodable a) => KRPCResponseExtractable a where
    extract :: KPRes.ProcedureResult -> Either ProtocolError a
    extract r = do
        checkProcedureResultError r
        maybe (Left ResponseEmpty) (decodePb) (KPRes.value r)


instance KRPCResponseExtractable Bool
instance KRPCResponseExtractable Float
instance KRPCResponseExtractable Double
instance KRPCResponseExtractable Int
instance KRPCResponseExtractable Int32
instance KRPCResponseExtractable Int64
instance KRPCResponseExtractable Word32
instance KRPCResponseExtractable Word64
instance KRPCResponseExtractable T.Text
instance (PbDecodable a, PbDecodable b)                               => KRPCResponseExtractable (a, b)
instance (PbDecodable a, PbDecodable b, PbDecodable c)                => KRPCResponseExtractable (a, b, c)
instance (PbDecodable a, PbDecodable b, PbDecodable c, PbDecodable d) => KRPCResponseExtractable (a, b, c, d)

-- as of krpc 0.4, stream ids are encapsulated in a message
instance KRPCResponseExtractable KStr.Stream


instance KRPCResponseExtractable () where
    extract = checkProcedureResultError


instance (PbDecodable a) => KRPCResponseExtractable [a] where
    extract r = do
        checkProcedureResultError r
        case (KPRes.value r) of
            Nothing    -> Right []
            Just bytes -> decodePb bytes


instance (Ord a, PbDecodable a) => KRPCResponseExtractable (Set.Set a) where
    extract r = do
        checkProcedureResultError r
        case (KPRes.value r) of
            Nothing    -> Right Set.empty
            Just bytes -> decodePb bytes


instance (Ord k, PbDecodable k, PbDecodable v) => KRPCResponseExtractable (M.Map k v) where
    extract r = do
        checkProcedureResultError r
        case (KPRes.value r) of
            Nothing    -> Right M.empty
            Just bytes -> decodePb bytes
