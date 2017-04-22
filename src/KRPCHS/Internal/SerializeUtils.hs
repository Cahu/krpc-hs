{-# LANGUAGE ScopedTypeVariables #-}

module KRPCHS.Internal.SerializeUtils
( PbSerializable
, decodePb
, encodePb
, messagePut
, messageGet
) where


import Data.Int
import Data.Word
import qualified Data.Text as T

import qualified Data.Foldable
import qualified Data.Set      as Set
import qualified Data.Sequence as Seq
import qualified Data.Map      as M

import KRPCHS.Internal.ProtocolError

import qualified Text.ProtocolBuffers             as P
import qualified Text.ProtocolBuffers.Get         as G
import qualified Text.ProtocolBuffers.Basic       as B
import qualified Text.ProtocolBuffers.WireMessage as W

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


decodePb_ :: (W.Wire a) => B.FieldType -> B.ByteString -> Either ProtocolError a
decodePb_ pbType bytes =
    case W.runGet (W.wireGet pbType) bytes of
        G.Finished _ _ v -> Right v
        G.Failed   _ s   -> Left $ DecodeFailure s
        _                -> Left $ DecodeFailure "partial"


class PbSerializable a where
    decodePb :: B.ByteString -> Either ProtocolError a
    encodePb :: a -> B.ByteString


instance PbSerializable () where
    decodePb _ = Right ()
    encodePb _ = undefined


instance PbSerializable Double where
    decodePb bytes = realToFrac <$> (decodePb_ 1 bytes :: Either ProtocolError B.Double)
    encodePb f     = W.runPut (W.wirePut 1 (realToFrac f :: B.Double))


instance PbSerializable Float where
    decodePb bytes = realToFrac <$> (decodePb_ 2 bytes :: Either ProtocolError B.Float)
    encodePb f     = W.runPut (W.wirePut 2 (realToFrac f :: B.Float))


instance PbSerializable Int where
    decodePb bytes = fromIntegral <$> (decodePb_ 4 bytes :: Either ProtocolError B.Word64)
    encodePb i     = W.runPut (W.wirePut 4 (fromIntegral i :: B.Word64))


instance PbSerializable Int32 where
    decodePb bytes = fromIntegral <$> (decodePb_ 5 bytes :: Either ProtocolError B.Int32)
    encodePb i     = W.runPut (W.wirePut 5 (fromIntegral i :: B.Int32))


instance PbSerializable Int64 where
    decodePb bytes = fromIntegral <$> (decodePb_ 3 bytes :: Either ProtocolError B.Int64)
    encodePb i     = W.runPut (W.wirePut 3 (fromIntegral i :: B.Int64))


instance PbSerializable Word32 where
    decodePb bytes = fromIntegral <$> (decodePb_ 13 bytes :: Either ProtocolError B.Word32)
    encodePb i     = W.runPut (W.wirePut 13 (fromIntegral i :: B.Word32))


instance PbSerializable Word64 where
    decodePb bytes = fromIntegral <$> (decodePb_ 4 bytes :: Either ProtocolError B.Word64)
    encodePb i     = W.runPut (W.wirePut 4 (fromIntegral i :: B.Word64))


instance PbSerializable Bool where
    decodePb   = decodePb_ 8
    encodePb b = W.runPut (W.wirePut 8 b)


instance PbSerializable T.Text where
    decodePb bytes = (T.pack . P.toString) <$> (decodePb_ 9 bytes :: Either ProtocolError B.Utf8)
    encodePb t     = W.runPut (W.wirePut 9 (P.fromString $ T.unpack t))


instance (PbSerializable a) => PbSerializable [a] where
    encodePb = messagePut . KList.List . Seq.fromList . map encodePb
    decodePb bytes = do
        l <- messageGet bytes
        mapM decodePb $ Data.Foldable.toList (KList.items l)


instance (Ord a, PbSerializable a) => PbSerializable (Set.Set a) where
    encodePb = messagePut . KSet.Set . Seq.fromList . map encodePb . Set.toList
    decodePb bytes = do
        m <- messageGet bytes
        Set.fromList <$> mapM decodePb (Data.Foldable.toList (KSet.items m))


instance (Ord k, PbSerializable k, PbSerializable v) => PbSerializable (M.Map k v) where
    encodePb = undefined
    decodePb bytes = do
        m <- messageGet bytes
        M.fromList <$> mapM extractKeyVal (Data.Foldable.toList $ KDict.entries m)
        where
            extractKeyVal e = do
                k <- maybe (Left $ DecodeFailure "No key") (decodePb) (KDictE.key   e)
                v <- maybe (Left $ DecodeFailure "No val") (decodePb) (KDictE.value e)
                return (k, v)


instance (PbSerializable a, PbSerializable b) => PbSerializable (a, b) where
    encodePb (a, b) =
        let a' = encodePb a
            b' = encodePb b
            tuple = KTuple.Tuple $ Seq.fromList [a', b']
        in
            messagePut tuple

    decodePb bytes = do
        s <- messageGet bytes
        let (a:b:_) = Data.Foldable.toList $ KTuple.items s
        a' <- decodePb a
        b' <- decodePb b
        return (a', b')


instance (PbSerializable a, PbSerializable b, PbSerializable c) => PbSerializable (a, b, c) where
    encodePb (a, b, c) =
        let a' = encodePb a
            b' = encodePb b
            c' = encodePb c
            tuple = KTuple.Tuple $ Seq.fromList [a', b', c']
        in
            messagePut tuple

    decodePb bytes = do
        s <- messageGet bytes
        let (a:b:c:_) = Data.Foldable.toList $ KTuple.items s
        a' <- decodePb a
        b' <- decodePb b
        c' <- decodePb c
        return (a', b', c')


instance (PbSerializable a, PbSerializable b, PbSerializable c, PbSerializable d) => PbSerializable (a, b, c, d) where
    encodePb (a, b, c, d) = 
        let a' = encodePb a
            b' = encodePb b
            c' = encodePb c
            d' = encodePb d
            tuple = KTuple.Tuple $ Seq.fromList [a', b', c', d']
        in
            messagePut tuple

    decodePb bytes = do
        s <- messageGet bytes
        let (a:b:c:d:_) = Data.Foldable.toList $ KTuple.items s
        a' <- decodePb a
        b' <- decodePb b
        c' <- decodePb c
        d' <- decodePb d
        return (a', b', c', d')
