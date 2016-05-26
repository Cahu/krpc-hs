{-# LANGUAGE ScopedTypeVariables #-}

module KRPCHS.SerializeUtils
( PbSerializable
, decodePb
, encodePb
) where


import Data.Int
import Data.Word
import Data.Text as T

import qualified Text.ProtocolBuffers             as P
import qualified Text.ProtocolBuffers.Get         as G
import qualified Text.ProtocolBuffers.Basic       as B
import qualified Text.ProtocolBuffers.WireMessage as W


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



decodePb_ :: (W.Wire a) => B.FieldType -> B.ByteString -> Either String a
decodePb_ pbType bytes =
    case W.runGet (W.wireGet pbType) bytes of
        G.Finished _ _ v -> Right v
        G.Failed   _ s   -> Left s
        _                -> Left "partial"


class PbSerializable a where
    decodePb :: B.ByteString -> Either String a
    encodePb :: a -> B.ByteString


instance PbSerializable Double where
    decodePb bytes = realToFrac <$> (decodePb_ 1 bytes :: Either String B.Double)
    encodePb f     = W.runPut (W.wirePut 1 (realToFrac f :: B.Double))


instance PbSerializable Float where
    decodePb bytes = realToFrac <$> (decodePb_ 2 bytes :: Either String B.Float)
    encodePb f     = W.runPut (W.wirePut 2 (realToFrac f :: B.Float))


instance PbSerializable Int where
    decodePb bytes = fromIntegral <$> (decodePb_ 4 bytes :: Either String B.Word64)
    encodePb i     = W.runPut (W.wirePut 4 (fromIntegral i :: B.Word64))


instance PbSerializable Int32 where
    decodePb bytes = fromIntegral <$> (decodePb_ 5 bytes :: Either String B.Int32)
    encodePb i     = W.runPut (W.wirePut 5 (fromIntegral i :: B.Int32))


instance PbSerializable Int64 where
    decodePb bytes = fromIntegral <$> (decodePb_ 3 bytes :: Either String B.Int64)
    encodePb i     = W.runPut (W.wirePut 3 (fromIntegral i :: B.Int64))


instance PbSerializable Word32 where
    decodePb bytes = fromIntegral <$> (decodePb_ 13 bytes :: Either String B.Word32)
    encodePb i     = W.runPut (W.wirePut 13 (fromIntegral i :: B.Word32))


instance PbSerializable Word64 where
    decodePb bytes = fromIntegral <$> (decodePb_ 4 bytes :: Either String B.Word64)
    encodePb i     = W.runPut (W.wirePut 4 (fromIntegral i :: B.Word64))


instance PbSerializable Bool where
    decodePb   = decodePb_ 8
    encodePb b = W.runPut (W.wirePut 8 b)


instance PbSerializable Text where
    decodePb bytes = case B.toUtf8 bytes of
        Left  i -> Left  $ show i
        Right u -> Right $ T.pack $ P.toString u
    encodePb = B.utf8 . P.fromString . T.unpack

