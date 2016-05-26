{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Response (Response(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Response = Response{time :: !(P'.Maybe P'.Double), has_error :: !(P'.Maybe P'.Bool), error :: !(P'.Maybe P'.Utf8),
                         has_return_value :: !(P'.Maybe P'.Bool), return_value :: !(P'.Maybe P'.ByteString)}
              deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Response where
  mergeAppend (Response x'1 x'2 x'3 x'4 x'5) (Response y'1 y'2 y'3 y'4 y'5)
   = Response (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3) (P'.mergeAppend x'4 y'4)
      (P'.mergeAppend x'5 y'5)

instance P'.Default Response where
  defaultValue = Response P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue

instance P'.Wire Response where
  wireSize ft' self'@(Response x'1 x'2 x'3 x'4 x'5)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size
         = (P'.wireSizeOpt 1 1 x'1 + P'.wireSizeOpt 1 8 x'2 + P'.wireSizeOpt 1 9 x'3 + P'.wireSizeOpt 1 8 x'4 +
             P'.wireSizeOpt 1 12 x'5)
  wirePut ft' self'@(Response x'1 x'2 x'3 x'4 x'5)
   = case ft' of
       10 -> put'Fields
       11 -> do
               P'.putSize (P'.wireSize 10 self')
               put'Fields
       _ -> P'.wirePutErr ft' self'
    where
        put'Fields
         = do
             P'.wirePutOpt 9 1 x'1
             P'.wirePutOpt 16 8 x'2
             P'.wirePutOpt 26 9 x'3
             P'.wirePutOpt 32 8 x'4
             P'.wirePutOpt 42 12 x'5
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             9 -> Prelude'.fmap (\ !new'Field -> old'Self{time = Prelude'.Just new'Field}) (P'.wireGet 1)
             16 -> Prelude'.fmap (\ !new'Field -> old'Self{has_error = Prelude'.Just new'Field}) (P'.wireGet 8)
             26 -> Prelude'.fmap (\ !new'Field -> old'Self{error = Prelude'.Just new'Field}) (P'.wireGet 9)
             32 -> Prelude'.fmap (\ !new'Field -> old'Self{has_return_value = Prelude'.Just new'Field}) (P'.wireGet 8)
             42 -> Prelude'.fmap (\ !new'Field -> old'Self{return_value = Prelude'.Just new'Field}) (P'.wireGet 12)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Response) Response where
  getVal m' f' = f' m'

instance P'.GPB Response

instance P'.ReflectDescriptor Response where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [9, 16, 26, 32, 42])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.Response\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Response\"}, descFilePath = [\"PB\",\"KRPC\",\"Response.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Response.time\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Response\"], baseName' = FName \"time\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 9}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 1}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Response.has_error\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Response\"], baseName' = FName \"has_error\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 16}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Response.error\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Response\"], baseName' = FName \"error\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 26}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Response.has_return_value\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Response\"], baseName' = FName \"has_return_value\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 4}, wireTag = WireTag {getWireTag = 32}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Response.return_value\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Response\"], baseName' = FName \"return_value\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 5}, wireTag = WireTag {getWireTag = 42}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 12}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Response where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Response where
  textPut msg
   = do
       P'.tellT "time" (time msg)
       P'.tellT "has_error" (has_error msg)
       P'.tellT "error" (error msg)
       P'.tellT "has_return_value" (has_return_value msg)
       P'.tellT "return_value" (return_value msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'time, parse'has_error, parse'error, parse'has_return_value, parse'return_value])
                P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'time
         = P'.try
            (do
               v <- P'.getT "time"
               Prelude'.return (\ o -> o{time = v}))
        parse'has_error
         = P'.try
            (do
               v <- P'.getT "has_error"
               Prelude'.return (\ o -> o{has_error = v}))
        parse'error
         = P'.try
            (do
               v <- P'.getT "error"
               Prelude'.return (\ o -> o{error = v}))
        parse'has_return_value
         = P'.try
            (do
               v <- P'.getT "has_return_value"
               Prelude'.return (\ o -> o{has_return_value = v}))
        parse'return_value
         = P'.try
            (do
               v <- P'.getT "return_value"
               Prelude'.return (\ o -> o{return_value = v}))