{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Parameter (Parameter(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Parameter = Parameter{name :: !(P'.Maybe P'.Utf8), type' :: !(P'.Maybe P'.Utf8), has_default_argument :: !(P'.Maybe P'.Bool),
                           default_argument :: !(P'.Maybe P'.ByteString)}
               deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Parameter where
  mergeAppend (Parameter x'1 x'2 x'3 x'4) (Parameter y'1 y'2 y'3 y'4)
   = Parameter (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3) (P'.mergeAppend x'4 y'4)

instance P'.Default Parameter where
  defaultValue = Parameter P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue

instance P'.Wire Parameter where
  wireSize ft' self'@(Parameter x'1 x'2 x'3 x'4)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 9 x'1 + P'.wireSizeOpt 1 9 x'2 + P'.wireSizeOpt 1 8 x'3 + P'.wireSizeOpt 1 12 x'4)
  wirePut ft' self'@(Parameter x'1 x'2 x'3 x'4)
   = case ft' of
       10 -> put'Fields
       11 -> do
               P'.putSize (P'.wireSize 10 self')
               put'Fields
       _ -> P'.wirePutErr ft' self'
    where
        put'Fields
         = do
             P'.wirePutOpt 10 9 x'1
             P'.wirePutOpt 18 9 x'2
             P'.wirePutOpt 24 8 x'3
             P'.wirePutOpt 34 12 x'4
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{name = Prelude'.Just new'Field}) (P'.wireGet 9)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{type' = Prelude'.Just new'Field}) (P'.wireGet 9)
             24 -> Prelude'.fmap (\ !new'Field -> old'Self{has_default_argument = Prelude'.Just new'Field}) (P'.wireGet 8)
             34 -> Prelude'.fmap (\ !new'Field -> old'Self{default_argument = Prelude'.Just new'Field}) (P'.wireGet 12)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Parameter) Parameter where
  getVal m' f' = f' m'

instance P'.GPB Parameter

instance P'.ReflectDescriptor Parameter where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10, 18, 24, 34])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.Parameter\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Parameter\"}, descFilePath = [\"PB\",\"KRPC\",\"Parameter.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Parameter.name\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Parameter\"], baseName' = FName \"name\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Parameter.type\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Parameter\"], baseName' = FName \"type'\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Parameter.has_default_argument\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Parameter\"], baseName' = FName \"has_default_argument\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 24}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Parameter.default_argument\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Parameter\"], baseName' = FName \"default_argument\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 4}, wireTag = WireTag {getWireTag = 34}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 12}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Parameter where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Parameter where
  textPut msg
   = do
       P'.tellT "name" (name msg)
       P'.tellT "type" (type' msg)
       P'.tellT "has_default_argument" (has_default_argument msg)
       P'.tellT "default_argument" (default_argument msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'name, parse'type', parse'has_default_argument, parse'default_argument]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'name
         = P'.try
            (do
               v <- P'.getT "name"
               Prelude'.return (\ o -> o{name = v}))
        parse'type'
         = P'.try
            (do
               v <- P'.getT "type"
               Prelude'.return (\ o -> o{type' = v}))
        parse'has_default_argument
         = P'.try
            (do
               v <- P'.getT "has_default_argument"
               Prelude'.return (\ o -> o{has_default_argument = v}))
        parse'default_argument
         = P'.try
            (do
               v <- P'.getT "default_argument"
               Prelude'.return (\ o -> o{default_argument = v}))