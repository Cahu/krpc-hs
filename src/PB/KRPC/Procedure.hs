{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Procedure (Procedure(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.Parameter as KRPC (Parameter)

data Procedure = Procedure{name :: !(P'.Maybe P'.Utf8), parameters :: !(P'.Seq KRPC.Parameter),
                           has_return_type :: !(P'.Maybe P'.Bool), return_type :: !(P'.Maybe P'.Utf8),
                           attributes :: !(P'.Seq P'.Utf8), documentation :: !(P'.Maybe P'.Utf8)}
               deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Procedure where
  mergeAppend (Procedure x'1 x'2 x'3 x'4 x'5 x'6) (Procedure y'1 y'2 y'3 y'4 y'5 y'6)
   = Procedure (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3) (P'.mergeAppend x'4 y'4)
      (P'.mergeAppend x'5 y'5)
      (P'.mergeAppend x'6 y'6)

instance P'.Default Procedure where
  defaultValue = Procedure P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue

instance P'.Wire Procedure where
  wireSize ft' self'@(Procedure x'1 x'2 x'3 x'4 x'5 x'6)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size
         = (P'.wireSizeOpt 1 9 x'1 + P'.wireSizeRep 1 11 x'2 + P'.wireSizeOpt 1 8 x'3 + P'.wireSizeOpt 1 9 x'4 +
             P'.wireSizeRep 1 9 x'5
             + P'.wireSizeOpt 1 9 x'6)
  wirePut ft' self'@(Procedure x'1 x'2 x'3 x'4 x'5 x'6)
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
             P'.wirePutRep 18 11 x'2
             P'.wirePutOpt 24 8 x'3
             P'.wirePutOpt 34 9 x'4
             P'.wirePutRep 42 9 x'5
             P'.wirePutOpt 50 9 x'6
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{name = Prelude'.Just new'Field}) (P'.wireGet 9)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{parameters = P'.append (parameters old'Self) new'Field}) (P'.wireGet 11)
             24 -> Prelude'.fmap (\ !new'Field -> old'Self{has_return_type = Prelude'.Just new'Field}) (P'.wireGet 8)
             34 -> Prelude'.fmap (\ !new'Field -> old'Self{return_type = Prelude'.Just new'Field}) (P'.wireGet 9)
             42 -> Prelude'.fmap (\ !new'Field -> old'Self{attributes = P'.append (attributes old'Self) new'Field}) (P'.wireGet 9)
             50 -> Prelude'.fmap (\ !new'Field -> old'Self{documentation = Prelude'.Just new'Field}) (P'.wireGet 9)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Procedure) Procedure where
  getVal m' f' = f' m'

instance P'.GPB Procedure

instance P'.ReflectDescriptor Procedure where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10, 18, 24, 34, 42, 50])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.Procedure\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Procedure\"}, descFilePath = [\"PB\",\"KRPC\",\"Procedure.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Procedure.name\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"name\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Procedure.parameters\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"parameters\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Parameter\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Parameter\"}), hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Procedure.has_return_type\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"has_return_type\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 24}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Procedure.return_type\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"return_type\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 4}, wireTag = WireTag {getWireTag = 34}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Procedure.attributes\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"attributes\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 5}, wireTag = WireTag {getWireTag = 42}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Procedure.documentation\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"documentation\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 6}, wireTag = WireTag {getWireTag = 50}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Procedure where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Procedure where
  textPut msg
   = do
       P'.tellT "name" (name msg)
       P'.tellT "parameters" (parameters msg)
       P'.tellT "has_return_type" (has_return_type msg)
       P'.tellT "return_type" (return_type msg)
       P'.tellT "attributes" (attributes msg)
       P'.tellT "documentation" (documentation msg)
  textGet
   = do
       mods <- P'.sepEndBy
                (P'.choice
                  [parse'name, parse'parameters, parse'has_return_type, parse'return_type, parse'attributes, parse'documentation])
                P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'name
         = P'.try
            (do
               v <- P'.getT "name"
               Prelude'.return (\ o -> o{name = v}))
        parse'parameters
         = P'.try
            (do
               v <- P'.getT "parameters"
               Prelude'.return (\ o -> o{parameters = P'.append (parameters o) v}))
        parse'has_return_type
         = P'.try
            (do
               v <- P'.getT "has_return_type"
               Prelude'.return (\ o -> o{has_return_type = v}))
        parse'return_type
         = P'.try
            (do
               v <- P'.getT "return_type"
               Prelude'.return (\ o -> o{return_type = v}))
        parse'attributes
         = P'.try
            (do
               v <- P'.getT "attributes"
               Prelude'.return (\ o -> o{attributes = P'.append (attributes o) v}))
        parse'documentation
         = P'.try
            (do
               v <- P'.getT "documentation"
               Prelude'.return (\ o -> o{documentation = v}))