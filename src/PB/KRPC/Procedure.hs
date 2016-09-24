{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Procedure (Procedure(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.Parameter as KRPC (Parameter)
import qualified PB.KRPC.Type as KRPC (Type)

data Procedure = Procedure{name :: !(P'.Maybe P'.Utf8), parameters :: !(P'.Seq KRPC.Parameter),
                           return_type :: !(P'.Maybe KRPC.Type), documentation :: !(P'.Maybe P'.Utf8)}
               deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Procedure where
  mergeAppend (Procedure x'1 x'2 x'3 x'4) (Procedure y'1 y'2 y'3 y'4)
   = Procedure (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3) (P'.mergeAppend x'4 y'4)

instance P'.Default Procedure where
  defaultValue = Procedure P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue

instance P'.Wire Procedure where
  wireSize ft' self'@(Procedure x'1 x'2 x'3 x'4)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 9 x'1 + P'.wireSizeRep 1 11 x'2 + P'.wireSizeOpt 1 11 x'3 + P'.wireSizeOpt 1 9 x'4)
  wirePut ft' self'@(Procedure x'1 x'2 x'3 x'4)
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
             P'.wirePutOpt 26 11 x'3
             P'.wirePutOpt 34 9 x'4
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
             26 -> Prelude'.fmap
                    (\ !new'Field -> old'Self{return_type = P'.mergeAppend (return_type old'Self) (Prelude'.Just new'Field)})
                    (P'.wireGet 11)
             34 -> Prelude'.fmap (\ !new'Field -> old'Self{documentation = Prelude'.Just new'Field}) (P'.wireGet 9)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Procedure) Procedure where
  getVal m' f' = f' m'

instance P'.GPB Procedure

instance P'.ReflectDescriptor Procedure where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10, 18, 26, 34])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".KRPC.Procedure\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Procedure\"}, descFilePath = [\"PB\",\"KRPC\",\"Procedure.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Procedure.name\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"name\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Procedure.parameters\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"parameters\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".KRPC.Parameter\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Parameter\"}), hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Procedure.return_type\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"return_type\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 26}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".KRPC.Type\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Type\"}), hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Procedure.documentation\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Procedure\"], baseName' = FName \"documentation\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 4}, wireTag = WireTag {getWireTag = 34}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Procedure where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Procedure where
  textPut msg
   = do
       P'.tellT "name" (name msg)
       P'.tellT "parameters" (parameters msg)
       P'.tellT "return_type" (return_type msg)
       P'.tellT "documentation" (documentation msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'name, parse'parameters, parse'return_type, parse'documentation]) P'.spaces
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
        parse'return_type
         = P'.try
            (do
               v <- P'.getT "return_type"
               Prelude'.return (\ o -> o{return_type = v}))
        parse'documentation
         = P'.try
            (do
               v <- P'.getT "documentation"
               Prelude'.return (\ o -> o{documentation = v}))