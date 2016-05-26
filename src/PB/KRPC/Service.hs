{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Service (Service(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.Class as KRPC (Class)
import qualified PB.KRPC.Enumeration as KRPC (Enumeration)
import qualified PB.KRPC.Procedure as KRPC (Procedure)

data Service = Service{name :: !(P'.Maybe P'.Utf8), procedures :: !(P'.Seq KRPC.Procedure), classes :: !(P'.Seq KRPC.Class),
                       enumerations :: !(P'.Seq KRPC.Enumeration), documentation :: !(P'.Maybe P'.Utf8)}
             deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Service where
  mergeAppend (Service x'1 x'2 x'3 x'4 x'5) (Service y'1 y'2 y'3 y'4 y'5)
   = Service (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3) (P'.mergeAppend x'4 y'4)
      (P'.mergeAppend x'5 y'5)

instance P'.Default Service where
  defaultValue = Service P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue

instance P'.Wire Service where
  wireSize ft' self'@(Service x'1 x'2 x'3 x'4 x'5)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size
         = (P'.wireSizeOpt 1 9 x'1 + P'.wireSizeRep 1 11 x'2 + P'.wireSizeRep 1 11 x'3 + P'.wireSizeRep 1 11 x'4 +
             P'.wireSizeOpt 1 9 x'5)
  wirePut ft' self'@(Service x'1 x'2 x'3 x'4 x'5)
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
             P'.wirePutRep 26 11 x'3
             P'.wirePutRep 34 11 x'4
             P'.wirePutOpt 42 9 x'5
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{name = Prelude'.Just new'Field}) (P'.wireGet 9)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{procedures = P'.append (procedures old'Self) new'Field}) (P'.wireGet 11)
             26 -> Prelude'.fmap (\ !new'Field -> old'Self{classes = P'.append (classes old'Self) new'Field}) (P'.wireGet 11)
             34 -> Prelude'.fmap (\ !new'Field -> old'Self{enumerations = P'.append (enumerations old'Self) new'Field})
                    (P'.wireGet 11)
             42 -> Prelude'.fmap (\ !new'Field -> old'Self{documentation = Prelude'.Just new'Field}) (P'.wireGet 9)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Service) Service where
  getVal m' f' = f' m'

instance P'.GPB Service

instance P'.ReflectDescriptor Service where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10, 18, 26, 34, 42])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.Service\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Service\"}, descFilePath = [\"PB\",\"KRPC\",\"Service.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Service.name\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Service\"], baseName' = FName \"name\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Service.procedures\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Service\"], baseName' = FName \"procedures\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Procedure\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Procedure\"}), hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Service.classes\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Service\"], baseName' = FName \"classes\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 26}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Class\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Class\"}), hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Service.enumerations\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Service\"], baseName' = FName \"enumerations\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 4}, wireTag = WireTag {getWireTag = 34}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Enumeration\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Enumeration\"}), hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Service.documentation\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Service\"], baseName' = FName \"documentation\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 5}, wireTag = WireTag {getWireTag = 42}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Service where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Service where
  textPut msg
   = do
       P'.tellT "name" (name msg)
       P'.tellT "procedures" (procedures msg)
       P'.tellT "classes" (classes msg)
       P'.tellT "enumerations" (enumerations msg)
       P'.tellT "documentation" (documentation msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'name, parse'procedures, parse'classes, parse'enumerations, parse'documentation])
                P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'name
         = P'.try
            (do
               v <- P'.getT "name"
               Prelude'.return (\ o -> o{name = v}))
        parse'procedures
         = P'.try
            (do
               v <- P'.getT "procedures"
               Prelude'.return (\ o -> o{procedures = P'.append (procedures o) v}))
        parse'classes
         = P'.try
            (do
               v <- P'.getT "classes"
               Prelude'.return (\ o -> o{classes = P'.append (classes o) v}))
        parse'enumerations
         = P'.try
            (do
               v <- P'.getT "enumerations"
               Prelude'.return (\ o -> o{enumerations = P'.append (enumerations o) v}))
        parse'documentation
         = P'.try
            (do
               v <- P'.getT "documentation"
               Prelude'.return (\ o -> o{documentation = v}))