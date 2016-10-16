{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.ProcedureCall (ProcedureCall(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.Argument as KRPC (Argument)

data ProcedureCall = ProcedureCall{service :: !(P'.Maybe P'.Utf8), procedure :: !(P'.Maybe P'.Utf8),
                                   arguments :: !(P'.Seq KRPC.Argument)}
                   deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable ProcedureCall where
  mergeAppend (ProcedureCall x'1 x'2 x'3) (ProcedureCall y'1 y'2 y'3)
   = ProcedureCall (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3)

instance P'.Default ProcedureCall where
  defaultValue = ProcedureCall P'.defaultValue P'.defaultValue P'.defaultValue

instance P'.Wire ProcedureCall where
  wireSize ft' self'@(ProcedureCall x'1 x'2 x'3)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 9 x'1 + P'.wireSizeOpt 1 9 x'2 + P'.wireSizeRep 1 11 x'3)
  wirePut ft' self'@(ProcedureCall x'1 x'2 x'3)
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
             P'.wirePutRep 26 11 x'3
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{service = Prelude'.Just new'Field}) (P'.wireGet 9)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{procedure = Prelude'.Just new'Field}) (P'.wireGet 9)
             26 -> Prelude'.fmap (\ !new'Field -> old'Self{arguments = P'.append (arguments old'Self) new'Field}) (P'.wireGet 11)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> ProcedureCall) ProcedureCall where
  getVal m' f' = f' m'

instance P'.GPB ProcedureCall

instance P'.ReflectDescriptor ProcedureCall where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10, 18, 26])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.ProcedureCall\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"ProcedureCall\"}, descFilePath = [\"PB\",\"KRPC\",\"ProcedureCall.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ProcedureCall.service\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ProcedureCall\"], baseName' = FName \"service\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ProcedureCall.procedure\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ProcedureCall\"], baseName' = FName \"procedure\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ProcedureCall.arguments\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ProcedureCall\"], baseName' = FName \"arguments\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 26}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Argument\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Argument\"}), hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType ProcedureCall where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg ProcedureCall where
  textPut msg
   = do
       P'.tellT "service" (service msg)
       P'.tellT "procedure" (procedure msg)
       P'.tellT "arguments" (arguments msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'service, parse'procedure, parse'arguments]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'service
         = P'.try
            (do
               v <- P'.getT "service"
               Prelude'.return (\ o -> o{service = v}))
        parse'procedure
         = P'.try
            (do
               v <- P'.getT "procedure"
               Prelude'.return (\ o -> o{procedure = v}))
        parse'arguments
         = P'.try
            (do
               v <- P'.getT "arguments"
               Prelude'.return (\ o -> o{arguments = P'.append (arguments o) v}))