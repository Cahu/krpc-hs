{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.StreamResult (StreamResult(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.ProcedureResult as KRPC (ProcedureResult)

data StreamResult = StreamResult{id :: !(P'.Maybe P'.Word64), result :: !(P'.Maybe KRPC.ProcedureResult)}
                  deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable StreamResult where
  mergeAppend (StreamResult x'1 x'2) (StreamResult y'1 y'2) = StreamResult (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2)

instance P'.Default StreamResult where
  defaultValue = StreamResult (Prelude'.Just 0) P'.defaultValue

instance P'.Wire StreamResult where
  wireSize ft' self'@(StreamResult x'1 x'2)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 4 x'1 + P'.wireSizeOpt 1 11 x'2)
  wirePut ft' self'@(StreamResult x'1 x'2)
   = case ft' of
       10 -> put'Fields
       11 -> do
               P'.putSize (P'.wireSize 10 self')
               put'Fields
       _ -> P'.wirePutErr ft' self'
    where
        put'Fields
         = do
             P'.wirePutOpt 8 4 x'1
             P'.wirePutOpt 18 11 x'2
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             8 -> Prelude'.fmap (\ !new'Field -> old'Self{id = Prelude'.Just new'Field}) (P'.wireGet 4)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{result = P'.mergeAppend (result old'Self) (Prelude'.Just new'Field)})
                    (P'.wireGet 11)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> StreamResult) StreamResult where
  getVal m' f' = f' m'

instance P'.GPB StreamResult

instance P'.ReflectDescriptor StreamResult where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [8, 18])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.StreamResult\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"StreamResult\"}, descFilePath = [\"PB\",\"KRPC\",\"StreamResult.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.StreamResult.id\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"StreamResult\"], baseName' = FName \"id\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 8}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 4}, typeName = Nothing, hsRawDefault = Just \"0\", hsDefault = Just (HsDef'Integer 0)},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.StreamResult.result\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"StreamResult\"], baseName' = FName \"result\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.ProcedureResult\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"ProcedureResult\"}), hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType StreamResult where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg StreamResult where
  textPut msg
   = do
       P'.tellT "id" (id msg)
       P'.tellT "result" (result msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'id, parse'result]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'id
         = P'.try
            (do
               v <- P'.getT "id"
               Prelude'.return (\ o -> o{id = v}))
        parse'result
         = P'.try
            (do
               v <- P'.getT "result"
               Prelude'.return (\ o -> o{result = v}))