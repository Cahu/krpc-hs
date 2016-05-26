{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.StreamResponse (StreamResponse(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.Response as KRPC (Response)

data StreamResponse = StreamResponse{id :: !(P'.Maybe P'.Word32), response :: !(P'.Maybe KRPC.Response)}
                    deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable StreamResponse where
  mergeAppend (StreamResponse x'1 x'2) (StreamResponse y'1 y'2) = StreamResponse (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2)

instance P'.Default StreamResponse where
  defaultValue = StreamResponse P'.defaultValue P'.defaultValue

instance P'.Wire StreamResponse where
  wireSize ft' self'@(StreamResponse x'1 x'2)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 13 x'1 + P'.wireSizeOpt 1 11 x'2)
  wirePut ft' self'@(StreamResponse x'1 x'2)
   = case ft' of
       10 -> put'Fields
       11 -> do
               P'.putSize (P'.wireSize 10 self')
               put'Fields
       _ -> P'.wirePutErr ft' self'
    where
        put'Fields
         = do
             P'.wirePutOpt 8 13 x'1
             P'.wirePutOpt 18 11 x'2
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             8 -> Prelude'.fmap (\ !new'Field -> old'Self{id = Prelude'.Just new'Field}) (P'.wireGet 13)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{response = P'.mergeAppend (response old'Self) (Prelude'.Just new'Field)})
                    (P'.wireGet 11)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> StreamResponse) StreamResponse where
  getVal m' f' = f' m'

instance P'.GPB StreamResponse

instance P'.ReflectDescriptor StreamResponse where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [8, 18])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.StreamResponse\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"StreamResponse\"}, descFilePath = [\"PB\",\"KRPC\",\"StreamResponse.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.StreamResponse.id\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"StreamResponse\"], baseName' = FName \"id\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 8}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 13}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.StreamResponse.response\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"StreamResponse\"], baseName' = FName \"response\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Response\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Response\"}), hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType StreamResponse where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg StreamResponse where
  textPut msg
   = do
       P'.tellT "id" (id msg)
       P'.tellT "response" (response msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'id, parse'response]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'id
         = P'.try
            (do
               v <- P'.getT "id"
               Prelude'.return (\ o -> o{id = v}))
        parse'response
         = P'.try
            (do
               v <- P'.getT "response"
               Prelude'.return (\ o -> o{response = v}))