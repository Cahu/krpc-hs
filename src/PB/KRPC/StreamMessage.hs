{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.StreamMessage (StreamMessage(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.StreamResponse as KRPC (StreamResponse)

data StreamMessage = StreamMessage{responses :: !(P'.Seq KRPC.StreamResponse)}
                   deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable StreamMessage where
  mergeAppend (StreamMessage x'1) (StreamMessage y'1) = StreamMessage (P'.mergeAppend x'1 y'1)

instance P'.Default StreamMessage where
  defaultValue = StreamMessage P'.defaultValue

instance P'.Wire StreamMessage where
  wireSize ft' self'@(StreamMessage x'1)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeRep 1 11 x'1)
  wirePut ft' self'@(StreamMessage x'1)
   = case ft' of
       10 -> put'Fields
       11 -> do
               P'.putSize (P'.wireSize 10 self')
               put'Fields
       _ -> P'.wirePutErr ft' self'
    where
        put'Fields
         = do
             P'.wirePutRep 10 11 x'1
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{responses = P'.append (responses old'Self) new'Field}) (P'.wireGet 11)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> StreamMessage) StreamMessage where
  getVal m' f' = f' m'

instance P'.GPB StreamMessage

instance P'.ReflectDescriptor StreamMessage where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.StreamMessage\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"StreamMessage\"}, descFilePath = [\"PB\",\"KRPC\",\"StreamMessage.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.StreamMessage.responses\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"StreamMessage\"], baseName' = FName \"responses\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.StreamResponse\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"StreamResponse\"}), hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType StreamMessage where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg StreamMessage where
  textPut msg
   = do
       P'.tellT "responses" (responses msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'responses]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'responses
         = P'.try
            (do
               v <- P'.getT "responses"
               Prelude'.return (\ o -> o{responses = P'.append (responses o) v}))