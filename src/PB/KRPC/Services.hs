{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Services (Services(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.Service as KRPC (Service)

data Services = Services{services :: !(P'.Seq KRPC.Service)}
              deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Services where
  mergeAppend (Services x'1) (Services y'1) = Services (P'.mergeAppend x'1 y'1)

instance P'.Default Services where
  defaultValue = Services P'.defaultValue

instance P'.Wire Services where
  wireSize ft' self'@(Services x'1)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeRep 1 11 x'1)
  wirePut ft' self'@(Services x'1)
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
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{services = P'.append (services old'Self) new'Field}) (P'.wireGet 11)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Services) Services where
  getVal m' f' = f' m'

instance P'.GPB Services

instance P'.ReflectDescriptor Services where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [10])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.Services\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Services\"}, descFilePath = [\"PB\",\"KRPC\",\"Services.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Services.services\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Services\"], baseName' = FName \"services\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = True, mightPack = False, typeCode = FieldType {getFieldType = 11}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.Service\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Service\"}), hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Services where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Services where
  textPut msg
   = do
       P'.tellT "services" (services msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'services]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'services
         = P'.try
            (do
               v <- P'.getT "services"
               Prelude'.return (\ o -> o{services = P'.append (services o) v}))