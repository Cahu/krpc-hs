{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Argument (Argument(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Argument = Argument{position :: !(P'.Maybe P'.Word32), value :: !(P'.Maybe P'.ByteString)}
              deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Argument where
  mergeAppend (Argument x'1 x'2) (Argument y'1 y'2) = Argument (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2)

instance P'.Default Argument where
  defaultValue = Argument (Prelude'.Just 0) P'.defaultValue

instance P'.Wire Argument where
  wireSize ft' self'@(Argument x'1 x'2)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 13 x'1 + P'.wireSizeOpt 1 12 x'2)
  wirePut ft' self'@(Argument x'1 x'2)
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
             P'.wirePutOpt 18 12 x'2
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             8 -> Prelude'.fmap (\ !new'Field -> old'Self{position = Prelude'.Just new'Field}) (P'.wireGet 13)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{value = Prelude'.Just new'Field}) (P'.wireGet 12)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Argument) Argument where
  getVal m' f' = f' m'

instance P'.GPB Argument

instance P'.ReflectDescriptor Argument where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [8, 18])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.Argument\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Argument\"}, descFilePath = [\"PB\",\"KRPC\",\"Argument.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Argument.position\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Argument\"], baseName' = FName \"position\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 8}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 13}, typeName = Nothing, hsRawDefault = Just \"0\", hsDefault = Just (HsDef'Integer 0)},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.Argument.value\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Argument\"], baseName' = FName \"value\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 12}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Argument where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Argument where
  textPut msg
   = do
       P'.tellT "position" (position msg)
       P'.tellT "value" (value msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'position, parse'value]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'position
         = P'.try
            (do
               v <- P'.getT "position"
               Prelude'.return (\ o -> o{position = v}))
        parse'value
         = P'.try
            (do
               v <- P'.getT "value"
               Prelude'.return (\ o -> o{value = v}))