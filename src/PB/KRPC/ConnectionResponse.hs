{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.ConnectionResponse (ConnectionResponse(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.ConnectionResponse.Status as KRPC.ConnectionResponse (Status)

data ConnectionResponse = ConnectionResponse{status :: !(P'.Maybe KRPC.ConnectionResponse.Status), message :: !(P'.Maybe P'.Utf8),
                                             client_identifier :: !(P'.Maybe P'.ByteString)}
                        deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable ConnectionResponse where
  mergeAppend (ConnectionResponse x'1 x'2 x'3) (ConnectionResponse y'1 y'2 y'3)
   = ConnectionResponse (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3)

instance P'.Default ConnectionResponse where
  defaultValue = ConnectionResponse (Prelude'.Just (Prelude'.read "OK")) P'.defaultValue P'.defaultValue

instance P'.Wire ConnectionResponse where
  wireSize ft' self'@(ConnectionResponse x'1 x'2 x'3)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 14 x'1 + P'.wireSizeOpt 1 9 x'2 + P'.wireSizeOpt 1 12 x'3)
  wirePut ft' self'@(ConnectionResponse x'1 x'2 x'3)
   = case ft' of
       10 -> put'Fields
       11 -> do
               P'.putSize (P'.wireSize 10 self')
               put'Fields
       _ -> P'.wirePutErr ft' self'
    where
        put'Fields
         = do
             P'.wirePutOpt 8 14 x'1
             P'.wirePutOpt 18 9 x'2
             P'.wirePutOpt 26 12 x'3
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             8 -> Prelude'.fmap (\ !new'Field -> old'Self{status = Prelude'.Just new'Field}) (P'.wireGet 14)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{message = Prelude'.Just new'Field}) (P'.wireGet 9)
             26 -> Prelude'.fmap (\ !new'Field -> old'Self{client_identifier = Prelude'.Just new'Field}) (P'.wireGet 12)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> ConnectionResponse) ConnectionResponse where
  getVal m' f' = f' m'

instance P'.GPB ConnectionResponse

instance P'.ReflectDescriptor ConnectionResponse where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [8, 18, 26])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.ConnectionResponse\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"ConnectionResponse\"}, descFilePath = [\"PB\",\"KRPC\",\"ConnectionResponse.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ConnectionResponse.status\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ConnectionResponse\"], baseName' = FName \"status\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 8}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 14}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.ConnectionResponse.Status\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\",MName \"ConnectionResponse\"], baseName = MName \"Status\"}), hsRawDefault = Just \"OK\", hsDefault = Just (HsDef'Enum \"OK\")},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ConnectionResponse.message\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ConnectionResponse\"], baseName' = FName \"message\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ConnectionResponse.client_identifier\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ConnectionResponse\"], baseName' = FName \"client_identifier\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 26}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 12}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType ConnectionResponse where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg ConnectionResponse where
  textPut msg
   = do
       P'.tellT "status" (status msg)
       P'.tellT "message" (message msg)
       P'.tellT "client_identifier" (client_identifier msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'status, parse'message, parse'client_identifier]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'status
         = P'.try
            (do
               v <- P'.getT "status"
               Prelude'.return (\ o -> o{status = v}))
        parse'message
         = P'.try
            (do
               v <- P'.getT "message"
               Prelude'.return (\ o -> o{message = v}))
        parse'client_identifier
         = P'.try
            (do
               v <- P'.getT "client_identifier"
               Prelude'.return (\ o -> o{client_identifier = v}))