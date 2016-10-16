{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.ConnectionRequest (ConnectionRequest(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'
import qualified PB.KRPC.ConnectionRequest.Type as KRPC.ConnectionRequest (Type)

data ConnectionRequest = ConnectionRequest{type' :: !(P'.Maybe KRPC.ConnectionRequest.Type), client_name :: !(P'.Maybe P'.Utf8),
                                           client_identifier :: !(P'.Maybe P'.ByteString)}
                       deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable ConnectionRequest where
  mergeAppend (ConnectionRequest x'1 x'2 x'3) (ConnectionRequest y'1 y'2 y'3)
   = ConnectionRequest (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3)

instance P'.Default ConnectionRequest where
  defaultValue = ConnectionRequest (Prelude'.Just (Prelude'.read "RPC")) P'.defaultValue P'.defaultValue

instance P'.Wire ConnectionRequest where
  wireSize ft' self'@(ConnectionRequest x'1 x'2 x'3)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size = (P'.wireSizeOpt 1 14 x'1 + P'.wireSizeOpt 1 9 x'2 + P'.wireSizeOpt 1 12 x'3)
  wirePut ft' self'@(ConnectionRequest x'1 x'2 x'3)
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
             8 -> Prelude'.fmap (\ !new'Field -> old'Self{type' = Prelude'.Just new'Field}) (P'.wireGet 14)
             18 -> Prelude'.fmap (\ !new'Field -> old'Self{client_name = Prelude'.Just new'Field}) (P'.wireGet 9)
             26 -> Prelude'.fmap (\ !new'Field -> old'Self{client_identifier = Prelude'.Just new'Field}) (P'.wireGet 12)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> ConnectionRequest) ConnectionRequest where
  getVal m' f' = f' m'

instance P'.GPB ConnectionRequest

instance P'.ReflectDescriptor ConnectionRequest where
  getMessageInfo _ = P'.GetMessageInfo (P'.fromDistinctAscList []) (P'.fromDistinctAscList [8, 18, 26])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".krpc.schema.ConnectionRequest\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"ConnectionRequest\"}, descFilePath = [\"PB\",\"KRPC\",\"ConnectionRequest.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ConnectionRequest.type\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ConnectionRequest\"], baseName' = FName \"type'\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 8}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 14}, typeName = Just (ProtoName {protobufName = FIName \".krpc.schema.ConnectionRequest.Type\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\",MName \"ConnectionRequest\"], baseName = MName \"Type\"}), hsRawDefault = Just \"RPC\", hsDefault = Just (HsDef'Enum \"RPC\")},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ConnectionRequest.client_name\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ConnectionRequest\"], baseName' = FName \"client_name\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 18}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".krpc.schema.ConnectionRequest.client_identifier\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"ConnectionRequest\"], baseName' = FName \"client_identifier\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 26}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 12}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType ConnectionRequest where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg ConnectionRequest where
  textPut msg
   = do
       P'.tellT "type" (type' msg)
       P'.tellT "client_name" (client_name msg)
       P'.tellT "client_identifier" (client_identifier msg)
  textGet
   = do
       mods <- P'.sepEndBy (P'.choice [parse'type', parse'client_name, parse'client_identifier]) P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'type'
         = P'.try
            (do
               v <- P'.getT "type"
               Prelude'.return (\ o -> o{type' = v}))
        parse'client_name
         = P'.try
            (do
               v <- P'.getT "client_name"
               Prelude'.return (\ o -> o{client_name = v}))
        parse'client_identifier
         = P'.try
            (do
               v <- P'.getT "client_identifier"
               Prelude'.return (\ o -> o{client_identifier = v}))