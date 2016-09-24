{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Status (Status(..)) where
import Prelude ((+), (/))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Status = Status{version :: !(P'.Maybe P'.Utf8), bytes_read :: !(P'.Maybe P'.Word64), bytes_written :: !(P'.Maybe P'.Word64),
                     bytes_read_rate :: !(P'.Maybe P'.Float), bytes_written_rate :: !(P'.Maybe P'.Float),
                     rpcs_executed :: !(P'.Maybe P'.Word64), rpc_rate :: !(P'.Maybe P'.Float),
                     one_rpc_per_update :: !(P'.Maybe P'.Bool), max_time_per_update :: !(P'.Maybe P'.Word32),
                     adaptive_rate_control :: !(P'.Maybe P'.Bool), blocking_recv :: !(P'.Maybe P'.Bool),
                     recv_timeout :: !(P'.Maybe P'.Word32), time_per_rpc_update :: !(P'.Maybe P'.Float),
                     poll_time_per_rpc_update :: !(P'.Maybe P'.Float), exec_time_per_rpc_update :: !(P'.Maybe P'.Float),
                     stream_rpcs :: !(P'.Maybe P'.Word32), stream_rpcs_executed :: !(P'.Maybe P'.Word64),
                     stream_rpc_rate :: !(P'.Maybe P'.Float), time_per_stream_update :: !(P'.Maybe P'.Float)}
            deriving (Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Status where
  mergeAppend (Status x'1 x'2 x'3 x'4 x'5 x'6 x'7 x'8 x'9 x'10 x'11 x'12 x'13 x'14 x'15 x'16 x'17 x'18 x'19)
   (Status y'1 y'2 y'3 y'4 y'5 y'6 y'7 y'8 y'9 y'10 y'11 y'12 y'13 y'14 y'15 y'16 y'17 y'18 y'19)
   = Status (P'.mergeAppend x'1 y'1) (P'.mergeAppend x'2 y'2) (P'.mergeAppend x'3 y'3) (P'.mergeAppend x'4 y'4)
      (P'.mergeAppend x'5 y'5)
      (P'.mergeAppend x'6 y'6)
      (P'.mergeAppend x'7 y'7)
      (P'.mergeAppend x'8 y'8)
      (P'.mergeAppend x'9 y'9)
      (P'.mergeAppend x'10 y'10)
      (P'.mergeAppend x'11 y'11)
      (P'.mergeAppend x'12 y'12)
      (P'.mergeAppend x'13 y'13)
      (P'.mergeAppend x'14 y'14)
      (P'.mergeAppend x'15 y'15)
      (P'.mergeAppend x'16 y'16)
      (P'.mergeAppend x'17 y'17)
      (P'.mergeAppend x'18 y'18)
      (P'.mergeAppend x'19 y'19)

instance P'.Default Status where
  defaultValue
   = Status P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue
      P'.defaultValue

instance P'.Wire Status where
  wireSize ft' self'@(Status x'1 x'2 x'3 x'4 x'5 x'6 x'7 x'8 x'9 x'10 x'11 x'12 x'13 x'14 x'15 x'16 x'17 x'18 x'19)
   = case ft' of
       10 -> calc'Size
       11 -> P'.prependMessageSize calc'Size
       _ -> P'.wireSizeErr ft' self'
    where
        calc'Size
         = (P'.wireSizeOpt 1 9 x'1 + P'.wireSizeOpt 1 4 x'2 + P'.wireSizeOpt 1 4 x'3 + P'.wireSizeOpt 1 2 x'4 +
             P'.wireSizeOpt 1 2 x'5
             + P'.wireSizeOpt 1 4 x'6
             + P'.wireSizeOpt 1 2 x'7
             + P'.wireSizeOpt 1 8 x'8
             + P'.wireSizeOpt 1 13 x'9
             + P'.wireSizeOpt 1 8 x'10
             + P'.wireSizeOpt 1 8 x'11
             + P'.wireSizeOpt 1 13 x'12
             + P'.wireSizeOpt 1 2 x'13
             + P'.wireSizeOpt 1 2 x'14
             + P'.wireSizeOpt 1 2 x'15
             + P'.wireSizeOpt 2 13 x'16
             + P'.wireSizeOpt 2 4 x'17
             + P'.wireSizeOpt 2 2 x'18
             + P'.wireSizeOpt 2 2 x'19)
  wirePut ft' self'@(Status x'1 x'2 x'3 x'4 x'5 x'6 x'7 x'8 x'9 x'10 x'11 x'12 x'13 x'14 x'15 x'16 x'17 x'18 x'19)
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
             P'.wirePutOpt 16 4 x'2
             P'.wirePutOpt 24 4 x'3
             P'.wirePutOpt 37 2 x'4
             P'.wirePutOpt 45 2 x'5
             P'.wirePutOpt 48 4 x'6
             P'.wirePutOpt 61 2 x'7
             P'.wirePutOpt 64 8 x'8
             P'.wirePutOpt 72 13 x'9
             P'.wirePutOpt 80 8 x'10
             P'.wirePutOpt 88 8 x'11
             P'.wirePutOpt 96 13 x'12
             P'.wirePutOpt 109 2 x'13
             P'.wirePutOpt 117 2 x'14
             P'.wirePutOpt 125 2 x'15
             P'.wirePutOpt 128 13 x'16
             P'.wirePutOpt 136 4 x'17
             P'.wirePutOpt 149 2 x'18
             P'.wirePutOpt 157 2 x'19
  wireGet ft'
   = case ft' of
       10 -> P'.getBareMessageWith update'Self
       11 -> P'.getMessageWith update'Self
       _ -> P'.wireGetErr ft'
    where
        update'Self wire'Tag old'Self
         = case wire'Tag of
             10 -> Prelude'.fmap (\ !new'Field -> old'Self{version = Prelude'.Just new'Field}) (P'.wireGet 9)
             16 -> Prelude'.fmap (\ !new'Field -> old'Self{bytes_read = Prelude'.Just new'Field}) (P'.wireGet 4)
             24 -> Prelude'.fmap (\ !new'Field -> old'Self{bytes_written = Prelude'.Just new'Field}) (P'.wireGet 4)
             37 -> Prelude'.fmap (\ !new'Field -> old'Self{bytes_read_rate = Prelude'.Just new'Field}) (P'.wireGet 2)
             45 -> Prelude'.fmap (\ !new'Field -> old'Self{bytes_written_rate = Prelude'.Just new'Field}) (P'.wireGet 2)
             48 -> Prelude'.fmap (\ !new'Field -> old'Self{rpcs_executed = Prelude'.Just new'Field}) (P'.wireGet 4)
             61 -> Prelude'.fmap (\ !new'Field -> old'Self{rpc_rate = Prelude'.Just new'Field}) (P'.wireGet 2)
             64 -> Prelude'.fmap (\ !new'Field -> old'Self{one_rpc_per_update = Prelude'.Just new'Field}) (P'.wireGet 8)
             72 -> Prelude'.fmap (\ !new'Field -> old'Self{max_time_per_update = Prelude'.Just new'Field}) (P'.wireGet 13)
             80 -> Prelude'.fmap (\ !new'Field -> old'Self{adaptive_rate_control = Prelude'.Just new'Field}) (P'.wireGet 8)
             88 -> Prelude'.fmap (\ !new'Field -> old'Self{blocking_recv = Prelude'.Just new'Field}) (P'.wireGet 8)
             96 -> Prelude'.fmap (\ !new'Field -> old'Self{recv_timeout = Prelude'.Just new'Field}) (P'.wireGet 13)
             109 -> Prelude'.fmap (\ !new'Field -> old'Self{time_per_rpc_update = Prelude'.Just new'Field}) (P'.wireGet 2)
             117 -> Prelude'.fmap (\ !new'Field -> old'Self{poll_time_per_rpc_update = Prelude'.Just new'Field}) (P'.wireGet 2)
             125 -> Prelude'.fmap (\ !new'Field -> old'Self{exec_time_per_rpc_update = Prelude'.Just new'Field}) (P'.wireGet 2)
             128 -> Prelude'.fmap (\ !new'Field -> old'Self{stream_rpcs = Prelude'.Just new'Field}) (P'.wireGet 13)
             136 -> Prelude'.fmap (\ !new'Field -> old'Self{stream_rpcs_executed = Prelude'.Just new'Field}) (P'.wireGet 4)
             149 -> Prelude'.fmap (\ !new'Field -> old'Self{stream_rpc_rate = Prelude'.Just new'Field}) (P'.wireGet 2)
             157 -> Prelude'.fmap (\ !new'Field -> old'Self{time_per_stream_update = Prelude'.Just new'Field}) (P'.wireGet 2)
             _ -> let (field'Number, wire'Type) = P'.splitWireTag wire'Tag in P'.unknown field'Number wire'Type old'Self

instance P'.MessageAPI msg' (msg' -> Status) Status where
  getVal m' f' = f' m'

instance P'.GPB Status

instance P'.ReflectDescriptor Status where
  getMessageInfo _
   = P'.GetMessageInfo (P'.fromDistinctAscList [])
      (P'.fromDistinctAscList [10, 16, 24, 37, 45, 48, 61, 64, 72, 80, 88, 96, 109, 117, 125, 128, 136, 149, 157])
  reflectDescriptorInfo _
   = Prelude'.read
      "DescriptorInfo {descName = ProtoName {protobufName = FIName \".KRPC.Status\", haskellPrefix = [MName \"PB\"], parentModule = [MName \"KRPC\"], baseName = MName \"Status\"}, descFilePath = [\"PB\",\"KRPC\",\"Status.hs\"], isGroup = False, fields = fromList [FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.version\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"version\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 1}, wireTag = WireTag {getWireTag = 10}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 9}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.bytes_read\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"bytes_read\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 2}, wireTag = WireTag {getWireTag = 16}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 4}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.bytes_written\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"bytes_written\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 3}, wireTag = WireTag {getWireTag = 24}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 4}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.bytes_read_rate\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"bytes_read_rate\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 4}, wireTag = WireTag {getWireTag = 37}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.bytes_written_rate\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"bytes_written_rate\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 5}, wireTag = WireTag {getWireTag = 45}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.rpcs_executed\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"rpcs_executed\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 6}, wireTag = WireTag {getWireTag = 48}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 4}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.rpc_rate\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"rpc_rate\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 7}, wireTag = WireTag {getWireTag = 61}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.one_rpc_per_update\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"one_rpc_per_update\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 8}, wireTag = WireTag {getWireTag = 64}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.max_time_per_update\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"max_time_per_update\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 9}, wireTag = WireTag {getWireTag = 72}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 13}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.adaptive_rate_control\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"adaptive_rate_control\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 10}, wireTag = WireTag {getWireTag = 80}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.blocking_recv\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"blocking_recv\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 11}, wireTag = WireTag {getWireTag = 88}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 8}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.recv_timeout\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"recv_timeout\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 12}, wireTag = WireTag {getWireTag = 96}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 13}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.time_per_rpc_update\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"time_per_rpc_update\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 13}, wireTag = WireTag {getWireTag = 109}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.poll_time_per_rpc_update\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"poll_time_per_rpc_update\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 14}, wireTag = WireTag {getWireTag = 117}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.exec_time_per_rpc_update\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"exec_time_per_rpc_update\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 15}, wireTag = WireTag {getWireTag = 125}, packedTag = Nothing, wireTagLength = 1, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.stream_rpcs\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"stream_rpcs\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 16}, wireTag = WireTag {getWireTag = 128}, packedTag = Nothing, wireTagLength = 2, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 13}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.stream_rpcs_executed\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"stream_rpcs_executed\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 17}, wireTag = WireTag {getWireTag = 136}, packedTag = Nothing, wireTagLength = 2, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 4}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.stream_rpc_rate\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"stream_rpc_rate\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 18}, wireTag = WireTag {getWireTag = 149}, packedTag = Nothing, wireTagLength = 2, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing},FieldInfo {fieldName = ProtoFName {protobufName' = FIName \".KRPC.Status.time_per_stream_update\", haskellPrefix' = [MName \"PB\"], parentModule' = [MName \"KRPC\",MName \"Status\"], baseName' = FName \"time_per_stream_update\", baseNamePrefix' = \"\"}, fieldNumber = FieldId {getFieldId = 19}, wireTag = WireTag {getWireTag = 157}, packedTag = Nothing, wireTagLength = 2, isPacked = False, isRequired = False, canRepeat = False, mightPack = False, typeCode = FieldType {getFieldType = 2}, typeName = Nothing, hsRawDefault = Nothing, hsDefault = Nothing}], descOneofs = fromList [], keys = fromList [], extRanges = [], knownKeys = fromList [], storeUnknown = False, lazyFields = False, makeLenses = False}"

instance P'.TextType Status where
  tellT = P'.tellSubMessage
  getT = P'.getSubMessage

instance P'.TextMsg Status where
  textPut msg
   = do
       P'.tellT "version" (version msg)
       P'.tellT "bytes_read" (bytes_read msg)
       P'.tellT "bytes_written" (bytes_written msg)
       P'.tellT "bytes_read_rate" (bytes_read_rate msg)
       P'.tellT "bytes_written_rate" (bytes_written_rate msg)
       P'.tellT "rpcs_executed" (rpcs_executed msg)
       P'.tellT "rpc_rate" (rpc_rate msg)
       P'.tellT "one_rpc_per_update" (one_rpc_per_update msg)
       P'.tellT "max_time_per_update" (max_time_per_update msg)
       P'.tellT "adaptive_rate_control" (adaptive_rate_control msg)
       P'.tellT "blocking_recv" (blocking_recv msg)
       P'.tellT "recv_timeout" (recv_timeout msg)
       P'.tellT "time_per_rpc_update" (time_per_rpc_update msg)
       P'.tellT "poll_time_per_rpc_update" (poll_time_per_rpc_update msg)
       P'.tellT "exec_time_per_rpc_update" (exec_time_per_rpc_update msg)
       P'.tellT "stream_rpcs" (stream_rpcs msg)
       P'.tellT "stream_rpcs_executed" (stream_rpcs_executed msg)
       P'.tellT "stream_rpc_rate" (stream_rpc_rate msg)
       P'.tellT "time_per_stream_update" (time_per_stream_update msg)
  textGet
   = do
       mods <- P'.sepEndBy
                (P'.choice
                  [parse'version, parse'bytes_read, parse'bytes_written, parse'bytes_read_rate, parse'bytes_written_rate,
                   parse'rpcs_executed, parse'rpc_rate, parse'one_rpc_per_update, parse'max_time_per_update,
                   parse'adaptive_rate_control, parse'blocking_recv, parse'recv_timeout, parse'time_per_rpc_update,
                   parse'poll_time_per_rpc_update, parse'exec_time_per_rpc_update, parse'stream_rpcs, parse'stream_rpcs_executed,
                   parse'stream_rpc_rate, parse'time_per_stream_update])
                P'.spaces
       Prelude'.return (Prelude'.foldl (\ v f -> f v) P'.defaultValue mods)
    where
        parse'version
         = P'.try
            (do
               v <- P'.getT "version"
               Prelude'.return (\ o -> o{version = v}))
        parse'bytes_read
         = P'.try
            (do
               v <- P'.getT "bytes_read"
               Prelude'.return (\ o -> o{bytes_read = v}))
        parse'bytes_written
         = P'.try
            (do
               v <- P'.getT "bytes_written"
               Prelude'.return (\ o -> o{bytes_written = v}))
        parse'bytes_read_rate
         = P'.try
            (do
               v <- P'.getT "bytes_read_rate"
               Prelude'.return (\ o -> o{bytes_read_rate = v}))
        parse'bytes_written_rate
         = P'.try
            (do
               v <- P'.getT "bytes_written_rate"
               Prelude'.return (\ o -> o{bytes_written_rate = v}))
        parse'rpcs_executed
         = P'.try
            (do
               v <- P'.getT "rpcs_executed"
               Prelude'.return (\ o -> o{rpcs_executed = v}))
        parse'rpc_rate
         = P'.try
            (do
               v <- P'.getT "rpc_rate"
               Prelude'.return (\ o -> o{rpc_rate = v}))
        parse'one_rpc_per_update
         = P'.try
            (do
               v <- P'.getT "one_rpc_per_update"
               Prelude'.return (\ o -> o{one_rpc_per_update = v}))
        parse'max_time_per_update
         = P'.try
            (do
               v <- P'.getT "max_time_per_update"
               Prelude'.return (\ o -> o{max_time_per_update = v}))
        parse'adaptive_rate_control
         = P'.try
            (do
               v <- P'.getT "adaptive_rate_control"
               Prelude'.return (\ o -> o{adaptive_rate_control = v}))
        parse'blocking_recv
         = P'.try
            (do
               v <- P'.getT "blocking_recv"
               Prelude'.return (\ o -> o{blocking_recv = v}))
        parse'recv_timeout
         = P'.try
            (do
               v <- P'.getT "recv_timeout"
               Prelude'.return (\ o -> o{recv_timeout = v}))
        parse'time_per_rpc_update
         = P'.try
            (do
               v <- P'.getT "time_per_rpc_update"
               Prelude'.return (\ o -> o{time_per_rpc_update = v}))
        parse'poll_time_per_rpc_update
         = P'.try
            (do
               v <- P'.getT "poll_time_per_rpc_update"
               Prelude'.return (\ o -> o{poll_time_per_rpc_update = v}))
        parse'exec_time_per_rpc_update
         = P'.try
            (do
               v <- P'.getT "exec_time_per_rpc_update"
               Prelude'.return (\ o -> o{exec_time_per_rpc_update = v}))
        parse'stream_rpcs
         = P'.try
            (do
               v <- P'.getT "stream_rpcs"
               Prelude'.return (\ o -> o{stream_rpcs = v}))
        parse'stream_rpcs_executed
         = P'.try
            (do
               v <- P'.getT "stream_rpcs_executed"
               Prelude'.return (\ o -> o{stream_rpcs_executed = v}))
        parse'stream_rpc_rate
         = P'.try
            (do
               v <- P'.getT "stream_rpc_rate"
               Prelude'.return (\ o -> o{stream_rpc_rate = v}))
        parse'time_per_stream_update
         = P'.try
            (do
               v <- P'.getT "time_per_stream_update"
               Prelude'.return (\ o -> o{time_per_stream_update = v}))