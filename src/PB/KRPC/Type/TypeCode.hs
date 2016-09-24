{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.Type.TypeCode (TypeCode(..)) where
import Prelude ((+), (/), (.))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data TypeCode = NONE
              | DOUBLE
              | FLOAT
              | SINT32
              | SINT64
              | UINT32
              | UINT64
              | BOOL
              | STRING
              | BYTES
              | CLASS
              | ENUMERATION
              | PROCEDURE_CALL
              | STREAM
              | STATUS
              | SERVICES
              | TUPLE
              | LIST
              | SET
              | DICTIONARY
              deriving (Prelude'.Read, Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable TypeCode

instance Prelude'.Bounded TypeCode where
  minBound = NONE
  maxBound = DICTIONARY

instance P'.Default TypeCode where
  defaultValue = NONE

toMaybe'Enum :: Prelude'.Int -> P'.Maybe TypeCode
toMaybe'Enum 0 = Prelude'.Just NONE
toMaybe'Enum 1 = Prelude'.Just DOUBLE
toMaybe'Enum 2 = Prelude'.Just FLOAT
toMaybe'Enum 3 = Prelude'.Just SINT32
toMaybe'Enum 4 = Prelude'.Just SINT64
toMaybe'Enum 5 = Prelude'.Just UINT32
toMaybe'Enum 6 = Prelude'.Just UINT64
toMaybe'Enum 7 = Prelude'.Just BOOL
toMaybe'Enum 8 = Prelude'.Just STRING
toMaybe'Enum 9 = Prelude'.Just BYTES
toMaybe'Enum 100 = Prelude'.Just CLASS
toMaybe'Enum 101 = Prelude'.Just ENUMERATION
toMaybe'Enum 200 = Prelude'.Just PROCEDURE_CALL
toMaybe'Enum 201 = Prelude'.Just STREAM
toMaybe'Enum 202 = Prelude'.Just STATUS
toMaybe'Enum 203 = Prelude'.Just SERVICES
toMaybe'Enum 300 = Prelude'.Just TUPLE
toMaybe'Enum 301 = Prelude'.Just LIST
toMaybe'Enum 302 = Prelude'.Just SET
toMaybe'Enum 303 = Prelude'.Just DICTIONARY
toMaybe'Enum _ = Prelude'.Nothing

instance Prelude'.Enum TypeCode where
  fromEnum NONE = 0
  fromEnum DOUBLE = 1
  fromEnum FLOAT = 2
  fromEnum SINT32 = 3
  fromEnum SINT64 = 4
  fromEnum UINT32 = 5
  fromEnum UINT64 = 6
  fromEnum BOOL = 7
  fromEnum STRING = 8
  fromEnum BYTES = 9
  fromEnum CLASS = 100
  fromEnum ENUMERATION = 101
  fromEnum PROCEDURE_CALL = 200
  fromEnum STREAM = 201
  fromEnum STATUS = 202
  fromEnum SERVICES = 203
  fromEnum TUPLE = 300
  fromEnum LIST = 301
  fromEnum SET = 302
  fromEnum DICTIONARY = 303
  toEnum = P'.fromMaybe (Prelude'.error "hprotoc generated code: toEnum failure for type PB.KRPC.Type.TypeCode") . toMaybe'Enum
  succ NONE = DOUBLE
  succ DOUBLE = FLOAT
  succ FLOAT = SINT32
  succ SINT32 = SINT64
  succ SINT64 = UINT32
  succ UINT32 = UINT64
  succ UINT64 = BOOL
  succ BOOL = STRING
  succ STRING = BYTES
  succ BYTES = CLASS
  succ CLASS = ENUMERATION
  succ ENUMERATION = PROCEDURE_CALL
  succ PROCEDURE_CALL = STREAM
  succ STREAM = STATUS
  succ STATUS = SERVICES
  succ SERVICES = TUPLE
  succ TUPLE = LIST
  succ LIST = SET
  succ SET = DICTIONARY
  succ _ = Prelude'.error "hprotoc generated code: succ failure for type PB.KRPC.Type.TypeCode"
  pred DOUBLE = NONE
  pred FLOAT = DOUBLE
  pred SINT32 = FLOAT
  pred SINT64 = SINT32
  pred UINT32 = SINT64
  pred UINT64 = UINT32
  pred BOOL = UINT64
  pred STRING = BOOL
  pred BYTES = STRING
  pred CLASS = BYTES
  pred ENUMERATION = CLASS
  pred PROCEDURE_CALL = ENUMERATION
  pred STREAM = PROCEDURE_CALL
  pred STATUS = STREAM
  pred SERVICES = STATUS
  pred TUPLE = SERVICES
  pred LIST = TUPLE
  pred SET = LIST
  pred DICTIONARY = SET
  pred _ = Prelude'.error "hprotoc generated code: pred failure for type PB.KRPC.Type.TypeCode"

instance P'.Wire TypeCode where
  wireSize ft' enum = P'.wireSize ft' (Prelude'.fromEnum enum)
  wirePut ft' enum = P'.wirePut ft' (Prelude'.fromEnum enum)
  wireGet 14 = P'.wireGetEnum toMaybe'Enum
  wireGet ft' = P'.wireGetErr ft'
  wireGetPacked 14 = P'.wireGetPackedEnum toMaybe'Enum
  wireGetPacked ft' = P'.wireGetErr ft'

instance P'.GPB TypeCode

instance P'.MessageAPI msg' (msg' -> TypeCode) TypeCode where
  getVal m' f' = f' m'

instance P'.ReflectEnum TypeCode where
  reflectEnum
   = [(0, "NONE", NONE), (1, "DOUBLE", DOUBLE), (2, "FLOAT", FLOAT), (3, "SINT32", SINT32), (4, "SINT64", SINT64),
      (5, "UINT32", UINT32), (6, "UINT64", UINT64), (7, "BOOL", BOOL), (8, "STRING", STRING), (9, "BYTES", BYTES),
      (100, "CLASS", CLASS), (101, "ENUMERATION", ENUMERATION), (200, "PROCEDURE_CALL", PROCEDURE_CALL), (201, "STREAM", STREAM),
      (202, "STATUS", STATUS), (203, "SERVICES", SERVICES), (300, "TUPLE", TUPLE), (301, "LIST", LIST), (302, "SET", SET),
      (303, "DICTIONARY", DICTIONARY)]
  reflectEnumInfo _
   = P'.EnumInfo (P'.makePNF (P'.pack ".KRPC.Type.TypeCode") ["PB"] ["KRPC", "Type"] "TypeCode")
      ["PB", "KRPC", "Type", "TypeCode.hs"]
      [(0, "NONE"), (1, "DOUBLE"), (2, "FLOAT"), (3, "SINT32"), (4, "SINT64"), (5, "UINT32"), (6, "UINT64"), (7, "BOOL"),
       (8, "STRING"), (9, "BYTES"), (100, "CLASS"), (101, "ENUMERATION"), (200, "PROCEDURE_CALL"), (201, "STREAM"), (202, "STATUS"),
       (203, "SERVICES"), (300, "TUPLE"), (301, "LIST"), (302, "SET"), (303, "DICTIONARY")]

instance P'.TextType TypeCode where
  tellT = P'.tellShow
  getT = P'.getRead