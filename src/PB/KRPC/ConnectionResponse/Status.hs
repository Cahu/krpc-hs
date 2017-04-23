{-# LANGUAGE BangPatterns, DeriveDataTypeable, DeriveGeneric, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.ConnectionResponse.Status (Status(..)) where
import Prelude ((+), (/), (.))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified GHC.Generics as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Status = OK
            | MALFORMED_MESSAGE
            | TIMEOUT
            | WRONG_TYPE
            deriving (Prelude'.Read, Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data, Prelude'.Generic)

instance P'.Mergeable Status

instance Prelude'.Bounded Status where
  minBound = OK
  maxBound = WRONG_TYPE

instance P'.Default Status where
  defaultValue = OK

toMaybe'Enum :: Prelude'.Int -> P'.Maybe Status
toMaybe'Enum 0 = Prelude'.Just OK
toMaybe'Enum 1 = Prelude'.Just MALFORMED_MESSAGE
toMaybe'Enum 2 = Prelude'.Just TIMEOUT
toMaybe'Enum 3 = Prelude'.Just WRONG_TYPE
toMaybe'Enum _ = Prelude'.Nothing

instance Prelude'.Enum Status where
  fromEnum OK = 0
  fromEnum MALFORMED_MESSAGE = 1
  fromEnum TIMEOUT = 2
  fromEnum WRONG_TYPE = 3
  toEnum
   = P'.fromMaybe (Prelude'.error "hprotoc generated code: toEnum failure for type PB.KRPC.ConnectionResponse.Status") .
      toMaybe'Enum
  succ OK = MALFORMED_MESSAGE
  succ MALFORMED_MESSAGE = TIMEOUT
  succ TIMEOUT = WRONG_TYPE
  succ _ = Prelude'.error "hprotoc generated code: succ failure for type PB.KRPC.ConnectionResponse.Status"
  pred MALFORMED_MESSAGE = OK
  pred TIMEOUT = MALFORMED_MESSAGE
  pred WRONG_TYPE = TIMEOUT
  pred _ = Prelude'.error "hprotoc generated code: pred failure for type PB.KRPC.ConnectionResponse.Status"

instance P'.Wire Status where
  wireSize ft' enum = P'.wireSize ft' (Prelude'.fromEnum enum)
  wirePut ft' enum = P'.wirePut ft' (Prelude'.fromEnum enum)
  wireGet 14 = P'.wireGetEnum toMaybe'Enum
  wireGet ft' = P'.wireGetErr ft'
  wireGetPacked 14 = P'.wireGetPackedEnum toMaybe'Enum
  wireGetPacked ft' = P'.wireGetErr ft'

instance P'.GPB Status

instance P'.MessageAPI msg' (msg' -> Status) Status where
  getVal m' f' = f' m'

instance P'.ReflectEnum Status where
  reflectEnum = [(0, "OK", OK), (1, "MALFORMED_MESSAGE", MALFORMED_MESSAGE), (2, "TIMEOUT", TIMEOUT), (3, "WRONG_TYPE", WRONG_TYPE)]
  reflectEnumInfo _
   = P'.EnumInfo (P'.makePNF (P'.pack ".krpc.schema.ConnectionResponse.Status") ["PB"] ["KRPC", "ConnectionResponse"] "Status")
      ["PB", "KRPC", "ConnectionResponse", "Status.hs"]
      [(0, "OK"), (1, "MALFORMED_MESSAGE"), (2, "TIMEOUT"), (3, "WRONG_TYPE")]

instance P'.TextType Status where
  tellT = P'.tellShow
  getT = P'.getRead