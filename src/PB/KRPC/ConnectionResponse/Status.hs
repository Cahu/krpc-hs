{-# LANGUAGE BangPatterns, DeriveDataTypeable, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.ConnectionResponse.Status (Status(..)) where
import Prelude ((+), (/), (.))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Status = OK
            | MALFORMED_HEADER
            | MALFORMED_MESSAGE
            | TIMEOUT
            deriving (Prelude'.Read, Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data)

instance P'.Mergeable Status

instance Prelude'.Bounded Status where
  minBound = OK
  maxBound = TIMEOUT

instance P'.Default Status where
  defaultValue = OK

toMaybe'Enum :: Prelude'.Int -> P'.Maybe Status
toMaybe'Enum 0 = Prelude'.Just OK
toMaybe'Enum 1 = Prelude'.Just MALFORMED_HEADER
toMaybe'Enum 2 = Prelude'.Just MALFORMED_MESSAGE
toMaybe'Enum 3 = Prelude'.Just TIMEOUT
toMaybe'Enum _ = Prelude'.Nothing

instance Prelude'.Enum Status where
  fromEnum OK = 0
  fromEnum MALFORMED_HEADER = 1
  fromEnum MALFORMED_MESSAGE = 2
  fromEnum TIMEOUT = 3
  toEnum
   = P'.fromMaybe (Prelude'.error "hprotoc generated code: toEnum failure for type PB.KRPC.ConnectionResponse.Status") .
      toMaybe'Enum
  succ OK = MALFORMED_HEADER
  succ MALFORMED_HEADER = MALFORMED_MESSAGE
  succ MALFORMED_MESSAGE = TIMEOUT
  succ _ = Prelude'.error "hprotoc generated code: succ failure for type PB.KRPC.ConnectionResponse.Status"
  pred MALFORMED_HEADER = OK
  pred MALFORMED_MESSAGE = MALFORMED_HEADER
  pred TIMEOUT = MALFORMED_MESSAGE
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
  reflectEnum
   = [(0, "OK", OK), (1, "MALFORMED_HEADER", MALFORMED_HEADER), (2, "MALFORMED_MESSAGE", MALFORMED_MESSAGE),
      (3, "TIMEOUT", TIMEOUT)]
  reflectEnumInfo _
   = P'.EnumInfo (P'.makePNF (P'.pack ".KRPC.ConnectionResponse.Status") ["PB"] ["KRPC", "ConnectionResponse"] "Status")
      ["PB", "KRPC", "ConnectionResponse", "Status.hs"]
      [(0, "OK"), (1, "MALFORMED_HEADER"), (2, "MALFORMED_MESSAGE"), (3, "TIMEOUT")]

instance P'.TextType Status where
  tellT = P'.tellShow
  getT = P'.getRead