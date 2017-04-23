{-# LANGUAGE BangPatterns, DeriveDataTypeable, DeriveGeneric, FlexibleInstances, MultiParamTypeClasses #-}
{-# OPTIONS_GHC  -fno-warn-unused-imports #-}
module PB.KRPC.ConnectionRequest.Type (Type(..)) where
import Prelude ((+), (/), (.))
import qualified Prelude as Prelude'
import qualified Data.Typeable as Prelude'
import qualified GHC.Generics as Prelude'
import qualified Data.Data as Prelude'
import qualified Text.ProtocolBuffers.Header as P'

data Type = RPC
          | STREAM
          deriving (Prelude'.Read, Prelude'.Show, Prelude'.Eq, Prelude'.Ord, Prelude'.Typeable, Prelude'.Data, Prelude'.Generic)

instance P'.Mergeable Type

instance Prelude'.Bounded Type where
  minBound = RPC
  maxBound = STREAM

instance P'.Default Type where
  defaultValue = RPC

toMaybe'Enum :: Prelude'.Int -> P'.Maybe Type
toMaybe'Enum 0 = Prelude'.Just RPC
toMaybe'Enum 1 = Prelude'.Just STREAM
toMaybe'Enum _ = Prelude'.Nothing

instance Prelude'.Enum Type where
  fromEnum RPC = 0
  fromEnum STREAM = 1
  toEnum
   = P'.fromMaybe (Prelude'.error "hprotoc generated code: toEnum failure for type PB.KRPC.ConnectionRequest.Type") . toMaybe'Enum
  succ RPC = STREAM
  succ _ = Prelude'.error "hprotoc generated code: succ failure for type PB.KRPC.ConnectionRequest.Type"
  pred STREAM = RPC
  pred _ = Prelude'.error "hprotoc generated code: pred failure for type PB.KRPC.ConnectionRequest.Type"

instance P'.Wire Type where
  wireSize ft' enum = P'.wireSize ft' (Prelude'.fromEnum enum)
  wirePut ft' enum = P'.wirePut ft' (Prelude'.fromEnum enum)
  wireGet 14 = P'.wireGetEnum toMaybe'Enum
  wireGet ft' = P'.wireGetErr ft'
  wireGetPacked 14 = P'.wireGetPackedEnum toMaybe'Enum
  wireGetPacked ft' = P'.wireGetErr ft'

instance P'.GPB Type

instance P'.MessageAPI msg' (msg' -> Type) Type where
  getVal m' f' = f' m'

instance P'.ReflectEnum Type where
  reflectEnum = [(0, "RPC", RPC), (1, "STREAM", STREAM)]
  reflectEnumInfo _
   = P'.EnumInfo (P'.makePNF (P'.pack ".krpc.schema.ConnectionRequest.Type") ["PB"] ["KRPC", "ConnectionRequest"] "Type")
      ["PB", "KRPC", "ConnectionRequest", "Type.hs"]
      [(0, "RPC"), (1, "STREAM")]

instance P'.TextType Type where
  tellT = P'.tellShow
  getT = P'.getRead