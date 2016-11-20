module KRPCHS.Internal.Requests.Call
( KRPCCall(..)
, KRPCCallReq(..)
, makeArgument
, makeCallRequest
) where


import qualified Data.Sequence as Seq


import KRPCHS.Internal.SerializeUtils


import qualified PB.KRPC.ProcedureCall as KPReq
import qualified PB.KRPC.Argument      as KArg

import qualified Text.ProtocolBuffers  as P

-- | A handle to retrieve a result from an RPC batch
newtype KRPCCall a = KRPCCall { batchId :: Int }
    deriving (Show)


-- | A prepared RPC.
newtype KRPCCallReq a = KRPCCallReq { procCall :: KPReq.ProcedureCall }
    deriving (Show)

instance PbEncodable (KRPCCallReq a) where
    encodePb = messagePut . procCall


makeArgument :: (PbEncodable a) => P.Word32 -> a -> KArg.Argument
makeArgument position arg = KArg.Argument (Just position) (Just $ encodePb arg)


makeCallRequest :: String -> String -> [KArg.Argument] -> KRPCCallReq a
makeCallRequest serviceName procName params = KRPCCallReq $
    KPReq.ProcedureCall
    { KPReq.service   = Just $ P.fromString serviceName
    , KPReq.procedure = Just $ P.fromString procName
    , KPReq.arguments = Seq.fromList params }
