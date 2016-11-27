module KRPCHS.Internal.Requests.Batch
( KRPCCallBatch(..)
, KRPCCallBatchReply(..)
, KRPCCall(..)
, emptyBatch
, batchAddCall
, batchLookupResult
, unpackBatchReply
) where


import KRPCHS.Internal.Requests.Call

import qualified PB.KRPC.Request         as KReq
import qualified PB.KRPC.Response        as KRes
import qualified PB.KRPC.ProcedureResult as KPRes

import Data.Sequence (Seq, (|>))
import qualified Data.Sequence as Seq


-- | A batch of prepared RPC calls.
newtype KRPCCallBatch = KRPCCallBatch { batch :: KReq.Request }
    deriving (Show)

-- | A handle to retrieve a result from an RPC batch
newtype KRPCCall a = KRPCCall { batchId :: Int }
    deriving (Show)

-- | Reply to a batch of requests from the RPC server.
newtype KRPCCallBatchReply = KRPCCallBatchReply { batchReply :: Seq KPRes.ProcedureResult }
    deriving (Show)


emptyBatch :: KRPCCallBatch
emptyBatch = KRPCCallBatch . KReq.Request $ Seq.empty


unpackBatchReply :: KRes.Response -> KRPCCallBatchReply
unpackBatchReply r = KRPCCallBatchReply $ KRes.results r


batchAddCall :: KRPCCallReq a -> KRPCCallBatch -> (KRPCCall a, KRPCCallBatch)
batchAddCall (KRPCCallReq call) (KRPCCallBatch req) = (h, b)
  where
    b = KRPCCallBatch $ KReq.Request $ s |> call
    h = KRPCCall $ Seq.length s
    s = KReq.calls req


batchLookupResult :: KRPCCall a -> KRPCCallBatchReply -> Maybe KPRes.ProcedureResult
batchLookupResult (KRPCCall i) (KRPCCallBatchReply r)
-- = Seq.lookup i r (available in later versions of 'containers')
    | i >= 0 && i < Seq.length r = Just $ r `Seq.index` i
    | otherwise                  = Nothing
