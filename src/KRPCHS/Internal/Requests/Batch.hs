{-# LANGUAGE GeneralizedNewtypeDeriving #-}

module KRPCHS.Internal.Requests.Batch
( KRPCCallHandle(..)
, KRPCCallBatch(..)
, KRPCCallBatchReply(..)
, KRPCCallBatchBuilder(..)
, emptyBatch
, rpcCall
, buildBatch
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

import Control.Monad.State


-- | A batch of prepared RPC calls.
newtype KRPCCallBatch = KRPCCallBatch { batch :: KReq.Request }
    deriving (Show)

-- | A handle to retrieve a result from an RPC batch.
newtype KRPCCallHandle a = KRPCCallHandle { batchId :: Int }
    deriving (Show)

-- | Reply to a batch of requests from the RPC server.
newtype KRPCCallBatchReply = KRPCCallBatchReply { batchReply :: Seq KPRes.ProcedureResult }
    deriving (Show)

-- | A state monad to simplify RPC batch building.
newtype KRPCCallBatchBuilder a = KRPCCallBatchBuilder { batchBuilder :: State KRPCCallBatch a }
    deriving (Functor, Applicative, Monad, MonadState KRPCCallBatch)


-- | Same as 'batchAddCall' but for use in a 'KRPCCallBatchBuilder'.
rpcCall :: KRPCCallReq a -> KRPCCallBatchBuilder (KRPCCallHandle a)
rpcCall r = state (batchAddCall r)


-- | Makes a batch using the given 'KRPCCallBatchBuilder'.
buildBatch :: KRPCCallBatch -> KRPCCallBatchBuilder a -> (a, KRPCCallBatch)
buildBatch = flip (runState . batchBuilder)


emptyBatch :: KRPCCallBatch
emptyBatch = KRPCCallBatch . KReq.Request $ Seq.empty


unpackBatchReply :: KRes.Response -> KRPCCallBatchReply
unpackBatchReply r = KRPCCallBatchReply $ KRes.results r


-- | Append a request to the given batch.
batchAddCall :: KRPCCallReq a -> KRPCCallBatch -> (KRPCCallHandle a, KRPCCallBatch)
batchAddCall (KRPCCallReq call) (KRPCCallBatch req) = (h, b)
  where
    b = KRPCCallBatch $ KReq.Request $ s |> call
    h = KRPCCallHandle $ Seq.length s
    s = KReq.calls req


batchLookupResult :: KRPCCallHandle a -> KRPCCallBatchReply -> Maybe KPRes.ProcedureResult
batchLookupResult (KRPCCallHandle i) (KRPCCallBatchReply r)
-- = Seq.lookup i r (available in later versions of 'containers')
    | i >= 0 && i < Seq.length r = Just $ r `Seq.index` i
    | otherwise                  = Nothing
