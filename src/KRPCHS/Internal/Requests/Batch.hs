{-# LANGUAGE ExistentialQuantification #-}

module KRPCHS.Internal.Requests.Batch
( KRPCCallBatch(..)
, KRPCCallBatchReply(..)
, makeBatch
) where


import KRPCHS.Internal.Requests.Call

import qualified PB.KRPC.Request  as KReq
import qualified PB.KRPC.Response as KRes

import qualified Data.Sequence as Seq


-- | A batch of prepared RPC calls.
newtype KRPCCallBatch = KRPCCallBatch { batch :: KReq.Request }
    deriving (Show)


-- | Reply to a batch of requests from the RPC server.
newtype KRPCCallBatchReply = KRPCCallBatchReply { batchReply :: KRes.Response }
    deriving (Show)


makeBatch :: forall a. [KRPCCallReq a] -> KRPCCallBatch
makeBatch = KRPCCallBatch . KReq.Request . fmap procCall . Seq.fromList
