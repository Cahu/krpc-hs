{-# LANGUAGE RecordWildCards #-}

module KRPCHS.Internal.Requests.Stream
( KRPCStream(..)
, KRPCStreamReq(..)
, KRPCStreamMsg()
, StreamID

, emptyKRPCStreamMsg
, unpackStreamMsg
, messageResultsCount
, messageLookupResult
, messageHasResultFor

, makeStreamReq
) where

import Data.Word
import Data.Maybe
import qualified Data.Map      as M
import qualified Data.Foldable as F

import KRPCHS.Internal.SerializeUtils
import KRPCHS.Internal.Requests.Call

import qualified PB.KRPC.Argument        as KArg
import qualified PB.KRPC.ProcedureResult as KPRes
import qualified PB.KRPC.Stream          as KStr
import qualified PB.KRPC.StreamUpdate    as KStreamMsg
import qualified PB.KRPC.StreamResult    as KStreamRes


type StreamID = Word64

-- | A handle to retrieve a result from a 'KRPCStreamMsg'.
newtype KRPCStream a = KRPCStream { streamId :: StreamID }
    deriving (Show)


-- | A prepared request for a stream.
newtype KRPCStreamReq a = KRPCStreamReq { streamCall :: KRPCCallReq KStr.Stream }
    deriving (Show)


-- | A message sent by the KRPC Stream server. It holds results of active
-- streams that can be extracted using a 'KRPCStream' handle.
newtype KRPCStreamMsg = KRPCStreamMsg { streamMsg :: M.Map StreamID KPRes.ProcedureResult }
    deriving (Show)


emptyKRPCStreamMsg :: KRPCStreamMsg
emptyKRPCStreamMsg = KRPCStreamMsg M.empty


extractStreamResponse :: KStreamRes.StreamResult -> Maybe (StreamID, KPRes.ProcedureResult)
extractStreamResponse streamRes = do
    sid <- KStreamRes.id     streamRes
    res <- KStreamRes.result streamRes
    return (sid, res)


extractStreamMessage :: KStreamMsg.StreamUpdate -> [(StreamID, KPRes.ProcedureResult)]
extractStreamMessage msg = mapMaybe extractStreamResponse responseList
    where responseList = F.toList (KStreamMsg.results msg)


unpackStreamMsg :: KStreamMsg.StreamUpdate -> KRPCStreamMsg
unpackStreamMsg res = KRPCStreamMsg $ M.fromList (extractStreamMessage res)


makeStreamReq :: KRPCCallReq a -> KRPCStreamReq a
makeStreamReq (KRPCCallReq r) = KRPCStreamReq $
    makeCallReq "KRPC" "AddStream" [KArg.Argument (Just 0) (Just $ messagePut r)]


-- | @'messageResultsCount' m@ returns the number of messages contained in the
-- 'KRPCStreamMsg' m.
messageResultsCount :: KRPCStreamMsg -> Int
messageResultsCount = M.size . streamMsg


-- | @'messageHasResultFor' s m@ returns true if the @KRPCStreamMsg@ contains a
-- result for the 'KRPCStream' @s@. It is then safe to call @'getStreamResult'
-- s m@.
messageHasResultFor :: KRPCStream a -> KRPCStreamMsg -> Bool
messageHasResultFor KRPCStream{..} KRPCStreamMsg{..} =
    M.member streamId streamMsg


messageLookupResult :: StreamID -> KRPCStreamMsg -> Maybe KPRes.ProcedureResult
messageLookupResult i (KRPCStreamMsg m) = M.lookup i m
