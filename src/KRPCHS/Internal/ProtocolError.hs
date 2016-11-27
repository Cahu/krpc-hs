module KRPCHS.Internal.ProtocolError
( ProtocolError(..)
) where

import Data.Typeable
import Control.Monad.Catch

data ProtocolError
    = UnknownError
    | ResponseEmpty
    | NotInBatch
    | NoSuchStream
    | KRPCError String
    | DecodeFailure String
    deriving (Typeable, Eq)


instance Show ProtocolError where
    show UnknownError      = "Unknown"
    show ResponseEmpty     = "Empty response"
    show NotInBatch        = "Response not found in batch"
    show NoSuchStream      = "No such stream"
    show (KRPCError e)     = "KRPC Error: '" ++ e ++ "'"
    show (DecodeFailure e) = "Decoding failure: " ++ e


instance Exception ProtocolError
