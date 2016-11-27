module Main
( main
) where


import KRPCHS
import KRPCHS.SpaceCenter


main :: IO ()
main = withRPCClient "Batch example" "127.0.0.1" "50000" $ \client ->
    runRPCProg client batchTest >>= print


batchTest :: RPCContext (Double, Vessel)
batchTest = do
    reply  <- performBatchRequest batch
    ut     <- batchGetResult utHandle     reply
    vessel <- batchGetResult vesselHandle reply
    return (ut, vessel)
  where
    ((utHandle, vesselHandle), batch) = flip buildBatch emptyBatch $ do
        u <- rpcCall $ getUTReq
        v <- rpcCall $ getActiveVesselReq
        return (u, v)
