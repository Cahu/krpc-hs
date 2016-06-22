{-# LANGUAGE RecordWildCards #-}

module Main
( main
) where


import KRPCHS
import KRPCHS.SpaceCenter

import Control.Monad
import Control.Monad.Catch
import Control.Monad.Trans

import Data.List


data TelemetryData = TelemetryData
    { altitude  :: Double
    , latitude  :: Double
    , longitude :: Double
    }


main :: IO ()
main =
    withRPCClient    "Flight recorder" "127.0.0.1" "50000" $ \client       -> do
    withStreamClient client            "127.0.0.1" "50001" $ \streamClient -> do
        runRPCProg client (telemetryProg streamClient)


telemetryProg :: StreamClient -> RPCContext ()
telemetryProg streamClient = do
    vessel <- getActiveVessel
    ref    <- getVesselSurfaceReferenceFrame vessel
    flight <- vesselFlight vessel ref

    -- install streams
    altitudeStream  <- getFlightMeanAltitudeStream flight
    latitudeStream  <- getFlightLatitudeStream     flight
    longitudeStream <- getFlightLongitudeStream    flight

    -- get results and print csv
    liftIO $ putStrLn "altitude;latitude;longitude"
    forever $ do
        msg <- getStreamMessage streamClient

        ok <- try (do
            altitude  <- getStreamResult altitudeStream  msg
            latitude  <- getStreamResult latitudeStream  msg
            longitude <- getStreamResult longitudeStream msg
            return $ TelemetryData altitude latitude longitude)

        case ok of
            Right telemetry    -> liftIO $ printTelemetryCSV telemetry
            Left  NoSuchStream -> return () -- this can happen
            Left  err          -> throwM err


printTelemetryCSV :: TelemetryData -> IO ()
printTelemetryCSV TelemetryData{..} =
    putStrLn $ intercalate ";" [show altitude, show latitude, show longitude]
