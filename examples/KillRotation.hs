{-# LANGUAGE MultiWayIf #-}
{-# LANGUAGE RecordWildCards #-}

module Main
( main
) where

{- NOTE:
 - If you are aiming for simplicity, killing the rotation of a vessel is more
 - easily done by activating the SAS or using kRPC's autopilot methods. This
 - example illustrates how it can be achieved using the basic pitch, yaw and
 - roll controls ... because it's more fun that way!
 -}

import KRPCHS
import KRPCHS.SpaceCenter

import Control.Monad
import Control.Monad.Catch



type Vect3 = (Double, Double, Double)


data VesselRotation = VesselRotation
    { transformedX    :: Vect3
    , transformedY    :: Vect3
    , transformedZ    :: Vect3
    , angularVelocity :: Vect3
    , availableTorque :: Vect3
    , momentOfInertia :: Vect3
    }


-- parameters to tune the algorithm : depend on the amount of torque available for instance
rotationTolerance :: Double
rotationTolerance = 0.00001

-- Some vector manipulation functions
vect3MagnitureSquared :: Vect3 -> Double
vect3MagnitureSquared (a,b,c) = a*a + b*b + c*c

vect3Magnitude :: Vect3 -> Double
vect3Magnitude = sqrt . vect3MagnitureSquared

vect3ScalarMult :: Double -> Vect3 -> Vect3
vect3ScalarMult k (x, y, z) = (k*x, k*y, k*z)

vect3DotProduct :: Vect3 -> Vect3 -> Double
vect3DotProduct (x1, y1, z1) (x2, y2, z2) = x1*x2 + y1*y2 + z1*z2

vect3Normalized :: Vect3 -> Vect3
vect3Normalized v@(x, y, z) = let n = vect3Magnitude v in (x/n, y/n, z/n)

-- Projection function
(-|) :: Vect3 -> Vect3 -> Vect3
(-|) v1 v2 =
    let
        unitV2        = vect3Normalized v2
        magnitudeV2   = vect3Magnitude v2
        dotProduct    = vect3DotProduct v1 v2
        projMagnitude = dotProduct / magnitudeV2
    in
        vect3ScalarMult projMagnitude unitV2


main :: IO ()
main =
    withRPCClient    "Kill Rotation" "127.0.0.1" "50000" $ \client       -> do
    withStreamClient client          "127.0.0.1" "50001" $ \streamClient -> do
        runRPCProg client (attitudeProg streamClient)


attitudeProg :: StreamClient -> RPCContext ()
attitudeProg streamClient = do
    -- vessel and reference frames
    vessel      <- getActiveVessel
    control     <- getVesselControl               vessel
    vesselRef   <- getVesselReferenceFrame        vessel
    orbitalRef  <- getVesselOrbitalReferenceFrame vessel

    -- install streams
    tranformedXStream     <- transformDirectionStream (1, 0, 0) vesselRef orbitalRef
    tranformedYStream     <- transformDirectionStream (0, 1, 0) vesselRef orbitalRef
    tranformedZStream     <- transformDirectionStream (0, 0, 1) vesselRef orbitalRef
    velocityStream        <- vesselAngularVelocityStream vessel orbitalRef
    availableTorqueStream <- getVesselAvailableReactionWheelTorqueStream vessel
    momentOfInertiaStream <- getVesselMomentOfInertiaStream vessel

    let
        -- Helper function to extract A VesselRotation from a stream message
        extractVesselRotation msg = do
            transformedX    <- getStreamResult tranformedXStream     msg
            transformedY    <- getStreamResult tranformedYStream     msg
            transformedZ    <- getStreamResult tranformedZStream     msg
            angularVelocity <- getStreamResult velocityStream        msg
            availableTorque <- getStreamResult availableTorqueStream msg
            momentOfInertia <- getStreamResult momentOfInertiaStream msg
            return VesselRotation{..}

    let
        {- Find out how much of the rotation can be killed in a particular 'direction' (yaw, roll, pitch).
         - To do that, we need to project the angular velocity vector on the
         - given direction and compute the appropriate amount of control (ie.
         - angular acceleration) to use to cancel the rotation.
         -}
        killRotation axisControlFunc moment torque velocity dir =
            let
                proj = velocity -| dir                -- project the velocity vector on the axis
                mag2 = vect3MagnitureSquared proj     -- get the magnitude of the projection
                prod = vect3DotProduct proj dir       -- find the direction of the rotation
                maxAccel   = torque / moment
                decelTime  = mag2 / maxAccel
                accel      = if decelTime > 0.2 then maxAccel else (mag2/0.2)
                accelRatio = realToFrac $ max 0.01 (accel / maxAccel)
            in
                if | mag2 > rotationTolerance && prod >= 0 -> void . axisControlFunc $ accelRatio
                   | mag2 > rotationTolerance && prod <  0 -> void . axisControlFunc $ (-accelRatio)
                   | otherwise -> void . axisControlFunc $ 0

        -- shortcut functions
        killPitch = killRotation (setControlPitch control)
        killRoll  = killRotation (setControlRoll  control)
        killYaw   = killRotation (setControlYaw   control)

    forever $ do
        msg <- getStreamMessage streamClient
        ok  <- try (extractVesselRotation msg)
        case ok of
            Left  NoSuchStream       -> return ()  -- this can happen during the first few loops
            Left  err                -> throwM err
            Right VesselRotation{..} -> do
                let (momentPitch, momentRoll, momentYaw) = momentOfInertia
                    (torquePitch, torqueRoll, torqueYaw) = availableTorque
                killPitch momentPitch torquePitch angularVelocity transformedX
                killRoll  momentRoll  torqueRoll  angularVelocity transformedY
                killYaw   momentYaw   torqueYaw   angularVelocity transformedZ
