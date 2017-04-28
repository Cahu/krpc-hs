module KRPCHS.InfernalRobotics
( Servo
, ServoGroup
, servoGroupWithName
, servoGroupWithNameStream
, servoGroupWithNameStreamReq
, servoGroupMoveCenter
, servoGroupMoveLeft
, servoGroupMoveNextPreset
, servoGroupMovePrevPreset
, servoGroupMoveRight
, servoGroupServoWithName
, servoGroupServoWithNameStream
, servoGroupServoWithNameStreamReq
, servoGroupStop
, getServoGroupExpanded
, getServoGroupExpandedStream
, getServoGroupExpandedStreamReq
, getServoGroupForwardKey
, getServoGroupForwardKeyStream
, getServoGroupForwardKeyStreamReq
, getServoGroupName
, getServoGroupNameStream
, getServoGroupNameStreamReq
, getServoGroupParts
, getServoGroupPartsStream
, getServoGroupPartsStreamReq
, getServoGroupReverseKey
, getServoGroupReverseKeyStream
, getServoGroupReverseKeyStreamReq
, getServoGroupServos
, getServoGroupServosStream
, getServoGroupServosStreamReq
, getServoGroupSpeed
, getServoGroupSpeedStream
, getServoGroupSpeedStreamReq
, setServoGroupExpanded
, setServoGroupForwardKey
, setServoGroupName
, setServoGroupReverseKey
, setServoGroupSpeed
, servoGroups
, servoGroupsStream
, servoGroupsStreamReq
, servoWithName
, servoWithNameStream
, servoWithNameStreamReq
, servoMoveCenter
, servoMoveLeft
, servoMoveNextPreset
, servoMovePrevPreset
, servoMoveRight
, servoMoveTo
, servoStop
, getServoAcceleration
, getServoAccelerationStream
, getServoAccelerationStreamReq
, getServoConfigSpeed
, getServoConfigSpeedStream
, getServoConfigSpeedStreamReq
, getServoCurrentSpeed
, getServoCurrentSpeedStream
, getServoCurrentSpeedStreamReq
, getServoIsAxisInverted
, getServoIsAxisInvertedStream
, getServoIsAxisInvertedStreamReq
, getServoIsFreeMoving
, getServoIsFreeMovingStream
, getServoIsFreeMovingStreamReq
, getServoIsLocked
, getServoIsLockedStream
, getServoIsLockedStreamReq
, getServoIsMoving
, getServoIsMovingStream
, getServoIsMovingStreamReq
, getServoMaxConfigPosition
, getServoMaxConfigPositionStream
, getServoMaxConfigPositionStreamReq
, getServoMaxPosition
, getServoMaxPositionStream
, getServoMaxPositionStreamReq
, getServoMinConfigPosition
, getServoMinConfigPositionStream
, getServoMinConfigPositionStreamReq
, getServoMinPosition
, getServoMinPositionStream
, getServoMinPositionStreamReq
, getServoName
, getServoNameStream
, getServoNameStreamReq
, getServoPart
, getServoPartStream
, getServoPartStreamReq
, getServoPosition
, getServoPositionStream
, getServoPositionStreamReq
, getServoSpeed
, getServoSpeedStream
, getServoSpeedStreamReq
, setServoAcceleration
, setServoCurrentSpeed
, setServoHighlight
, setServoIsAxisInverted
, setServoIsLocked
, setServoMaxPosition
, setServoMinPosition
, setServoName
, setServoSpeed
, getAvailable
, getAvailableStream
, getAvailableStreamReq
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import Control.Monad.Catch
import Control.Monad.IO.Class

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - Represents a servo. Obtained using
 - <see cref="M:InfernalRobotics.ServoGroup.Servos" />,
 - <see cref="M:InfernalRobotics.ServoGroup.ServoWithName" />
 - or <see cref="M:InfernalRobotics.ServoWithName" />.
 -}
newtype Servo = Servo { servoId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Servo where
    encodePb   = encodePb . servoId
    decodePb b = Servo <$> decodePb b

instance KRPCResponseExtractable Servo

{-
 - A group of servos, obtained by calling <see cref="M:InfernalRobotics.ServoGroups" />
 - or <see cref="M:InfernalRobotics.ServoGroupWithName" />. Represents the "Servo Groups"
 - in the InfernalRobotics UI.
 -}
newtype ServoGroup = ServoGroup { servoGroupId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ServoGroup where
    encodePb   = encodePb . servoGroupId
    decodePb b = ServoGroup <$> decodePb b

instance KRPCResponseExtractable ServoGroup



{-
 - Returns the servo group in the given <paramref name="vessel" /> with the given <paramref name="name" />,
 - ornullif none exists. If multiple servo groups have the same name, only one of them is returned.<param name="vessel">Vessel to check.<param name="name">Name of servo group to find.
 -}
servoGroupWithName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> m (KRPCHS.InfernalRobotics.ServoGroup)
servoGroupWithName vesselArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroupWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

servoGroupWithNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCStreamReq (KRPCHS.InfernalRobotics.ServoGroup)
servoGroupWithNameStreamReq vesselArg nameArg =
    let req = makeRequest "InfernalRobotics" "ServoGroupWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
    in  makeStream req

servoGroupWithNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> m (KRPCStream (KRPCHS.InfernalRobotics.ServoGroup))
servoGroupWithNameStream vesselArg nameArg = requestStream $ servoGroupWithNameStreamReq vesselArg nameArg 

{-
 - Moves all of the servos in the group to the center.
 -}
servoGroupMoveCenter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ()
servoGroupMoveCenter thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveCenter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves all of the servos in the group to the left.
 -}
servoGroupMoveLeft :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ()
servoGroupMoveLeft thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveLeft" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves all of the servos in the group to the next preset.
 -}
servoGroupMoveNextPreset :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ()
servoGroupMoveNextPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveNextPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves all of the servos in the group to the previous preset.
 -}
servoGroupMovePrevPreset :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ()
servoGroupMovePrevPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MovePrevPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves all of the servos in the group to the right.
 -}
servoGroupMoveRight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ()
servoGroupMoveRight thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveRight" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns the servo with the given <paramref name="name" /> from this group,
 - ornullif none exists.<param name="name">Name of servo to find.
 -}
servoGroupServoWithName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> m (KRPCHS.InfernalRobotics.Servo)
servoGroupServoWithName thisArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_ServoWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

servoGroupServoWithNameStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> KRPCStreamReq (KRPCHS.InfernalRobotics.Servo)
servoGroupServoWithNameStreamReq thisArg nameArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_ServoWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

servoGroupServoWithNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> m (KRPCStream (KRPCHS.InfernalRobotics.Servo))
servoGroupServoWithNameStream thisArg nameArg = requestStream $ servoGroupServoWithNameStreamReq thisArg nameArg 

{-
 - Stops the servos in the group.
 -}
servoGroupStop :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ()
servoGroupStop thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_Stop" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the group is expanded in the InfernalRobotics UI.
 -}
getServoGroupExpanded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (Bool)
getServoGroupExpanded thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Expanded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupExpandedStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Bool)
getServoGroupExpandedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_Expanded" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupExpandedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream (Bool))
getServoGroupExpandedStream thisArg = requestStream $ getServoGroupExpandedStreamReq thisArg 

{-
 - The key assigned to be the "forward" key for the group.
 -}
getServoGroupForwardKey :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (Data.Text.Text)
getServoGroupForwardKey thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_ForwardKey" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupForwardKeyStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Data.Text.Text)
getServoGroupForwardKeyStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_ForwardKey" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupForwardKeyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream (Data.Text.Text))
getServoGroupForwardKeyStream thisArg = requestStream $ getServoGroupForwardKeyStreamReq thisArg 

{-
 - The name of the group.
 -}
getServoGroupName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (Data.Text.Text)
getServoGroupName thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupNameStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Data.Text.Text)
getServoGroupNameStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream (Data.Text.Text))
getServoGroupNameStream thisArg = requestStream $ getServoGroupNameStreamReq thisArg 

{-
 - The parts containing the servos in the group.
 -}
getServoGroupParts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ([KRPCHS.SpaceCenter.Part])
getServoGroupParts thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Parts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupPartsStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getServoGroupPartsStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_Parts" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupPartsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getServoGroupPartsStream thisArg = requestStream $ getServoGroupPartsStreamReq thisArg 

{-
 - The key assigned to be the "reverse" key for the group.
 -}
getServoGroupReverseKey :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (Data.Text.Text)
getServoGroupReverseKey thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_ReverseKey" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupReverseKeyStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Data.Text.Text)
getServoGroupReverseKeyStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_ReverseKey" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupReverseKeyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream (Data.Text.Text))
getServoGroupReverseKeyStream thisArg = requestStream $ getServoGroupReverseKeyStreamReq thisArg 

{-
 - The servos that are in the group.
 -}
getServoGroupServos :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m ([KRPCHS.InfernalRobotics.Servo])
getServoGroupServos thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Servos" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupServosStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq ([KRPCHS.InfernalRobotics.Servo])
getServoGroupServosStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_Servos" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupServosStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream ([KRPCHS.InfernalRobotics.Servo]))
getServoGroupServosStream thisArg = requestStream $ getServoGroupServosStreamReq thisArg 

{-
 - The speed multiplier for the group.
 -}
getServoGroupSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (Float)
getServoGroupSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoGroupSpeedStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Float)
getServoGroupSpeedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "ServoGroup_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getServoGroupSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> m (KRPCStream (Float))
getServoGroupSpeedStream thisArg = requestStream $ getServoGroupSpeedStreamReq thisArg 

{-
 - Whether the group is expanded in the InfernalRobotics UI.
 -}
setServoGroupExpanded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Bool -> m ()
setServoGroupExpanded thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_Expanded" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The key assigned to be the "forward" key for the group.
 -}
setServoGroupForwardKey :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> m ()
setServoGroupForwardKey thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_ForwardKey" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The name of the group.
 -}
setServoGroupName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> m ()
setServoGroupName thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The key assigned to be the "reverse" key for the group.
 -}
setServoGroupReverseKey :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> m ()
setServoGroupReverseKey thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_ReverseKey" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The speed multiplier for the group.
 -}
setServoGroupSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.ServoGroup -> Float -> m ()
setServoGroupSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_Speed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - A list of all the servo groups in the given <paramref name="vessel" />.
 -}
servoGroups :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m ([KRPCHS.InfernalRobotics.ServoGroup])
servoGroups vesselArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroups" [makeArgument 0 vesselArg]
    res <- sendRequest r
    processResponse res

servoGroupsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ([KRPCHS.InfernalRobotics.ServoGroup])
servoGroupsStreamReq vesselArg =
    let req = makeRequest "InfernalRobotics" "ServoGroups" [makeArgument 0 vesselArg]
    in  makeStream req

servoGroupsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream ([KRPCHS.InfernalRobotics.ServoGroup]))
servoGroupsStream vesselArg = requestStream $ servoGroupsStreamReq vesselArg 

{-
 - Returns the servo in the given <paramref name="vessel" /> with the given <paramref name="name" /> ornullif none exists. If multiple servos have the same name, only one of them is returned.<param name="vessel">Vessel to check.<param name="name">Name of the servo to find.
 -}
servoWithName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> m (KRPCHS.InfernalRobotics.Servo)
servoWithName vesselArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

servoWithNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCStreamReq (KRPCHS.InfernalRobotics.Servo)
servoWithNameStreamReq vesselArg nameArg =
    let req = makeRequest "InfernalRobotics" "ServoWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
    in  makeStream req

servoWithNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> m (KRPCStream (KRPCHS.InfernalRobotics.Servo))
servoWithNameStream vesselArg nameArg = requestStream $ servoWithNameStreamReq vesselArg nameArg 

{-
 - Moves the servo to the center.
 -}
servoMoveCenter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m ()
servoMoveCenter thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveCenter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves the servo to the left.
 -}
servoMoveLeft :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m ()
servoMoveLeft thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveLeft" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves the servo to the next preset.
 -}
servoMoveNextPreset :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m ()
servoMoveNextPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveNextPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves the servo to the previous preset.
 -}
servoMovePrevPreset :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m ()
servoMovePrevPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MovePrevPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves the servo to the right.
 -}
servoMoveRight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m ()
servoMoveRight thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveRight" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Moves the servo to <paramref name="position" /> and sets the
 - speed multiplier to <paramref name="speed" />.<param name="position">The position to move the servo to.<param name="speed">Speed multiplier for the movement.
 -}
servoMoveTo :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Float -> Float -> m ()
servoMoveTo thisArg positionArg speedArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveTo" [makeArgument 0 thisArg, makeArgument 1 positionArg, makeArgument 2 speedArg]
    res <- sendRequest r
    processResponse res 

{-
 - Stops the servo.
 -}
servoStop :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m ()
servoStop thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_Stop" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current speed multiplier set in the UI.
 -}
getServoAcceleration :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoAcceleration thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Acceleration" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoAccelerationStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoAccelerationStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_Acceleration" [makeArgument 0 thisArg]
    in  makeStream req

getServoAccelerationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoAccelerationStream thisArg = requestStream $ getServoAccelerationStreamReq thisArg 

{-
 - The speed multiplier of the servo, specified by the part configuration.
 -}
getServoConfigSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoConfigSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_ConfigSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoConfigSpeedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoConfigSpeedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_ConfigSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getServoConfigSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoConfigSpeedStream thisArg = requestStream $ getServoConfigSpeedStreamReq thisArg 

{-
 - The current speed at which the servo is moving.
 -}
getServoCurrentSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoCurrentSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_CurrentSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoCurrentSpeedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoCurrentSpeedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_CurrentSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getServoCurrentSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoCurrentSpeedStream thisArg = requestStream $ getServoCurrentSpeedStreamReq thisArg 

{-
 - Whether the servos axis is inverted.
 -}
getServoIsAxisInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Bool)
getServoIsAxisInverted thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsAxisInverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoIsAxisInvertedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsAxisInvertedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_IsAxisInverted" [makeArgument 0 thisArg]
    in  makeStream req

getServoIsAxisInvertedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Bool))
getServoIsAxisInvertedStream thisArg = requestStream $ getServoIsAxisInvertedStreamReq thisArg 

{-
 - Whether the servo is freely moving.
 -}
getServoIsFreeMoving :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Bool)
getServoIsFreeMoving thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsFreeMoving" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoIsFreeMovingStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsFreeMovingStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_IsFreeMoving" [makeArgument 0 thisArg]
    in  makeStream req

getServoIsFreeMovingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Bool))
getServoIsFreeMovingStream thisArg = requestStream $ getServoIsFreeMovingStreamReq thisArg 

{-
 - Whether the servo is locked.
 -}
getServoIsLocked :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Bool)
getServoIsLocked thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoIsLockedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsLockedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_IsLocked" [makeArgument 0 thisArg]
    in  makeStream req

getServoIsLockedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Bool))
getServoIsLockedStream thisArg = requestStream $ getServoIsLockedStreamReq thisArg 

{-
 - Whether the servo is moving.
 -}
getServoIsMoving :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Bool)
getServoIsMoving thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsMoving" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoIsMovingStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsMovingStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_IsMoving" [makeArgument 0 thisArg]
    in  makeStream req

getServoIsMovingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Bool))
getServoIsMovingStream thisArg = requestStream $ getServoIsMovingStreamReq thisArg 

{-
 - The maximum position of the servo, specified by the part configuration.
 -}
getServoMaxConfigPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoMaxConfigPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxConfigPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoMaxConfigPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMaxConfigPositionStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_MaxConfigPosition" [makeArgument 0 thisArg]
    in  makeStream req

getServoMaxConfigPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoMaxConfigPositionStream thisArg = requestStream $ getServoMaxConfigPositionStreamReq thisArg 

{-
 - The maximum position of the servo, specified by the in-game tweak menu.
 -}
getServoMaxPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoMaxPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoMaxPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMaxPositionStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_MaxPosition" [makeArgument 0 thisArg]
    in  makeStream req

getServoMaxPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoMaxPositionStream thisArg = requestStream $ getServoMaxPositionStreamReq thisArg 

{-
 - The minimum position of the servo, specified by the part configuration.
 -}
getServoMinConfigPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoMinConfigPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinConfigPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoMinConfigPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMinConfigPositionStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_MinConfigPosition" [makeArgument 0 thisArg]
    in  makeStream req

getServoMinConfigPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoMinConfigPositionStream thisArg = requestStream $ getServoMinConfigPositionStreamReq thisArg 

{-
 - The minimum position of the servo, specified by the in-game tweak menu.
 -}
getServoMinPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoMinPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoMinPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMinPositionStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_MinPosition" [makeArgument 0 thisArg]
    in  makeStream req

getServoMinPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoMinPositionStream thisArg = requestStream $ getServoMinPositionStreamReq thisArg 

{-
 - The name of the servo.
 -}
getServoName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Data.Text.Text)
getServoName thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoNameStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Data.Text.Text)
getServoNameStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getServoNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Data.Text.Text))
getServoNameStream thisArg = requestStream $ getServoNameStreamReq thisArg 

{-
 - The part containing the servo.
 -}
getServoPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCHS.SpaceCenter.Part)
getServoPart thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoPartStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getServoPartStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getServoPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getServoPartStream thisArg = requestStream $ getServoPartStreamReq thisArg 

{-
 - The position of the servo.
 -}
getServoPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoPositionStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_Position" [makeArgument 0 thisArg]
    in  makeStream req

getServoPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoPositionStream thisArg = requestStream $ getServoPositionStreamReq thisArg 

{-
 - The speed multiplier of the servo, specified by the in-game tweak menu.
 -}
getServoSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (Float)
getServoSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getServoSpeedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoSpeedStreamReq thisArg =
    let req = makeRequest "InfernalRobotics" "Servo_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getServoSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> m (KRPCStream (Float))
getServoSpeedStream thisArg = requestStream $ getServoSpeedStreamReq thisArg 

{-
 - The current speed multiplier set in the UI.
 -}
setServoAcceleration :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Float -> m ()
setServoAcceleration thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Acceleration" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current speed at which the servo is moving.
 -}
setServoCurrentSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Float -> m ()
setServoCurrentSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_CurrentSpeed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the servo should be highlighted in-game.
 -}
setServoHighlight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Bool -> m ()
setServoHighlight thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Highlight" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the servos axis is inverted.
 -}
setServoIsAxisInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Bool -> m ()
setServoIsAxisInverted thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_IsAxisInverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the servo is locked.
 -}
setServoIsLocked :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Bool -> m ()
setServoIsLocked thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_IsLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The maximum position of the servo, specified by the in-game tweak menu.
 -}
setServoMaxPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Float -> m ()
setServoMaxPosition thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_MaxPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The minimum position of the servo, specified by the in-game tweak menu.
 -}
setServoMinPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Float -> m ()
setServoMinPosition thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_MinPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The name of the servo.
 -}
setServoName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Data.Text.Text -> m ()
setServoName thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The speed multiplier of the servo, specified by the in-game tweak menu.
 -}
setServoSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.InfernalRobotics.Servo -> Float -> m ()
setServoSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Speed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether Infernal Robotics is installed.
 -}
getAvailable :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Bool)
getAvailable  = do
    let r = makeRequest "InfernalRobotics" "get_Available" []
    res <- sendRequest r
    processResponse res

getAvailableStreamReq :: KRPCStreamReq (Bool)
getAvailableStreamReq  =
    let req = makeRequest "InfernalRobotics" "get_Available" []
    in  makeStream req

getAvailableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Bool))
getAvailableStream  = requestStream $ getAvailableStreamReq  

