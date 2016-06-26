module KRPCHS.InfernalRobotics
( Servo
, ServoGroup
, servoGroupWithName
, servoGroupWithNameStream
, servoGroupMoveCenter
, servoGroupMoveLeft
, servoGroupMoveNextPreset
, servoGroupMovePrevPreset
, servoGroupMoveRight
, servoGroupServoWithName
, servoGroupServoWithNameStream
, servoGroupStop
, getServoGroupExpanded
, getServoGroupExpandedStream
, getServoGroupForwardKey
, getServoGroupForwardKeyStream
, getServoGroupName
, getServoGroupNameStream
, getServoGroupParts
, getServoGroupPartsStream
, getServoGroupReverseKey
, getServoGroupReverseKeyStream
, getServoGroupServos
, getServoGroupServosStream
, getServoGroupSpeed
, getServoGroupSpeedStream
, setServoGroupExpanded
, setServoGroupForwardKey
, setServoGroupName
, setServoGroupReverseKey
, setServoGroupSpeed
, servoGroups
, servoGroupsStream
, servoWithName
, servoWithNameStream
, servoMoveCenter
, servoMoveLeft
, servoMoveNextPreset
, servoMovePrevPreset
, servoMoveRight
, servoMoveTo
, servoStop
, getServoAcceleration
, getServoAccelerationStream
, getServoConfigSpeed
, getServoConfigSpeedStream
, getServoCurrentSpeed
, getServoCurrentSpeedStream
, getServoIsAxisInverted
, getServoIsAxisInvertedStream
, getServoIsFreeMoving
, getServoIsFreeMovingStream
, getServoIsLocked
, getServoIsLockedStream
, getServoIsMoving
, getServoIsMovingStream
, getServoMaxConfigPosition
, getServoMaxConfigPositionStream
, getServoMaxPosition
, getServoMaxPositionStream
, getServoMinConfigPosition
, getServoMinConfigPositionStream
, getServoMinPosition
, getServoMinPositionStream
, getServoName
, getServoNameStream
, getServoPart
, getServoPartStream
, getServoPosition
, getServoPositionStream
, getServoSpeed
, getServoSpeedStream
, setServoAcceleration
, setServoCurrentSpeed
, setServoHighlight
, setServoIsAxisInverted
, setServoIsLocked
, setServoMaxPosition
, setServoMinPosition
, setServoName
, setServoSpeed
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

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
servoGroupWithName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCHS.InfernalRobotics.ServoGroup)
servoGroupWithName vesselArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroupWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

servoGroupWithNameStream :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.InfernalRobotics.ServoGroup))
servoGroupWithNameStream vesselArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroupWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Moves all of the servos in the group to the center.
 -}
servoGroupMoveCenter :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
servoGroupMoveCenter thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveCenter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves all of the servos in the group to the left.
 -}
servoGroupMoveLeft :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
servoGroupMoveLeft thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveLeft" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves all of the servos in the group to the next preset.
 -}
servoGroupMoveNextPreset :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
servoGroupMoveNextPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveNextPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves all of the servos in the group to the previous preset.
 -}
servoGroupMovePrevPreset :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
servoGroupMovePrevPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MovePrevPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves all of the servos in the group to the right.
 -}
servoGroupMoveRight :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
servoGroupMoveRight thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_MoveRight" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Returns the servo with the given <paramref name="name" /> from this group,
 - ornullif none exists.<param name="name">Name of servo to find.
 -}
servoGroupServoWithName :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (KRPCHS.InfernalRobotics.Servo)
servoGroupServoWithName thisArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_ServoWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

servoGroupServoWithNameStream :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.InfernalRobotics.Servo))
servoGroupServoWithNameStream thisArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_ServoWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Stops the servos in the group.
 -}
servoGroupStop :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
servoGroupStop thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_Stop" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the group is expanded in the InfernalRobotics UI.
 -}
getServoGroupExpanded :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
getServoGroupExpanded thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Expanded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupExpandedStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Bool))
getServoGroupExpandedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Expanded" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The key assigned to be the "forward" key for the group.
 -}
getServoGroupForwardKey :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Data.Text.Text)
getServoGroupForwardKey thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_ForwardKey" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupForwardKeyStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Data.Text.Text))
getServoGroupForwardKeyStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_ForwardKey" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the group.
 -}
getServoGroupName :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Data.Text.Text)
getServoGroupName thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupNameStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Data.Text.Text))
getServoGroupNameStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The parts containing the servos in the group.
 -}
getServoGroupParts :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ([KRPCHS.SpaceCenter.Part])
getServoGroupParts thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Parts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupPartsStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getServoGroupPartsStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Parts" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The key assigned to be the "reverse" key for the group.
 -}
getServoGroupReverseKey :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Data.Text.Text)
getServoGroupReverseKey thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_ReverseKey" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupReverseKeyStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Data.Text.Text))
getServoGroupReverseKeyStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_ReverseKey" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The servos that are in the group.
 -}
getServoGroupServos :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ([KRPCHS.InfernalRobotics.Servo])
getServoGroupServos thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Servos" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupServosStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream ([KRPCHS.InfernalRobotics.Servo]))
getServoGroupServosStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Servos" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The speed multiplier for the group.
 -}
getServoGroupSpeed :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Float)
getServoGroupSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoGroupSpeedStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Float))
getServoGroupSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_get_Speed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the group is expanded in the InfernalRobotics UI.
 -}
setServoGroupExpanded :: KRPCHS.InfernalRobotics.ServoGroup -> Bool -> RPCContext (Bool)
setServoGroupExpanded thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_Expanded" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The key assigned to be the "forward" key for the group.
 -}
setServoGroupForwardKey :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (Bool)
setServoGroupForwardKey thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_ForwardKey" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The name of the group.
 -}
setServoGroupName :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (Bool)
setServoGroupName thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The key assigned to be the "reverse" key for the group.
 -}
setServoGroupReverseKey :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (Bool)
setServoGroupReverseKey thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_ReverseKey" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The speed multiplier for the group.
 -}
setServoGroupSpeed :: KRPCHS.InfernalRobotics.ServoGroup -> Float -> RPCContext (Bool)
setServoGroupSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroup_set_Speed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - A list of all the servo groups in the given <paramref name="vessel" />.
 -}
servoGroups :: KRPCHS.SpaceCenter.Vessel -> RPCContext ([KRPCHS.InfernalRobotics.ServoGroup])
servoGroups vesselArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroups" [makeArgument 0 vesselArg]
    res <- sendRequest r
    processResponse extract res 

servoGroupsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ([KRPCHS.InfernalRobotics.ServoGroup]))
servoGroupsStream vesselArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroups" [makeArgument 0 vesselArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the servo in the given <paramref name="vessel" /> with the given <paramref name="name" /> ornullif none exists. If multiple servos have the same name, only one of them is returned.<param name="vessel">Vessel to check.<param name="name">Name of the servo to find.
 -}
servoWithName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCHS.InfernalRobotics.Servo)
servoWithName vesselArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

servoWithNameStream :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.InfernalRobotics.Servo))
servoWithNameStream vesselArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Moves the servo to the center.
 -}
servoMoveCenter :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
servoMoveCenter thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveCenter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves the servo to the left.
 -}
servoMoveLeft :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
servoMoveLeft thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveLeft" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves the servo to the next preset.
 -}
servoMoveNextPreset :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
servoMoveNextPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveNextPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves the servo to the previous preset.
 -}
servoMovePrevPreset :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
servoMovePrevPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MovePrevPreset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves the servo to the right.
 -}
servoMoveRight :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
servoMoveRight thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveRight" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Moves the servo to <paramref name="position" /> and sets the
 - speed multiplier to <paramref name="speed" />.<param name="position">The position to move the servo to.<param name="speed">Speed multiplier for the movement.
 -}
servoMoveTo :: KRPCHS.InfernalRobotics.Servo -> Float -> Float -> RPCContext (Bool)
servoMoveTo thisArg positionArg speedArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveTo" [makeArgument 0 thisArg, makeArgument 1 positionArg, makeArgument 2 speedArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Stops the servo.
 -}
servoStop :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
servoStop thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_Stop" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The current speed multiplier set in the UI.
 -}
getServoAcceleration :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoAcceleration thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Acceleration" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoAccelerationStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoAccelerationStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Acceleration" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The speed multiplier of the servo, specified by the part configuration.
 -}
getServoConfigSpeed :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoConfigSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_ConfigSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoConfigSpeedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoConfigSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_ConfigSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current speed at which the servo is moving.
 -}
getServoCurrentSpeed :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoCurrentSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_CurrentSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoCurrentSpeedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoCurrentSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_CurrentSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the servos axis is inverted.
 -}
getServoIsAxisInverted :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsAxisInverted thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsAxisInverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoIsAxisInvertedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsAxisInvertedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsAxisInverted" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the servo is freely moving.
 -}
getServoIsFreeMoving :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsFreeMoving thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsFreeMoving" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoIsFreeMovingStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsFreeMovingStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsFreeMoving" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the servo is locked.
 -}
getServoIsLocked :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsLocked thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoIsLockedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsLockedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsLocked" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the servo is moving.
 -}
getServoIsMoving :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsMoving thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsMoving" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoIsMovingStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsMovingStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsMoving" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum position of the servo, specified by the part configuration.
 -}
getServoMaxConfigPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMaxConfigPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxConfigPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoMaxConfigPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMaxConfigPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxConfigPosition" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum position of the servo, specified by the in-game tweak menu.
 -}
getServoMaxPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMaxPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoMaxPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMaxPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxPosition" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The minimum position of the servo, specified by the part configuration.
 -}
getServoMinConfigPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMinConfigPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinConfigPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoMinConfigPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMinConfigPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinConfigPosition" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The minimum position of the servo, specified by the in-game tweak menu.
 -}
getServoMinPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMinPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoMinPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMinPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinPosition" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the servo.
 -}
getServoName :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Data.Text.Text)
getServoName thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoNameStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Data.Text.Text))
getServoNameStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part containing the servo.
 -}
getServoPart :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCHS.SpaceCenter.Part)
getServoPart thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoPartStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getServoPartStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position of the servo.
 -}
getServoPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Position" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The speed multiplier of the servo, specified by the in-game tweak menu.
 -}
getServoSpeed :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getServoSpeedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Speed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current speed multiplier set in the UI.
 -}
setServoAcceleration :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext (Bool)
setServoAcceleration thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Acceleration" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The current speed at which the servo is moving.
 -}
setServoCurrentSpeed :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext (Bool)
setServoCurrentSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_CurrentSpeed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the servo should be highlighted in-game.
 -}
setServoHighlight :: KRPCHS.InfernalRobotics.Servo -> Bool -> RPCContext (Bool)
setServoHighlight thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Highlight" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the servos axis is inverted.
 -}
setServoIsAxisInverted :: KRPCHS.InfernalRobotics.Servo -> Bool -> RPCContext (Bool)
setServoIsAxisInverted thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_IsAxisInverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the servo is locked.
 -}
setServoIsLocked :: KRPCHS.InfernalRobotics.Servo -> Bool -> RPCContext (Bool)
setServoIsLocked thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_IsLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The maximum position of the servo, specified by the in-game tweak menu.
 -}
setServoMaxPosition :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext (Bool)
setServoMaxPosition thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_MaxPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The minimum position of the servo, specified by the in-game tweak menu.
 -}
setServoMinPosition :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext (Bool)
setServoMinPosition thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_MinPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The name of the servo.
 -}
setServoName :: KRPCHS.InfernalRobotics.Servo -> Data.Text.Text -> RPCContext (Bool)
setServoName thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The speed multiplier of the servo, specified by the in-game tweak menu.
 -}
setServoSpeed :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext (Bool)
setServoSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Speed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


