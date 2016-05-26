{-# LANGUAGE RecordWildCards #-}
module KRPCHS.InfernalRobotics
( ControlGroup(..)
, Servo(..)
, servoGroupWithName
, servoWithName
, getServoGroups
, controlGroupServoWithName
, controlGroupMoveRight
, controlGroupMoveLeft
, controlGroupMoveCenter
, controlGroupMoveNextPreset
, controlGroupMovePrevPreset
, controlGroupStop
, getControlGroupName
, setControlGroupName
, getControlGroupForwardKey
, setControlGroupForwardKey
, getControlGroupReverseKey
, setControlGroupReverseKey
, getControlGroupSpeed
, setControlGroupSpeed
, getControlGroupExpanded
, setControlGroupExpanded
, getControlGroupServos
, servoMoveRight
, servoMoveLeft
, servoMoveCenter
, servoMoveNextPreset
, servoMovePrevPreset
, servoMoveTo
, servoStop
, getServoName
, setServoName
, setServoHighlight
, getServoPosition
, getServoMinConfigPosition
, getServoMaxConfigPosition
, getServoMinPosition
, setServoMinPosition
, getServoMaxPosition
, setServoMaxPosition
, getServoConfigSpeed
, getServoSpeed
, setServoSpeed
, getServoCurrentSpeed
, setServoCurrentSpeed
, getServoAcceleration
, setServoAcceleration
, getServoIsMoving
, getServoIsFreeMoving
, getServoIsLocked
, setServoIsLocked
, getServoIsAxisInverted
, setServoIsAxisInverted
, servoGroupWithNameStream
, servoWithNameStream
, getServoGroupsStream
, controlGroupServoWithNameStream
, getControlGroupNameStream
, getControlGroupForwardKeyStream
, getControlGroupReverseKeyStream
, getControlGroupSpeedStream
, getControlGroupExpandedStream
, getControlGroupServosStream
, getServoNameStream
, getServoPositionStream
, getServoMinConfigPositionStream
, getServoMaxConfigPositionStream
, getServoMinPositionStream
, getServoMaxPositionStream
, getServoConfigSpeedStream
, getServoSpeedStream
, getServoCurrentSpeedStream
, getServoAccelerationStream
, getServoIsMovingStream
, getServoIsFreeMovingStream
, getServoIsLockedStream
, getServoIsAxisInvertedStream
) where


import Data.Text

import KRPCHS.Requests
import KRPCHS.SerializeUtils


newtype ControlGroup = ControlGroup { controlGroupId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ControlGroup where
    encodePb   = encodePb . controlGroupId
    decodePb b = ControlGroup <$> decodePb b

newtype Servo = Servo { servoId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Servo where
    encodePb   = encodePb . servoId
    decodePb b = Servo <$> decodePb b

servoGroupWithName :: Text -> RPCContext (ControlGroup)
servoGroupWithName nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroupWithName" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

servoGroupWithNameStream :: Text -> RPCContext (KRPCStream (ControlGroup))
servoGroupWithNameStream nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoGroupWithName" [ makeArgument 0 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

servoWithName :: Text -> RPCContext (Servo)
servoWithName nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoWithName" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

servoWithNameStream :: Text -> RPCContext (KRPCStream (Servo))
servoWithNameStream nameArg = do
    let r = makeRequest "InfernalRobotics" "ServoWithName" [ makeArgument 0 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoGroups :: RPCContext ([ControlGroup])
getServoGroups  = do
    let r = makeRequest "InfernalRobotics" "get_ServoGroups" [  ]
    res <- sendRequest r
    processResponse extractList res

getServoGroupsStream :: RPCContext (KRPCStream ([ControlGroup]))
getServoGroupsStream  = do
    let r = makeRequest "InfernalRobotics" "get_ServoGroups" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

controlGroupServoWithName :: ControlGroup -> Text -> RPCContext (Servo)
controlGroupServoWithName thisArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_ServoWithName" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

controlGroupServoWithNameStream :: ControlGroup -> Text -> RPCContext (KRPCStream (Servo))
controlGroupServoWithNameStream thisArg nameArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_ServoWithName" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

controlGroupMoveRight :: ControlGroup -> RPCContext (Bool)
controlGroupMoveRight thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_MoveRight" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

controlGroupMoveLeft :: ControlGroup -> RPCContext (Bool)
controlGroupMoveLeft thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_MoveLeft" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

controlGroupMoveCenter :: ControlGroup -> RPCContext (Bool)
controlGroupMoveCenter thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_MoveCenter" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

controlGroupMoveNextPreset :: ControlGroup -> RPCContext (Bool)
controlGroupMoveNextPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_MoveNextPreset" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

controlGroupMovePrevPreset :: ControlGroup -> RPCContext (Bool)
controlGroupMovePrevPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_MovePrevPreset" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

controlGroupStop :: ControlGroup -> RPCContext (Bool)
controlGroupStop thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_Stop" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGroupName :: ControlGroup -> RPCContext (Text)
getControlGroupName thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Name" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlGroupNameStream :: ControlGroup -> RPCContext (KRPCStream (Text))
getControlGroupNameStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Name" [ makeArgument 0 (controlGroupId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlGroupName :: ControlGroup -> Text -> RPCContext (Bool)
setControlGroupName thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_set_Name" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGroupForwardKey :: ControlGroup -> RPCContext (Text)
getControlGroupForwardKey thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_ForwardKey" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlGroupForwardKeyStream :: ControlGroup -> RPCContext (KRPCStream (Text))
getControlGroupForwardKeyStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_ForwardKey" [ makeArgument 0 (controlGroupId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlGroupForwardKey :: ControlGroup -> Text -> RPCContext (Bool)
setControlGroupForwardKey thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_set_ForwardKey" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGroupReverseKey :: ControlGroup -> RPCContext (Text)
getControlGroupReverseKey thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_ReverseKey" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlGroupReverseKeyStream :: ControlGroup -> RPCContext (KRPCStream (Text))
getControlGroupReverseKeyStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_ReverseKey" [ makeArgument 0 (controlGroupId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlGroupReverseKey :: ControlGroup -> Text -> RPCContext (Bool)
setControlGroupReverseKey thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_set_ReverseKey" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGroupSpeed :: ControlGroup -> RPCContext (Float)
getControlGroupSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Speed" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlGroupSpeedStream :: ControlGroup -> RPCContext (KRPCStream (Float))
getControlGroupSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Speed" [ makeArgument 0 (controlGroupId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlGroupSpeed :: ControlGroup -> Float -> RPCContext (Bool)
setControlGroupSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_set_Speed" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGroupExpanded :: ControlGroup -> RPCContext (Bool)
getControlGroupExpanded thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Expanded" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlGroupExpandedStream :: ControlGroup -> RPCContext (KRPCStream (Bool))
getControlGroupExpandedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Expanded" [ makeArgument 0 (controlGroupId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlGroupExpanded :: ControlGroup -> Bool -> RPCContext (Bool)
setControlGroupExpanded thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_set_Expanded" [ makeArgument 0 (controlGroupId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGroupServos :: ControlGroup -> RPCContext ([Servo])
getControlGroupServos thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Servos" [ makeArgument 0 (controlGroupId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getControlGroupServosStream :: ControlGroup -> RPCContext (KRPCStream ([Servo]))
getControlGroupServosStream thisArg = do
    let r = makeRequest "InfernalRobotics" "ControlGroup_get_Servos" [ makeArgument 0 (controlGroupId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

servoMoveRight :: Servo -> RPCContext (Bool)
servoMoveRight thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveRight" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

servoMoveLeft :: Servo -> RPCContext (Bool)
servoMoveLeft thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveLeft" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

servoMoveCenter :: Servo -> RPCContext (Bool)
servoMoveCenter thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveCenter" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

servoMoveNextPreset :: Servo -> RPCContext (Bool)
servoMoveNextPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveNextPreset" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

servoMovePrevPreset :: Servo -> RPCContext (Bool)
servoMovePrevPreset thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MovePrevPreset" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

servoMoveTo :: Servo -> Float -> Float -> RPCContext (Bool)
servoMoveTo thisArg positionArg speedArg = do
    let r = makeRequest "InfernalRobotics" "Servo_MoveTo" [ makeArgument 0 (servoId thisArg), makeArgument 1 positionArg, makeArgument 2 speedArg ]
    res <- sendRequest r
    processResponse extractNothing res

servoStop :: Servo -> RPCContext (Bool)
servoStop thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_Stop" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getServoName :: Servo -> RPCContext (Text)
getServoName thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Name" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoNameStream :: Servo -> RPCContext (KRPCStream (Text))
getServoNameStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Name" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoName :: Servo -> Text -> RPCContext (Bool)
setServoName thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Name" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

setServoHighlight :: Servo -> Bool -> RPCContext (Bool)
setServoHighlight thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Highlight" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoPosition :: Servo -> RPCContext (Float)
getServoPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Position" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoPositionStream :: Servo -> RPCContext (KRPCStream (Float))
getServoPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Position" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoMinConfigPosition :: Servo -> RPCContext (Float)
getServoMinConfigPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinConfigPosition" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoMinConfigPositionStream :: Servo -> RPCContext (KRPCStream (Float))
getServoMinConfigPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinConfigPosition" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoMaxConfigPosition :: Servo -> RPCContext (Float)
getServoMaxConfigPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxConfigPosition" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoMaxConfigPositionStream :: Servo -> RPCContext (KRPCStream (Float))
getServoMaxConfigPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxConfigPosition" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoMinPosition :: Servo -> RPCContext (Float)
getServoMinPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinPosition" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoMinPositionStream :: Servo -> RPCContext (KRPCStream (Float))
getServoMinPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MinPosition" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoMinPosition :: Servo -> Float -> RPCContext (Bool)
setServoMinPosition thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_MinPosition" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoMaxPosition :: Servo -> RPCContext (Float)
getServoMaxPosition thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxPosition" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoMaxPositionStream :: Servo -> RPCContext (KRPCStream (Float))
getServoMaxPositionStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_MaxPosition" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoMaxPosition :: Servo -> Float -> RPCContext (Bool)
setServoMaxPosition thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_MaxPosition" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoConfigSpeed :: Servo -> RPCContext (Float)
getServoConfigSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_ConfigSpeed" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoConfigSpeedStream :: Servo -> RPCContext (KRPCStream (Float))
getServoConfigSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_ConfigSpeed" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoSpeed :: Servo -> RPCContext (Float)
getServoSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Speed" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoSpeedStream :: Servo -> RPCContext (KRPCStream (Float))
getServoSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Speed" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoSpeed :: Servo -> Float -> RPCContext (Bool)
setServoSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Speed" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoCurrentSpeed :: Servo -> RPCContext (Float)
getServoCurrentSpeed thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_CurrentSpeed" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoCurrentSpeedStream :: Servo -> RPCContext (KRPCStream (Float))
getServoCurrentSpeedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_CurrentSpeed" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoCurrentSpeed :: Servo -> Float -> RPCContext (Bool)
setServoCurrentSpeed thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_CurrentSpeed" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoAcceleration :: Servo -> RPCContext (Float)
getServoAcceleration thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Acceleration" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoAccelerationStream :: Servo -> RPCContext (KRPCStream (Float))
getServoAccelerationStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_Acceleration" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoAcceleration :: Servo -> Float -> RPCContext (Bool)
setServoAcceleration thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_Acceleration" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoIsMoving :: Servo -> RPCContext (Bool)
getServoIsMoving thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsMoving" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoIsMovingStream :: Servo -> RPCContext (KRPCStream (Bool))
getServoIsMovingStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsMoving" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoIsFreeMoving :: Servo -> RPCContext (Bool)
getServoIsFreeMoving thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsFreeMoving" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoIsFreeMovingStream :: Servo -> RPCContext (KRPCStream (Bool))
getServoIsFreeMovingStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsFreeMoving" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getServoIsLocked :: Servo -> RPCContext (Bool)
getServoIsLocked thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsLocked" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoIsLockedStream :: Servo -> RPCContext (KRPCStream (Bool))
getServoIsLockedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsLocked" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoIsLocked :: Servo -> Bool -> RPCContext (Bool)
setServoIsLocked thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_IsLocked" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getServoIsAxisInverted :: Servo -> RPCContext (Bool)
getServoIsAxisInverted thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsAxisInverted" [ makeArgument 0 (servoId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getServoIsAxisInvertedStream :: Servo -> RPCContext (KRPCStream (Bool))
getServoIsAxisInvertedStream thisArg = do
    let r = makeRequest "InfernalRobotics" "Servo_get_IsAxisInverted" [ makeArgument 0 (servoId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setServoIsAxisInverted :: Servo -> Bool -> RPCContext (Bool)
setServoIsAxisInverted thisArg valueArg = do
    let r = makeRequest "InfernalRobotics" "Servo_set_IsAxisInverted" [ makeArgument 0 (servoId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

