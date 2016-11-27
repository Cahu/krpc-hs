module KRPCHS.InfernalRobotics
( Servo
, ServoGroup
, servoGroupWithName
, servoGroupWithNameReq
, servoGroupWithNameStream
, servoGroupWithNameStreamReq
, servoGroupMoveCenter
, servoGroupMoveCenterReq
, servoGroupMoveLeft
, servoGroupMoveLeftReq
, servoGroupMoveNextPreset
, servoGroupMoveNextPresetReq
, servoGroupMovePrevPreset
, servoGroupMovePrevPresetReq
, servoGroupMoveRight
, servoGroupMoveRightReq
, servoGroupServoWithName
, servoGroupServoWithNameReq
, servoGroupServoWithNameStream
, servoGroupServoWithNameStreamReq
, servoGroupStop
, servoGroupStopReq
, getServoGroupExpanded
, getServoGroupExpandedReq
, getServoGroupExpandedStream
, getServoGroupExpandedStreamReq
, getServoGroupForwardKey
, getServoGroupForwardKeyReq
, getServoGroupForwardKeyStream
, getServoGroupForwardKeyStreamReq
, getServoGroupName
, getServoGroupNameReq
, getServoGroupNameStream
, getServoGroupNameStreamReq
, getServoGroupParts
, getServoGroupPartsReq
, getServoGroupPartsStream
, getServoGroupPartsStreamReq
, getServoGroupReverseKey
, getServoGroupReverseKeyReq
, getServoGroupReverseKeyStream
, getServoGroupReverseKeyStreamReq
, getServoGroupServos
, getServoGroupServosReq
, getServoGroupServosStream
, getServoGroupServosStreamReq
, getServoGroupSpeed
, getServoGroupSpeedReq
, getServoGroupSpeedStream
, getServoGroupSpeedStreamReq
, setServoGroupExpanded
, setServoGroupExpandedReq
, setServoGroupForwardKey
, setServoGroupForwardKeyReq
, setServoGroupName
, setServoGroupNameReq
, setServoGroupReverseKey
, setServoGroupReverseKeyReq
, setServoGroupSpeed
, setServoGroupSpeedReq
, servoGroups
, servoGroupsReq
, servoGroupsStream
, servoGroupsStreamReq
, servoWithName
, servoWithNameReq
, servoWithNameStream
, servoWithNameStreamReq
, servoMoveCenter
, servoMoveCenterReq
, servoMoveLeft
, servoMoveLeftReq
, servoMoveNextPreset
, servoMoveNextPresetReq
, servoMovePrevPreset
, servoMovePrevPresetReq
, servoMoveRight
, servoMoveRightReq
, servoMoveTo
, servoMoveToReq
, servoStop
, servoStopReq
, getServoAcceleration
, getServoAccelerationReq
, getServoAccelerationStream
, getServoAccelerationStreamReq
, getServoConfigSpeed
, getServoConfigSpeedReq
, getServoConfigSpeedStream
, getServoConfigSpeedStreamReq
, getServoCurrentSpeed
, getServoCurrentSpeedReq
, getServoCurrentSpeedStream
, getServoCurrentSpeedStreamReq
, getServoIsAxisInverted
, getServoIsAxisInvertedReq
, getServoIsAxisInvertedStream
, getServoIsAxisInvertedStreamReq
, getServoIsFreeMoving
, getServoIsFreeMovingReq
, getServoIsFreeMovingStream
, getServoIsFreeMovingStreamReq
, getServoIsLocked
, getServoIsLockedReq
, getServoIsLockedStream
, getServoIsLockedStreamReq
, getServoIsMoving
, getServoIsMovingReq
, getServoIsMovingStream
, getServoIsMovingStreamReq
, getServoMaxConfigPosition
, getServoMaxConfigPositionReq
, getServoMaxConfigPositionStream
, getServoMaxConfigPositionStreamReq
, getServoMaxPosition
, getServoMaxPositionReq
, getServoMaxPositionStream
, getServoMaxPositionStreamReq
, getServoMinConfigPosition
, getServoMinConfigPositionReq
, getServoMinConfigPositionStream
, getServoMinConfigPositionStreamReq
, getServoMinPosition
, getServoMinPositionReq
, getServoMinPositionStream
, getServoMinPositionStreamReq
, getServoName
, getServoNameReq
, getServoNameStream
, getServoNameStreamReq
, getServoPart
, getServoPartReq
, getServoPartStream
, getServoPartStreamReq
, getServoPosition
, getServoPositionReq
, getServoPositionStream
, getServoPositionStreamReq
, getServoSpeed
, getServoSpeedReq
, getServoSpeedStream
, getServoSpeedStreamReq
, setServoAcceleration
, setServoAccelerationReq
, setServoCurrentSpeed
, setServoCurrentSpeedReq
, setServoHighlight
, setServoHighlightReq
, setServoIsAxisInverted
, setServoIsAxisInvertedReq
, setServoIsLocked
, setServoIsLockedReq
, setServoMaxPosition
, setServoMaxPositionReq
, setServoMinPosition
, setServoMinPositionReq
, setServoName
, setServoNameReq
, setServoSpeed
, setServoSpeedReq
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


{-|
Represents a servo. Obtained using
<see cref="M:InfernalRobotics.ServoGroup.Servos" />,
<see cref="M:InfernalRobotics.ServoGroup.ServoWithName" />
or <see cref="M:InfernalRobotics.ServoWithName" />.
 -}
newtype Servo = Servo { servoId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Servo where
    encodePb = encodePb . servoId

instance PbDecodable Servo where
    decodePb b = Servo <$> decodePb b

instance KRPCResponseExtractable Servo

{-|
A group of servos, obtained by calling <see cref="M:InfernalRobotics.ServoGroups" />
or <see cref="M:InfernalRobotics.ServoGroupWithName" />. Represents the "Servo Groups"
in the InfernalRobotics UI.
 -}
newtype ServoGroup = ServoGroup { servoGroupId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ServoGroup where
    encodePb = encodePb . servoGroupId

instance PbDecodable ServoGroup where
    decodePb b = ServoGroup <$> decodePb b

instance KRPCResponseExtractable ServoGroup



{-|
Returns the servo group in the given <paramref name="vessel" /> with the given <paramref name="name" />,
ornullif none exists. If multiple servo groups have the same name, only one of them is returned.<param name="vessel">Vessel to check.<param name="name">Name of servo group to find.
 -}
servoGroupWithNameReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCCallReq (KRPCHS.InfernalRobotics.ServoGroup)
servoGroupWithNameReq vesselArg nameArg = makeCallReq "InfernalRobotics" "ServoGroupWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]

servoGroupWithName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCHS.InfernalRobotics.ServoGroup)
servoGroupWithName vesselArg nameArg = simpleRequest $ servoGroupWithNameReq vesselArg nameArg

servoGroupWithNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCStreamReq (KRPCHS.InfernalRobotics.ServoGroup)
servoGroupWithNameStreamReq vesselArg nameArg = makeStreamReq $ servoGroupWithNameReq vesselArg nameArg

servoGroupWithNameStream :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.InfernalRobotics.ServoGroup))
servoGroupWithNameStream vesselArg nameArg = requestAddStream $ servoGroupWithNameStreamReq vesselArg nameArg 

{-|
Moves all of the servos in the group to the center.
 -}
servoGroupMoveCenterReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ()
servoGroupMoveCenterReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_MoveCenter" [makeArgument 0 thisArg]

servoGroupMoveCenter :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ()
servoGroupMoveCenter thisArg = simpleRequest $ servoGroupMoveCenterReq thisArg 

{-|
Moves all of the servos in the group to the left.
 -}
servoGroupMoveLeftReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ()
servoGroupMoveLeftReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_MoveLeft" [makeArgument 0 thisArg]

servoGroupMoveLeft :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ()
servoGroupMoveLeft thisArg = simpleRequest $ servoGroupMoveLeftReq thisArg 

{-|
Moves all of the servos in the group to the next preset.
 -}
servoGroupMoveNextPresetReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ()
servoGroupMoveNextPresetReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_MoveNextPreset" [makeArgument 0 thisArg]

servoGroupMoveNextPreset :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ()
servoGroupMoveNextPreset thisArg = simpleRequest $ servoGroupMoveNextPresetReq thisArg 

{-|
Moves all of the servos in the group to the previous preset.
 -}
servoGroupMovePrevPresetReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ()
servoGroupMovePrevPresetReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_MovePrevPreset" [makeArgument 0 thisArg]

servoGroupMovePrevPreset :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ()
servoGroupMovePrevPreset thisArg = simpleRequest $ servoGroupMovePrevPresetReq thisArg 

{-|
Moves all of the servos in the group to the right.
 -}
servoGroupMoveRightReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ()
servoGroupMoveRightReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_MoveRight" [makeArgument 0 thisArg]

servoGroupMoveRight :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ()
servoGroupMoveRight thisArg = simpleRequest $ servoGroupMoveRightReq thisArg 

{-|
Returns the servo with the given <paramref name="name" /> from this group,
ornullif none exists.<param name="name">Name of servo to find.
 -}
servoGroupServoWithNameReq :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> KRPCCallReq (KRPCHS.InfernalRobotics.Servo)
servoGroupServoWithNameReq thisArg nameArg = makeCallReq "InfernalRobotics" "ServoGroup_ServoWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]

servoGroupServoWithName :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (KRPCHS.InfernalRobotics.Servo)
servoGroupServoWithName thisArg nameArg = simpleRequest $ servoGroupServoWithNameReq thisArg nameArg

servoGroupServoWithNameStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> KRPCStreamReq (KRPCHS.InfernalRobotics.Servo)
servoGroupServoWithNameStreamReq thisArg nameArg = makeStreamReq $ servoGroupServoWithNameReq thisArg nameArg

servoGroupServoWithNameStream :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.InfernalRobotics.Servo))
servoGroupServoWithNameStream thisArg nameArg = requestAddStream $ servoGroupServoWithNameStreamReq thisArg nameArg 

{-|
Stops the servos in the group.
 -}
servoGroupStopReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ()
servoGroupStopReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_Stop" [makeArgument 0 thisArg]

servoGroupStop :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ()
servoGroupStop thisArg = simpleRequest $ servoGroupStopReq thisArg 

{-|
Whether the group is expanded in the InfernalRobotics UI.
 -}
getServoGroupExpandedReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq (Bool)
getServoGroupExpandedReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_Expanded" [makeArgument 0 thisArg]

getServoGroupExpanded :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Bool)
getServoGroupExpanded thisArg = simpleRequest $ getServoGroupExpandedReq thisArg

getServoGroupExpandedStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Bool)
getServoGroupExpandedStreamReq thisArg = makeStreamReq $ getServoGroupExpandedReq thisArg

getServoGroupExpandedStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Bool))
getServoGroupExpandedStream thisArg = requestAddStream $ getServoGroupExpandedStreamReq thisArg 

{-|
The key assigned to be the "forward" key for the group.
 -}
getServoGroupForwardKeyReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq (Data.Text.Text)
getServoGroupForwardKeyReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_ForwardKey" [makeArgument 0 thisArg]

getServoGroupForwardKey :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Data.Text.Text)
getServoGroupForwardKey thisArg = simpleRequest $ getServoGroupForwardKeyReq thisArg

getServoGroupForwardKeyStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Data.Text.Text)
getServoGroupForwardKeyStreamReq thisArg = makeStreamReq $ getServoGroupForwardKeyReq thisArg

getServoGroupForwardKeyStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Data.Text.Text))
getServoGroupForwardKeyStream thisArg = requestAddStream $ getServoGroupForwardKeyStreamReq thisArg 

{-|
The name of the group.
 -}
getServoGroupNameReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq (Data.Text.Text)
getServoGroupNameReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_Name" [makeArgument 0 thisArg]

getServoGroupName :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Data.Text.Text)
getServoGroupName thisArg = simpleRequest $ getServoGroupNameReq thisArg

getServoGroupNameStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Data.Text.Text)
getServoGroupNameStreamReq thisArg = makeStreamReq $ getServoGroupNameReq thisArg

getServoGroupNameStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Data.Text.Text))
getServoGroupNameStream thisArg = requestAddStream $ getServoGroupNameStreamReq thisArg 

{-|
The parts containing the servos in the group.
 -}
getServoGroupPartsReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
getServoGroupPartsReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_Parts" [makeArgument 0 thisArg]

getServoGroupParts :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ([KRPCHS.SpaceCenter.Part])
getServoGroupParts thisArg = simpleRequest $ getServoGroupPartsReq thisArg

getServoGroupPartsStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getServoGroupPartsStreamReq thisArg = makeStreamReq $ getServoGroupPartsReq thisArg

getServoGroupPartsStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getServoGroupPartsStream thisArg = requestAddStream $ getServoGroupPartsStreamReq thisArg 

{-|
The key assigned to be the "reverse" key for the group.
 -}
getServoGroupReverseKeyReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq (Data.Text.Text)
getServoGroupReverseKeyReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_ReverseKey" [makeArgument 0 thisArg]

getServoGroupReverseKey :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Data.Text.Text)
getServoGroupReverseKey thisArg = simpleRequest $ getServoGroupReverseKeyReq thisArg

getServoGroupReverseKeyStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Data.Text.Text)
getServoGroupReverseKeyStreamReq thisArg = makeStreamReq $ getServoGroupReverseKeyReq thisArg

getServoGroupReverseKeyStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Data.Text.Text))
getServoGroupReverseKeyStream thisArg = requestAddStream $ getServoGroupReverseKeyStreamReq thisArg 

{-|
The servos that are in the group.
 -}
getServoGroupServosReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq ([KRPCHS.InfernalRobotics.Servo])
getServoGroupServosReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_Servos" [makeArgument 0 thisArg]

getServoGroupServos :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext ([KRPCHS.InfernalRobotics.Servo])
getServoGroupServos thisArg = simpleRequest $ getServoGroupServosReq thisArg

getServoGroupServosStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq ([KRPCHS.InfernalRobotics.Servo])
getServoGroupServosStreamReq thisArg = makeStreamReq $ getServoGroupServosReq thisArg

getServoGroupServosStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream ([KRPCHS.InfernalRobotics.Servo]))
getServoGroupServosStream thisArg = requestAddStream $ getServoGroupServosStreamReq thisArg 

{-|
The speed multiplier for the group.
 -}
getServoGroupSpeedReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCCallReq (Float)
getServoGroupSpeedReq thisArg = makeCallReq "InfernalRobotics" "ServoGroup_get_Speed" [makeArgument 0 thisArg]

getServoGroupSpeed :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (Float)
getServoGroupSpeed thisArg = simpleRequest $ getServoGroupSpeedReq thisArg

getServoGroupSpeedStreamReq :: KRPCHS.InfernalRobotics.ServoGroup -> KRPCStreamReq (Float)
getServoGroupSpeedStreamReq thisArg = makeStreamReq $ getServoGroupSpeedReq thisArg

getServoGroupSpeedStream :: KRPCHS.InfernalRobotics.ServoGroup -> RPCContext (KRPCStream (Float))
getServoGroupSpeedStream thisArg = requestAddStream $ getServoGroupSpeedStreamReq thisArg 

{-|
Whether the group is expanded in the InfernalRobotics UI.
 -}
setServoGroupExpandedReq :: KRPCHS.InfernalRobotics.ServoGroup -> Bool -> KRPCCallReq ()
setServoGroupExpandedReq thisArg valueArg = makeCallReq "InfernalRobotics" "ServoGroup_set_Expanded" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoGroupExpanded :: KRPCHS.InfernalRobotics.ServoGroup -> Bool -> RPCContext ()
setServoGroupExpanded thisArg valueArg = simpleRequest $ setServoGroupExpandedReq thisArg valueArg 

{-|
The key assigned to be the "forward" key for the group.
 -}
setServoGroupForwardKeyReq :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> KRPCCallReq ()
setServoGroupForwardKeyReq thisArg valueArg = makeCallReq "InfernalRobotics" "ServoGroup_set_ForwardKey" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoGroupForwardKey :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext ()
setServoGroupForwardKey thisArg valueArg = simpleRequest $ setServoGroupForwardKeyReq thisArg valueArg 

{-|
The name of the group.
 -}
setServoGroupNameReq :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> KRPCCallReq ()
setServoGroupNameReq thisArg valueArg = makeCallReq "InfernalRobotics" "ServoGroup_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoGroupName :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext ()
setServoGroupName thisArg valueArg = simpleRequest $ setServoGroupNameReq thisArg valueArg 

{-|
The key assigned to be the "reverse" key for the group.
 -}
setServoGroupReverseKeyReq :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> KRPCCallReq ()
setServoGroupReverseKeyReq thisArg valueArg = makeCallReq "InfernalRobotics" "ServoGroup_set_ReverseKey" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoGroupReverseKey :: KRPCHS.InfernalRobotics.ServoGroup -> Data.Text.Text -> RPCContext ()
setServoGroupReverseKey thisArg valueArg = simpleRequest $ setServoGroupReverseKeyReq thisArg valueArg 

{-|
The speed multiplier for the group.
 -}
setServoGroupSpeedReq :: KRPCHS.InfernalRobotics.ServoGroup -> Float -> KRPCCallReq ()
setServoGroupSpeedReq thisArg valueArg = makeCallReq "InfernalRobotics" "ServoGroup_set_Speed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoGroupSpeed :: KRPCHS.InfernalRobotics.ServoGroup -> Float -> RPCContext ()
setServoGroupSpeed thisArg valueArg = simpleRequest $ setServoGroupSpeedReq thisArg valueArg 

{-|
A list of all the servo groups in the given <paramref name="vessel" />.
 -}
servoGroupsReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ([KRPCHS.InfernalRobotics.ServoGroup])
servoGroupsReq vesselArg = makeCallReq "InfernalRobotics" "ServoGroups" [makeArgument 0 vesselArg]

servoGroups :: KRPCHS.SpaceCenter.Vessel -> RPCContext ([KRPCHS.InfernalRobotics.ServoGroup])
servoGroups vesselArg = simpleRequest $ servoGroupsReq vesselArg

servoGroupsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ([KRPCHS.InfernalRobotics.ServoGroup])
servoGroupsStreamReq vesselArg = makeStreamReq $ servoGroupsReq vesselArg

servoGroupsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ([KRPCHS.InfernalRobotics.ServoGroup]))
servoGroupsStream vesselArg = requestAddStream $ servoGroupsStreamReq vesselArg 

{-|
Returns the servo in the given <paramref name="vessel" /> with the given <paramref name="name" /> ornullif none exists. If multiple servos have the same name, only one of them is returned.<param name="vessel">Vessel to check.<param name="name">Name of the servo to find.
 -}
servoWithNameReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCCallReq (KRPCHS.InfernalRobotics.Servo)
servoWithNameReq vesselArg nameArg = makeCallReq "InfernalRobotics" "ServoWithName" [makeArgument 0 vesselArg, makeArgument 1 nameArg]

servoWithName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCHS.InfernalRobotics.Servo)
servoWithName vesselArg nameArg = simpleRequest $ servoWithNameReq vesselArg nameArg

servoWithNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCStreamReq (KRPCHS.InfernalRobotics.Servo)
servoWithNameStreamReq vesselArg nameArg = makeStreamReq $ servoWithNameReq vesselArg nameArg

servoWithNameStream :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.InfernalRobotics.Servo))
servoWithNameStream vesselArg nameArg = requestAddStream $ servoWithNameStreamReq vesselArg nameArg 

{-|
Moves the servo to the center.
 -}
servoMoveCenterReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq ()
servoMoveCenterReq thisArg = makeCallReq "InfernalRobotics" "Servo_MoveCenter" [makeArgument 0 thisArg]

servoMoveCenter :: KRPCHS.InfernalRobotics.Servo -> RPCContext ()
servoMoveCenter thisArg = simpleRequest $ servoMoveCenterReq thisArg 

{-|
Moves the servo to the left.
 -}
servoMoveLeftReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq ()
servoMoveLeftReq thisArg = makeCallReq "InfernalRobotics" "Servo_MoveLeft" [makeArgument 0 thisArg]

servoMoveLeft :: KRPCHS.InfernalRobotics.Servo -> RPCContext ()
servoMoveLeft thisArg = simpleRequest $ servoMoveLeftReq thisArg 

{-|
Moves the servo to the next preset.
 -}
servoMoveNextPresetReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq ()
servoMoveNextPresetReq thisArg = makeCallReq "InfernalRobotics" "Servo_MoveNextPreset" [makeArgument 0 thisArg]

servoMoveNextPreset :: KRPCHS.InfernalRobotics.Servo -> RPCContext ()
servoMoveNextPreset thisArg = simpleRequest $ servoMoveNextPresetReq thisArg 

{-|
Moves the servo to the previous preset.
 -}
servoMovePrevPresetReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq ()
servoMovePrevPresetReq thisArg = makeCallReq "InfernalRobotics" "Servo_MovePrevPreset" [makeArgument 0 thisArg]

servoMovePrevPreset :: KRPCHS.InfernalRobotics.Servo -> RPCContext ()
servoMovePrevPreset thisArg = simpleRequest $ servoMovePrevPresetReq thisArg 

{-|
Moves the servo to the right.
 -}
servoMoveRightReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq ()
servoMoveRightReq thisArg = makeCallReq "InfernalRobotics" "Servo_MoveRight" [makeArgument 0 thisArg]

servoMoveRight :: KRPCHS.InfernalRobotics.Servo -> RPCContext ()
servoMoveRight thisArg = simpleRequest $ servoMoveRightReq thisArg 

{-|
Moves the servo to <paramref name="position" /> and sets the
speed multiplier to <paramref name="speed" />.<param name="position">The position to move the servo to.<param name="speed">Speed multiplier for the movement.
 -}
servoMoveToReq :: KRPCHS.InfernalRobotics.Servo -> Float -> Float -> KRPCCallReq ()
servoMoveToReq thisArg positionArg speedArg = makeCallReq "InfernalRobotics" "Servo_MoveTo" [makeArgument 0 thisArg, makeArgument 1 positionArg, makeArgument 2 speedArg]

servoMoveTo :: KRPCHS.InfernalRobotics.Servo -> Float -> Float -> RPCContext ()
servoMoveTo thisArg positionArg speedArg = simpleRequest $ servoMoveToReq thisArg positionArg speedArg 

{-|
Stops the servo.
 -}
servoStopReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq ()
servoStopReq thisArg = makeCallReq "InfernalRobotics" "Servo_Stop" [makeArgument 0 thisArg]

servoStop :: KRPCHS.InfernalRobotics.Servo -> RPCContext ()
servoStop thisArg = simpleRequest $ servoStopReq thisArg 

{-|
The current speed multiplier set in the UI.
 -}
getServoAccelerationReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoAccelerationReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_Acceleration" [makeArgument 0 thisArg]

getServoAcceleration :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoAcceleration thisArg = simpleRequest $ getServoAccelerationReq thisArg

getServoAccelerationStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoAccelerationStreamReq thisArg = makeStreamReq $ getServoAccelerationReq thisArg

getServoAccelerationStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoAccelerationStream thisArg = requestAddStream $ getServoAccelerationStreamReq thisArg 

{-|
The speed multiplier of the servo, specified by the part configuration.
 -}
getServoConfigSpeedReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoConfigSpeedReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_ConfigSpeed" [makeArgument 0 thisArg]

getServoConfigSpeed :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoConfigSpeed thisArg = simpleRequest $ getServoConfigSpeedReq thisArg

getServoConfigSpeedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoConfigSpeedStreamReq thisArg = makeStreamReq $ getServoConfigSpeedReq thisArg

getServoConfigSpeedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoConfigSpeedStream thisArg = requestAddStream $ getServoConfigSpeedStreamReq thisArg 

{-|
The current speed at which the servo is moving.
 -}
getServoCurrentSpeedReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoCurrentSpeedReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_CurrentSpeed" [makeArgument 0 thisArg]

getServoCurrentSpeed :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoCurrentSpeed thisArg = simpleRequest $ getServoCurrentSpeedReq thisArg

getServoCurrentSpeedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoCurrentSpeedStreamReq thisArg = makeStreamReq $ getServoCurrentSpeedReq thisArg

getServoCurrentSpeedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoCurrentSpeedStream thisArg = requestAddStream $ getServoCurrentSpeedStreamReq thisArg 

{-|
Whether the servos axis is inverted.
 -}
getServoIsAxisInvertedReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Bool)
getServoIsAxisInvertedReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_IsAxisInverted" [makeArgument 0 thisArg]

getServoIsAxisInverted :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsAxisInverted thisArg = simpleRequest $ getServoIsAxisInvertedReq thisArg

getServoIsAxisInvertedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsAxisInvertedStreamReq thisArg = makeStreamReq $ getServoIsAxisInvertedReq thisArg

getServoIsAxisInvertedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsAxisInvertedStream thisArg = requestAddStream $ getServoIsAxisInvertedStreamReq thisArg 

{-|
Whether the servo is freely moving.
 -}
getServoIsFreeMovingReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Bool)
getServoIsFreeMovingReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_IsFreeMoving" [makeArgument 0 thisArg]

getServoIsFreeMoving :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsFreeMoving thisArg = simpleRequest $ getServoIsFreeMovingReq thisArg

getServoIsFreeMovingStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsFreeMovingStreamReq thisArg = makeStreamReq $ getServoIsFreeMovingReq thisArg

getServoIsFreeMovingStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsFreeMovingStream thisArg = requestAddStream $ getServoIsFreeMovingStreamReq thisArg 

{-|
Whether the servo is locked.
 -}
getServoIsLockedReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Bool)
getServoIsLockedReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_IsLocked" [makeArgument 0 thisArg]

getServoIsLocked :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsLocked thisArg = simpleRequest $ getServoIsLockedReq thisArg

getServoIsLockedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsLockedStreamReq thisArg = makeStreamReq $ getServoIsLockedReq thisArg

getServoIsLockedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsLockedStream thisArg = requestAddStream $ getServoIsLockedStreamReq thisArg 

{-|
Whether the servo is moving.
 -}
getServoIsMovingReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Bool)
getServoIsMovingReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_IsMoving" [makeArgument 0 thisArg]

getServoIsMoving :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Bool)
getServoIsMoving thisArg = simpleRequest $ getServoIsMovingReq thisArg

getServoIsMovingStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Bool)
getServoIsMovingStreamReq thisArg = makeStreamReq $ getServoIsMovingReq thisArg

getServoIsMovingStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Bool))
getServoIsMovingStream thisArg = requestAddStream $ getServoIsMovingStreamReq thisArg 

{-|
The maximum position of the servo, specified by the part configuration.
 -}
getServoMaxConfigPositionReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoMaxConfigPositionReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_MaxConfigPosition" [makeArgument 0 thisArg]

getServoMaxConfigPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMaxConfigPosition thisArg = simpleRequest $ getServoMaxConfigPositionReq thisArg

getServoMaxConfigPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMaxConfigPositionStreamReq thisArg = makeStreamReq $ getServoMaxConfigPositionReq thisArg

getServoMaxConfigPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMaxConfigPositionStream thisArg = requestAddStream $ getServoMaxConfigPositionStreamReq thisArg 

{-|
The maximum position of the servo, specified by the in-game tweak menu.
 -}
getServoMaxPositionReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoMaxPositionReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_MaxPosition" [makeArgument 0 thisArg]

getServoMaxPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMaxPosition thisArg = simpleRequest $ getServoMaxPositionReq thisArg

getServoMaxPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMaxPositionStreamReq thisArg = makeStreamReq $ getServoMaxPositionReq thisArg

getServoMaxPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMaxPositionStream thisArg = requestAddStream $ getServoMaxPositionStreamReq thisArg 

{-|
The minimum position of the servo, specified by the part configuration.
 -}
getServoMinConfigPositionReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoMinConfigPositionReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_MinConfigPosition" [makeArgument 0 thisArg]

getServoMinConfigPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMinConfigPosition thisArg = simpleRequest $ getServoMinConfigPositionReq thisArg

getServoMinConfigPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMinConfigPositionStreamReq thisArg = makeStreamReq $ getServoMinConfigPositionReq thisArg

getServoMinConfigPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMinConfigPositionStream thisArg = requestAddStream $ getServoMinConfigPositionStreamReq thisArg 

{-|
The minimum position of the servo, specified by the in-game tweak menu.
 -}
getServoMinPositionReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoMinPositionReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_MinPosition" [makeArgument 0 thisArg]

getServoMinPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoMinPosition thisArg = simpleRequest $ getServoMinPositionReq thisArg

getServoMinPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoMinPositionStreamReq thisArg = makeStreamReq $ getServoMinPositionReq thisArg

getServoMinPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoMinPositionStream thisArg = requestAddStream $ getServoMinPositionStreamReq thisArg 

{-|
The name of the servo.
 -}
getServoNameReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Data.Text.Text)
getServoNameReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_Name" [makeArgument 0 thisArg]

getServoName :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Data.Text.Text)
getServoName thisArg = simpleRequest $ getServoNameReq thisArg

getServoNameStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Data.Text.Text)
getServoNameStreamReq thisArg = makeStreamReq $ getServoNameReq thisArg

getServoNameStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Data.Text.Text))
getServoNameStream thisArg = requestAddStream $ getServoNameStreamReq thisArg 

{-|
The part containing the servo.
 -}
getServoPartReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getServoPartReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_Part" [makeArgument 0 thisArg]

getServoPart :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCHS.SpaceCenter.Part)
getServoPart thisArg = simpleRequest $ getServoPartReq thisArg

getServoPartStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getServoPartStreamReq thisArg = makeStreamReq $ getServoPartReq thisArg

getServoPartStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getServoPartStream thisArg = requestAddStream $ getServoPartStreamReq thisArg 

{-|
The position of the servo.
 -}
getServoPositionReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoPositionReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_Position" [makeArgument 0 thisArg]

getServoPosition :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoPosition thisArg = simpleRequest $ getServoPositionReq thisArg

getServoPositionStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoPositionStreamReq thisArg = makeStreamReq $ getServoPositionReq thisArg

getServoPositionStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoPositionStream thisArg = requestAddStream $ getServoPositionStreamReq thisArg 

{-|
The speed multiplier of the servo, specified by the in-game tweak menu.
 -}
getServoSpeedReq :: KRPCHS.InfernalRobotics.Servo -> KRPCCallReq (Float)
getServoSpeedReq thisArg = makeCallReq "InfernalRobotics" "Servo_get_Speed" [makeArgument 0 thisArg]

getServoSpeed :: KRPCHS.InfernalRobotics.Servo -> RPCContext (Float)
getServoSpeed thisArg = simpleRequest $ getServoSpeedReq thisArg

getServoSpeedStreamReq :: KRPCHS.InfernalRobotics.Servo -> KRPCStreamReq (Float)
getServoSpeedStreamReq thisArg = makeStreamReq $ getServoSpeedReq thisArg

getServoSpeedStream :: KRPCHS.InfernalRobotics.Servo -> RPCContext (KRPCStream (Float))
getServoSpeedStream thisArg = requestAddStream $ getServoSpeedStreamReq thisArg 

{-|
The current speed multiplier set in the UI.
 -}
setServoAccelerationReq :: KRPCHS.InfernalRobotics.Servo -> Float -> KRPCCallReq ()
setServoAccelerationReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_Acceleration" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoAcceleration :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext ()
setServoAcceleration thisArg valueArg = simpleRequest $ setServoAccelerationReq thisArg valueArg 

{-|
The current speed at which the servo is moving.
 -}
setServoCurrentSpeedReq :: KRPCHS.InfernalRobotics.Servo -> Float -> KRPCCallReq ()
setServoCurrentSpeedReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_CurrentSpeed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoCurrentSpeed :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext ()
setServoCurrentSpeed thisArg valueArg = simpleRequest $ setServoCurrentSpeedReq thisArg valueArg 

{-|
Whether the servo should be highlighted in-game.
 -}
setServoHighlightReq :: KRPCHS.InfernalRobotics.Servo -> Bool -> KRPCCallReq ()
setServoHighlightReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_Highlight" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoHighlight :: KRPCHS.InfernalRobotics.Servo -> Bool -> RPCContext ()
setServoHighlight thisArg valueArg = simpleRequest $ setServoHighlightReq thisArg valueArg 

{-|
Whether the servos axis is inverted.
 -}
setServoIsAxisInvertedReq :: KRPCHS.InfernalRobotics.Servo -> Bool -> KRPCCallReq ()
setServoIsAxisInvertedReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_IsAxisInverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoIsAxisInverted :: KRPCHS.InfernalRobotics.Servo -> Bool -> RPCContext ()
setServoIsAxisInverted thisArg valueArg = simpleRequest $ setServoIsAxisInvertedReq thisArg valueArg 

{-|
Whether the servo is locked.
 -}
setServoIsLockedReq :: KRPCHS.InfernalRobotics.Servo -> Bool -> KRPCCallReq ()
setServoIsLockedReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_IsLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoIsLocked :: KRPCHS.InfernalRobotics.Servo -> Bool -> RPCContext ()
setServoIsLocked thisArg valueArg = simpleRequest $ setServoIsLockedReq thisArg valueArg 

{-|
The maximum position of the servo, specified by the in-game tweak menu.
 -}
setServoMaxPositionReq :: KRPCHS.InfernalRobotics.Servo -> Float -> KRPCCallReq ()
setServoMaxPositionReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_MaxPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoMaxPosition :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext ()
setServoMaxPosition thisArg valueArg = simpleRequest $ setServoMaxPositionReq thisArg valueArg 

{-|
The minimum position of the servo, specified by the in-game tweak menu.
 -}
setServoMinPositionReq :: KRPCHS.InfernalRobotics.Servo -> Float -> KRPCCallReq ()
setServoMinPositionReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_MinPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoMinPosition :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext ()
setServoMinPosition thisArg valueArg = simpleRequest $ setServoMinPositionReq thisArg valueArg 

{-|
The name of the servo.
 -}
setServoNameReq :: KRPCHS.InfernalRobotics.Servo -> Data.Text.Text -> KRPCCallReq ()
setServoNameReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoName :: KRPCHS.InfernalRobotics.Servo -> Data.Text.Text -> RPCContext ()
setServoName thisArg valueArg = simpleRequest $ setServoNameReq thisArg valueArg 

{-|
The speed multiplier of the servo, specified by the in-game tweak menu.
 -}
setServoSpeedReq :: KRPCHS.InfernalRobotics.Servo -> Float -> KRPCCallReq ()
setServoSpeedReq thisArg valueArg = makeCallReq "InfernalRobotics" "Servo_set_Speed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setServoSpeed :: KRPCHS.InfernalRobotics.Servo -> Float -> RPCContext ()
setServoSpeed thisArg valueArg = simpleRequest $ setServoSpeedReq thisArg valueArg 

