module KRPCHS.KerbalAlarmClock
( AlarmAction(..)
, AlarmType(..)
, Alarm
, alarmWithName
, alarmWithNameStream
, alarmWithNameStreamReq
, alarmRemove
, getAlarmAction
, getAlarmActionStream
, getAlarmActionStreamReq
, getAlarmID
, getAlarmIDStream
, getAlarmIDStreamReq
, getAlarmMargin
, getAlarmMarginStream
, getAlarmMarginStreamReq
, getAlarmName
, getAlarmNameStream
, getAlarmNameStreamReq
, getAlarmNotes
, getAlarmNotesStream
, getAlarmNotesStreamReq
, getAlarmRemaining
, getAlarmRemainingStream
, getAlarmRemainingStreamReq
, getAlarmRepeat
, getAlarmRepeatStream
, getAlarmRepeatStreamReq
, getAlarmRepeatPeriod
, getAlarmRepeatPeriodStream
, getAlarmRepeatPeriodStreamReq
, getAlarmTime
, getAlarmTimeStream
, getAlarmTimeStreamReq
, getAlarmType
, getAlarmTypeStream
, getAlarmTypeStreamReq
, getAlarmVessel
, getAlarmVesselStream
, getAlarmVesselStreamReq
, getAlarmXferOriginBody
, getAlarmXferOriginBodyStream
, getAlarmXferOriginBodyStreamReq
, getAlarmXferTargetBody
, getAlarmXferTargetBodyStream
, getAlarmXferTargetBodyStreamReq
, setAlarmAction
, setAlarmMargin
, setAlarmName
, setAlarmNotes
, setAlarmRepeat
, setAlarmRepeatPeriod
, setAlarmTime
, setAlarmVessel
, setAlarmXferOriginBody
, setAlarmXferTargetBody
, alarmsWithType
, alarmsWithTypeStream
, alarmsWithTypeStreamReq
, createAlarm
, createAlarmStream
, createAlarmStreamReq
, getAlarms
, getAlarmsStream
, getAlarmsStreamReq
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - Represents an alarm. Obtained by calling
 - <see cref="M:KerbalAlarmClock.Alarms" />,
 - <see cref="M:KerbalAlarmClock.AlarmWithName" /> or
 - <see cref="M:KerbalAlarmClock.AlarmsWithType" />.
 -}
newtype Alarm = Alarm { alarmId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Alarm where
    encodePb   = encodePb . alarmId
    decodePb b = Alarm <$> decodePb b

instance KRPCResponseExtractable Alarm


{-
 - The action performed by an alarm when it fires.
 -}
data AlarmAction
    = AlarmAction'DoNothing
    | AlarmAction'DoNothingDeleteWhenPassed
    | AlarmAction'KillWarp
    | AlarmAction'KillWarpOnly
    | AlarmAction'MessageOnly
    | AlarmAction'PauseGame
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable AlarmAction where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable AlarmAction

{-
 - The type of an alarm.
 -}
data AlarmType
    = AlarmType'Raw
    | AlarmType'Maneuver
    | AlarmType'Crew
    | AlarmType'Distance
    | AlarmType'EarthTime
    | AlarmType'LaunchRendevous
    | AlarmType'SOIChange
    | AlarmType'SOIChangeAuto
    | AlarmType'Transfer
    | AlarmType'TransferModelled
    | AlarmType'ManeuverAuto
    | AlarmType'Apoapsis
    | AlarmType'Periapsis
    | AlarmType'AscendingNode
    | AlarmType'DescendingNode
    | AlarmType'Closest
    | AlarmType'Contract
    | AlarmType'ContractAuto
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable AlarmType where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable AlarmType


{-
 - Get the alarm with the given <paramref name="name" />, ornullif no alarms have that name. If more than one alarm has the name,
 - only returns one of them.<param name="name">Name of the alarm to search for.
 -}
alarmWithName :: Data.Text.Text -> RPCContext (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithName nameArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmWithName" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res

alarmWithNameStreamReq :: Data.Text.Text -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithNameStreamReq nameArg =
    let req = makeRequest "KerbalAlarmClock" "AlarmWithName" [makeArgument 0 nameArg]
    in  makeStream req

alarmWithNameStream :: Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
alarmWithNameStream nameArg = requestStream $ alarmWithNameStreamReq nameArg 

{-
 - Removes the alarm.
 -}
alarmRemove :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext ()
alarmRemove thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The action that the alarm triggers.
 -}
getAlarmAction :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmAction thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmActionStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmActionStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmActionStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmAction))
getAlarmActionStream thisArg = requestStream $ getAlarmActionStreamReq thisArg 

{-
 - The unique identifier for the alarm.
 -}
getAlarmID :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmID thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmIDStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmIDStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmIDStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmIDStream thisArg = requestStream $ getAlarmIDStreamReq thisArg 

{-
 - The number of seconds before the event that the alarm will fire.
 -}
getAlarmMargin :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmMargin thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmMarginStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmMarginStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmMarginStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmMarginStream thisArg = requestStream $ getAlarmMarginStreamReq thisArg 

{-
 - The short name of the alarm.
 -}
getAlarmName :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmName thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmNameStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmNameStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmNameStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmNameStream thisArg = requestStream $ getAlarmNameStreamReq thisArg 

{-
 - The long description of the alarm.
 -}
getAlarmNotes :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmNotes thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmNotesStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmNotesStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmNotesStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmNotesStream thisArg = requestStream $ getAlarmNotesStreamReq thisArg 

{-
 - The number of seconds until the alarm will fire.
 -}
getAlarmRemaining :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmRemaining thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmRemainingStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmRemainingStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmRemainingStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmRemainingStream thisArg = requestStream $ getAlarmRemainingStreamReq thisArg 

{-
 - Whether the alarm will be repeated after it has fired.
 -}
getAlarmRepeat :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Bool)
getAlarmRepeat thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmRepeatStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Bool)
getAlarmRepeatStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmRepeatStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Bool))
getAlarmRepeatStream thisArg = requestStream $ getAlarmRepeatStreamReq thisArg 

{-
 - The time delay to automatically create an alarm after it has fired.
 -}
getAlarmRepeatPeriod :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmRepeatPeriod thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmRepeatPeriodStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmRepeatPeriodStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmRepeatPeriodStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmRepeatPeriodStream thisArg = requestStream $ getAlarmRepeatPeriodStreamReq thisArg 

{-
 - The time at which the alarm will fire.
 -}
getAlarmTime :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmTime thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmTimeStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmTimeStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmTimeStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmTimeStream thisArg = requestStream $ getAlarmTimeStreamReq thisArg 

{-
 - The type of the alarm.
 -}
getAlarmType :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmType thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmTypeStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmTypeStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmTypeStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmType))
getAlarmTypeStream thisArg = requestStream $ getAlarmTypeStreamReq thisArg 

{-
 - The vessel that the alarm is attached to.
 -}
getAlarmVessel :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getAlarmVessel thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmVesselStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getAlarmVesselStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmVesselStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAlarmVesselStream thisArg = requestStream $ getAlarmVesselStreamReq thisArg 

{-
 - The celestial body the vessel is departing from.
 -}
getAlarmXferOriginBody :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmXferOriginBodyStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBodyStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmXferOriginBodyStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferOriginBodyStream thisArg = requestStream $ getAlarmXferOriginBodyStreamReq thisArg 

{-
 - The celestial body the vessel is arriving at.
 -}
getAlarmXferTargetBody :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmXferTargetBodyStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBodyStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmXferTargetBodyStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferTargetBodyStream thisArg = requestStream $ getAlarmXferTargetBodyStreamReq thisArg 

{-
 - The action that the alarm triggers.
 -}
setAlarmAction :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.KerbalAlarmClock.AlarmAction -> RPCContext ()
setAlarmAction thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Action" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The number of seconds before the event that the alarm will fire.
 -}
setAlarmMargin :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext ()
setAlarmMargin thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Margin" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The short name of the alarm.
 -}
setAlarmName :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> RPCContext ()
setAlarmName thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The long description of the alarm.
 -}
setAlarmNotes :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> RPCContext ()
setAlarmNotes thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Notes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the alarm will be repeated after it has fired.
 -}
setAlarmRepeat :: KRPCHS.KerbalAlarmClock.Alarm -> Bool -> RPCContext ()
setAlarmRepeat thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Repeat" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time delay to automatically create an alarm after it has fired.
 -}
setAlarmRepeatPeriod :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext ()
setAlarmRepeatPeriod thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_RepeatPeriod" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time at which the alarm will fire.
 -}
setAlarmTime :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext ()
setAlarmTime thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Time" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The vessel that the alarm is attached to.
 -}
setAlarmVessel :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setAlarmVessel thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Vessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The celestial body the vessel is departing from.
 -}
setAlarmXferOriginBody :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setAlarmXferOriginBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferOriginBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The celestial body the vessel is arriving at.
 -}
setAlarmXferTargetBody :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setAlarmXferTargetBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferTargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Get a list of alarms of the specified <paramref name="type" />.<param name="type">Type of alarm to return.
 -}
alarmsWithType :: KRPCHS.KerbalAlarmClock.AlarmType -> RPCContext ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithType typeArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]
    res <- sendRequest r
    processResponse res

alarmsWithTypeStreamReq :: KRPCHS.KerbalAlarmClock.AlarmType -> KRPCStreamReq ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithTypeStreamReq typeArg =
    let req = makeRequest "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]
    in  makeStream req

alarmsWithTypeStream :: KRPCHS.KerbalAlarmClock.AlarmType -> RPCContext (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
alarmsWithTypeStream typeArg = requestStream $ alarmsWithTypeStreamReq typeArg 

{-
 - Create a new alarm and return it.<param name="type">Type of the new alarm.<param name="name">Name of the new alarm.<param name="ut">Time at which the new alarm should trigger.
 -}
createAlarm :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> RPCContext (KRPCHS.KerbalAlarmClock.Alarm)
createAlarm typeArg nameArg utArg = do
    let r = makeRequest "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]
    res <- sendRequest r
    processResponse res

createAlarmStreamReq :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.Alarm)
createAlarmStreamReq typeArg nameArg utArg =
    let req = makeRequest "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]
    in  makeStream req

createAlarmStream :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
createAlarmStream typeArg nameArg utArg = requestStream $ createAlarmStreamReq typeArg nameArg utArg 

{-
 - A list of all the alarms.
 -}
getAlarms :: RPCContext ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarms  = do
    let r = makeRequest "KerbalAlarmClock" "get_Alarms" []
    res <- sendRequest r
    processResponse res

getAlarmsStreamReq :: KRPCStreamReq ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarmsStreamReq  =
    let req = makeRequest "KerbalAlarmClock" "get_Alarms" []
    in  makeStream req

getAlarmsStream :: RPCContext (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
getAlarmsStream  = requestStream $ getAlarmsStreamReq  

