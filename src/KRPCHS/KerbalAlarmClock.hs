module KRPCHS.KerbalAlarmClock
( AlarmAction(..)
, AlarmType(..)
, Alarm
, alarmWithName
, alarmWithNameReq
, alarmWithNameStream
, alarmWithNameStreamReq
, alarmRemove
, alarmRemoveReq
, getAlarmAction
, getAlarmActionReq
, getAlarmActionStream
, getAlarmActionStreamReq
, getAlarmID
, getAlarmIDReq
, getAlarmIDStream
, getAlarmIDStreamReq
, getAlarmMargin
, getAlarmMarginReq
, getAlarmMarginStream
, getAlarmMarginStreamReq
, getAlarmName
, getAlarmNameReq
, getAlarmNameStream
, getAlarmNameStreamReq
, getAlarmNotes
, getAlarmNotesReq
, getAlarmNotesStream
, getAlarmNotesStreamReq
, getAlarmRemaining
, getAlarmRemainingReq
, getAlarmRemainingStream
, getAlarmRemainingStreamReq
, getAlarmRepeat
, getAlarmRepeatReq
, getAlarmRepeatStream
, getAlarmRepeatStreamReq
, getAlarmRepeatPeriod
, getAlarmRepeatPeriodReq
, getAlarmRepeatPeriodStream
, getAlarmRepeatPeriodStreamReq
, getAlarmTime
, getAlarmTimeReq
, getAlarmTimeStream
, getAlarmTimeStreamReq
, getAlarmType
, getAlarmTypeReq
, getAlarmTypeStream
, getAlarmTypeStreamReq
, getAlarmVessel
, getAlarmVesselReq
, getAlarmVesselStream
, getAlarmVesselStreamReq
, getAlarmXferOriginBody
, getAlarmXferOriginBodyReq
, getAlarmXferOriginBodyStream
, getAlarmXferOriginBodyStreamReq
, getAlarmXferTargetBody
, getAlarmXferTargetBodyReq
, getAlarmXferTargetBodyStream
, getAlarmXferTargetBodyStreamReq
, setAlarmAction
, setAlarmActionReq
, setAlarmMargin
, setAlarmMarginReq
, setAlarmName
, setAlarmNameReq
, setAlarmNotes
, setAlarmNotesReq
, setAlarmRepeat
, setAlarmRepeatReq
, setAlarmRepeatPeriod
, setAlarmRepeatPeriodReq
, setAlarmTime
, setAlarmTimeReq
, setAlarmVessel
, setAlarmVesselReq
, setAlarmXferOriginBody
, setAlarmXferOriginBodyReq
, setAlarmXferTargetBody
, setAlarmXferTargetBodyReq
, alarmsWithType
, alarmsWithTypeReq
, alarmsWithTypeStream
, alarmsWithTypeStreamReq
, createAlarm
, createAlarmReq
, createAlarmStream
, createAlarmStreamReq
, getAlarms
, getAlarmsReq
, getAlarmsStream
, getAlarmsStreamReq
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


{-|
Represents an alarm. Obtained by calling
<see cref="M:KerbalAlarmClock.Alarms" />,
<see cref="M:KerbalAlarmClock.AlarmWithName" /> or
<see cref="M:KerbalAlarmClock.AlarmsWithType" />.
 -}
newtype Alarm = Alarm { alarmId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Alarm where
    encodePb = encodePb . alarmId

instance PbDecodable Alarm where
    decodePb b = Alarm <$> decodePb b

instance KRPCResponseExtractable Alarm


{-|
The action performed by an alarm when it fires.
 -}
data AlarmAction
    = AlarmAction'DoNothing
    | AlarmAction'DoNothingDeleteWhenPassed
    | AlarmAction'KillWarp
    | AlarmAction'KillWarpOnly
    | AlarmAction'MessageOnly
    | AlarmAction'PauseGame
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable AlarmAction where
    encodePb = encodePb . fromEnum

instance PbDecodable AlarmAction where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable AlarmAction

{-|
The type of an alarm.
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

instance PbEncodable AlarmType where
    encodePb = encodePb . fromEnum

instance PbDecodable AlarmType where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable AlarmType


{-|
Get the alarm with the given <paramref name="name" />, ornullif no alarms have that name. If more than one alarm has the name,
only returns one of them.<param name="name">Name of the alarm to search for.
 -}
alarmWithNameReq :: Data.Text.Text -> KRPCCallReq (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithNameReq nameArg = makeCallReq "KerbalAlarmClock" "AlarmWithName" [makeArgument 0 nameArg]

alarmWithName :: Data.Text.Text -> RPCContext (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithName nameArg = simpleRequest $ alarmWithNameReq nameArg

alarmWithNameStreamReq :: Data.Text.Text -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithNameStreamReq nameArg = makeStreamReq $ alarmWithNameReq nameArg

alarmWithNameStream :: Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
alarmWithNameStream nameArg = requestAddStream $ alarmWithNameStreamReq nameArg 

{-|
Removes the alarm.
 -}
alarmRemoveReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq ()
alarmRemoveReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_Remove" [makeArgument 0 thisArg]

alarmRemove :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext ()
alarmRemove thisArg = simpleRequest $ alarmRemoveReq thisArg 

{-|
The action that the alarm triggers.
 -}
getAlarmActionReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmActionReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]

getAlarmAction :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmAction thisArg = simpleRequest $ getAlarmActionReq thisArg

getAlarmActionStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmActionStreamReq thisArg = makeStreamReq $ getAlarmActionReq thisArg

getAlarmActionStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmAction))
getAlarmActionStream thisArg = requestAddStream $ getAlarmActionStreamReq thisArg 

{-|
The unique identifier for the alarm.
 -}
getAlarmIDReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Data.Text.Text)
getAlarmIDReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]

getAlarmID :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmID thisArg = simpleRequest $ getAlarmIDReq thisArg

getAlarmIDStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmIDStreamReq thisArg = makeStreamReq $ getAlarmIDReq thisArg

getAlarmIDStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmIDStream thisArg = requestAddStream $ getAlarmIDStreamReq thisArg 

{-|
The number of seconds before the event that the alarm will fire.
 -}
getAlarmMarginReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Double)
getAlarmMarginReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]

getAlarmMargin :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmMargin thisArg = simpleRequest $ getAlarmMarginReq thisArg

getAlarmMarginStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmMarginStreamReq thisArg = makeStreamReq $ getAlarmMarginReq thisArg

getAlarmMarginStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmMarginStream thisArg = requestAddStream $ getAlarmMarginStreamReq thisArg 

{-|
The short name of the alarm.
 -}
getAlarmNameReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Data.Text.Text)
getAlarmNameReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]

getAlarmName :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmName thisArg = simpleRequest $ getAlarmNameReq thisArg

getAlarmNameStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmNameStreamReq thisArg = makeStreamReq $ getAlarmNameReq thisArg

getAlarmNameStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmNameStream thisArg = requestAddStream $ getAlarmNameStreamReq thisArg 

{-|
The long description of the alarm.
 -}
getAlarmNotesReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Data.Text.Text)
getAlarmNotesReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]

getAlarmNotes :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmNotes thisArg = simpleRequest $ getAlarmNotesReq thisArg

getAlarmNotesStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmNotesStreamReq thisArg = makeStreamReq $ getAlarmNotesReq thisArg

getAlarmNotesStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmNotesStream thisArg = requestAddStream $ getAlarmNotesStreamReq thisArg 

{-|
The number of seconds until the alarm will fire.
 -}
getAlarmRemainingReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Double)
getAlarmRemainingReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]

getAlarmRemaining :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmRemaining thisArg = simpleRequest $ getAlarmRemainingReq thisArg

getAlarmRemainingStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmRemainingStreamReq thisArg = makeStreamReq $ getAlarmRemainingReq thisArg

getAlarmRemainingStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmRemainingStream thisArg = requestAddStream $ getAlarmRemainingStreamReq thisArg 

{-|
Whether the alarm will be repeated after it has fired.
 -}
getAlarmRepeatReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Bool)
getAlarmRepeatReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]

getAlarmRepeat :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Bool)
getAlarmRepeat thisArg = simpleRequest $ getAlarmRepeatReq thisArg

getAlarmRepeatStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Bool)
getAlarmRepeatStreamReq thisArg = makeStreamReq $ getAlarmRepeatReq thisArg

getAlarmRepeatStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Bool))
getAlarmRepeatStream thisArg = requestAddStream $ getAlarmRepeatStreamReq thisArg 

{-|
The time delay to automatically create an alarm after it has fired.
 -}
getAlarmRepeatPeriodReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Double)
getAlarmRepeatPeriodReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]

getAlarmRepeatPeriod :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmRepeatPeriod thisArg = simpleRequest $ getAlarmRepeatPeriodReq thisArg

getAlarmRepeatPeriodStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmRepeatPeriodStreamReq thisArg = makeStreamReq $ getAlarmRepeatPeriodReq thisArg

getAlarmRepeatPeriodStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmRepeatPeriodStream thisArg = requestAddStream $ getAlarmRepeatPeriodStreamReq thisArg 

{-|
The time at which the alarm will fire.
 -}
getAlarmTimeReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (Double)
getAlarmTimeReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]

getAlarmTime :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmTime thisArg = simpleRequest $ getAlarmTimeReq thisArg

getAlarmTimeStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmTimeStreamReq thisArg = makeStreamReq $ getAlarmTimeReq thisArg

getAlarmTimeStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmTimeStream thisArg = requestAddStream $ getAlarmTimeStreamReq thisArg 

{-|
The type of the alarm.
 -}
getAlarmTypeReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmTypeReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]

getAlarmType :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmType thisArg = simpleRequest $ getAlarmTypeReq thisArg

getAlarmTypeStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmTypeStreamReq thisArg = makeStreamReq $ getAlarmTypeReq thisArg

getAlarmTypeStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmType))
getAlarmTypeStream thisArg = requestAddStream $ getAlarmTypeStreamReq thisArg 

{-|
The vessel that the alarm is attached to.
 -}
getAlarmVesselReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getAlarmVesselReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]

getAlarmVessel :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getAlarmVessel thisArg = simpleRequest $ getAlarmVesselReq thisArg

getAlarmVesselStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getAlarmVesselStreamReq thisArg = makeStreamReq $ getAlarmVesselReq thisArg

getAlarmVesselStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAlarmVesselStream thisArg = requestAddStream $ getAlarmVesselStreamReq thisArg 

{-|
The celestial body the vessel is departing from.
 -}
getAlarmXferOriginBodyReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBodyReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]

getAlarmXferOriginBody :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBody thisArg = simpleRequest $ getAlarmXferOriginBodyReq thisArg

getAlarmXferOriginBodyStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBodyStreamReq thisArg = makeStreamReq $ getAlarmXferOriginBodyReq thisArg

getAlarmXferOriginBodyStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferOriginBodyStream thisArg = requestAddStream $ getAlarmXferOriginBodyStreamReq thisArg 

{-|
The celestial body the vessel is arriving at.
 -}
getAlarmXferTargetBodyReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBodyReq thisArg = makeCallReq "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]

getAlarmXferTargetBody :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBody thisArg = simpleRequest $ getAlarmXferTargetBodyReq thisArg

getAlarmXferTargetBodyStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBodyStreamReq thisArg = makeStreamReq $ getAlarmXferTargetBodyReq thisArg

getAlarmXferTargetBodyStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferTargetBodyStream thisArg = requestAddStream $ getAlarmXferTargetBodyStreamReq thisArg 

{-|
The action that the alarm triggers.
 -}
setAlarmActionReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.KerbalAlarmClock.AlarmAction -> KRPCCallReq ()
setAlarmActionReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Action" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmAction :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.KerbalAlarmClock.AlarmAction -> RPCContext ()
setAlarmAction thisArg valueArg = simpleRequest $ setAlarmActionReq thisArg valueArg 

{-|
The number of seconds before the event that the alarm will fire.
 -}
setAlarmMarginReq :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> KRPCCallReq ()
setAlarmMarginReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Margin" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmMargin :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext ()
setAlarmMargin thisArg valueArg = simpleRequest $ setAlarmMarginReq thisArg valueArg 

{-|
The short name of the alarm.
 -}
setAlarmNameReq :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> KRPCCallReq ()
setAlarmNameReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmName :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> RPCContext ()
setAlarmName thisArg valueArg = simpleRequest $ setAlarmNameReq thisArg valueArg 

{-|
The long description of the alarm.
 -}
setAlarmNotesReq :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> KRPCCallReq ()
setAlarmNotesReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Notes" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmNotes :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> RPCContext ()
setAlarmNotes thisArg valueArg = simpleRequest $ setAlarmNotesReq thisArg valueArg 

{-|
Whether the alarm will be repeated after it has fired.
 -}
setAlarmRepeatReq :: KRPCHS.KerbalAlarmClock.Alarm -> Bool -> KRPCCallReq ()
setAlarmRepeatReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Repeat" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmRepeat :: KRPCHS.KerbalAlarmClock.Alarm -> Bool -> RPCContext ()
setAlarmRepeat thisArg valueArg = simpleRequest $ setAlarmRepeatReq thisArg valueArg 

{-|
The time delay to automatically create an alarm after it has fired.
 -}
setAlarmRepeatPeriodReq :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> KRPCCallReq ()
setAlarmRepeatPeriodReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_RepeatPeriod" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmRepeatPeriod :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext ()
setAlarmRepeatPeriod thisArg valueArg = simpleRequest $ setAlarmRepeatPeriodReq thisArg valueArg 

{-|
The time at which the alarm will fire.
 -}
setAlarmTimeReq :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> KRPCCallReq ()
setAlarmTimeReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Time" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmTime :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext ()
setAlarmTime thisArg valueArg = simpleRequest $ setAlarmTimeReq thisArg valueArg 

{-|
The vessel that the alarm is attached to.
 -}
setAlarmVesselReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ()
setAlarmVesselReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_Vessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmVessel :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setAlarmVessel thisArg valueArg = simpleRequest $ setAlarmVesselReq thisArg valueArg 

{-|
The celestial body the vessel is departing from.
 -}
setAlarmXferOriginBodyReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ()
setAlarmXferOriginBodyReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_XferOriginBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmXferOriginBody :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setAlarmXferOriginBody thisArg valueArg = simpleRequest $ setAlarmXferOriginBodyReq thisArg valueArg 

{-|
The celestial body the vessel is arriving at.
 -}
setAlarmXferTargetBodyReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ()
setAlarmXferTargetBodyReq thisArg valueArg = makeCallReq "KerbalAlarmClock" "Alarm_set_XferTargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAlarmXferTargetBody :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setAlarmXferTargetBody thisArg valueArg = simpleRequest $ setAlarmXferTargetBodyReq thisArg valueArg 

{-|
Get a list of alarms of the specified <paramref name="type" />.<param name="type">Type of alarm to return.
 -}
alarmsWithTypeReq :: KRPCHS.KerbalAlarmClock.AlarmType -> KRPCCallReq ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithTypeReq typeArg = makeCallReq "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]

alarmsWithType :: KRPCHS.KerbalAlarmClock.AlarmType -> RPCContext ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithType typeArg = simpleRequest $ alarmsWithTypeReq typeArg

alarmsWithTypeStreamReq :: KRPCHS.KerbalAlarmClock.AlarmType -> KRPCStreamReq ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithTypeStreamReq typeArg = makeStreamReq $ alarmsWithTypeReq typeArg

alarmsWithTypeStream :: KRPCHS.KerbalAlarmClock.AlarmType -> RPCContext (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
alarmsWithTypeStream typeArg = requestAddStream $ alarmsWithTypeStreamReq typeArg 

{-|
Create a new alarm and return it.<param name="type">Type of the new alarm.<param name="name">Name of the new alarm.<param name="ut">Time at which the new alarm should trigger.
 -}
createAlarmReq :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> KRPCCallReq (KRPCHS.KerbalAlarmClock.Alarm)
createAlarmReq typeArg nameArg utArg = makeCallReq "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]

createAlarm :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> RPCContext (KRPCHS.KerbalAlarmClock.Alarm)
createAlarm typeArg nameArg utArg = simpleRequest $ createAlarmReq typeArg nameArg utArg

createAlarmStreamReq :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.Alarm)
createAlarmStreamReq typeArg nameArg utArg = makeStreamReq $ createAlarmReq typeArg nameArg utArg

createAlarmStream :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
createAlarmStream typeArg nameArg utArg = requestAddStream $ createAlarmStreamReq typeArg nameArg utArg 

{-|
A list of all the alarms.
 -}
getAlarmsReq :: KRPCCallReq ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarmsReq  = makeCallReq "KerbalAlarmClock" "get_Alarms" []

getAlarms :: RPCContext ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarms  = simpleRequest $ getAlarmsReq 

getAlarmsStreamReq :: KRPCStreamReq ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarmsStreamReq  = makeStreamReq $ getAlarmsReq 

getAlarmsStream :: RPCContext (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
getAlarmsStream  = requestAddStream $ getAlarmsStreamReq  

