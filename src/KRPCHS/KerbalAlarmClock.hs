module KRPCHS.KerbalAlarmClock
( AlarmAction(..)
, AlarmType(..)
, Alarm
, alarmWithName
, alarmWithNameStream
, alarmRemove
, getAlarmAction
, getAlarmActionStream
, getAlarmID
, getAlarmIDStream
, getAlarmMargin
, getAlarmMarginStream
, getAlarmName
, getAlarmNameStream
, getAlarmNotes
, getAlarmNotesStream
, getAlarmRemaining
, getAlarmRemainingStream
, getAlarmRepeat
, getAlarmRepeatStream
, getAlarmRepeatPeriod
, getAlarmRepeatPeriodStream
, getAlarmTime
, getAlarmTimeStream
, getAlarmType
, getAlarmTypeStream
, getAlarmVessel
, getAlarmVesselStream
, getAlarmXferOriginBody
, getAlarmXferOriginBodyStream
, getAlarmXferTargetBody
, getAlarmXferTargetBodyStream
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
, createAlarm
, createAlarmStream
, getAlarms
, getAlarmsStream
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
    processResponse extract res 

alarmWithNameStream :: Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
alarmWithNameStream nameArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmWithName" [makeArgument 0 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Removes the alarm.
 -}
alarmRemove :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Bool)
alarmRemove thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The action that the alarm triggers.
 -}
getAlarmAction :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmAction thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmActionStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmAction))
getAlarmActionStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unique identifier for the alarm.
 -}
getAlarmID :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmID thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmIDStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmIDStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The number of seconds before the event that the alarm will fire.
 -}
getAlarmMargin :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmMargin thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmMarginStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmMarginStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The short name of the alarm.
 -}
getAlarmName :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmName thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmNameStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmNameStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The long description of the alarm.
 -}
getAlarmNotes :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Data.Text.Text)
getAlarmNotes thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmNotesStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Data.Text.Text))
getAlarmNotesStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The number of seconds until the alarm will fire.
 -}
getAlarmRemaining :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmRemaining thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmRemainingStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmRemainingStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the alarm will be repeated after it has fired.
 -}
getAlarmRepeat :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Bool)
getAlarmRepeat thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmRepeatStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Bool))
getAlarmRepeatStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time delay to automatically create an alarm after it has fired.
 -}
getAlarmRepeatPeriod :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmRepeatPeriod thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmRepeatPeriodStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmRepeatPeriodStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time at which the alarm will fire.
 -}
getAlarmTime :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (Double)
getAlarmTime thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmTimeStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (Double))
getAlarmTimeStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The type of the alarm.
 -}
getAlarmType :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmType thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmTypeStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmType))
getAlarmTypeStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vessel that the alarm is attached to.
 -}
getAlarmVessel :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getAlarmVessel thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmVesselStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAlarmVesselStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The celestial body the vessel is departing from.
 -}
getAlarmXferOriginBody :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmXferOriginBodyStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferOriginBodyStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The celestial body the vessel is arriving at.
 -}
getAlarmXferTargetBody :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAlarmXferTargetBodyStream :: KRPCHS.KerbalAlarmClock.Alarm -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferTargetBodyStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The action that the alarm triggers.
 -}
setAlarmAction :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.KerbalAlarmClock.AlarmAction -> RPCContext (Bool)
setAlarmAction thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Action" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The number of seconds before the event that the alarm will fire.
 -}
setAlarmMargin :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext (Bool)
setAlarmMargin thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Margin" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The short name of the alarm.
 -}
setAlarmName :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> RPCContext (Bool)
setAlarmName thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The long description of the alarm.
 -}
setAlarmNotes :: KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> RPCContext (Bool)
setAlarmNotes thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Notes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the alarm will be repeated after it has fired.
 -}
setAlarmRepeat :: KRPCHS.KerbalAlarmClock.Alarm -> Bool -> RPCContext (Bool)
setAlarmRepeat thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Repeat" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The time delay to automatically create an alarm after it has fired.
 -}
setAlarmRepeatPeriod :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext (Bool)
setAlarmRepeatPeriod thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_RepeatPeriod" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The time at which the alarm will fire.
 -}
setAlarmTime :: KRPCHS.KerbalAlarmClock.Alarm -> Double -> RPCContext (Bool)
setAlarmTime thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Time" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The vessel that the alarm is attached to.
 -}
setAlarmVessel :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
setAlarmVessel thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Vessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The celestial body the vessel is departing from.
 -}
setAlarmXferOriginBody :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
setAlarmXferOriginBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferOriginBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The celestial body the vessel is arriving at.
 -}
setAlarmXferTargetBody :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
setAlarmXferTargetBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferTargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Get a list of alarms of the specified <paramref name="type" />.<param name="type">Type of alarm to return.
 -}
alarmsWithType :: KRPCHS.KerbalAlarmClock.AlarmType -> RPCContext ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithType typeArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]
    res <- sendRequest r
    processResponse extract res 

alarmsWithTypeStream :: KRPCHS.KerbalAlarmClock.AlarmType -> RPCContext (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
alarmsWithTypeStream typeArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Create a new alarm and return it.<param name="type">Type of the new alarm.<param name="name">Name of the new alarm.<param name="ut">Time at which the new alarm should trigger.
 -}
createAlarm :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> RPCContext (KRPCHS.KerbalAlarmClock.Alarm)
createAlarm typeArg nameArg utArg = do
    let r = makeRequest "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]
    res <- sendRequest r
    processResponse extract res 

createAlarmStream :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> RPCContext (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
createAlarmStream typeArg nameArg utArg = do
    let r = makeRequest "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all the alarms.
 -}
getAlarms :: RPCContext ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarms  = do
    let r = makeRequest "KerbalAlarmClock" "get_Alarms" []
    res <- sendRequest r
    processResponse extract res 

getAlarmsStream :: RPCContext (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
getAlarmsStream  = do
    let r = makeRequest "KerbalAlarmClock" "get_Alarms" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


