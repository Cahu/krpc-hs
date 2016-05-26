{-# LANGUAGE RecordWildCards #-}
module KRPCHS.KerbalAlarmClock
( Alarm(..)
, AlarmAction(..)
, AlarmType(..)
, alarmWithName
, alarmsWithType
, createAlarm
, getAlarms
, alarmRemove
, getAlarmAction
, setAlarmAction
, getAlarmMargin
, setAlarmMargin
, getAlarmTime
, setAlarmTime
, getAlarmType
, getAlarmID
, getAlarmName
, setAlarmName
, getAlarmNotes
, setAlarmNotes
, getAlarmRemaining
, getAlarmRepeat
, setAlarmRepeat
, getAlarmRepeatPeriod
, setAlarmRepeatPeriod
, getAlarmVessel
, setAlarmVessel
, getAlarmXferOriginBody
, setAlarmXferOriginBody
, getAlarmXferTargetBody
, setAlarmXferTargetBody
, alarmWithNameStream
, alarmsWithTypeStream
, createAlarmStream
, getAlarmsStream
, getAlarmActionStream
, getAlarmMarginStream
, getAlarmTimeStream
, getAlarmTypeStream
, getAlarmIDStream
, getAlarmNameStream
, getAlarmNotesStream
, getAlarmRemainingStream
, getAlarmRepeatStream
, getAlarmRepeatPeriodStream
, getAlarmVesselStream
, getAlarmXferOriginBodyStream
, getAlarmXferTargetBodyStream
) where


import Data.Text

import KRPCHS.Requests
import KRPCHS.SerializeUtils

import KRPCHS.SpaceCenter


newtype Alarm = Alarm { alarmId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Alarm where
    encodePb   = encodePb . alarmId
    decodePb b = Alarm <$> decodePb b

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

data AlarmType
    = AlarmType'Raw
    | AlarmType'Maneuver
    | AlarmType'ManeuverAuto
    | AlarmType'Apoapsis
    | AlarmType'Periapsis
    | AlarmType'AscendingNode
    | AlarmType'DescendingNode
    | AlarmType'Closest
    | AlarmType'Contract
    | AlarmType'ContractAuto
    | AlarmType'Crew
    | AlarmType'Distance
    | AlarmType'EarthTime
    | AlarmType'LaunchRendevous
    | AlarmType'SOIChange
    | AlarmType'SOIChangeAuto
    | AlarmType'Transfer
    | AlarmType'TransferModelled
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable AlarmType where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

alarmWithName :: Text -> RPCContext (Alarm)
alarmWithName nameArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmWithName" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

alarmWithNameStream :: Text -> RPCContext (KRPCStream (Alarm))
alarmWithNameStream nameArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmWithName" [ makeArgument 0 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

alarmsWithType :: AlarmType -> RPCContext ([Alarm])
alarmsWithType typeArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmsWithType" [ makeArgument 0 typeArg ]
    res <- sendRequest r
    processResponse extractList res

alarmsWithTypeStream :: AlarmType -> RPCContext (KRPCStream ([Alarm]))
alarmsWithTypeStream typeArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmsWithType" [ makeArgument 0 typeArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

createAlarm :: AlarmType -> Text -> Double -> RPCContext (Alarm)
createAlarm typeArg nameArg utArg = do
    let r = makeRequest "KerbalAlarmClock" "CreateAlarm" [ makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg ]
    res <- sendRequest r
    processResponse extractValue res

createAlarmStream :: AlarmType -> Text -> Double -> RPCContext (KRPCStream (Alarm))
createAlarmStream typeArg nameArg utArg = do
    let r = makeRequest "KerbalAlarmClock" "CreateAlarm" [ makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getAlarms :: RPCContext ([Alarm])
getAlarms  = do
    let r = makeRequest "KerbalAlarmClock" "get_Alarms" [  ]
    res <- sendRequest r
    processResponse extractList res

getAlarmsStream :: RPCContext (KRPCStream ([Alarm]))
getAlarmsStream  = do
    let r = makeRequest "KerbalAlarmClock" "get_Alarms" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

alarmRemove :: Alarm -> RPCContext (Bool)
alarmRemove thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_Remove" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmAction :: Alarm -> RPCContext (AlarmAction)
getAlarmAction thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmActionStream :: Alarm -> RPCContext (KRPCStream (AlarmAction))
getAlarmActionStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmAction :: Alarm -> AlarmAction -> RPCContext (Bool)
setAlarmAction thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Action" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmMargin :: Alarm -> RPCContext (Double)
getAlarmMargin thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmMarginStream :: Alarm -> RPCContext (KRPCStream (Double))
getAlarmMarginStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmMargin :: Alarm -> Double -> RPCContext (Bool)
setAlarmMargin thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Margin" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmTime :: Alarm -> RPCContext (Double)
getAlarmTime thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmTimeStream :: Alarm -> RPCContext (KRPCStream (Double))
getAlarmTimeStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmTime :: Alarm -> Double -> RPCContext (Bool)
setAlarmTime thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Time" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmType :: Alarm -> RPCContext (AlarmType)
getAlarmType thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmTypeStream :: Alarm -> RPCContext (KRPCStream (AlarmType))
getAlarmTypeStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getAlarmID :: Alarm -> RPCContext (Text)
getAlarmID thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmIDStream :: Alarm -> RPCContext (KRPCStream (Text))
getAlarmIDStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getAlarmName :: Alarm -> RPCContext (Text)
getAlarmName thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmNameStream :: Alarm -> RPCContext (KRPCStream (Text))
getAlarmNameStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmName :: Alarm -> Text -> RPCContext (Bool)
setAlarmName thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Name" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmNotes :: Alarm -> RPCContext (Text)
getAlarmNotes thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmNotesStream :: Alarm -> RPCContext (KRPCStream (Text))
getAlarmNotesStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmNotes :: Alarm -> Text -> RPCContext (Bool)
setAlarmNotes thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Notes" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmRemaining :: Alarm -> RPCContext (Double)
getAlarmRemaining thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmRemainingStream :: Alarm -> RPCContext (KRPCStream (Double))
getAlarmRemainingStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getAlarmRepeat :: Alarm -> RPCContext (Bool)
getAlarmRepeat thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmRepeatStream :: Alarm -> RPCContext (KRPCStream (Bool))
getAlarmRepeatStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmRepeat :: Alarm -> Bool -> RPCContext (Bool)
setAlarmRepeat thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Repeat" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmRepeatPeriod :: Alarm -> RPCContext (Double)
getAlarmRepeatPeriod thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmRepeatPeriodStream :: Alarm -> RPCContext (KRPCStream (Double))
getAlarmRepeatPeriodStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmRepeatPeriod :: Alarm -> Double -> RPCContext (Bool)
setAlarmRepeatPeriod thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_RepeatPeriod" [ makeArgument 0 (alarmId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmVessel :: Alarm -> RPCContext (Vessel)
getAlarmVessel thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmVesselStream :: Alarm -> RPCContext (KRPCStream (Vessel))
getAlarmVesselStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmVessel :: Alarm -> Vessel -> RPCContext (Bool)
setAlarmVessel thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Vessel" [ makeArgument 0 (alarmId thisArg), makeArgument 1 (vesselId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmXferOriginBody :: Alarm -> RPCContext (CelestialBody)
getAlarmXferOriginBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmXferOriginBodyStream :: Alarm -> RPCContext (KRPCStream (CelestialBody))
getAlarmXferOriginBodyStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmXferOriginBody :: Alarm -> CelestialBody -> RPCContext (Bool)
setAlarmXferOriginBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferOriginBody" [ makeArgument 0 (alarmId thisArg), makeArgument 1 (celestialBodyId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getAlarmXferTargetBody :: Alarm -> RPCContext (CelestialBody)
getAlarmXferTargetBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [ makeArgument 0 (alarmId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAlarmXferTargetBodyStream :: Alarm -> RPCContext (KRPCStream (CelestialBody))
getAlarmXferTargetBodyStream thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [ makeArgument 0 (alarmId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAlarmXferTargetBody :: Alarm -> CelestialBody -> RPCContext (Bool)
setAlarmXferTargetBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferTargetBody" [ makeArgument 0 (alarmId thisArg), makeArgument 1 (celestialBodyId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

