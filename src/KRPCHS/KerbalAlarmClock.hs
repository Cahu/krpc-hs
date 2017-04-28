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
alarmWithName :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithName nameArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmWithName" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res

alarmWithNameStreamReq :: Data.Text.Text -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.Alarm)
alarmWithNameStreamReq nameArg =
    let req = makeRequest "KerbalAlarmClock" "AlarmWithName" [makeArgument 0 nameArg]
    in  makeStream req

alarmWithNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
alarmWithNameStream nameArg = requestStream $ alarmWithNameStreamReq nameArg 

{-
 - Removes the alarm.
 -}
alarmRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m ()
alarmRemove thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The action that the alarm triggers.
 -}
getAlarmAction :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmAction thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmActionStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.AlarmAction)
getAlarmActionStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Action" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmActionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmAction))
getAlarmActionStream thisArg = requestStream $ getAlarmActionStreamReq thisArg 

{-
 - The unique identifier for the alarm.
 -}
getAlarmID :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Data.Text.Text)
getAlarmID thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmIDStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmIDStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_ID" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmIDStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Data.Text.Text))
getAlarmIDStream thisArg = requestStream $ getAlarmIDStreamReq thisArg 

{-
 - The number of seconds before the event that the alarm will fire.
 -}
getAlarmMargin :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Double)
getAlarmMargin thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmMarginStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmMarginStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Margin" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmMarginStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Double))
getAlarmMarginStream thisArg = requestStream $ getAlarmMarginStreamReq thisArg 

{-
 - The short name of the alarm.
 -}
getAlarmName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Data.Text.Text)
getAlarmName thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmNameStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmNameStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Data.Text.Text))
getAlarmNameStream thisArg = requestStream $ getAlarmNameStreamReq thisArg 

{-
 - The long description of the alarm.
 -}
getAlarmNotes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Data.Text.Text)
getAlarmNotes thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmNotesStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Data.Text.Text)
getAlarmNotesStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Notes" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmNotesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Data.Text.Text))
getAlarmNotesStream thisArg = requestStream $ getAlarmNotesStreamReq thisArg 

{-
 - The number of seconds until the alarm will fire.
 -}
getAlarmRemaining :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Double)
getAlarmRemaining thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmRemainingStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmRemainingStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Remaining" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmRemainingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Double))
getAlarmRemainingStream thisArg = requestStream $ getAlarmRemainingStreamReq thisArg 

{-
 - Whether the alarm will be repeated after it has fired.
 -}
getAlarmRepeat :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Bool)
getAlarmRepeat thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmRepeatStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Bool)
getAlarmRepeatStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Repeat" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmRepeatStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Bool))
getAlarmRepeatStream thisArg = requestStream $ getAlarmRepeatStreamReq thisArg 

{-
 - The time delay to automatically create an alarm after it has fired.
 -}
getAlarmRepeatPeriod :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Double)
getAlarmRepeatPeriod thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmRepeatPeriodStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmRepeatPeriodStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_RepeatPeriod" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmRepeatPeriodStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Double))
getAlarmRepeatPeriodStream thisArg = requestStream $ getAlarmRepeatPeriodStreamReq thisArg 

{-
 - The time at which the alarm will fire.
 -}
getAlarmTime :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (Double)
getAlarmTime thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmTimeStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (Double)
getAlarmTimeStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Time" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmTimeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (Double))
getAlarmTimeStream thisArg = requestStream $ getAlarmTimeStreamReq thisArg 

{-
 - The type of the alarm.
 -}
getAlarmType :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmType thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmTypeStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.AlarmType)
getAlarmTypeStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Type" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmTypeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (KRPCHS.KerbalAlarmClock.AlarmType))
getAlarmTypeStream thisArg = requestStream $ getAlarmTypeStreamReq thisArg 

{-
 - The vessel that the alarm is attached to.
 -}
getAlarmVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCHS.SpaceCenter.Vessel)
getAlarmVessel thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmVesselStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getAlarmVesselStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_Vessel" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAlarmVesselStream thisArg = requestStream $ getAlarmVesselStreamReq thisArg 

{-
 - The celestial body the vessel is departing from.
 -}
getAlarmXferOriginBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmXferOriginBodyStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferOriginBodyStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_XferOriginBody" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmXferOriginBodyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferOriginBodyStream thisArg = requestStream $ getAlarmXferOriginBodyStreamReq thisArg 

{-
 - The celestial body the vessel is arriving at.
 -}
getAlarmXferTargetBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBody thisArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAlarmXferTargetBodyStreamReq :: KRPCHS.KerbalAlarmClock.Alarm -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAlarmXferTargetBodyStreamReq thisArg =
    let req = makeRequest "KerbalAlarmClock" "Alarm_get_XferTargetBody" [makeArgument 0 thisArg]
    in  makeStream req

getAlarmXferTargetBodyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> m (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAlarmXferTargetBodyStream thisArg = requestStream $ getAlarmXferTargetBodyStreamReq thisArg 

{-
 - The action that the alarm triggers.
 -}
setAlarmAction :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.KerbalAlarmClock.AlarmAction -> m ()
setAlarmAction thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Action" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The number of seconds before the event that the alarm will fire.
 -}
setAlarmMargin :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> Double -> m ()
setAlarmMargin thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Margin" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The short name of the alarm.
 -}
setAlarmName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> m ()
setAlarmName thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The long description of the alarm.
 -}
setAlarmNotes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> Data.Text.Text -> m ()
setAlarmNotes thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Notes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the alarm will be repeated after it has fired.
 -}
setAlarmRepeat :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> Bool -> m ()
setAlarmRepeat thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Repeat" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time delay to automatically create an alarm after it has fired.
 -}
setAlarmRepeatPeriod :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> Double -> m ()
setAlarmRepeatPeriod thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_RepeatPeriod" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time at which the alarm will fire.
 -}
setAlarmTime :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> Double -> m ()
setAlarmTime thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Time" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The vessel that the alarm is attached to.
 -}
setAlarmVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.Vessel -> m ()
setAlarmVessel thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_Vessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The celestial body the vessel is departing from.
 -}
setAlarmXferOriginBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> m ()
setAlarmXferOriginBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferOriginBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The celestial body the vessel is arriving at.
 -}
setAlarmXferTargetBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.Alarm -> KRPCHS.SpaceCenter.CelestialBody -> m ()
setAlarmXferTargetBody thisArg valueArg = do
    let r = makeRequest "KerbalAlarmClock" "Alarm_set_XferTargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Get a list of alarms of the specified <paramref name="type" />.<param name="type">Type of alarm to return.
 -}
alarmsWithType :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.AlarmType -> m ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithType typeArg = do
    let r = makeRequest "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]
    res <- sendRequest r
    processResponse res

alarmsWithTypeStreamReq :: KRPCHS.KerbalAlarmClock.AlarmType -> KRPCStreamReq ([KRPCHS.KerbalAlarmClock.Alarm])
alarmsWithTypeStreamReq typeArg =
    let req = makeRequest "KerbalAlarmClock" "AlarmsWithType" [makeArgument 0 typeArg]
    in  makeStream req

alarmsWithTypeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.AlarmType -> m (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
alarmsWithTypeStream typeArg = requestStream $ alarmsWithTypeStreamReq typeArg 

{-
 - Create a new alarm and return it.<param name="type">Type of the new alarm.<param name="name">Name of the new alarm.<param name="ut">Time at which the new alarm should trigger.
 -}
createAlarm :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> m (KRPCHS.KerbalAlarmClock.Alarm)
createAlarm typeArg nameArg utArg = do
    let r = makeRequest "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]
    res <- sendRequest r
    processResponse res

createAlarmStreamReq :: KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> KRPCStreamReq (KRPCHS.KerbalAlarmClock.Alarm)
createAlarmStreamReq typeArg nameArg utArg =
    let req = makeRequest "KerbalAlarmClock" "CreateAlarm" [makeArgument 0 typeArg, makeArgument 1 nameArg, makeArgument 2 utArg]
    in  makeStream req

createAlarmStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.KerbalAlarmClock.AlarmType -> Data.Text.Text -> Double -> m (KRPCStream (KRPCHS.KerbalAlarmClock.Alarm))
createAlarmStream typeArg nameArg utArg = requestStream $ createAlarmStreamReq typeArg nameArg utArg 

{-
 - A list of all the alarms.
 -}
getAlarms :: (MonadIO m, MonadThrow m, MonadRPC m) => m ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarms  = do
    let r = makeRequest "KerbalAlarmClock" "get_Alarms" []
    res <- sendRequest r
    processResponse res

getAlarmsStreamReq :: KRPCStreamReq ([KRPCHS.KerbalAlarmClock.Alarm])
getAlarmsStreamReq  =
    let req = makeRequest "KerbalAlarmClock" "get_Alarms" []
    in  makeStream req

getAlarmsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream ([KRPCHS.KerbalAlarmClock.Alarm]))
getAlarmsStream  = requestStream $ getAlarmsStreamReq  

{-
 - Whether Kerbal Alarm Clock is available.
 -}
getAvailable :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Bool)
getAvailable  = do
    let r = makeRequest "KerbalAlarmClock" "get_Available" []
    res <- sendRequest r
    processResponse res

getAvailableStreamReq :: KRPCStreamReq (Bool)
getAvailableStreamReq  =
    let req = makeRequest "KerbalAlarmClock" "get_Available" []
    in  makeStream req

getAvailableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Bool))
getAvailableStream  = requestStream $ getAvailableStreamReq  

