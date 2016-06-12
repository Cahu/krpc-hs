module KRPCHS.RemoteTech
( Target(..)
, Antenna
, Comms
, antenna
, antennaStream
, getAntennaHasConnection
, getAntennaHasConnectionStream
, getAntennaPart
, getAntennaPartStream
, getAntennaTarget
, getAntennaTargetStream
, getAntennaTargetBody
, getAntennaTargetBodyStream
, getAntennaTargetGroundStation
, getAntennaTargetGroundStationStream
, getAntennaTargetVessel
, getAntennaTargetVesselStream
, setAntennaTarget
, setAntennaTargetBody
, setAntennaTargetGroundStation
, setAntennaTargetVessel
, comms
, commsStream
, commsSignalDelayToVessel
, commsSignalDelayToVesselStream
, getCommsAntennas
, getCommsAntennasStream
, getCommsHasConnection
, getCommsHasConnectionStream
, getCommsHasConnectionToGroundStation
, getCommsHasConnectionToGroundStationStream
, getCommsHasFlightComputer
, getCommsHasFlightComputerStream
, getCommsHasLocalControl
, getCommsHasLocalControlStream
, getCommsSignalDelay
, getCommsSignalDelayStream
, getCommsSignalDelayToGroundStation
, getCommsSignalDelayToGroundStationStream
, getCommsVessel
, getCommsVesselStream
, getGroundStations
, getGroundStationsStream
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - A RemoteTech antenna. Obtained by calling <see cref="M:RemoteTech.Comms.Antennas" /> or  <see cref="M:RemoteTech.Antenna" />.
 -}
newtype Antenna = Antenna { antennaId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Antenna where
    encodePb   = encodePb . antennaId
    decodePb b = Antenna <$> decodePb b

instance KRPCResponseExtractable Antenna

{-
 - Communications for a vessel.
 -}
newtype Comms = Comms { commsId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Comms where
    encodePb   = encodePb . commsId
    decodePb b = Comms <$> decodePb b

instance KRPCResponseExtractable Comms


{-
 - The type of object an antenna is targetting.
 - See <see cref="M:RemoteTech.Antenna.Target" />.
 -}
data Target
    = Target'ActiveVessel
    | Target'CelestialBody
    | Target'GroundStation
    | Target'Vessel
    | Target'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable Target where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable Target


{-
 - Get the antenna object for a particular part.
 -}
antenna :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.RemoteTech.Antenna)
antenna partArg = do
    let r = makeRequest "RemoteTech" "Antenna" [makeArgument 0 partArg]
    res <- sendRequest r
    processResponse extract res 

antennaStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Antenna))
antennaStream partArg = do
    let r = makeRequest "RemoteTech" "Antenna" [makeArgument 0 partArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the antenna has a connection.
 -}
getAntennaHasConnection :: KRPCHS.RemoteTech.Antenna -> RPCContext (Bool)
getAntennaHasConnection thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_HasConnection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAntennaHasConnectionStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (Bool))
getAntennaHasConnectionStream thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_HasConnection" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Get the part containing this antenna.
 -}
getAntennaPart :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.Part)
getAntennaPart thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAntennaPartStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getAntennaPartStream thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The object that the antenna is targetting.
 - This property can be used to set the target to <see cref="M:RemoteTech.Target.None" /> or <see cref="M:RemoteTech.Target.ActiveVessel" />.
 - To set the target to a celestial body, ground station or vessel see <see cref="M:RemoteTech.Antenna.TargetBody" />,
 - <see cref="M:RemoteTech.Antenna.TargetGroundStation" /> and <see cref="M:RemoteTech.Antenna.TargetVessel" />.
 -}
getAntennaTarget :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.RemoteTech.Target)
getAntennaTarget thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_Target" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAntennaTargetStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Target))
getAntennaTargetStream thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_Target" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The celestial body the antenna is targetting.
 -}
getAntennaTargetBody :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAntennaTargetBody thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAntennaTargetBodyStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAntennaTargetBodyStream thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetBody" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The ground station the antenna is targetting.
 -}
getAntennaTargetGroundStation :: KRPCHS.RemoteTech.Antenna -> RPCContext (Data.Text.Text)
getAntennaTargetGroundStation thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetGroundStation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAntennaTargetGroundStationStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (Data.Text.Text))
getAntennaTargetGroundStationStream thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetGroundStation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vessel the antenna is targetting.
 -}
getAntennaTargetVessel :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getAntennaTargetVessel thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetVessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAntennaTargetVesselStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAntennaTargetVesselStream thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetVessel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The object that the antenna is targetting.
 - This property can be used to set the target to <see cref="M:RemoteTech.Target.None" /> or <see cref="M:RemoteTech.Target.ActiveVessel" />.
 - To set the target to a celestial body, ground station or vessel see <see cref="M:RemoteTech.Antenna.TargetBody" />,
 - <see cref="M:RemoteTech.Antenna.TargetGroundStation" /> and <see cref="M:RemoteTech.Antenna.TargetVessel" />.
 -}
setAntennaTarget :: KRPCHS.RemoteTech.Antenna -> KRPCHS.RemoteTech.Target -> RPCContext (Bool)
setAntennaTarget thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_Target" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The celestial body the antenna is targetting.
 -}
setAntennaTargetBody :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
setAntennaTargetBody thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_TargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The ground station the antenna is targetting.
 -}
setAntennaTargetGroundStation :: KRPCHS.RemoteTech.Antenna -> Data.Text.Text -> RPCContext (Bool)
setAntennaTargetGroundStation thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_TargetGroundStation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The vessel the antenna is targetting.
 -}
setAntennaTargetVessel :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
setAntennaTargetVessel thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_TargetVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Get a communications object, representing the communication capability of a particular vessel.
 -}
comms :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.RemoteTech.Comms)
comms vesselArg = do
    let r = makeRequest "RemoteTech" "Comms" [makeArgument 0 vesselArg]
    res <- sendRequest r
    processResponse extract res 

commsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Comms))
commsStream vesselArg = do
    let r = makeRequest "RemoteTech" "Comms" [makeArgument 0 vesselArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The signal delay between the this vessel and another vessel, in seconds.<param name="other">
 -}
commsSignalDelayToVessel :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> RPCContext (Double)
commsSignalDelayToVessel thisArg otherArg = do
    let r = makeRequest "RemoteTech" "Comms_SignalDelayToVessel" [makeArgument 0 thisArg, makeArgument 1 otherArg]
    res <- sendRequest r
    processResponse extract res 

commsSignalDelayToVesselStream :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Double))
commsSignalDelayToVesselStream thisArg otherArg = do
    let r = makeRequest "RemoteTech" "Comms_SignalDelayToVessel" [makeArgument 0 thisArg, makeArgument 1 otherArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The antennas for this vessel.
 -}
getCommsAntennas :: KRPCHS.RemoteTech.Comms -> RPCContext ([KRPCHS.RemoteTech.Antenna])
getCommsAntennas thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_Antennas" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsAntennasStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream ([KRPCHS.RemoteTech.Antenna]))
getCommsAntennasStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_Antennas" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the vessel has any connection.
 -}
getCommsHasConnection :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasConnection thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasConnection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsHasConnectionStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasConnection" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the vessel has a connection to a ground station.
 -}
getCommsHasConnectionToGroundStation :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasConnectionToGroundStation thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasConnectionToGroundStation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsHasConnectionToGroundStationStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionToGroundStationStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasConnectionToGroundStation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the vessel has a flight computer on board.
 -}
getCommsHasFlightComputer :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasFlightComputer thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasFlightComputer" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsHasFlightComputerStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasFlightComputerStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasFlightComputer" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the vessel can be controlled locally.
 -}
getCommsHasLocalControl :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasLocalControl thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasLocalControl" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsHasLocalControlStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasLocalControlStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasLocalControl" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The shortest signal delay to the vessel, in seconds.
 -}
getCommsSignalDelay :: KRPCHS.RemoteTech.Comms -> RPCContext (Double)
getCommsSignalDelay thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_SignalDelay" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsSignalDelayStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_SignalDelay" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The signal delay between the vessel and the closest ground station, in seconds.
 -}
getCommsSignalDelayToGroundStation :: KRPCHS.RemoteTech.Comms -> RPCContext (Double)
getCommsSignalDelayToGroundStation thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_SignalDelayToGroundStation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsSignalDelayToGroundStationStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayToGroundStationStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_SignalDelayToGroundStation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Get the vessel.
 -}
getCommsVessel :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getCommsVessel thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCommsVesselStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCommsVesselStream thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_Vessel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The names of the ground stations.
 -}
getGroundStations :: RPCContext ([Data.Text.Text])
getGroundStations  = do
    let r = makeRequest "RemoteTech" "get_GroundStations" []
    res <- sendRequest r
    processResponse extract res 

getGroundStationsStream :: RPCContext (KRPCStream ([Data.Text.Text]))
getGroundStationsStream  = do
    let r = makeRequest "RemoteTech" "get_GroundStations" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


