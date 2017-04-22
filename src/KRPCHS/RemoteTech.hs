module KRPCHS.RemoteTech
( Target(..)
, Antenna
, Comms
, antenna
, antennaStream
, antennaStreamReq
, getAntennaHasConnection
, getAntennaHasConnectionStream
, getAntennaHasConnectionStreamReq
, getAntennaPart
, getAntennaPartStream
, getAntennaPartStreamReq
, getAntennaTarget
, getAntennaTargetStream
, getAntennaTargetStreamReq
, getAntennaTargetBody
, getAntennaTargetBodyStream
, getAntennaTargetBodyStreamReq
, getAntennaTargetGroundStation
, getAntennaTargetGroundStationStream
, getAntennaTargetGroundStationStreamReq
, getAntennaTargetVessel
, getAntennaTargetVesselStream
, getAntennaTargetVesselStreamReq
, setAntennaTarget
, setAntennaTargetBody
, setAntennaTargetGroundStation
, setAntennaTargetVessel
, comms
, commsStream
, commsStreamReq
, commsSignalDelayToVessel
, commsSignalDelayToVesselStream
, commsSignalDelayToVesselStreamReq
, getCommsAntennas
, getCommsAntennasStream
, getCommsAntennasStreamReq
, getCommsHasConnection
, getCommsHasConnectionStream
, getCommsHasConnectionStreamReq
, getCommsHasConnectionToGroundStation
, getCommsHasConnectionToGroundStationStream
, getCommsHasConnectionToGroundStationStreamReq
, getCommsHasFlightComputer
, getCommsHasFlightComputerStream
, getCommsHasFlightComputerStreamReq
, getCommsHasLocalControl
, getCommsHasLocalControlStream
, getCommsHasLocalControlStreamReq
, getCommsSignalDelay
, getCommsSignalDelayStream
, getCommsSignalDelayStreamReq
, getCommsSignalDelayToGroundStation
, getCommsSignalDelayToGroundStationStream
, getCommsSignalDelayToGroundStationStreamReq
, getCommsVessel
, getCommsVesselStream
, getCommsVesselStreamReq
, getAvailable
, getAvailableStream
, getAvailableStreamReq
, getGroundStations
, getGroundStationsStream
, getGroundStationsStreamReq
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - A RemoteTech antenna. Obtained by calling <see cref="M:RemoteTech.Comms.Antennas" /> or <see cref="M:RemoteTech.Antenna" />.
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
    processResponse res

antennaStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.RemoteTech.Antenna)
antennaStreamReq partArg =
    let req = makeRequest "RemoteTech" "Antenna" [makeArgument 0 partArg]
    in  makeStream req

antennaStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Antenna))
antennaStream partArg = requestStream $ antennaStreamReq partArg 

{-
 - Whether the antenna has a connection.
 -}
getAntennaHasConnection :: KRPCHS.RemoteTech.Antenna -> RPCContext (Bool)
getAntennaHasConnection thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_HasConnection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaHasConnectionStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (Bool)
getAntennaHasConnectionStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Antenna_get_HasConnection" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaHasConnectionStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (Bool))
getAntennaHasConnectionStream thisArg = requestStream $ getAntennaHasConnectionStreamReq thisArg 

{-
 - Get the part containing this antenna.
 -}
getAntennaPart :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.Part)
getAntennaPart thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaPartStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getAntennaPartStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Antenna_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaPartStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getAntennaPartStream thisArg = requestStream $ getAntennaPartStreamReq thisArg 

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
    processResponse res

getAntennaTargetStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.RemoteTech.Target)
getAntennaTargetStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Antenna_get_Target" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaTargetStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Target))
getAntennaTargetStream thisArg = requestStream $ getAntennaTargetStreamReq thisArg 

{-
 - The celestial body the antenna is targetting.
 -}
getAntennaTargetBody :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAntennaTargetBody thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaTargetBodyStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAntennaTargetBodyStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Antenna_get_TargetBody" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaTargetBodyStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAntennaTargetBodyStream thisArg = requestStream $ getAntennaTargetBodyStreamReq thisArg 

{-
 - The ground station the antenna is targetting.
 -}
getAntennaTargetGroundStation :: KRPCHS.RemoteTech.Antenna -> RPCContext (Data.Text.Text)
getAntennaTargetGroundStation thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetGroundStation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaTargetGroundStationStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (Data.Text.Text)
getAntennaTargetGroundStationStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Antenna_get_TargetGroundStation" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaTargetGroundStationStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (Data.Text.Text))
getAntennaTargetGroundStationStream thisArg = requestStream $ getAntennaTargetGroundStationStreamReq thisArg 

{-
 - The vessel the antenna is targetting.
 -}
getAntennaTargetVessel :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getAntennaTargetVessel thisArg = do
    let r = makeRequest "RemoteTech" "Antenna_get_TargetVessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaTargetVesselStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getAntennaTargetVesselStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Antenna_get_TargetVessel" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaTargetVesselStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAntennaTargetVesselStream thisArg = requestStream $ getAntennaTargetVesselStreamReq thisArg 

{-
 - The object that the antenna is targetting.
 - This property can be used to set the target to <see cref="M:RemoteTech.Target.None" /> or <see cref="M:RemoteTech.Target.ActiveVessel" />.
 - To set the target to a celestial body, ground station or vessel see <see cref="M:RemoteTech.Antenna.TargetBody" />,
 - <see cref="M:RemoteTech.Antenna.TargetGroundStation" /> and <see cref="M:RemoteTech.Antenna.TargetVessel" />.
 -}
setAntennaTarget :: KRPCHS.RemoteTech.Antenna -> KRPCHS.RemoteTech.Target -> RPCContext ()
setAntennaTarget thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_Target" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The celestial body the antenna is targetting.
 -}
setAntennaTargetBody :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setAntennaTargetBody thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_TargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The ground station the antenna is targetting.
 -}
setAntennaTargetGroundStation :: KRPCHS.RemoteTech.Antenna -> Data.Text.Text -> RPCContext ()
setAntennaTargetGroundStation thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_TargetGroundStation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The vessel the antenna is targetting.
 -}
setAntennaTargetVessel :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setAntennaTargetVessel thisArg valueArg = do
    let r = makeRequest "RemoteTech" "Antenna_set_TargetVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Get a communications object, representing the communication capability of a particular vessel.
 -}
comms :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.RemoteTech.Comms)
comms vesselArg = do
    let r = makeRequest "RemoteTech" "Comms" [makeArgument 0 vesselArg]
    res <- sendRequest r
    processResponse res

commsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.RemoteTech.Comms)
commsStreamReq vesselArg =
    let req = makeRequest "RemoteTech" "Comms" [makeArgument 0 vesselArg]
    in  makeStream req

commsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Comms))
commsStream vesselArg = requestStream $ commsStreamReq vesselArg 

{-
 - The signal delay between the this vessel and another vessel, in seconds.<param name="other">
 -}
commsSignalDelayToVessel :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> RPCContext (Double)
commsSignalDelayToVessel thisArg otherArg = do
    let r = makeRequest "RemoteTech" "Comms_SignalDelayToVessel" [makeArgument 0 thisArg, makeArgument 1 otherArg]
    res <- sendRequest r
    processResponse res

commsSignalDelayToVesselStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Double)
commsSignalDelayToVesselStreamReq thisArg otherArg =
    let req = makeRequest "RemoteTech" "Comms_SignalDelayToVessel" [makeArgument 0 thisArg, makeArgument 1 otherArg]
    in  makeStream req

commsSignalDelayToVesselStream :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Double))
commsSignalDelayToVesselStream thisArg otherArg = requestStream $ commsSignalDelayToVesselStreamReq thisArg otherArg 

{-
 - The antennas for this vessel.
 -}
getCommsAntennas :: KRPCHS.RemoteTech.Comms -> RPCContext ([KRPCHS.RemoteTech.Antenna])
getCommsAntennas thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_Antennas" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsAntennasStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq ([KRPCHS.RemoteTech.Antenna])
getCommsAntennasStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_Antennas" [makeArgument 0 thisArg]
    in  makeStream req

getCommsAntennasStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream ([KRPCHS.RemoteTech.Antenna]))
getCommsAntennasStream thisArg = requestStream $ getCommsAntennasStreamReq thisArg 

{-
 - Whether the vessel has any connection.
 -}
getCommsHasConnection :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasConnection thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasConnection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsHasConnectionStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasConnectionStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_HasConnection" [makeArgument 0 thisArg]
    in  makeStream req

getCommsHasConnectionStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionStream thisArg = requestStream $ getCommsHasConnectionStreamReq thisArg 

{-
 - Whether the vessel has a connection to a ground station.
 -}
getCommsHasConnectionToGroundStation :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasConnectionToGroundStation thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasConnectionToGroundStation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsHasConnectionToGroundStationStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasConnectionToGroundStationStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_HasConnectionToGroundStation" [makeArgument 0 thisArg]
    in  makeStream req

getCommsHasConnectionToGroundStationStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionToGroundStationStream thisArg = requestStream $ getCommsHasConnectionToGroundStationStreamReq thisArg 

{-
 - Whether the vessel has a flight computer on board.
 -}
getCommsHasFlightComputer :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasFlightComputer thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasFlightComputer" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsHasFlightComputerStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasFlightComputerStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_HasFlightComputer" [makeArgument 0 thisArg]
    in  makeStream req

getCommsHasFlightComputerStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasFlightComputerStream thisArg = requestStream $ getCommsHasFlightComputerStreamReq thisArg 

{-
 - Whether the vessel can be controlled locally.
 -}
getCommsHasLocalControl :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasLocalControl thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_HasLocalControl" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsHasLocalControlStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasLocalControlStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_HasLocalControl" [makeArgument 0 thisArg]
    in  makeStream req

getCommsHasLocalControlStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasLocalControlStream thisArg = requestStream $ getCommsHasLocalControlStreamReq thisArg 

{-
 - The shortest signal delay to the vessel, in seconds.
 -}
getCommsSignalDelay :: KRPCHS.RemoteTech.Comms -> RPCContext (Double)
getCommsSignalDelay thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_SignalDelay" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsSignalDelayStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Double)
getCommsSignalDelayStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_SignalDelay" [makeArgument 0 thisArg]
    in  makeStream req

getCommsSignalDelayStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayStream thisArg = requestStream $ getCommsSignalDelayStreamReq thisArg 

{-
 - The signal delay between the vessel and the closest ground station, in seconds.
 -}
getCommsSignalDelayToGroundStation :: KRPCHS.RemoteTech.Comms -> RPCContext (Double)
getCommsSignalDelayToGroundStation thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_SignalDelayToGroundStation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsSignalDelayToGroundStationStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Double)
getCommsSignalDelayToGroundStationStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_SignalDelayToGroundStation" [makeArgument 0 thisArg]
    in  makeStream req

getCommsSignalDelayToGroundStationStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayToGroundStationStream thisArg = requestStream $ getCommsSignalDelayToGroundStationStreamReq thisArg 

{-
 - Get the vessel.
 -}
getCommsVessel :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getCommsVessel thisArg = do
    let r = makeRequest "RemoteTech" "Comms_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsVesselStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getCommsVesselStreamReq thisArg =
    let req = makeRequest "RemoteTech" "Comms_get_Vessel" [makeArgument 0 thisArg]
    in  makeStream req

getCommsVesselStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCommsVesselStream thisArg = requestStream $ getCommsVesselStreamReq thisArg 

{-
 - Whether RemoteTech is installed.
 -}
getAvailable :: RPCContext (Bool)
getAvailable  = do
    let r = makeRequest "RemoteTech" "get_Available" []
    res <- sendRequest r
    processResponse res

getAvailableStreamReq :: KRPCStreamReq (Bool)
getAvailableStreamReq  =
    let req = makeRequest "RemoteTech" "get_Available" []
    in  makeStream req

getAvailableStream :: RPCContext (KRPCStream (Bool))
getAvailableStream  = requestStream $ getAvailableStreamReq  

{-
 - The names of the ground stations.
 -}
getGroundStations :: RPCContext ([Data.Text.Text])
getGroundStations  = do
    let r = makeRequest "RemoteTech" "get_GroundStations" []
    res <- sendRequest r
    processResponse res

getGroundStationsStreamReq :: KRPCStreamReq ([Data.Text.Text])
getGroundStationsStreamReq  =
    let req = makeRequest "RemoteTech" "get_GroundStations" []
    in  makeStream req

getGroundStationsStream :: RPCContext (KRPCStream ([Data.Text.Text]))
getGroundStationsStream  = requestStream $ getGroundStationsStreamReq  

