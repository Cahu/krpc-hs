module KRPCHS.RemoteTech
( Target(..)
, Antenna
, Comms
, antenna
, antennaReq
, antennaStream
, antennaStreamReq
, getAntennaHasConnection
, getAntennaHasConnectionReq
, getAntennaHasConnectionStream
, getAntennaHasConnectionStreamReq
, getAntennaPart
, getAntennaPartReq
, getAntennaPartStream
, getAntennaPartStreamReq
, getAntennaTarget
, getAntennaTargetReq
, getAntennaTargetStream
, getAntennaTargetStreamReq
, getAntennaTargetBody
, getAntennaTargetBodyReq
, getAntennaTargetBodyStream
, getAntennaTargetBodyStreamReq
, getAntennaTargetGroundStation
, getAntennaTargetGroundStationReq
, getAntennaTargetGroundStationStream
, getAntennaTargetGroundStationStreamReq
, getAntennaTargetVessel
, getAntennaTargetVesselReq
, getAntennaTargetVesselStream
, getAntennaTargetVesselStreamReq
, setAntennaTarget
, setAntennaTargetReq
, setAntennaTargetBody
, setAntennaTargetBodyReq
, setAntennaTargetGroundStation
, setAntennaTargetGroundStationReq
, setAntennaTargetVessel
, setAntennaTargetVesselReq
, comms
, commsReq
, commsStream
, commsStreamReq
, commsSignalDelayToVessel
, commsSignalDelayToVesselReq
, commsSignalDelayToVesselStream
, commsSignalDelayToVesselStreamReq
, getCommsAntennas
, getCommsAntennasReq
, getCommsAntennasStream
, getCommsAntennasStreamReq
, getCommsHasConnection
, getCommsHasConnectionReq
, getCommsHasConnectionStream
, getCommsHasConnectionStreamReq
, getCommsHasConnectionToGroundStation
, getCommsHasConnectionToGroundStationReq
, getCommsHasConnectionToGroundStationStream
, getCommsHasConnectionToGroundStationStreamReq
, getCommsHasFlightComputer
, getCommsHasFlightComputerReq
, getCommsHasFlightComputerStream
, getCommsHasFlightComputerStreamReq
, getCommsHasLocalControl
, getCommsHasLocalControlReq
, getCommsHasLocalControlStream
, getCommsHasLocalControlStreamReq
, getCommsSignalDelay
, getCommsSignalDelayReq
, getCommsSignalDelayStream
, getCommsSignalDelayStreamReq
, getCommsSignalDelayToGroundStation
, getCommsSignalDelayToGroundStationReq
, getCommsSignalDelayToGroundStationStream
, getCommsSignalDelayToGroundStationStreamReq
, getCommsVessel
, getCommsVesselReq
, getCommsVesselStream
, getCommsVesselStreamReq
, getGroundStations
, getGroundStationsReq
, getGroundStationsStream
, getGroundStationsStreamReq
) where

import qualified Data.Text
import qualified KRPCHS.SpaceCenter

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


{-|
A RemoteTech antenna. Obtained by calling <see cref="M:RemoteTech.Comms.Antennas" /> or  <see cref="M:RemoteTech.Antenna" />.
 -}
newtype Antenna = Antenna { antennaId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Antenna where
    encodePb = encodePb . antennaId

instance PbDecodable Antenna where
    decodePb b = Antenna <$> decodePb b

instance KRPCResponseExtractable Antenna

{-|
Communications for a vessel.
 -}
newtype Comms = Comms { commsId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Comms where
    encodePb = encodePb . commsId

instance PbDecodable Comms where
    decodePb b = Comms <$> decodePb b

instance KRPCResponseExtractable Comms


{-|
The type of object an antenna is targetting.
See <see cref="M:RemoteTech.Antenna.Target" />.
 -}
data Target
    = Target'ActiveVessel
    | Target'CelestialBody
    | Target'GroundStation
    | Target'Vessel
    | Target'None
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable Target where
    encodePb = encodePb . fromEnum

instance PbDecodable Target where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable Target


{-|
Get the antenna object for a particular part.
 -}
antennaReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.RemoteTech.Antenna)
antennaReq partArg = makeCallReq "RemoteTech" "Antenna" [makeArgument 0 partArg]

antenna :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.RemoteTech.Antenna)
antenna partArg = simpleRequest $ antennaReq partArg

antennaStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.RemoteTech.Antenna)
antennaStreamReq partArg = makeStreamReq $ antennaReq partArg

antennaStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Antenna))
antennaStream partArg = requestAddStream $ antennaStreamReq partArg 

{-|
Whether the antenna has a connection.
 -}
getAntennaHasConnectionReq :: KRPCHS.RemoteTech.Antenna -> KRPCCallReq (Bool)
getAntennaHasConnectionReq thisArg = makeCallReq "RemoteTech" "Antenna_get_HasConnection" [makeArgument 0 thisArg]

getAntennaHasConnection :: KRPCHS.RemoteTech.Antenna -> RPCContext (Bool)
getAntennaHasConnection thisArg = simpleRequest $ getAntennaHasConnectionReq thisArg

getAntennaHasConnectionStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (Bool)
getAntennaHasConnectionStreamReq thisArg = makeStreamReq $ getAntennaHasConnectionReq thisArg

getAntennaHasConnectionStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (Bool))
getAntennaHasConnectionStream thisArg = requestAddStream $ getAntennaHasConnectionStreamReq thisArg 

{-|
Get the part containing this antenna.
 -}
getAntennaPartReq :: KRPCHS.RemoteTech.Antenna -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getAntennaPartReq thisArg = makeCallReq "RemoteTech" "Antenna_get_Part" [makeArgument 0 thisArg]

getAntennaPart :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.Part)
getAntennaPart thisArg = simpleRequest $ getAntennaPartReq thisArg

getAntennaPartStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getAntennaPartStreamReq thisArg = makeStreamReq $ getAntennaPartReq thisArg

getAntennaPartStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getAntennaPartStream thisArg = requestAddStream $ getAntennaPartStreamReq thisArg 

{-|
The object that the antenna is targetting.
This property can be used to set the target to <see cref="M:RemoteTech.Target.None" /> or <see cref="M:RemoteTech.Target.ActiveVessel" />.
To set the target to a celestial body, ground station or vessel see <see cref="M:RemoteTech.Antenna.TargetBody" />,
<see cref="M:RemoteTech.Antenna.TargetGroundStation" /> and <see cref="M:RemoteTech.Antenna.TargetVessel" />.
 -}
getAntennaTargetReq :: KRPCHS.RemoteTech.Antenna -> KRPCCallReq (KRPCHS.RemoteTech.Target)
getAntennaTargetReq thisArg = makeCallReq "RemoteTech" "Antenna_get_Target" [makeArgument 0 thisArg]

getAntennaTarget :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.RemoteTech.Target)
getAntennaTarget thisArg = simpleRequest $ getAntennaTargetReq thisArg

getAntennaTargetStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.RemoteTech.Target)
getAntennaTargetStreamReq thisArg = makeStreamReq $ getAntennaTargetReq thisArg

getAntennaTargetStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Target))
getAntennaTargetStream thisArg = requestAddStream $ getAntennaTargetStreamReq thisArg 

{-|
The celestial body the antenna is targetting.
 -}
getAntennaTargetBodyReq :: KRPCHS.RemoteTech.Antenna -> KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getAntennaTargetBodyReq thisArg = makeCallReq "RemoteTech" "Antenna_get_TargetBody" [makeArgument 0 thisArg]

getAntennaTargetBody :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getAntennaTargetBody thisArg = simpleRequest $ getAntennaTargetBodyReq thisArg

getAntennaTargetBodyStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getAntennaTargetBodyStreamReq thisArg = makeStreamReq $ getAntennaTargetBodyReq thisArg

getAntennaTargetBodyStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getAntennaTargetBodyStream thisArg = requestAddStream $ getAntennaTargetBodyStreamReq thisArg 

{-|
The ground station the antenna is targetting.
 -}
getAntennaTargetGroundStationReq :: KRPCHS.RemoteTech.Antenna -> KRPCCallReq (Data.Text.Text)
getAntennaTargetGroundStationReq thisArg = makeCallReq "RemoteTech" "Antenna_get_TargetGroundStation" [makeArgument 0 thisArg]

getAntennaTargetGroundStation :: KRPCHS.RemoteTech.Antenna -> RPCContext (Data.Text.Text)
getAntennaTargetGroundStation thisArg = simpleRequest $ getAntennaTargetGroundStationReq thisArg

getAntennaTargetGroundStationStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (Data.Text.Text)
getAntennaTargetGroundStationStreamReq thisArg = makeStreamReq $ getAntennaTargetGroundStationReq thisArg

getAntennaTargetGroundStationStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (Data.Text.Text))
getAntennaTargetGroundStationStream thisArg = requestAddStream $ getAntennaTargetGroundStationStreamReq thisArg 

{-|
The vessel the antenna is targetting.
 -}
getAntennaTargetVesselReq :: KRPCHS.RemoteTech.Antenna -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getAntennaTargetVesselReq thisArg = makeCallReq "RemoteTech" "Antenna_get_TargetVessel" [makeArgument 0 thisArg]

getAntennaTargetVessel :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getAntennaTargetVessel thisArg = simpleRequest $ getAntennaTargetVesselReq thisArg

getAntennaTargetVesselStreamReq :: KRPCHS.RemoteTech.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getAntennaTargetVesselStreamReq thisArg = makeStreamReq $ getAntennaTargetVesselReq thisArg

getAntennaTargetVesselStream :: KRPCHS.RemoteTech.Antenna -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getAntennaTargetVesselStream thisArg = requestAddStream $ getAntennaTargetVesselStreamReq thisArg 

{-|
The object that the antenna is targetting.
This property can be used to set the target to <see cref="M:RemoteTech.Target.None" /> or <see cref="M:RemoteTech.Target.ActiveVessel" />.
To set the target to a celestial body, ground station or vessel see <see cref="M:RemoteTech.Antenna.TargetBody" />,
<see cref="M:RemoteTech.Antenna.TargetGroundStation" /> and <see cref="M:RemoteTech.Antenna.TargetVessel" />.
 -}
setAntennaTargetReq :: KRPCHS.RemoteTech.Antenna -> KRPCHS.RemoteTech.Target -> KRPCCallReq ()
setAntennaTargetReq thisArg valueArg = makeCallReq "RemoteTech" "Antenna_set_Target" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAntennaTarget :: KRPCHS.RemoteTech.Antenna -> KRPCHS.RemoteTech.Target -> RPCContext ()
setAntennaTarget thisArg valueArg = simpleRequest $ setAntennaTargetReq thisArg valueArg 

{-|
The celestial body the antenna is targetting.
 -}
setAntennaTargetBodyReq :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ()
setAntennaTargetBodyReq thisArg valueArg = makeCallReq "RemoteTech" "Antenna_set_TargetBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAntennaTargetBody :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setAntennaTargetBody thisArg valueArg = simpleRequest $ setAntennaTargetBodyReq thisArg valueArg 

{-|
The ground station the antenna is targetting.
 -}
setAntennaTargetGroundStationReq :: KRPCHS.RemoteTech.Antenna -> Data.Text.Text -> KRPCCallReq ()
setAntennaTargetGroundStationReq thisArg valueArg = makeCallReq "RemoteTech" "Antenna_set_TargetGroundStation" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAntennaTargetGroundStation :: KRPCHS.RemoteTech.Antenna -> Data.Text.Text -> RPCContext ()
setAntennaTargetGroundStation thisArg valueArg = simpleRequest $ setAntennaTargetGroundStationReq thisArg valueArg 

{-|
The vessel the antenna is targetting.
 -}
setAntennaTargetVesselReq :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ()
setAntennaTargetVesselReq thisArg valueArg = makeCallReq "RemoteTech" "Antenna_set_TargetVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAntennaTargetVessel :: KRPCHS.RemoteTech.Antenna -> KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setAntennaTargetVessel thisArg valueArg = simpleRequest $ setAntennaTargetVesselReq thisArg valueArg 

{-|
Get a communications object, representing the communication capability of a particular vessel.
 -}
commsReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.RemoteTech.Comms)
commsReq vesselArg = makeCallReq "RemoteTech" "Comms" [makeArgument 0 vesselArg]

comms :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.RemoteTech.Comms)
comms vesselArg = simpleRequest $ commsReq vesselArg

commsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.RemoteTech.Comms)
commsStreamReq vesselArg = makeStreamReq $ commsReq vesselArg

commsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.RemoteTech.Comms))
commsStream vesselArg = requestAddStream $ commsStreamReq vesselArg 

{-|
The signal delay between the this vessel and another vessel, in seconds.<param name="other">
 -}
commsSignalDelayToVesselReq :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Double)
commsSignalDelayToVesselReq thisArg otherArg = makeCallReq "RemoteTech" "Comms_SignalDelayToVessel" [makeArgument 0 thisArg, makeArgument 1 otherArg]

commsSignalDelayToVessel :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> RPCContext (Double)
commsSignalDelayToVessel thisArg otherArg = simpleRequest $ commsSignalDelayToVesselReq thisArg otherArg

commsSignalDelayToVesselStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Double)
commsSignalDelayToVesselStreamReq thisArg otherArg = makeStreamReq $ commsSignalDelayToVesselReq thisArg otherArg

commsSignalDelayToVesselStream :: KRPCHS.RemoteTech.Comms -> KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Double))
commsSignalDelayToVesselStream thisArg otherArg = requestAddStream $ commsSignalDelayToVesselStreamReq thisArg otherArg 

{-|
The antennas for this vessel.
 -}
getCommsAntennasReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq ([KRPCHS.RemoteTech.Antenna])
getCommsAntennasReq thisArg = makeCallReq "RemoteTech" "Comms_get_Antennas" [makeArgument 0 thisArg]

getCommsAntennas :: KRPCHS.RemoteTech.Comms -> RPCContext ([KRPCHS.RemoteTech.Antenna])
getCommsAntennas thisArg = simpleRequest $ getCommsAntennasReq thisArg

getCommsAntennasStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq ([KRPCHS.RemoteTech.Antenna])
getCommsAntennasStreamReq thisArg = makeStreamReq $ getCommsAntennasReq thisArg

getCommsAntennasStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream ([KRPCHS.RemoteTech.Antenna]))
getCommsAntennasStream thisArg = requestAddStream $ getCommsAntennasStreamReq thisArg 

{-|
Whether the vessel has any connection.
 -}
getCommsHasConnectionReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (Bool)
getCommsHasConnectionReq thisArg = makeCallReq "RemoteTech" "Comms_get_HasConnection" [makeArgument 0 thisArg]

getCommsHasConnection :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasConnection thisArg = simpleRequest $ getCommsHasConnectionReq thisArg

getCommsHasConnectionStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasConnectionStreamReq thisArg = makeStreamReq $ getCommsHasConnectionReq thisArg

getCommsHasConnectionStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionStream thisArg = requestAddStream $ getCommsHasConnectionStreamReq thisArg 

{-|
Whether the vessel has a connection to a ground station.
 -}
getCommsHasConnectionToGroundStationReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (Bool)
getCommsHasConnectionToGroundStationReq thisArg = makeCallReq "RemoteTech" "Comms_get_HasConnectionToGroundStation" [makeArgument 0 thisArg]

getCommsHasConnectionToGroundStation :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasConnectionToGroundStation thisArg = simpleRequest $ getCommsHasConnectionToGroundStationReq thisArg

getCommsHasConnectionToGroundStationStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasConnectionToGroundStationStreamReq thisArg = makeStreamReq $ getCommsHasConnectionToGroundStationReq thisArg

getCommsHasConnectionToGroundStationStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionToGroundStationStream thisArg = requestAddStream $ getCommsHasConnectionToGroundStationStreamReq thisArg 

{-|
Whether the vessel has a flight computer on board.
 -}
getCommsHasFlightComputerReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (Bool)
getCommsHasFlightComputerReq thisArg = makeCallReq "RemoteTech" "Comms_get_HasFlightComputer" [makeArgument 0 thisArg]

getCommsHasFlightComputer :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasFlightComputer thisArg = simpleRequest $ getCommsHasFlightComputerReq thisArg

getCommsHasFlightComputerStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasFlightComputerStreamReq thisArg = makeStreamReq $ getCommsHasFlightComputerReq thisArg

getCommsHasFlightComputerStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasFlightComputerStream thisArg = requestAddStream $ getCommsHasFlightComputerStreamReq thisArg 

{-|
Whether the vessel can be controlled locally.
 -}
getCommsHasLocalControlReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (Bool)
getCommsHasLocalControlReq thisArg = makeCallReq "RemoteTech" "Comms_get_HasLocalControl" [makeArgument 0 thisArg]

getCommsHasLocalControl :: KRPCHS.RemoteTech.Comms -> RPCContext (Bool)
getCommsHasLocalControl thisArg = simpleRequest $ getCommsHasLocalControlReq thisArg

getCommsHasLocalControlStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Bool)
getCommsHasLocalControlStreamReq thisArg = makeStreamReq $ getCommsHasLocalControlReq thisArg

getCommsHasLocalControlStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Bool))
getCommsHasLocalControlStream thisArg = requestAddStream $ getCommsHasLocalControlStreamReq thisArg 

{-|
The shortest signal delay to the vessel, in seconds.
 -}
getCommsSignalDelayReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (Double)
getCommsSignalDelayReq thisArg = makeCallReq "RemoteTech" "Comms_get_SignalDelay" [makeArgument 0 thisArg]

getCommsSignalDelay :: KRPCHS.RemoteTech.Comms -> RPCContext (Double)
getCommsSignalDelay thisArg = simpleRequest $ getCommsSignalDelayReq thisArg

getCommsSignalDelayStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Double)
getCommsSignalDelayStreamReq thisArg = makeStreamReq $ getCommsSignalDelayReq thisArg

getCommsSignalDelayStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayStream thisArg = requestAddStream $ getCommsSignalDelayStreamReq thisArg 

{-|
The signal delay between the vessel and the closest ground station, in seconds.
 -}
getCommsSignalDelayToGroundStationReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (Double)
getCommsSignalDelayToGroundStationReq thisArg = makeCallReq "RemoteTech" "Comms_get_SignalDelayToGroundStation" [makeArgument 0 thisArg]

getCommsSignalDelayToGroundStation :: KRPCHS.RemoteTech.Comms -> RPCContext (Double)
getCommsSignalDelayToGroundStation thisArg = simpleRequest $ getCommsSignalDelayToGroundStationReq thisArg

getCommsSignalDelayToGroundStationStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (Double)
getCommsSignalDelayToGroundStationStreamReq thisArg = makeStreamReq $ getCommsSignalDelayToGroundStationReq thisArg

getCommsSignalDelayToGroundStationStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayToGroundStationStream thisArg = requestAddStream $ getCommsSignalDelayToGroundStationStreamReq thisArg 

{-|
Get the vessel.
 -}
getCommsVesselReq :: KRPCHS.RemoteTech.Comms -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getCommsVesselReq thisArg = makeCallReq "RemoteTech" "Comms_get_Vessel" [makeArgument 0 thisArg]

getCommsVessel :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getCommsVessel thisArg = simpleRequest $ getCommsVesselReq thisArg

getCommsVesselStreamReq :: KRPCHS.RemoteTech.Comms -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getCommsVesselStreamReq thisArg = makeStreamReq $ getCommsVesselReq thisArg

getCommsVesselStream :: KRPCHS.RemoteTech.Comms -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCommsVesselStream thisArg = requestAddStream $ getCommsVesselStreamReq thisArg 

{-|
The names of the ground stations.
 -}
getGroundStationsReq :: KRPCCallReq ([Data.Text.Text])
getGroundStationsReq  = makeCallReq "RemoteTech" "get_GroundStations" []

getGroundStations :: RPCContext ([Data.Text.Text])
getGroundStations  = simpleRequest $ getGroundStationsReq 

getGroundStationsStreamReq :: KRPCStreamReq ([Data.Text.Text])
getGroundStationsStreamReq  = makeStreamReq $ getGroundStationsReq 

getGroundStationsStream :: RPCContext (KRPCStream ([Data.Text.Text]))
getGroundStationsStream  = requestAddStream $ getGroundStationsStreamReq  

