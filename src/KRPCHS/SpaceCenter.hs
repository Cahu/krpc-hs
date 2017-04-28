module KRPCHS.SpaceCenter
( AntennaState(..)
, CameraMode(..)
, CargoBayState(..)
, CommLinkType(..)
, ContractState(..)
, ControlSource(..)
, ControlState(..)
, DockingPortState(..)
, LegState(..)
, MotorState(..)
, ParachuteState(..)
, RadiatorState(..)
, ResourceConverterState(..)
, ResourceFlowMode(..)
, ResourceHarvesterState(..)
, SASMode(..)
, SolarPanelState(..)
, SpeedMode(..)
, VesselSituation(..)
, VesselType(..)
, WarpMode(..)
, WheelState(..)
, Antenna
, AutoPilot
, Camera
, CargoBay
, CelestialBody
, CommLink
, CommNode
, Comms
, Contract
, ContractManager
, ContractParameter
, Control
, ControlSurface
, Decoupler
, DockingPort
, Engine
, Experiment
, Fairing
, Flight
, Force
, Intake
, LaunchClamp
, Leg
, Light
, Module
, Node
, Orbit
, Parachute
, Part
, Parts
, Propellant
, RCS
, Radiator
, ReactionWheel
, ReferenceFrame
, Resource
, ResourceConverter
, ResourceHarvester
, ResourceTransfer
, Resources
, ScienceData
, ScienceSubject
, Sensor
, SolarPanel
, Thruster
, Vessel
, Waypoint
, WaypointManager
, Wheel
, antennaCancel
, antennaTransmit
, getAntennaAllowPartial
, getAntennaAllowPartialStream
, getAntennaAllowPartialStreamReq
, getAntennaCanTransmit
, getAntennaCanTransmitStream
, getAntennaCanTransmitStreamReq
, getAntennaCombinable
, getAntennaCombinableStream
, getAntennaCombinableStreamReq
, getAntennaCombinableExponent
, getAntennaCombinableExponentStream
, getAntennaCombinableExponentStreamReq
, getAntennaDeployable
, getAntennaDeployableStream
, getAntennaDeployableStreamReq
, getAntennaDeployed
, getAntennaDeployedStream
, getAntennaDeployedStreamReq
, getAntennaPacketInterval
, getAntennaPacketIntervalStream
, getAntennaPacketIntervalStreamReq
, getAntennaPacketResourceCost
, getAntennaPacketResourceCostStream
, getAntennaPacketResourceCostStreamReq
, getAntennaPacketSize
, getAntennaPacketSizeStream
, getAntennaPacketSizeStreamReq
, getAntennaPart
, getAntennaPartStream
, getAntennaPartStreamReq
, getAntennaPower
, getAntennaPowerStream
, getAntennaPowerStreamReq
, getAntennaState
, getAntennaStateStream
, getAntennaStateStreamReq
, setAntennaAllowPartial
, setAntennaDeployed
, autoPilotDisengage
, autoPilotEngage
, autoPilotTargetPitchAndHeading
, autoPilotWait
, getAutoPilotAttenuationAngle
, getAutoPilotAttenuationAngleStream
, getAutoPilotAttenuationAngleStreamReq
, getAutoPilotAutoTune
, getAutoPilotAutoTuneStream
, getAutoPilotAutoTuneStreamReq
, getAutoPilotDecelerationTime
, getAutoPilotDecelerationTimeStream
, getAutoPilotDecelerationTimeStreamReq
, getAutoPilotError
, getAutoPilotErrorStream
, getAutoPilotErrorStreamReq
, getAutoPilotHeadingError
, getAutoPilotHeadingErrorStream
, getAutoPilotHeadingErrorStreamReq
, getAutoPilotOvershoot
, getAutoPilotOvershootStream
, getAutoPilotOvershootStreamReq
, getAutoPilotPitchError
, getAutoPilotPitchErrorStream
, getAutoPilotPitchErrorStreamReq
, getAutoPilotPitchPIDGains
, getAutoPilotPitchPIDGainsStream
, getAutoPilotPitchPIDGainsStreamReq
, getAutoPilotReferenceFrame
, getAutoPilotReferenceFrameStream
, getAutoPilotReferenceFrameStreamReq
, getAutoPilotRollError
, getAutoPilotRollErrorStream
, getAutoPilotRollErrorStreamReq
, getAutoPilotRollPIDGains
, getAutoPilotRollPIDGainsStream
, getAutoPilotRollPIDGainsStreamReq
, getAutoPilotRollThreshold
, getAutoPilotRollThresholdStream
, getAutoPilotRollThresholdStreamReq
, getAutoPilotSAS
, getAutoPilotSASStream
, getAutoPilotSASStreamReq
, getAutoPilotSASMode
, getAutoPilotSASModeStream
, getAutoPilotSASModeStreamReq
, getAutoPilotStoppingTime
, getAutoPilotStoppingTimeStream
, getAutoPilotStoppingTimeStreamReq
, getAutoPilotTargetDirection
, getAutoPilotTargetDirectionStream
, getAutoPilotTargetDirectionStreamReq
, getAutoPilotTargetHeading
, getAutoPilotTargetHeadingStream
, getAutoPilotTargetHeadingStreamReq
, getAutoPilotTargetPitch
, getAutoPilotTargetPitchStream
, getAutoPilotTargetPitchStreamReq
, getAutoPilotTargetRoll
, getAutoPilotTargetRollStream
, getAutoPilotTargetRollStreamReq
, getAutoPilotTimeToPeak
, getAutoPilotTimeToPeakStream
, getAutoPilotTimeToPeakStreamReq
, getAutoPilotYawPIDGains
, getAutoPilotYawPIDGainsStream
, getAutoPilotYawPIDGainsStreamReq
, setAutoPilotAttenuationAngle
, setAutoPilotAutoTune
, setAutoPilotDecelerationTime
, setAutoPilotOvershoot
, setAutoPilotPitchPIDGains
, setAutoPilotReferenceFrame
, setAutoPilotRollPIDGains
, setAutoPilotRollThreshold
, setAutoPilotSAS
, setAutoPilotSASMode
, setAutoPilotStoppingTime
, setAutoPilotTargetDirection
, setAutoPilotTargetHeading
, setAutoPilotTargetPitch
, setAutoPilotTargetRoll
, setAutoPilotTimeToPeak
, setAutoPilotYawPIDGains
, getCameraDefaultDistance
, getCameraDefaultDistanceStream
, getCameraDefaultDistanceStreamReq
, getCameraDistance
, getCameraDistanceStream
, getCameraDistanceStreamReq
, getCameraFocussedBody
, getCameraFocussedBodyStream
, getCameraFocussedBodyStreamReq
, getCameraFocussedNode
, getCameraFocussedNodeStream
, getCameraFocussedNodeStreamReq
, getCameraFocussedVessel
, getCameraFocussedVesselStream
, getCameraFocussedVesselStreamReq
, getCameraHeading
, getCameraHeadingStream
, getCameraHeadingStreamReq
, getCameraMaxDistance
, getCameraMaxDistanceStream
, getCameraMaxDistanceStreamReq
, getCameraMaxPitch
, getCameraMaxPitchStream
, getCameraMaxPitchStreamReq
, getCameraMinDistance
, getCameraMinDistanceStream
, getCameraMinDistanceStreamReq
, getCameraMinPitch
, getCameraMinPitchStream
, getCameraMinPitchStreamReq
, getCameraMode
, getCameraModeStream
, getCameraModeStreamReq
, getCameraPitch
, getCameraPitchStream
, getCameraPitchStreamReq
, setCameraDistance
, setCameraFocussedBody
, setCameraFocussedNode
, setCameraFocussedVessel
, setCameraHeading
, setCameraMode
, setCameraPitch
, canRailsWarpAt
, canRailsWarpAtStream
, canRailsWarpAtStreamReq
, getCargoBayOpen
, getCargoBayOpenStream
, getCargoBayOpenStreamReq
, getCargoBayPart
, getCargoBayPartStream
, getCargoBayPartStreamReq
, getCargoBayState
, getCargoBayStateStream
, getCargoBayStateStreamReq
, setCargoBayOpen
, celestialBodyAngularVelocity
, celestialBodyAngularVelocityStream
, celestialBodyAngularVelocityStreamReq
, celestialBodyBedrockHeight
, celestialBodyBedrockHeightStream
, celestialBodyBedrockHeightStreamReq
, celestialBodyBedrockPosition
, celestialBodyBedrockPositionStream
, celestialBodyBedrockPositionStreamReq
, celestialBodyBiomeAt
, celestialBodyBiomeAtStream
, celestialBodyBiomeAtStreamReq
, celestialBodyDirection
, celestialBodyDirectionStream
, celestialBodyDirectionStreamReq
, celestialBodyMSLPosition
, celestialBodyMSLPositionStream
, celestialBodyMSLPositionStreamReq
, celestialBodyPosition
, celestialBodyPositionStream
, celestialBodyPositionStreamReq
, celestialBodyRotation
, celestialBodyRotationStream
, celestialBodyRotationStreamReq
, celestialBodySurfaceHeight
, celestialBodySurfaceHeightStream
, celestialBodySurfaceHeightStreamReq
, celestialBodySurfacePosition
, celestialBodySurfacePositionStream
, celestialBodySurfacePositionStreamReq
, celestialBodyVelocity
, celestialBodyVelocityStream
, celestialBodyVelocityStreamReq
, getCelestialBodyAtmosphereDepth
, getCelestialBodyAtmosphereDepthStream
, getCelestialBodyAtmosphereDepthStreamReq
, getCelestialBodyBiomes
, getCelestialBodyBiomesStream
, getCelestialBodyBiomesStreamReq
, getCelestialBodyEquatorialRadius
, getCelestialBodyEquatorialRadiusStream
, getCelestialBodyEquatorialRadiusStreamReq
, getCelestialBodyFlyingHighAltitudeThreshold
, getCelestialBodyFlyingHighAltitudeThresholdStream
, getCelestialBodyFlyingHighAltitudeThresholdStreamReq
, getCelestialBodyGravitationalParameter
, getCelestialBodyGravitationalParameterStream
, getCelestialBodyGravitationalParameterStreamReq
, getCelestialBodyHasAtmosphere
, getCelestialBodyHasAtmosphereStream
, getCelestialBodyHasAtmosphereStreamReq
, getCelestialBodyHasAtmosphericOxygen
, getCelestialBodyHasAtmosphericOxygenStream
, getCelestialBodyHasAtmosphericOxygenStreamReq
, getCelestialBodyInitialRotation
, getCelestialBodyInitialRotationStream
, getCelestialBodyInitialRotationStreamReq
, getCelestialBodyMass
, getCelestialBodyMassStream
, getCelestialBodyMassStreamReq
, getCelestialBodyName
, getCelestialBodyNameStream
, getCelestialBodyNameStreamReq
, getCelestialBodyNonRotatingReferenceFrame
, getCelestialBodyNonRotatingReferenceFrameStream
, getCelestialBodyNonRotatingReferenceFrameStreamReq
, getCelestialBodyOrbit
, getCelestialBodyOrbitStream
, getCelestialBodyOrbitStreamReq
, getCelestialBodyOrbitalReferenceFrame
, getCelestialBodyOrbitalReferenceFrameStream
, getCelestialBodyOrbitalReferenceFrameStreamReq
, getCelestialBodyReferenceFrame
, getCelestialBodyReferenceFrameStream
, getCelestialBodyReferenceFrameStreamReq
, getCelestialBodyRotationAngle
, getCelestialBodyRotationAngleStream
, getCelestialBodyRotationAngleStreamReq
, getCelestialBodyRotationalPeriod
, getCelestialBodyRotationalPeriodStream
, getCelestialBodyRotationalPeriodStreamReq
, getCelestialBodyRotationalSpeed
, getCelestialBodyRotationalSpeedStream
, getCelestialBodyRotationalSpeedStreamReq
, getCelestialBodySatellites
, getCelestialBodySatellitesStream
, getCelestialBodySatellitesStreamReq
, getCelestialBodySpaceHighAltitudeThreshold
, getCelestialBodySpaceHighAltitudeThresholdStream
, getCelestialBodySpaceHighAltitudeThresholdStreamReq
, getCelestialBodySphereOfInfluence
, getCelestialBodySphereOfInfluenceStream
, getCelestialBodySphereOfInfluenceStreamReq
, getCelestialBodySurfaceGravity
, getCelestialBodySurfaceGravityStream
, getCelestialBodySurfaceGravityStreamReq
, clearTarget
, getCommLinkEnd
, getCommLinkEndStream
, getCommLinkEndStreamReq
, getCommLinkSignalStrength
, getCommLinkSignalStrengthStream
, getCommLinkSignalStrengthStreamReq
, getCommLinkStart
, getCommLinkStartStream
, getCommLinkStartStreamReq
, getCommLinkType
, getCommLinkTypeStream
, getCommLinkTypeStreamReq
, getCommNodeIsControlPoint
, getCommNodeIsControlPointStream
, getCommNodeIsControlPointStreamReq
, getCommNodeIsHome
, getCommNodeIsHomeStream
, getCommNodeIsHomeStreamReq
, getCommNodeIsVessel
, getCommNodeIsVesselStream
, getCommNodeIsVesselStreamReq
, getCommNodeName
, getCommNodeNameStream
, getCommNodeNameStreamReq
, getCommNodeVessel
, getCommNodeVesselStream
, getCommNodeVesselStreamReq
, getCommsCanCommunicate
, getCommsCanCommunicateStream
, getCommsCanCommunicateStreamReq
, getCommsCanTransmitScience
, getCommsCanTransmitScienceStream
, getCommsCanTransmitScienceStreamReq
, getCommsControlPath
, getCommsControlPathStream
, getCommsControlPathStreamReq
, getCommsPower
, getCommsPowerStream
, getCommsPowerStreamReq
, getCommsSignalDelay
, getCommsSignalDelayStream
, getCommsSignalDelayStreamReq
, getCommsSignalStrength
, getCommsSignalStrengthStream
, getCommsSignalStrengthStreamReq
, getContractManagerActiveContracts
, getContractManagerActiveContractsStream
, getContractManagerActiveContractsStreamReq
, getContractManagerAllContracts
, getContractManagerAllContractsStream
, getContractManagerAllContractsStreamReq
, getContractManagerCompletedContracts
, getContractManagerCompletedContractsStream
, getContractManagerCompletedContractsStreamReq
, getContractManagerFailedContracts
, getContractManagerFailedContractsStream
, getContractManagerFailedContractsStreamReq
, getContractManagerOfferedContracts
, getContractManagerOfferedContractsStream
, getContractManagerOfferedContractsStreamReq
, getContractManagerTypes
, getContractManagerTypesStream
, getContractManagerTypesStreamReq
, getContractParameterChildren
, getContractParameterChildrenStream
, getContractParameterChildrenStreamReq
, getContractParameterCompleted
, getContractParameterCompletedStream
, getContractParameterCompletedStreamReq
, getContractParameterFailed
, getContractParameterFailedStream
, getContractParameterFailedStreamReq
, getContractParameterFundsCompletion
, getContractParameterFundsCompletionStream
, getContractParameterFundsCompletionStreamReq
, getContractParameterFundsFailure
, getContractParameterFundsFailureStream
, getContractParameterFundsFailureStreamReq
, getContractParameterNotes
, getContractParameterNotesStream
, getContractParameterNotesStreamReq
, getContractParameterOptional
, getContractParameterOptionalStream
, getContractParameterOptionalStreamReq
, getContractParameterReputationCompletion
, getContractParameterReputationCompletionStream
, getContractParameterReputationCompletionStreamReq
, getContractParameterReputationFailure
, getContractParameterReputationFailureStream
, getContractParameterReputationFailureStreamReq
, getContractParameterScienceCompletion
, getContractParameterScienceCompletionStream
, getContractParameterScienceCompletionStreamReq
, getContractParameterTitle
, getContractParameterTitleStream
, getContractParameterTitleStreamReq
, contractAccept
, contractCancel
, contractDecline
, getContractActive
, getContractActiveStream
, getContractActiveStreamReq
, getContractCanBeCanceled
, getContractCanBeCanceledStream
, getContractCanBeCanceledStreamReq
, getContractCanBeDeclined
, getContractCanBeDeclinedStream
, getContractCanBeDeclinedStreamReq
, getContractCanBeFailed
, getContractCanBeFailedStream
, getContractCanBeFailedStreamReq
, getContractDescription
, getContractDescriptionStream
, getContractDescriptionStreamReq
, getContractFailed
, getContractFailedStream
, getContractFailedStreamReq
, getContractFundsAdvance
, getContractFundsAdvanceStream
, getContractFundsAdvanceStreamReq
, getContractFundsCompletion
, getContractFundsCompletionStream
, getContractFundsCompletionStreamReq
, getContractFundsFailure
, getContractFundsFailureStream
, getContractFundsFailureStreamReq
, getContractKeywords
, getContractKeywordsStream
, getContractKeywordsStreamReq
, getContractNotes
, getContractNotesStream
, getContractNotesStreamReq
, getContractParameters
, getContractParametersStream
, getContractParametersStreamReq
, getContractRead
, getContractReadStream
, getContractReadStreamReq
, getContractReputationCompletion
, getContractReputationCompletionStream
, getContractReputationCompletionStreamReq
, getContractReputationFailure
, getContractReputationFailureStream
, getContractReputationFailureStreamReq
, getContractScienceCompletion
, getContractScienceCompletionStream
, getContractScienceCompletionStreamReq
, getContractSeen
, getContractSeenStream
, getContractSeenStreamReq
, getContractState
, getContractStateStream
, getContractStateStreamReq
, getContractSynopsis
, getContractSynopsisStream
, getContractSynopsisStreamReq
, getContractTitle
, getContractTitleStream
, getContractTitleStreamReq
, getContractType
, getContractTypeStream
, getContractTypeStreamReq
, getControlSurfaceAuthorityLimiter
, getControlSurfaceAuthorityLimiterStream
, getControlSurfaceAuthorityLimiterStreamReq
, getControlSurfaceAvailableTorque
, getControlSurfaceAvailableTorqueStream
, getControlSurfaceAvailableTorqueStreamReq
, getControlSurfaceDeployed
, getControlSurfaceDeployedStream
, getControlSurfaceDeployedStreamReq
, getControlSurfaceInverted
, getControlSurfaceInvertedStream
, getControlSurfaceInvertedStreamReq
, getControlSurfacePart
, getControlSurfacePartStream
, getControlSurfacePartStreamReq
, getControlSurfacePitchEnabled
, getControlSurfacePitchEnabledStream
, getControlSurfacePitchEnabledStreamReq
, getControlSurfaceRollEnabled
, getControlSurfaceRollEnabledStream
, getControlSurfaceRollEnabledStreamReq
, getControlSurfaceSurfaceArea
, getControlSurfaceSurfaceAreaStream
, getControlSurfaceSurfaceAreaStreamReq
, getControlSurfaceYawEnabled
, getControlSurfaceYawEnabledStream
, getControlSurfaceYawEnabledStreamReq
, setControlSurfaceAuthorityLimiter
, setControlSurfaceDeployed
, setControlSurfaceInverted
, setControlSurfacePitchEnabled
, setControlSurfaceRollEnabled
, setControlSurfaceYawEnabled
, controlActivateNextStage
, controlActivateNextStageStream
, controlActivateNextStageStreamReq
, controlAddNode
, controlAddNodeStream
, controlAddNodeStreamReq
, controlGetActionGroup
, controlGetActionGroupStream
, controlGetActionGroupStreamReq
, controlRemoveNodes
, controlSetActionGroup
, controlToggleActionGroup
, getControlAbort
, getControlAbortStream
, getControlAbortStreamReq
, getControlAntennas
, getControlAntennasStream
, getControlAntennasStreamReq
, getControlBrakes
, getControlBrakesStream
, getControlBrakesStreamReq
, getControlCargoBays
, getControlCargoBaysStream
, getControlCargoBaysStreamReq
, getControlCurrentStage
, getControlCurrentStageStream
, getControlCurrentStageStreamReq
, getControlForward
, getControlForwardStream
, getControlForwardStreamReq
, getControlGear
, getControlGearStream
, getControlGearStreamReq
, getControlIntakes
, getControlIntakesStream
, getControlIntakesStreamReq
, getControlLegs
, getControlLegsStream
, getControlLegsStreamReq
, getControlLights
, getControlLightsStream
, getControlLightsStreamReq
, getControlNodes
, getControlNodesStream
, getControlNodesStreamReq
, getControlParachutes
, getControlParachutesStream
, getControlParachutesStreamReq
, getControlPitch
, getControlPitchStream
, getControlPitchStreamReq
, getControlRCS
, getControlRCSStream
, getControlRCSStreamReq
, getControlRadiators
, getControlRadiatorsStream
, getControlRadiatorsStreamReq
, getControlReactionWheels
, getControlReactionWheelsStream
, getControlReactionWheelsStreamReq
, getControlResourceHarvesters
, getControlResourceHarvestersStream
, getControlResourceHarvestersStreamReq
, getControlResourceHarvestersActive
, getControlResourceHarvestersActiveStream
, getControlResourceHarvestersActiveStreamReq
, getControlRight
, getControlRightStream
, getControlRightStreamReq
, getControlRoll
, getControlRollStream
, getControlRollStreamReq
, getControlSAS
, getControlSASStream
, getControlSASStreamReq
, getControlSASMode
, getControlSASModeStream
, getControlSASModeStreamReq
, getControlSolarPanels
, getControlSolarPanelsStream
, getControlSolarPanelsStreamReq
, getControlSource
, getControlSourceStream
, getControlSourceStreamReq
, getControlSpeedMode
, getControlSpeedModeStream
, getControlSpeedModeStreamReq
, getControlState
, getControlStateStream
, getControlStateStreamReq
, getControlThrottle
, getControlThrottleStream
, getControlThrottleStreamReq
, getControlUp
, getControlUpStream
, getControlUpStreamReq
, getControlWheelSteering
, getControlWheelSteeringStream
, getControlWheelSteeringStreamReq
, getControlWheelThrottle
, getControlWheelThrottleStream
, getControlWheelThrottleStreamReq
, getControlWheels
, getControlWheelsStream
, getControlWheelsStreamReq
, getControlYaw
, getControlYawStream
, getControlYawStreamReq
, setControlAbort
, setControlAntennas
, setControlBrakes
, setControlCargoBays
, setControlForward
, setControlGear
, setControlIntakes
, setControlLegs
, setControlLights
, setControlParachutes
, setControlPitch
, setControlRCS
, setControlRadiators
, setControlReactionWheels
, setControlResourceHarvesters
, setControlResourceHarvestersActive
, setControlRight
, setControlRoll
, setControlSAS
, setControlSASMode
, setControlSolarPanels
, setControlSpeedMode
, setControlThrottle
, setControlUp
, setControlWheelSteering
, setControlWheelThrottle
, setControlWheels
, setControlYaw
, decouplerDecouple
, decouplerDecoupleStream
, decouplerDecoupleStreamReq
, getDecouplerDecoupled
, getDecouplerDecoupledStream
, getDecouplerDecoupledStreamReq
, getDecouplerImpulse
, getDecouplerImpulseStream
, getDecouplerImpulseStreamReq
, getDecouplerPart
, getDecouplerPartStream
, getDecouplerPartStreamReq
, getDecouplerStaged
, getDecouplerStagedStream
, getDecouplerStagedStreamReq
, dockingPortDirection
, dockingPortDirectionStream
, dockingPortDirectionStreamReq
, dockingPortPosition
, dockingPortPositionStream
, dockingPortPositionStreamReq
, dockingPortRotation
, dockingPortRotationStream
, dockingPortRotationStreamReq
, dockingPortUndock
, dockingPortUndockStream
, dockingPortUndockStreamReq
, getDockingPortDockedPart
, getDockingPortDockedPartStream
, getDockingPortDockedPartStreamReq
, getDockingPortHasShield
, getDockingPortHasShieldStream
, getDockingPortHasShieldStreamReq
, getDockingPortPart
, getDockingPortPartStream
, getDockingPortPartStreamReq
, getDockingPortReengageDistance
, getDockingPortReengageDistanceStream
, getDockingPortReengageDistanceStreamReq
, getDockingPortReferenceFrame
, getDockingPortReferenceFrameStream
, getDockingPortReferenceFrameStreamReq
, getDockingPortShielded
, getDockingPortShieldedStream
, getDockingPortShieldedStreamReq
, getDockingPortState
, getDockingPortStateStream
, getDockingPortStateStreamReq
, setDockingPortShielded
, engineToggleMode
, getEngineActive
, getEngineActiveStream
, getEngineActiveStreamReq
, getEngineAutoModeSwitch
, getEngineAutoModeSwitchStream
, getEngineAutoModeSwitchStreamReq
, getEngineAvailableThrust
, getEngineAvailableThrustStream
, getEngineAvailableThrustStreamReq
, getEngineAvailableTorque
, getEngineAvailableTorqueStream
, getEngineAvailableTorqueStreamReq
, getEngineCanRestart
, getEngineCanRestartStream
, getEngineCanRestartStreamReq
, getEngineCanShutdown
, getEngineCanShutdownStream
, getEngineCanShutdownStreamReq
, getEngineGimbalLimit
, getEngineGimbalLimitStream
, getEngineGimbalLimitStreamReq
, getEngineGimbalLocked
, getEngineGimbalLockedStream
, getEngineGimbalLockedStreamReq
, getEngineGimbalRange
, getEngineGimbalRangeStream
, getEngineGimbalRangeStreamReq
, getEngineGimballed
, getEngineGimballedStream
, getEngineGimballedStreamReq
, getEngineHasFuel
, getEngineHasFuelStream
, getEngineHasFuelStreamReq
, getEngineHasModes
, getEngineHasModesStream
, getEngineHasModesStreamReq
, getEngineKerbinSeaLevelSpecificImpulse
, getEngineKerbinSeaLevelSpecificImpulseStream
, getEngineKerbinSeaLevelSpecificImpulseStreamReq
, getEngineMaxThrust
, getEngineMaxThrustStream
, getEngineMaxThrustStreamReq
, getEngineMaxVacuumThrust
, getEngineMaxVacuumThrustStream
, getEngineMaxVacuumThrustStreamReq
, getEngineMode
, getEngineModeStream
, getEngineModeStreamReq
, getEngineModes
, getEngineModesStream
, getEngineModesStreamReq
, getEnginePart
, getEnginePartStream
, getEnginePartStreamReq
, getEnginePropellantNames
, getEnginePropellantNamesStream
, getEnginePropellantNamesStreamReq
, getEnginePropellantRatios
, getEnginePropellantRatiosStream
, getEnginePropellantRatiosStreamReq
, getEnginePropellants
, getEnginePropellantsStream
, getEnginePropellantsStreamReq
, getEngineSpecificImpulse
, getEngineSpecificImpulseStream
, getEngineSpecificImpulseStreamReq
, getEngineThrottle
, getEngineThrottleStream
, getEngineThrottleStreamReq
, getEngineThrottleLocked
, getEngineThrottleLockedStream
, getEngineThrottleLockedStreamReq
, getEngineThrust
, getEngineThrustStream
, getEngineThrustStreamReq
, getEngineThrustLimit
, getEngineThrustLimitStream
, getEngineThrustLimitStreamReq
, getEngineThrusters
, getEngineThrustersStream
, getEngineThrustersStreamReq
, getEngineVacuumSpecificImpulse
, getEngineVacuumSpecificImpulseStream
, getEngineVacuumSpecificImpulseStreamReq
, setEngineActive
, setEngineAutoModeSwitch
, setEngineGimbalLimit
, setEngineGimbalLocked
, setEngineMode
, setEngineThrustLimit
, experimentDump
, experimentReset
, experimentRun
, experimentTransmit
, getExperimentAvailable
, getExperimentAvailableStream
, getExperimentAvailableStreamReq
, getExperimentBiome
, getExperimentBiomeStream
, getExperimentBiomeStreamReq
, getExperimentData
, getExperimentDataStream
, getExperimentDataStreamReq
, getExperimentDeployed
, getExperimentDeployedStream
, getExperimentDeployedStreamReq
, getExperimentHasData
, getExperimentHasDataStream
, getExperimentHasDataStreamReq
, getExperimentInoperable
, getExperimentInoperableStream
, getExperimentInoperableStreamReq
, getExperimentPart
, getExperimentPartStream
, getExperimentPartStreamReq
, getExperimentRerunnable
, getExperimentRerunnableStream
, getExperimentRerunnableStreamReq
, getExperimentScienceSubject
, getExperimentScienceSubjectStream
, getExperimentScienceSubjectStreamReq
, fairingJettison
, getFairingJettisoned
, getFairingJettisonedStream
, getFairingJettisonedStreamReq
, getFairingPart
, getFairingPartStream
, getFairingPartStreamReq
, getFlightAerodynamicForce
, getFlightAerodynamicForceStream
, getFlightAerodynamicForceStreamReq
, getFlightAngleOfAttack
, getFlightAngleOfAttackStream
, getFlightAngleOfAttackStreamReq
, getFlightAntiNormal
, getFlightAntiNormalStream
, getFlightAntiNormalStreamReq
, getFlightAntiRadial
, getFlightAntiRadialStream
, getFlightAntiRadialStreamReq
, getFlightAtmosphereDensity
, getFlightAtmosphereDensityStream
, getFlightAtmosphereDensityStreamReq
, getFlightBallisticCoefficient
, getFlightBallisticCoefficientStream
, getFlightBallisticCoefficientStreamReq
, getFlightBedrockAltitude
, getFlightBedrockAltitudeStream
, getFlightBedrockAltitudeStreamReq
, getFlightCenterOfMass
, getFlightCenterOfMassStream
, getFlightCenterOfMassStreamReq
, getFlightDirection
, getFlightDirectionStream
, getFlightDirectionStreamReq
, getFlightDrag
, getFlightDragStream
, getFlightDragStreamReq
, getFlightDragCoefficient
, getFlightDragCoefficientStream
, getFlightDragCoefficientStreamReq
, getFlightDynamicPressure
, getFlightDynamicPressureStream
, getFlightDynamicPressureStreamReq
, getFlightElevation
, getFlightElevationStream
, getFlightElevationStreamReq
, getFlightEquivalentAirSpeed
, getFlightEquivalentAirSpeedStream
, getFlightEquivalentAirSpeedStreamReq
, getFlightGForce
, getFlightGForceStream
, getFlightGForceStreamReq
, getFlightHeading
, getFlightHeadingStream
, getFlightHeadingStreamReq
, getFlightHorizontalSpeed
, getFlightHorizontalSpeedStream
, getFlightHorizontalSpeedStreamReq
, getFlightLatitude
, getFlightLatitudeStream
, getFlightLatitudeStreamReq
, getFlightLift
, getFlightLiftStream
, getFlightLiftStreamReq
, getFlightLiftCoefficient
, getFlightLiftCoefficientStream
, getFlightLiftCoefficientStreamReq
, getFlightLongitude
, getFlightLongitudeStream
, getFlightLongitudeStreamReq
, getFlightMach
, getFlightMachStream
, getFlightMachStreamReq
, getFlightMeanAltitude
, getFlightMeanAltitudeStream
, getFlightMeanAltitudeStreamReq
, getFlightNormal
, getFlightNormalStream
, getFlightNormalStreamReq
, getFlightPitch
, getFlightPitchStream
, getFlightPitchStreamReq
, getFlightPrograde
, getFlightProgradeStream
, getFlightProgradeStreamReq
, getFlightRadial
, getFlightRadialStream
, getFlightRadialStreamReq
, getFlightRetrograde
, getFlightRetrogradeStream
, getFlightRetrogradeStreamReq
, getFlightReynoldsNumber
, getFlightReynoldsNumberStream
, getFlightReynoldsNumberStreamReq
, getFlightRoll
, getFlightRollStream
, getFlightRollStreamReq
, getFlightRotation
, getFlightRotationStream
, getFlightRotationStreamReq
, getFlightSideslipAngle
, getFlightSideslipAngleStream
, getFlightSideslipAngleStreamReq
, getFlightSpeed
, getFlightSpeedStream
, getFlightSpeedStreamReq
, getFlightSpeedOfSound
, getFlightSpeedOfSoundStream
, getFlightSpeedOfSoundStreamReq
, getFlightStallFraction
, getFlightStallFractionStream
, getFlightStallFractionStreamReq
, getFlightStaticAirTemperature
, getFlightStaticAirTemperatureStream
, getFlightStaticAirTemperatureStreamReq
, getFlightStaticPressure
, getFlightStaticPressureStream
, getFlightStaticPressureStreamReq
, getFlightStaticPressureAtMSL
, getFlightStaticPressureAtMSLStream
, getFlightStaticPressureAtMSLStreamReq
, getFlightSurfaceAltitude
, getFlightSurfaceAltitudeStream
, getFlightSurfaceAltitudeStreamReq
, getFlightTerminalVelocity
, getFlightTerminalVelocityStream
, getFlightTerminalVelocityStreamReq
, getFlightThrustSpecificFuelConsumption
, getFlightThrustSpecificFuelConsumptionStream
, getFlightThrustSpecificFuelConsumptionStreamReq
, getFlightTotalAirTemperature
, getFlightTotalAirTemperatureStream
, getFlightTotalAirTemperatureStreamReq
, getFlightTrueAirSpeed
, getFlightTrueAirSpeedStream
, getFlightTrueAirSpeedStreamReq
, getFlightVelocity
, getFlightVelocityStream
, getFlightVelocityStreamReq
, getFlightVerticalSpeed
, getFlightVerticalSpeedStream
, getFlightVerticalSpeedStreamReq
, forceRemove
, getForceForceVector
, getForceForceVectorStream
, getForceForceVectorStreamReq
, getForcePart
, getForcePartStream
, getForcePartStreamReq
, getForcePosition
, getForcePositionStream
, getForcePositionStreamReq
, getForceReferenceFrame
, getForceReferenceFrameStream
, getForceReferenceFrameStreamReq
, setForceForceVector
, setForcePosition
, setForceReferenceFrame
, getIntakeArea
, getIntakeAreaStream
, getIntakeAreaStreamReq
, getIntakeFlow
, getIntakeFlowStream
, getIntakeFlowStreamReq
, getIntakeOpen
, getIntakeOpenStream
, getIntakeOpenStreamReq
, getIntakePart
, getIntakePartStream
, getIntakePartStreamReq
, getIntakeSpeed
, getIntakeSpeedStream
, getIntakeSpeedStreamReq
, setIntakeOpen
, launchClampRelease
, getLaunchClampPart
, getLaunchClampPartStream
, getLaunchClampPartStreamReq
, launchVessel
, launchVesselFromSPH
, launchVesselFromVAB
, launchableVessels
, launchableVesselsStream
, launchableVesselsStreamReq
, getLegDeployable
, getLegDeployableStream
, getLegDeployableStreamReq
, getLegDeployed
, getLegDeployedStream
, getLegDeployedStreamReq
, getLegIsGrounded
, getLegIsGroundedStream
, getLegIsGroundedStreamReq
, getLegPart
, getLegPartStream
, getLegPartStreamReq
, getLegState
, getLegStateStream
, getLegStateStreamReq
, setLegDeployed
, getLightActive
, getLightActiveStream
, getLightActiveStreamReq
, getLightColor
, getLightColorStream
, getLightColorStreamReq
, getLightPart
, getLightPartStream
, getLightPartStreamReq
, getLightPowerUsage
, getLightPowerUsageStream
, getLightPowerUsageStreamReq
, setLightActive
, setLightColor
, load
, moduleGetField
, moduleGetFieldStream
, moduleGetFieldStreamReq
, moduleHasAction
, moduleHasActionStream
, moduleHasActionStreamReq
, moduleHasEvent
, moduleHasEventStream
, moduleHasEventStreamReq
, moduleHasField
, moduleHasFieldStream
, moduleHasFieldStreamReq
, moduleResetField
, moduleSetAction
, moduleSetFieldFloat
, moduleSetFieldInt
, moduleSetFieldString
, moduleTriggerEvent
, getModuleActions
, getModuleActionsStream
, getModuleActionsStreamReq
, getModuleEvents
, getModuleEventsStream
, getModuleEventsStreamReq
, getModuleFields
, getModuleFieldsStream
, getModuleFieldsStreamReq
, getModuleName
, getModuleNameStream
, getModuleNameStreamReq
, getModulePart
, getModulePartStream
, getModulePartStreamReq
, nodeBurnVector
, nodeBurnVectorStream
, nodeBurnVectorStreamReq
, nodeDirection
, nodeDirectionStream
, nodeDirectionStreamReq
, nodePosition
, nodePositionStream
, nodePositionStreamReq
, nodeRemainingBurnVector
, nodeRemainingBurnVectorStream
, nodeRemainingBurnVectorStreamReq
, nodeRemove
, getNodeDeltaV
, getNodeDeltaVStream
, getNodeDeltaVStreamReq
, getNodeNormal
, getNodeNormalStream
, getNodeNormalStreamReq
, getNodeOrbit
, getNodeOrbitStream
, getNodeOrbitStreamReq
, getNodeOrbitalReferenceFrame
, getNodeOrbitalReferenceFrameStream
, getNodeOrbitalReferenceFrameStreamReq
, getNodePrograde
, getNodeProgradeStream
, getNodeProgradeStreamReq
, getNodeRadial
, getNodeRadialStream
, getNodeRadialStreamReq
, getNodeReferenceFrame
, getNodeReferenceFrameStream
, getNodeReferenceFrameStreamReq
, getNodeRemainingDeltaV
, getNodeRemainingDeltaVStream
, getNodeRemainingDeltaVStreamReq
, getNodeTimeTo
, getNodeTimeToStream
, getNodeTimeToStreamReq
, getNodeUT
, getNodeUTStream
, getNodeUTStreamReq
, setNodeDeltaV
, setNodeNormal
, setNodePrograde
, setNodeRadial
, setNodeUT
, orbitEccentricAnomalyAtUT
, orbitEccentricAnomalyAtUTStream
, orbitEccentricAnomalyAtUTStreamReq
, orbitOrbitalSpeedAt
, orbitOrbitalSpeedAtStream
, orbitOrbitalSpeedAtStreamReq
, orbitRadiusAtTrueAnomaly
, orbitRadiusAtTrueAnomalyStream
, orbitRadiusAtTrueAnomalyStreamReq
, orbitReferencePlaneDirection
, orbitReferencePlaneDirectionStream
, orbitReferencePlaneDirectionStreamReq
, orbitReferencePlaneNormal
, orbitReferencePlaneNormalStream
, orbitReferencePlaneNormalStreamReq
, orbitTrueAnomalyAtRadius
, orbitTrueAnomalyAtRadiusStream
, orbitTrueAnomalyAtRadiusStreamReq
, orbitTrueAnomalyAtUT
, orbitTrueAnomalyAtUTStream
, orbitTrueAnomalyAtUTStreamReq
, orbitUTAtTrueAnomaly
, orbitUTAtTrueAnomalyStream
, orbitUTAtTrueAnomalyStreamReq
, getOrbitApoapsis
, getOrbitApoapsisStream
, getOrbitApoapsisStreamReq
, getOrbitApoapsisAltitude
, getOrbitApoapsisAltitudeStream
, getOrbitApoapsisAltitudeStreamReq
, getOrbitArgumentOfPeriapsis
, getOrbitArgumentOfPeriapsisStream
, getOrbitArgumentOfPeriapsisStreamReq
, getOrbitBody
, getOrbitBodyStream
, getOrbitBodyStreamReq
, getOrbitEccentricAnomaly
, getOrbitEccentricAnomalyStream
, getOrbitEccentricAnomalyStreamReq
, getOrbitEccentricity
, getOrbitEccentricityStream
, getOrbitEccentricityStreamReq
, getOrbitEpoch
, getOrbitEpochStream
, getOrbitEpochStreamReq
, getOrbitInclination
, getOrbitInclinationStream
, getOrbitInclinationStreamReq
, getOrbitLongitudeOfAscendingNode
, getOrbitLongitudeOfAscendingNodeStream
, getOrbitLongitudeOfAscendingNodeStreamReq
, getOrbitMeanAnomaly
, getOrbitMeanAnomalyStream
, getOrbitMeanAnomalyStreamReq
, getOrbitMeanAnomalyAtEpoch
, getOrbitMeanAnomalyAtEpochStream
, getOrbitMeanAnomalyAtEpochStreamReq
, getOrbitNextOrbit
, getOrbitNextOrbitStream
, getOrbitNextOrbitStreamReq
, getOrbitOrbitalSpeed
, getOrbitOrbitalSpeedStream
, getOrbitOrbitalSpeedStreamReq
, getOrbitPeriapsis
, getOrbitPeriapsisStream
, getOrbitPeriapsisStreamReq
, getOrbitPeriapsisAltitude
, getOrbitPeriapsisAltitudeStream
, getOrbitPeriapsisAltitudeStreamReq
, getOrbitPeriod
, getOrbitPeriodStream
, getOrbitPeriodStreamReq
, getOrbitRadius
, getOrbitRadiusStream
, getOrbitRadiusStreamReq
, getOrbitSemiMajorAxis
, getOrbitSemiMajorAxisStream
, getOrbitSemiMajorAxisStreamReq
, getOrbitSemiMinorAxis
, getOrbitSemiMinorAxisStream
, getOrbitSemiMinorAxisStreamReq
, getOrbitSpeed
, getOrbitSpeedStream
, getOrbitSpeedStreamReq
, getOrbitTimeToApoapsis
, getOrbitTimeToApoapsisStream
, getOrbitTimeToApoapsisStreamReq
, getOrbitTimeToPeriapsis
, getOrbitTimeToPeriapsisStream
, getOrbitTimeToPeriapsisStreamReq
, getOrbitTimeToSOIChange
, getOrbitTimeToSOIChangeStream
, getOrbitTimeToSOIChangeStreamReq
, getOrbitTrueAnomaly
, getOrbitTrueAnomalyStream
, getOrbitTrueAnomalyStreamReq
, parachuteArm
, parachuteDeploy
, getParachuteArmed
, getParachuteArmedStream
, getParachuteArmedStreamReq
, getParachuteDeployAltitude
, getParachuteDeployAltitudeStream
, getParachuteDeployAltitudeStreamReq
, getParachuteDeployMinPressure
, getParachuteDeployMinPressureStream
, getParachuteDeployMinPressureStreamReq
, getParachuteDeployed
, getParachuteDeployedStream
, getParachuteDeployedStreamReq
, getParachutePart
, getParachutePartStream
, getParachutePartStreamReq
, getParachuteState
, getParachuteStateStream
, getParachuteStateStreamReq
, setParachuteDeployAltitude
, setParachuteDeployMinPressure
, partAddForce
, partAddForceStream
, partAddForceStreamReq
, partBoundingBox
, partBoundingBoxStream
, partBoundingBoxStreamReq
, partCenterOfMass
, partCenterOfMassStream
, partCenterOfMassStreamReq
, partDirection
, partDirectionStream
, partDirectionStreamReq
, partInstantaneousForce
, partPosition
, partPositionStream
, partPositionStreamReq
, partRotation
, partRotationStream
, partRotationStreamReq
, partVelocity
, partVelocityStream
, partVelocityStreamReq
, getPartAntenna
, getPartAntennaStream
, getPartAntennaStreamReq
, getPartAxiallyAttached
, getPartAxiallyAttachedStream
, getPartAxiallyAttachedStreamReq
, getPartCargoBay
, getPartCargoBayStream
, getPartCargoBayStreamReq
, getPartCenterOfMassReferenceFrame
, getPartCenterOfMassReferenceFrameStream
, getPartCenterOfMassReferenceFrameStreamReq
, getPartChildren
, getPartChildrenStream
, getPartChildrenStreamReq
, getPartControlSurface
, getPartControlSurfaceStream
, getPartControlSurfaceStreamReq
, getPartCost
, getPartCostStream
, getPartCostStreamReq
, getPartCrossfeed
, getPartCrossfeedStream
, getPartCrossfeedStreamReq
, getPartDecoupleStage
, getPartDecoupleStageStream
, getPartDecoupleStageStreamReq
, getPartDecoupler
, getPartDecouplerStream
, getPartDecouplerStreamReq
, getPartDockingPort
, getPartDockingPortStream
, getPartDockingPortStreamReq
, getPartDryMass
, getPartDryMassStream
, getPartDryMassStreamReq
, getPartDynamicPressure
, getPartDynamicPressureStream
, getPartDynamicPressureStreamReq
, getPartEngine
, getPartEngineStream
, getPartEngineStreamReq
, getPartExperiment
, getPartExperimentStream
, getPartExperimentStreamReq
, getPartFairing
, getPartFairingStream
, getPartFairingStreamReq
, getPartFuelLinesFrom
, getPartFuelLinesFromStream
, getPartFuelLinesFromStreamReq
, getPartFuelLinesTo
, getPartFuelLinesToStream
, getPartFuelLinesToStreamReq
, getPartHighlightColor
, getPartHighlightColorStream
, getPartHighlightColorStreamReq
, getPartHighlighted
, getPartHighlightedStream
, getPartHighlightedStreamReq
, getPartImpactTolerance
, getPartImpactToleranceStream
, getPartImpactToleranceStreamReq
, getPartInertiaTensor
, getPartInertiaTensorStream
, getPartInertiaTensorStreamReq
, getPartIntake
, getPartIntakeStream
, getPartIntakeStreamReq
, getPartIsFuelLine
, getPartIsFuelLineStream
, getPartIsFuelLineStreamReq
, getPartLaunchClamp
, getPartLaunchClampStream
, getPartLaunchClampStreamReq
, getPartLeg
, getPartLegStream
, getPartLegStreamReq
, getPartLight
, getPartLightStream
, getPartLightStreamReq
, getPartMass
, getPartMassStream
, getPartMassStreamReq
, getPartMassless
, getPartMasslessStream
, getPartMasslessStreamReq
, getPartMaxSkinTemperature
, getPartMaxSkinTemperatureStream
, getPartMaxSkinTemperatureStreamReq
, getPartMaxTemperature
, getPartMaxTemperatureStream
, getPartMaxTemperatureStreamReq
, getPartModules
, getPartModulesStream
, getPartModulesStreamReq
, getPartMomentOfInertia
, getPartMomentOfInertiaStream
, getPartMomentOfInertiaStreamReq
, getPartName
, getPartNameStream
, getPartNameStreamReq
, getPartParachute
, getPartParachuteStream
, getPartParachuteStreamReq
, getPartParent
, getPartParentStream
, getPartParentStreamReq
, getPartRCS
, getPartRCSStream
, getPartRCSStreamReq
, getPartRadiallyAttached
, getPartRadiallyAttachedStream
, getPartRadiallyAttachedStreamReq
, getPartRadiator
, getPartRadiatorStream
, getPartRadiatorStreamReq
, getPartReactionWheel
, getPartReactionWheelStream
, getPartReactionWheelStreamReq
, getPartReferenceFrame
, getPartReferenceFrameStream
, getPartReferenceFrameStreamReq
, getPartResourceConverter
, getPartResourceConverterStream
, getPartResourceConverterStreamReq
, getPartResourceHarvester
, getPartResourceHarvesterStream
, getPartResourceHarvesterStreamReq
, getPartResources
, getPartResourcesStream
, getPartResourcesStreamReq
, getPartSensor
, getPartSensorStream
, getPartSensorStreamReq
, getPartShielded
, getPartShieldedStream
, getPartShieldedStreamReq
, getPartSkinTemperature
, getPartSkinTemperatureStream
, getPartSkinTemperatureStreamReq
, getPartSolarPanel
, getPartSolarPanelStream
, getPartSolarPanelStreamReq
, getPartStage
, getPartStageStream
, getPartStageStreamReq
, getPartTag
, getPartTagStream
, getPartTagStreamReq
, getPartTemperature
, getPartTemperatureStream
, getPartTemperatureStreamReq
, getPartThermalConductionFlux
, getPartThermalConductionFluxStream
, getPartThermalConductionFluxStreamReq
, getPartThermalConvectionFlux
, getPartThermalConvectionFluxStream
, getPartThermalConvectionFluxStreamReq
, getPartThermalInternalFlux
, getPartThermalInternalFluxStream
, getPartThermalInternalFluxStreamReq
, getPartThermalMass
, getPartThermalMassStream
, getPartThermalMassStreamReq
, getPartThermalRadiationFlux
, getPartThermalRadiationFluxStream
, getPartThermalRadiationFluxStreamReq
, getPartThermalResourceMass
, getPartThermalResourceMassStream
, getPartThermalResourceMassStreamReq
, getPartThermalSkinMass
, getPartThermalSkinMassStream
, getPartThermalSkinMassStreamReq
, getPartThermalSkinToInternalFlux
, getPartThermalSkinToInternalFluxStream
, getPartThermalSkinToInternalFluxStreamReq
, getPartTitle
, getPartTitleStream
, getPartTitleStreamReq
, getPartVessel
, getPartVesselStream
, getPartVesselStreamReq
, getPartWheel
, getPartWheelStream
, getPartWheelStreamReq
, setPartHighlightColor
, setPartHighlighted
, setPartTag
, partsInDecoupleStage
, partsInDecoupleStageStream
, partsInDecoupleStageStreamReq
, partsInStage
, partsInStageStream
, partsInStageStreamReq
, partsModulesWithName
, partsModulesWithNameStream
, partsModulesWithNameStreamReq
, partsWithModule
, partsWithModuleStream
, partsWithModuleStreamReq
, partsWithName
, partsWithNameStream
, partsWithNameStreamReq
, partsWithTag
, partsWithTagStream
, partsWithTagStreamReq
, partsWithTitle
, partsWithTitleStream
, partsWithTitleStreamReq
, getPartsAll
, getPartsAllStream
, getPartsAllStreamReq
, getPartsAntennas
, getPartsAntennasStream
, getPartsAntennasStreamReq
, getPartsCargoBays
, getPartsCargoBaysStream
, getPartsCargoBaysStreamReq
, getPartsControlSurfaces
, getPartsControlSurfacesStream
, getPartsControlSurfacesStreamReq
, getPartsControlling
, getPartsControllingStream
, getPartsControllingStreamReq
, getPartsDecouplers
, getPartsDecouplersStream
, getPartsDecouplersStreamReq
, getPartsDockingPorts
, getPartsDockingPortsStream
, getPartsDockingPortsStreamReq
, getPartsEngines
, getPartsEnginesStream
, getPartsEnginesStreamReq
, getPartsExperiments
, getPartsExperimentsStream
, getPartsExperimentsStreamReq
, getPartsFairings
, getPartsFairingsStream
, getPartsFairingsStreamReq
, getPartsIntakes
, getPartsIntakesStream
, getPartsIntakesStreamReq
, getPartsLaunchClamps
, getPartsLaunchClampsStream
, getPartsLaunchClampsStreamReq
, getPartsLegs
, getPartsLegsStream
, getPartsLegsStreamReq
, getPartsLights
, getPartsLightsStream
, getPartsLightsStreamReq
, getPartsParachutes
, getPartsParachutesStream
, getPartsParachutesStreamReq
, getPartsRCS
, getPartsRCSStream
, getPartsRCSStreamReq
, getPartsRadiators
, getPartsRadiatorsStream
, getPartsRadiatorsStreamReq
, getPartsReactionWheels
, getPartsReactionWheelsStream
, getPartsReactionWheelsStreamReq
, getPartsResourceConverters
, getPartsResourceConvertersStream
, getPartsResourceConvertersStreamReq
, getPartsResourceHarvesters
, getPartsResourceHarvestersStream
, getPartsResourceHarvestersStreamReq
, getPartsRoot
, getPartsRootStream
, getPartsRootStreamReq
, getPartsSensors
, getPartsSensorsStream
, getPartsSensorsStreamReq
, getPartsSolarPanels
, getPartsSolarPanelsStream
, getPartsSolarPanelsStreamReq
, getPartsWheels
, getPartsWheelsStream
, getPartsWheelsStreamReq
, setPartsControlling
, getPropellantCurrentAmount
, getPropellantCurrentAmountStream
, getPropellantCurrentAmountStreamReq
, getPropellantCurrentRequirement
, getPropellantCurrentRequirementStream
, getPropellantCurrentRequirementStreamReq
, getPropellantDrawStackGauge
, getPropellantDrawStackGaugeStream
, getPropellantDrawStackGaugeStreamReq
, getPropellantIgnoreForIsp
, getPropellantIgnoreForIspStream
, getPropellantIgnoreForIspStreamReq
, getPropellantIgnoreForThrustCurve
, getPropellantIgnoreForThrustCurveStream
, getPropellantIgnoreForThrustCurveStreamReq
, getPropellantIsDeprived
, getPropellantIsDeprivedStream
, getPropellantIsDeprivedStreamReq
, getPropellantName
, getPropellantNameStream
, getPropellantNameStreamReq
, getPropellantRatio
, getPropellantRatioStream
, getPropellantRatioStreamReq
, getPropellantTotalResourceAvailable
, getPropellantTotalResourceAvailableStream
, getPropellantTotalResourceAvailableStreamReq
, getPropellantTotalResourceCapacity
, getPropellantTotalResourceCapacityStream
, getPropellantTotalResourceCapacityStreamReq
, quickload
, quicksave
, getRCSActive
, getRCSActiveStream
, getRCSActiveStreamReq
, getRCSAvailableTorque
, getRCSAvailableTorqueStream
, getRCSAvailableTorqueStreamReq
, getRCSEnabled
, getRCSEnabledStream
, getRCSEnabledStreamReq
, getRCSForwardEnabled
, getRCSForwardEnabledStream
, getRCSForwardEnabledStreamReq
, getRCSHasFuel
, getRCSHasFuelStream
, getRCSHasFuelStreamReq
, getRCSKerbinSeaLevelSpecificImpulse
, getRCSKerbinSeaLevelSpecificImpulseStream
, getRCSKerbinSeaLevelSpecificImpulseStreamReq
, getRCSMaxThrust
, getRCSMaxThrustStream
, getRCSMaxThrustStreamReq
, getRCSMaxVacuumThrust
, getRCSMaxVacuumThrustStream
, getRCSMaxVacuumThrustStreamReq
, getRCSPart
, getRCSPartStream
, getRCSPartStreamReq
, getRCSPitchEnabled
, getRCSPitchEnabledStream
, getRCSPitchEnabledStreamReq
, getRCSPropellantRatios
, getRCSPropellantRatiosStream
, getRCSPropellantRatiosStreamReq
, getRCSPropellants
, getRCSPropellantsStream
, getRCSPropellantsStreamReq
, getRCSRightEnabled
, getRCSRightEnabledStream
, getRCSRightEnabledStreamReq
, getRCSRollEnabled
, getRCSRollEnabledStream
, getRCSRollEnabledStreamReq
, getRCSSpecificImpulse
, getRCSSpecificImpulseStream
, getRCSSpecificImpulseStreamReq
, getRCSThrusters
, getRCSThrustersStream
, getRCSThrustersStreamReq
, getRCSUpEnabled
, getRCSUpEnabledStream
, getRCSUpEnabledStreamReq
, getRCSVacuumSpecificImpulse
, getRCSVacuumSpecificImpulseStream
, getRCSVacuumSpecificImpulseStreamReq
, getRCSYawEnabled
, getRCSYawEnabledStream
, getRCSYawEnabledStreamReq
, setRCSEnabled
, setRCSForwardEnabled
, setRCSPitchEnabled
, setRCSRightEnabled
, setRCSRollEnabled
, setRCSUpEnabled
, setRCSYawEnabled
, getRadiatorDeployable
, getRadiatorDeployableStream
, getRadiatorDeployableStreamReq
, getRadiatorDeployed
, getRadiatorDeployedStream
, getRadiatorDeployedStreamReq
, getRadiatorPart
, getRadiatorPartStream
, getRadiatorPartStreamReq
, getRadiatorState
, getRadiatorStateStream
, getRadiatorStateStreamReq
, setRadiatorDeployed
, getReactionWheelActive
, getReactionWheelActiveStream
, getReactionWheelActiveStreamReq
, getReactionWheelAvailableTorque
, getReactionWheelAvailableTorqueStream
, getReactionWheelAvailableTorqueStreamReq
, getReactionWheelBroken
, getReactionWheelBrokenStream
, getReactionWheelBrokenStreamReq
, getReactionWheelMaxTorque
, getReactionWheelMaxTorqueStream
, getReactionWheelMaxTorqueStreamReq
, getReactionWheelPart
, getReactionWheelPartStream
, getReactionWheelPartStreamReq
, setReactionWheelActive
, referenceFrameCreateHybrid
, referenceFrameCreateHybridStream
, referenceFrameCreateHybridStreamReq
, referenceFrameCreateRelative
, referenceFrameCreateRelativeStream
, referenceFrameCreateRelativeStreamReq
, resourceConverterActive
, resourceConverterActiveStream
, resourceConverterActiveStreamReq
, resourceConverterInputs
, resourceConverterInputsStream
, resourceConverterInputsStreamReq
, resourceConverterName
, resourceConverterNameStream
, resourceConverterNameStreamReq
, resourceConverterOutputs
, resourceConverterOutputsStream
, resourceConverterOutputsStreamReq
, resourceConverterStart
, resourceConverterState
, resourceConverterStateStream
, resourceConverterStateStreamReq
, resourceConverterStatusInfo
, resourceConverterStatusInfoStream
, resourceConverterStatusInfoStreamReq
, resourceConverterStop
, getResourceConverterCount
, getResourceConverterCountStream
, getResourceConverterCountStreamReq
, getResourceConverterPart
, getResourceConverterPartStream
, getResourceConverterPartStreamReq
, getResourceHarvesterActive
, getResourceHarvesterActiveStream
, getResourceHarvesterActiveStreamReq
, getResourceHarvesterCoreTemperature
, getResourceHarvesterCoreTemperatureStream
, getResourceHarvesterCoreTemperatureStreamReq
, getResourceHarvesterDeployed
, getResourceHarvesterDeployedStream
, getResourceHarvesterDeployedStreamReq
, getResourceHarvesterExtractionRate
, getResourceHarvesterExtractionRateStream
, getResourceHarvesterExtractionRateStreamReq
, getResourceHarvesterOptimumCoreTemperature
, getResourceHarvesterOptimumCoreTemperatureStream
, getResourceHarvesterOptimumCoreTemperatureStreamReq
, getResourceHarvesterPart
, getResourceHarvesterPartStream
, getResourceHarvesterPartStreamReq
, getResourceHarvesterState
, getResourceHarvesterStateStream
, getResourceHarvesterStateStreamReq
, getResourceHarvesterThermalEfficiency
, getResourceHarvesterThermalEfficiencyStream
, getResourceHarvesterThermalEfficiencyStreamReq
, setResourceHarvesterActive
, setResourceHarvesterDeployed
, resourceTransferStart
, resourceTransferStartStream
, resourceTransferStartStreamReq
, getResourceTransferAmount
, getResourceTransferAmountStream
, getResourceTransferAmountStreamReq
, getResourceTransferComplete
, getResourceTransferCompleteStream
, getResourceTransferCompleteStreamReq
, getResourceAmount
, getResourceAmountStream
, getResourceAmountStreamReq
, getResourceDensity
, getResourceDensityStream
, getResourceDensityStreamReq
, getResourceEnabled
, getResourceEnabledStream
, getResourceEnabledStreamReq
, getResourceFlowMode
, getResourceFlowModeStream
, getResourceFlowModeStreamReq
, getResourceMax
, getResourceMaxStream
, getResourceMaxStreamReq
, getResourceName
, getResourceNameStream
, getResourceNameStreamReq
, getResourcePart
, getResourcePartStream
, getResourcePartStreamReq
, setResourceEnabled
, resourcesAmount
, resourcesAmountStream
, resourcesAmountStreamReq
, resourcesDensity
, resourcesDensityStream
, resourcesDensityStreamReq
, resourcesFlowMode
, resourcesFlowModeStream
, resourcesFlowModeStreamReq
, resourcesHasResource
, resourcesHasResourceStream
, resourcesHasResourceStreamReq
, resourcesMax
, resourcesMaxStream
, resourcesMaxStreamReq
, resourcesWithResource
, resourcesWithResourceStream
, resourcesWithResourceStreamReq
, getResourcesAll
, getResourcesAllStream
, getResourcesAllStreamReq
, getResourcesEnabled
, getResourcesEnabledStream
, getResourcesEnabledStreamReq
, getResourcesNames
, getResourcesNamesStream
, getResourcesNamesStreamReq
, setResourcesEnabled
, save
, getScienceDataDataAmount
, getScienceDataDataAmountStream
, getScienceDataDataAmountStreamReq
, getScienceDataScienceValue
, getScienceDataScienceValueStream
, getScienceDataScienceValueStreamReq
, getScienceDataTransmitValue
, getScienceDataTransmitValueStream
, getScienceDataTransmitValueStreamReq
, getScienceSubjectDataScale
, getScienceSubjectDataScaleStream
, getScienceSubjectDataScaleStreamReq
, getScienceSubjectIsComplete
, getScienceSubjectIsCompleteStream
, getScienceSubjectIsCompleteStreamReq
, getScienceSubjectScience
, getScienceSubjectScienceStream
, getScienceSubjectScienceStreamReq
, getScienceSubjectScienceCap
, getScienceSubjectScienceCapStream
, getScienceSubjectScienceCapStreamReq
, getScienceSubjectScientificValue
, getScienceSubjectScientificValueStream
, getScienceSubjectScientificValueStreamReq
, getScienceSubjectSubjectValue
, getScienceSubjectSubjectValueStream
, getScienceSubjectSubjectValueStreamReq
, getScienceSubjectTitle
, getScienceSubjectTitleStream
, getScienceSubjectTitleStreamReq
, getSensorActive
, getSensorActiveStream
, getSensorActiveStreamReq
, getSensorPart
, getSensorPartStream
, getSensorPartStreamReq
, getSensorValue
, getSensorValueStream
, getSensorValueStreamReq
, setSensorActive
, getSolarPanelDeployable
, getSolarPanelDeployableStream
, getSolarPanelDeployableStreamReq
, getSolarPanelDeployed
, getSolarPanelDeployedStream
, getSolarPanelDeployedStreamReq
, getSolarPanelEnergyFlow
, getSolarPanelEnergyFlowStream
, getSolarPanelEnergyFlowStreamReq
, getSolarPanelPart
, getSolarPanelPartStream
, getSolarPanelPartStreamReq
, getSolarPanelState
, getSolarPanelStateStream
, getSolarPanelStateStreamReq
, getSolarPanelSunExposure
, getSolarPanelSunExposureStream
, getSolarPanelSunExposureStreamReq
, setSolarPanelDeployed
, thrusterGimbalPosition
, thrusterGimbalPositionStream
, thrusterGimbalPositionStreamReq
, thrusterInitialThrustDirection
, thrusterInitialThrustDirectionStream
, thrusterInitialThrustDirectionStreamReq
, thrusterInitialThrustPosition
, thrusterInitialThrustPositionStream
, thrusterInitialThrustPositionStreamReq
, thrusterThrustDirection
, thrusterThrustDirectionStream
, thrusterThrustDirectionStreamReq
, thrusterThrustPosition
, thrusterThrustPositionStream
, thrusterThrustPositionStreamReq
, getThrusterGimbalAngle
, getThrusterGimbalAngleStream
, getThrusterGimbalAngleStreamReq
, getThrusterGimballed
, getThrusterGimballedStream
, getThrusterGimballedStreamReq
, getThrusterPart
, getThrusterPartStream
, getThrusterPartStreamReq
, getThrusterThrustReferenceFrame
, getThrusterThrustReferenceFrameStream
, getThrusterThrustReferenceFrameStreamReq
, transformDirection
, transformDirectionStream
, transformDirectionStreamReq
, transformPosition
, transformPositionStream
, transformPositionStreamReq
, transformRotation
, transformRotationStream
, transformRotationStreamReq
, transformVelocity
, transformVelocityStream
, transformVelocityStreamReq
, vesselAngularVelocity
, vesselAngularVelocityStream
, vesselAngularVelocityStreamReq
, vesselBoundingBox
, vesselBoundingBoxStream
, vesselBoundingBoxStreamReq
, vesselDirection
, vesselDirectionStream
, vesselDirectionStreamReq
, vesselFlight
, vesselFlightStream
, vesselFlightStreamReq
, vesselPosition
, vesselPositionStream
, vesselPositionStreamReq
, vesselRecover
, vesselResourcesInDecoupleStage
, vesselResourcesInDecoupleStageStream
, vesselResourcesInDecoupleStageStreamReq
, vesselRotation
, vesselRotationStream
, vesselRotationStreamReq
, vesselVelocity
, vesselVelocityStream
, vesselVelocityStreamReq
, getVesselAutoPilot
, getVesselAutoPilotStream
, getVesselAutoPilotStreamReq
, getVesselAvailableControlSurfaceTorque
, getVesselAvailableControlSurfaceTorqueStream
, getVesselAvailableControlSurfaceTorqueStreamReq
, getVesselAvailableEngineTorque
, getVesselAvailableEngineTorqueStream
, getVesselAvailableEngineTorqueStreamReq
, getVesselAvailableOtherTorque
, getVesselAvailableOtherTorqueStream
, getVesselAvailableOtherTorqueStreamReq
, getVesselAvailableRCSTorque
, getVesselAvailableRCSTorqueStream
, getVesselAvailableRCSTorqueStreamReq
, getVesselAvailableReactionWheelTorque
, getVesselAvailableReactionWheelTorqueStream
, getVesselAvailableReactionWheelTorqueStreamReq
, getVesselAvailableThrust
, getVesselAvailableThrustStream
, getVesselAvailableThrustStreamReq
, getVesselAvailableTorque
, getVesselAvailableTorqueStream
, getVesselAvailableTorqueStreamReq
, getVesselBiome
, getVesselBiomeStream
, getVesselBiomeStreamReq
, getVesselComms
, getVesselCommsStream
, getVesselCommsStreamReq
, getVesselControl
, getVesselControlStream
, getVesselControlStreamReq
, getVesselDryMass
, getVesselDryMassStream
, getVesselDryMassStreamReq
, getVesselInertiaTensor
, getVesselInertiaTensorStream
, getVesselInertiaTensorStreamReq
, getVesselKerbinSeaLevelSpecificImpulse
, getVesselKerbinSeaLevelSpecificImpulseStream
, getVesselKerbinSeaLevelSpecificImpulseStreamReq
, getVesselMET
, getVesselMETStream
, getVesselMETStreamReq
, getVesselMass
, getVesselMassStream
, getVesselMassStreamReq
, getVesselMaxThrust
, getVesselMaxThrustStream
, getVesselMaxThrustStreamReq
, getVesselMaxVacuumThrust
, getVesselMaxVacuumThrustStream
, getVesselMaxVacuumThrustStreamReq
, getVesselMomentOfInertia
, getVesselMomentOfInertiaStream
, getVesselMomentOfInertiaStreamReq
, getVesselName
, getVesselNameStream
, getVesselNameStreamReq
, getVesselOrbit
, getVesselOrbitStream
, getVesselOrbitStreamReq
, getVesselOrbitalReferenceFrame
, getVesselOrbitalReferenceFrameStream
, getVesselOrbitalReferenceFrameStreamReq
, getVesselParts
, getVesselPartsStream
, getVesselPartsStreamReq
, getVesselRecoverable
, getVesselRecoverableStream
, getVesselRecoverableStreamReq
, getVesselReferenceFrame
, getVesselReferenceFrameStream
, getVesselReferenceFrameStreamReq
, getVesselResources
, getVesselResourcesStream
, getVesselResourcesStreamReq
, getVesselSituation
, getVesselSituationStream
, getVesselSituationStreamReq
, getVesselSpecificImpulse
, getVesselSpecificImpulseStream
, getVesselSpecificImpulseStreamReq
, getVesselSurfaceReferenceFrame
, getVesselSurfaceReferenceFrameStream
, getVesselSurfaceReferenceFrameStreamReq
, getVesselSurfaceVelocityReferenceFrame
, getVesselSurfaceVelocityReferenceFrameStream
, getVesselSurfaceVelocityReferenceFrameStreamReq
, getVesselThrust
, getVesselThrustStream
, getVesselThrustStreamReq
, getVesselType
, getVesselTypeStream
, getVesselTypeStreamReq
, getVesselVacuumSpecificImpulse
, getVesselVacuumSpecificImpulseStream
, getVesselVacuumSpecificImpulseStreamReq
, setVesselName
, setVesselType
, warpTo
, waypointManagerAddWaypoint
, waypointManagerAddWaypointStream
, waypointManagerAddWaypointStreamReq
, getWaypointManagerColors
, getWaypointManagerColorsStream
, getWaypointManagerColorsStreamReq
, getWaypointManagerIcons
, getWaypointManagerIconsStream
, getWaypointManagerIconsStreamReq
, getWaypointManagerWaypoints
, getWaypointManagerWaypointsStream
, getWaypointManagerWaypointsStreamReq
, waypointRemove
, getWaypointBedrockAltitude
, getWaypointBedrockAltitudeStream
, getWaypointBedrockAltitudeStreamReq
, getWaypointBody
, getWaypointBodyStream
, getWaypointBodyStreamReq
, getWaypointClustered
, getWaypointClusteredStream
, getWaypointClusteredStreamReq
, getWaypointColor
, getWaypointColorStream
, getWaypointColorStreamReq
, getWaypointContract
, getWaypointContractStream
, getWaypointContractStreamReq
, getWaypointGrounded
, getWaypointGroundedStream
, getWaypointGroundedStreamReq
, getWaypointHasContract
, getWaypointHasContractStream
, getWaypointHasContractStreamReq
, getWaypointIcon
, getWaypointIconStream
, getWaypointIconStreamReq
, getWaypointIndex
, getWaypointIndexStream
, getWaypointIndexStreamReq
, getWaypointLatitude
, getWaypointLatitudeStream
, getWaypointLatitudeStreamReq
, getWaypointLongitude
, getWaypointLongitudeStream
, getWaypointLongitudeStreamReq
, getWaypointMeanAltitude
, getWaypointMeanAltitudeStream
, getWaypointMeanAltitudeStreamReq
, getWaypointName
, getWaypointNameStream
, getWaypointNameStreamReq
, getWaypointNearSurface
, getWaypointNearSurfaceStream
, getWaypointNearSurfaceStreamReq
, getWaypointSurfaceAltitude
, getWaypointSurfaceAltitudeStream
, getWaypointSurfaceAltitudeStreamReq
, setWaypointBedrockAltitude
, setWaypointBody
, setWaypointColor
, setWaypointIcon
, setWaypointLatitude
, setWaypointLongitude
, setWaypointMeanAltitude
, setWaypointName
, setWaypointSurfaceAltitude
, getWheelAutoFrictionControl
, getWheelAutoFrictionControlStream
, getWheelAutoFrictionControlStreamReq
, getWheelBrakes
, getWheelBrakesStream
, getWheelBrakesStreamReq
, getWheelBroken
, getWheelBrokenStream
, getWheelBrokenStreamReq
, getWheelDeflection
, getWheelDeflectionStream
, getWheelDeflectionStreamReq
, getWheelDeployable
, getWheelDeployableStream
, getWheelDeployableStreamReq
, getWheelDeployed
, getWheelDeployedStream
, getWheelDeployedStreamReq
, getWheelDriveLimiter
, getWheelDriveLimiterStream
, getWheelDriveLimiterStreamReq
, getWheelGrounded
, getWheelGroundedStream
, getWheelGroundedStreamReq
, getWheelHasBrakes
, getWheelHasBrakesStream
, getWheelHasBrakesStreamReq
, getWheelHasSuspension
, getWheelHasSuspensionStream
, getWheelHasSuspensionStreamReq
, getWheelManualFrictionControl
, getWheelManualFrictionControlStream
, getWheelManualFrictionControlStreamReq
, getWheelMotorEnabled
, getWheelMotorEnabledStream
, getWheelMotorEnabledStreamReq
, getWheelMotorInverted
, getWheelMotorInvertedStream
, getWheelMotorInvertedStreamReq
, getWheelMotorOutput
, getWheelMotorOutputStream
, getWheelMotorOutputStreamReq
, getWheelMotorState
, getWheelMotorStateStream
, getWheelMotorStateStreamReq
, getWheelPart
, getWheelPartStream
, getWheelPartStreamReq
, getWheelPowered
, getWheelPoweredStream
, getWheelPoweredStreamReq
, getWheelRadius
, getWheelRadiusStream
, getWheelRadiusStreamReq
, getWheelRepairable
, getWheelRepairableStream
, getWheelRepairableStreamReq
, getWheelSlip
, getWheelSlipStream
, getWheelSlipStreamReq
, getWheelState
, getWheelStateStream
, getWheelStateStreamReq
, getWheelSteerable
, getWheelSteerableStream
, getWheelSteerableStreamReq
, getWheelSteeringEnabled
, getWheelSteeringEnabledStream
, getWheelSteeringEnabledStreamReq
, getWheelSteeringInverted
, getWheelSteeringInvertedStream
, getWheelSteeringInvertedStreamReq
, getWheelStress
, getWheelStressStream
, getWheelStressStreamReq
, getWheelStressPercentage
, getWheelStressPercentageStream
, getWheelStressPercentageStreamReq
, getWheelStressTolerance
, getWheelStressToleranceStream
, getWheelStressToleranceStreamReq
, getWheelSuspensionDamperStrength
, getWheelSuspensionDamperStrengthStream
, getWheelSuspensionDamperStrengthStreamReq
, getWheelSuspensionSpringStrength
, getWheelSuspensionSpringStrengthStream
, getWheelSuspensionSpringStrengthStreamReq
, getWheelTractionControl
, getWheelTractionControlStream
, getWheelTractionControlStreamReq
, getWheelTractionControlEnabled
, getWheelTractionControlEnabledStream
, getWheelTractionControlEnabledStreamReq
, setWheelAutoFrictionControl
, setWheelBrakes
, setWheelDeployed
, setWheelDriveLimiter
, setWheelManualFrictionControl
, setWheelMotorEnabled
, setWheelMotorInverted
, setWheelSteeringEnabled
, setWheelSteeringInverted
, setWheelTractionControl
, setWheelTractionControlEnabled
, getActiveVessel
, getActiveVesselStream
, getActiveVesselStreamReq
, getBodies
, getBodiesStream
, getBodiesStreamReq
, getCamera
, getCameraStream
, getCameraStreamReq
, getContractManager
, getContractManagerStream
, getContractManagerStreamReq
, getFARAvailable
, getFARAvailableStream
, getFARAvailableStreamReq
, getG
, getGStream
, getGStreamReq
, getMaximumRailsWarpFactor
, getMaximumRailsWarpFactorStream
, getMaximumRailsWarpFactorStreamReq
, getNavball
, getNavballStream
, getNavballStreamReq
, getPhysicsWarpFactor
, getPhysicsWarpFactorStream
, getPhysicsWarpFactorStreamReq
, getRailsWarpFactor
, getRailsWarpFactorStream
, getRailsWarpFactorStreamReq
, getTargetBody
, getTargetBodyStream
, getTargetBodyStreamReq
, getTargetDockingPort
, getTargetDockingPortStream
, getTargetDockingPortStreamReq
, getTargetVessel
, getTargetVesselStream
, getTargetVesselStreamReq
, getUIVisible
, getUIVisibleStream
, getUIVisibleStreamReq
, getUT
, getUTStream
, getUTStreamReq
, getVessels
, getVesselsStream
, getVesselsStreamReq
, getWarpFactor
, getWarpFactorStream
, getWarpFactorStreamReq
, getWarpMode
, getWarpModeStream
, getWarpModeStreamReq
, getWarpRate
, getWarpRateStream
, getWarpRateStreamReq
, getWaypointManager
, getWaypointManagerStream
, getWaypointManagerStreamReq
, setActiveVessel
, setNavball
, setPhysicsWarpFactor
, setRailsWarpFactor
, setTargetBody
, setTargetDockingPort
, setTargetVessel
, setUIVisible
) where

import qualified Data.Int
import qualified Data.Map
import qualified Data.Set
import qualified Data.Text
import qualified Data.Word

import Control.Monad.Catch
import Control.Monad.IO.Class

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - An antenna. Obtained by calling <see cref="M:SpaceCenter.Part.Antenna" />.
 -}
newtype Antenna = Antenna { antennaId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Antenna where
    encodePb   = encodePb . antennaId
    decodePb b = Antenna <$> decodePb b

instance KRPCResponseExtractable Antenna

{-
 - Provides basic auto-piloting utilities for a vessel.
 - Created by calling <see cref="M:SpaceCenter.Vessel.AutoPilot" />.If a client engages the auto-pilot and then closes its connection to the server,
 - the auto-pilot will be disengaged and its target reference frame, direction and roll reset to default.
 -}
newtype AutoPilot = AutoPilot { autoPilotId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable AutoPilot where
    encodePb   = encodePb . autoPilotId
    decodePb b = AutoPilot <$> decodePb b

instance KRPCResponseExtractable AutoPilot

{-
 - Controls the game's camera.
 - Obtained by calling <see cref="M:SpaceCenter.Camera" />.
 -}
newtype Camera = Camera { cameraId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Camera where
    encodePb   = encodePb . cameraId
    decodePb b = Camera <$> decodePb b

instance KRPCResponseExtractable Camera

{-
 - A cargo bay. Obtained by calling <see cref="M:SpaceCenter.Part.CargoBay" />.
 -}
newtype CargoBay = CargoBay { cargoBayId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CargoBay where
    encodePb   = encodePb . cargoBayId
    decodePb b = CargoBay <$> decodePb b

instance KRPCResponseExtractable CargoBay

{-
 - Represents a celestial body (such as a planet or moon).
 - See <see cref="M:SpaceCenter.Bodies" />.
 -}
newtype CelestialBody = CelestialBody { celestialBodyId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CelestialBody where
    encodePb   = encodePb . celestialBodyId
    decodePb b = CelestialBody <$> decodePb b

instance KRPCResponseExtractable CelestialBody

{-
 - Represents a communication node in the network. For example, a vessel or the KSC.
 -}
newtype CommLink = CommLink { commLinkId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CommLink where
    encodePb   = encodePb . commLinkId
    decodePb b = CommLink <$> decodePb b

instance KRPCResponseExtractable CommLink

{-
 - Represents a communication node in the network. For example, a vessel or the KSC.
 -}
newtype CommNode = CommNode { commNodeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CommNode where
    encodePb   = encodePb . commNodeId
    decodePb b = CommNode <$> decodePb b

instance KRPCResponseExtractable CommNode

{-
 - Used to interact with CommNet for a given vessel.
 - Obtained by calling <see cref="M:SpaceCenter.Vessel.Comms" />.
 -}
newtype Comms = Comms { commsId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Comms where
    encodePb   = encodePb . commsId
    decodePb b = Comms <$> decodePb b

instance KRPCResponseExtractable Comms

{-
 - A contract. Can be accessed using <see cref="M:SpaceCenter.ContractManager" />.
 -}
newtype Contract = Contract { contractId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Contract where
    encodePb   = encodePb . contractId
    decodePb b = Contract <$> decodePb b

instance KRPCResponseExtractable Contract

{-
 - Contracts manager.
 - Obtained by calling <see cref="M:SpaceCenter.WaypointManager" />.
 -}
newtype ContractManager = ContractManager { contractManagerId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ContractManager where
    encodePb   = encodePb . contractManagerId
    decodePb b = ContractManager <$> decodePb b

instance KRPCResponseExtractable ContractManager

{-
 - A contract parameter. See <see cref="M:SpaceCenter.Contract.Parameters" />.
 -}
newtype ContractParameter = ContractParameter { contractParameterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ContractParameter where
    encodePb   = encodePb . contractParameterId
    decodePb b = ContractParameter <$> decodePb b

instance KRPCResponseExtractable ContractParameter

{-
 - Used to manipulate the controls of a vessel. This includes adjusting the
 - throttle, enabling/disabling systems such as SAS and RCS, or altering the
 - direction in which the vessel is pointing.
 - Obtained by calling <see cref="M:SpaceCenter.Vessel.Control" />.Control inputs (such as pitch, yaw and roll) are zeroed when all clients
 - that have set one or more of these inputs are no longer connected.
 -}
newtype Control = Control { controlId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Control where
    encodePb   = encodePb . controlId
    decodePb b = Control <$> decodePb b

instance KRPCResponseExtractable Control

{-
 - An aerodynamic control surface. Obtained by calling <see cref="M:SpaceCenter.Part.ControlSurface" />.
 -}
newtype ControlSurface = ControlSurface { controlSurfaceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ControlSurface where
    encodePb   = encodePb . controlSurfaceId
    decodePb b = ControlSurface <$> decodePb b

instance KRPCResponseExtractable ControlSurface

{-
 - A decoupler. Obtained by calling <see cref="M:SpaceCenter.Part.Decoupler" />
 -}
newtype Decoupler = Decoupler { decouplerId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Decoupler where
    encodePb   = encodePb . decouplerId
    decodePb b = Decoupler <$> decodePb b

instance KRPCResponseExtractable Decoupler

{-
 - A docking port. Obtained by calling <see cref="M:SpaceCenter.Part.DockingPort" />
 -}
newtype DockingPort = DockingPort { dockingPortId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable DockingPort where
    encodePb   = encodePb . dockingPortId
    decodePb b = DockingPort <$> decodePb b

instance KRPCResponseExtractable DockingPort

{-
 - An engine, including ones of various types.
 - For example liquid fuelled gimballed engines, solid rocket boosters and jet engines.
 - Obtained by calling <see cref="M:SpaceCenter.Part.Engine" />.For RCS thrusters <see cref="M:SpaceCenter.Part.RCS" />.
 -}
newtype Engine = Engine { engineId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Engine where
    encodePb   = encodePb . engineId
    decodePb b = Engine <$> decodePb b

instance KRPCResponseExtractable Engine

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Experiment" />.
 -}
newtype Experiment = Experiment { experimentId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Experiment where
    encodePb   = encodePb . experimentId
    decodePb b = Experiment <$> decodePb b

instance KRPCResponseExtractable Experiment

{-
 - A fairing. Obtained by calling <see cref="M:SpaceCenter.Part.Fairing" />.
 -}
newtype Fairing = Fairing { fairingId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Fairing where
    encodePb   = encodePb . fairingId
    decodePb b = Fairing <$> decodePb b

instance KRPCResponseExtractable Fairing

{-
 - Used to get flight telemetry for a vessel, by calling <see cref="M:SpaceCenter.Vessel.Flight" />.
 - All of the information returned by this class is given in the reference frame
 - passed to that method.
 - Obtained by calling <see cref="M:SpaceCenter.Vessel.Flight" />.To get orbital information, such as the apoapsis or inclination, see <see cref="T:SpaceCenter.Orbit" />.
 -}
newtype Flight = Flight { flightId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Flight where
    encodePb   = encodePb . flightId
    decodePb b = Flight <$> decodePb b

instance KRPCResponseExtractable Flight

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.AddForce" />.
 -}
newtype Force = Force { forceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Force where
    encodePb   = encodePb . forceId
    decodePb b = Force <$> decodePb b

instance KRPCResponseExtractable Force

{-
 - An air intake. Obtained by calling <see cref="M:SpaceCenter.Part.Intake" />.
 -}
newtype Intake = Intake { intakeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Intake where
    encodePb   = encodePb . intakeId
    decodePb b = Intake <$> decodePb b

instance KRPCResponseExtractable Intake

{-
 - A launch clamp. Obtained by calling <see cref="M:SpaceCenter.Part.LaunchClamp" />.
 -}
newtype LaunchClamp = LaunchClamp { launchClampId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LaunchClamp where
    encodePb   = encodePb . launchClampId
    decodePb b = LaunchClamp <$> decodePb b

instance KRPCResponseExtractable LaunchClamp

{-
 - A landing leg. Obtained by calling <see cref="M:SpaceCenter.Part.Leg" />.
 -}
newtype Leg = Leg { legId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Leg where
    encodePb   = encodePb . legId
    decodePb b = Leg <$> decodePb b

instance KRPCResponseExtractable Leg

{-
 - A light. Obtained by calling <see cref="M:SpaceCenter.Part.Light" />.
 -}
newtype Light = Light { lightId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Light where
    encodePb   = encodePb . lightId
    decodePb b = Light <$> decodePb b

instance KRPCResponseExtractable Light

{-
 - This can be used to interact with a specific part module. This includes part modules in stock KSP,
 - and those added by mods.
 - 
 - In KSP, each part has zero or more
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation#MODULES">PartModulesassociated with it. Each one contains some of the functionality of the part.
 - For example, an engine has a "ModuleEngines" part module that contains all the
 - functionality of an engine.
 -}
newtype Module = Module { moduleId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Module where
    encodePb   = encodePb . moduleId
    decodePb b = Module <$> decodePb b

instance KRPCResponseExtractable Module

{-
 - Represents a maneuver node. Can be created using <see cref="M:SpaceCenter.Control.AddNode" />.
 -}
newtype Node = Node { nodeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Node where
    encodePb   = encodePb . nodeId
    decodePb b = Node <$> decodePb b

instance KRPCResponseExtractable Node

{-
 - Describes an orbit. For example, the orbit of a vessel, obtained by calling
 - <see cref="M:SpaceCenter.Vessel.Orbit" />, or a celestial body, obtained by calling
 - <see cref="M:SpaceCenter.CelestialBody.Orbit" />.
 -}
newtype Orbit = Orbit { orbitId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Orbit where
    encodePb   = encodePb . orbitId
    decodePb b = Orbit <$> decodePb b

instance KRPCResponseExtractable Orbit

{-
 - A parachute. Obtained by calling <see cref="M:SpaceCenter.Part.Parachute" />.
 -}
newtype Parachute = Parachute { parachuteId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Parachute where
    encodePb   = encodePb . parachuteId
    decodePb b = Parachute <$> decodePb b

instance KRPCResponseExtractable Parachute

{-
 - Represents an individual part. Vessels are made up of multiple parts.
 - Instances of this class can be obtained by several methods in <see cref="T:SpaceCenter.Parts" />.
 -}
newtype Part = Part { partId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Part where
    encodePb   = encodePb . partId
    decodePb b = Part <$> decodePb b

instance KRPCResponseExtractable Part

{-
 - Instances of this class are used to interact with the parts of a vessel.
 - An instance can be obtained by calling <see cref="M:SpaceCenter.Vessel.Parts" />.
 -}
newtype Parts = Parts { partsId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Parts where
    encodePb   = encodePb . partsId
    decodePb b = Parts <$> decodePb b

instance KRPCResponseExtractable Parts

{-
 - A propellant for an engine. Obtains by calling <see cref="M:SpaceCenter.Engine.Propellants" />.
 -}
newtype Propellant = Propellant { propellantId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Propellant where
    encodePb   = encodePb . propellantId
    decodePb b = Propellant <$> decodePb b

instance KRPCResponseExtractable Propellant

{-
 - An RCS block or thruster. Obtained by calling <see cref="M:SpaceCenter.Part.RCS" />.
 -}
newtype RCS = RCS { rCSId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable RCS where
    encodePb   = encodePb . rCSId
    decodePb b = RCS <$> decodePb b

instance KRPCResponseExtractable RCS

{-
 - A radiator. Obtained by calling <see cref="M:SpaceCenter.Part.Radiator" />.
 -}
newtype Radiator = Radiator { radiatorId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Radiator where
    encodePb   = encodePb . radiatorId
    decodePb b = Radiator <$> decodePb b

instance KRPCResponseExtractable Radiator

{-
 - A reaction wheel. Obtained by calling <see cref="M:SpaceCenter.Part.ReactionWheel" />.
 -}
newtype ReactionWheel = ReactionWheel { reactionWheelId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ReactionWheel where
    encodePb   = encodePb . reactionWheelId
    decodePb b = ReactionWheel <$> decodePb b

instance KRPCResponseExtractable ReactionWheel

{-
 - Represents a reference frame for positions, rotations and
 - velocities. Contains:
 - <list type="bullet">The position of the origin.The directions of the x, y and z axes.The linear velocity of the frame.The angular velocity of the frame.This class does not contain any properties or methods. It is only
 - used as a parameter to other functions.
 -}
newtype ReferenceFrame = ReferenceFrame { referenceFrameId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ReferenceFrame where
    encodePb   = encodePb . referenceFrameId
    decodePb b = ReferenceFrame <$> decodePb b

instance KRPCResponseExtractable ReferenceFrame

{-
 - An individual resource stored within a part.
 - Created using methods in the <see cref="T:SpaceCenter.Resources" /> class.
 -}
newtype Resource = Resource { resourceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Resource where
    encodePb   = encodePb . resourceId
    decodePb b = Resource <$> decodePb b

instance KRPCResponseExtractable Resource

{-
 - A resource converter. Obtained by calling <see cref="M:SpaceCenter.Part.ResourceConverter" />.
 -}
newtype ResourceConverter = ResourceConverter { resourceConverterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceConverter where
    encodePb   = encodePb . resourceConverterId
    decodePb b = ResourceConverter <$> decodePb b

instance KRPCResponseExtractable ResourceConverter

{-
 - A resource harvester (drill). Obtained by calling <see cref="M:SpaceCenter.Part.ResourceHarvester" />.
 -}
newtype ResourceHarvester = ResourceHarvester { resourceHarvesterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceHarvester where
    encodePb   = encodePb . resourceHarvesterId
    decodePb b = ResourceHarvester <$> decodePb b

instance KRPCResponseExtractable ResourceHarvester

{-
 - Transfer resources between parts.
 -}
newtype ResourceTransfer = ResourceTransfer { resourceTransferId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceTransfer where
    encodePb   = encodePb . resourceTransferId
    decodePb b = ResourceTransfer <$> decodePb b

instance KRPCResponseExtractable ResourceTransfer

{-
 - Represents the collection of resources stored in a vessel, stage or part.
 - Created by calling <see cref="M:SpaceCenter.Vessel.Resources" />,
 - <see cref="M:SpaceCenter.Vessel.ResourcesInDecoupleStage" /> or
 - <see cref="M:SpaceCenter.Part.Resources" />.
 -}
newtype Resources = Resources { resourcesId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Resources where
    encodePb   = encodePb . resourcesId
    decodePb b = Resources <$> decodePb b

instance KRPCResponseExtractable Resources

{-
 - Obtained by calling <see cref="M:SpaceCenter.Experiment.Data" />.
 -}
newtype ScienceData = ScienceData { scienceDataId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ScienceData where
    encodePb   = encodePb . scienceDataId
    decodePb b = ScienceData <$> decodePb b

instance KRPCResponseExtractable ScienceData

{-
 - Obtained by calling <see cref="M:SpaceCenter.Experiment.ScienceSubject" />.
 -}
newtype ScienceSubject = ScienceSubject { scienceSubjectId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ScienceSubject where
    encodePb   = encodePb . scienceSubjectId
    decodePb b = ScienceSubject <$> decodePb b

instance KRPCResponseExtractable ScienceSubject

{-
 - A sensor, such as a thermometer. Obtained by calling <see cref="M:SpaceCenter.Part.Sensor" />.
 -}
newtype Sensor = Sensor { sensorId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Sensor where
    encodePb   = encodePb . sensorId
    decodePb b = Sensor <$> decodePb b

instance KRPCResponseExtractable Sensor

{-
 - A solar panel. Obtained by calling <see cref="M:SpaceCenter.Part.SolarPanel" />.
 -}
newtype SolarPanel = SolarPanel { solarPanelId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable SolarPanel where
    encodePb   = encodePb . solarPanelId
    decodePb b = SolarPanel <$> decodePb b

instance KRPCResponseExtractable SolarPanel

{-
 - The component of an <see cref="T:SpaceCenter.Engine" /> or <see cref="T:SpaceCenter.RCS" /> part that generates thrust.
 - Can obtained by calling <see cref="M:SpaceCenter.Engine.Thrusters" /> or <see cref="M:SpaceCenter.RCS.Thrusters" />.Engines can consist of multiple thrusters.
 - For example, the S3 KS-25x4 "Mammoth" has four rocket nozzels, and so consists of four thrusters.
 -}
newtype Thruster = Thruster { thrusterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Thruster where
    encodePb   = encodePb . thrusterId
    decodePb b = Thruster <$> decodePb b

instance KRPCResponseExtractable Thruster

{-
 - These objects are used to interact with vessels in KSP. This includes getting
 - orbital and flight data, manipulating control inputs and managing resources.
 - Created using <see cref="M:SpaceCenter.ActiveVessel" /> or <see cref="M:SpaceCenter.Vessels" />.
 -}
newtype Vessel = Vessel { vesselId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Vessel where
    encodePb   = encodePb . vesselId
    decodePb b = Vessel <$> decodePb b

instance KRPCResponseExtractable Vessel

{-
 - Represents a waypoint. Can be created using <see cref="M:SpaceCenter.WaypointManager.AddWaypoint" />.
 -}
newtype Waypoint = Waypoint { waypointId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Waypoint where
    encodePb   = encodePb . waypointId
    decodePb b = Waypoint <$> decodePb b

instance KRPCResponseExtractable Waypoint

{-
 - Waypoints are the location markers you can see on the map view showing you where contracts are targeted for.
 - With this structure, you can obtain coordinate data for the locations of these waypoints.
 - Obtained by calling <see cref="M:SpaceCenter.WaypointManager" />.
 -}
newtype WaypointManager = WaypointManager { waypointManagerId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable WaypointManager where
    encodePb   = encodePb . waypointManagerId
    decodePb b = WaypointManager <$> decodePb b

instance KRPCResponseExtractable WaypointManager

{-
 - A wheel. Includes landing gear and rover wheels. Obtained by calling <see cref="M:SpaceCenter.Part.Wheel" />.
 - Can be used to control the motors, steering and deployment of wheels, among other things.
 -}
newtype Wheel = Wheel { wheelId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Wheel where
    encodePb   = encodePb . wheelId
    decodePb b = Wheel <$> decodePb b

instance KRPCResponseExtractable Wheel


{-
 - The state of an antenna. See <see cref="M:SpaceCenter.Antenna.State" />.
 -}
data AntennaState
    = AntennaState'Deployed
    | AntennaState'Retracted
    | AntennaState'Deploying
    | AntennaState'Retracting
    | AntennaState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable AntennaState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable AntennaState

{-
 - See <see cref="M:SpaceCenter.Camera.Mode" />.
 -}
data CameraMode
    = CameraMode'Automatic
    | CameraMode'Free
    | CameraMode'Chase
    | CameraMode'Locked
    | CameraMode'Orbital
    | CameraMode'IVA
    | CameraMode'Map
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable CameraMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable CameraMode

{-
 - The state of a cargo bay. See <see cref="M:SpaceCenter.CargoBay.State" />.
 -}
data CargoBayState
    = CargoBayState'Open
    | CargoBayState'Closed
    | CargoBayState'Opening
    | CargoBayState'Closing
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable CargoBayState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable CargoBayState

{-
 - The type of a communication link.
 - See <see cref="M:SpaceCenter.CommLink.Type" />.
 -}
data CommLinkType
    = CommLinkType'Home
    | CommLinkType'Control
    | CommLinkType'Relay
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable CommLinkType where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable CommLinkType

{-
 - The state of a contract. See <see cref="M:SpaceCenter.Contract.State" />.
 -}
data ContractState
    = ContractState'Active
    | ContractState'Canceled
    | ContractState'Completed
    | ContractState'DeadlineExpired
    | ContractState'Declined
    | ContractState'Failed
    | ContractState'Generated
    | ContractState'Offered
    | ContractState'OfferExpired
    | ContractState'Withdrawn
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ContractState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ContractState

{-
 - The control source of a vessel.
 - See <see cref="M:SpaceCenter.Control.Source" />.
 -}
data ControlSource
    = ControlSource'Kerbal
    | ControlSource'Probe
    | ControlSource'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ControlSource where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ControlSource

{-
 - The control state of a vessel.
 - See <see cref="M:SpaceCenter.Control.State" />.
 -}
data ControlState
    = ControlState'Full
    | ControlState'Partial
    | ControlState'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ControlState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ControlState

{-
 - The state of a docking port. See <see cref="M:SpaceCenter.DockingPort.State" />.
 -}
data DockingPortState
    = DockingPortState'Ready
    | DockingPortState'Docked
    | DockingPortState'Docking
    | DockingPortState'Undocking
    | DockingPortState'Shielded
    | DockingPortState'Moving
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable DockingPortState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable DockingPortState

{-
 - The state of a landing leg. See <see cref="M:SpaceCenter.Leg.State" />.
 -}
data LegState
    = LegState'Deployed
    | LegState'Retracted
    | LegState'Deploying
    | LegState'Retracting
    | LegState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable LegState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable LegState

{-
 - The state of the motor on a powered wheel. See <see cref="M:SpaceCenter.Wheel.MotorState" />.
 -}
data MotorState
    = MotorState'Idle
    | MotorState'Running
    | MotorState'Disabled
    | MotorState'Inoperable
    | MotorState'NotEnoughResources
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable MotorState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable MotorState

{-
 - The state of a parachute. See <see cref="M:SpaceCenter.Parachute.State" />.
 -}
data ParachuteState
    = ParachuteState'Stowed
    | ParachuteState'Armed
    | ParachuteState'Active
    | ParachuteState'SemiDeployed
    | ParachuteState'Deployed
    | ParachuteState'Cut
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ParachuteState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ParachuteState

{-
 - The state of a radiator. <see cref="T:SpaceCenter.RadiatorState" />
 -}
data RadiatorState
    = RadiatorState'Extended
    | RadiatorState'Retracted
    | RadiatorState'Extending
    | RadiatorState'Retracting
    | RadiatorState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable RadiatorState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable RadiatorState

{-
 - The state of a resource converter. See <see cref="M:SpaceCenter.ResourceConverter.State" />.
 -}
data ResourceConverterState
    = ResourceConverterState'Running
    | ResourceConverterState'Idle
    | ResourceConverterState'MissingResource
    | ResourceConverterState'StorageFull
    | ResourceConverterState'Capacity
    | ResourceConverterState'Unknown
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ResourceConverterState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ResourceConverterState

{-
 - The way in which a resource flows between parts. See <see cref="M:SpaceCenter.Resources.FlowMode" />.
 -}
data ResourceFlowMode
    = ResourceFlowMode'Vessel
    | ResourceFlowMode'Stage
    | ResourceFlowMode'Adjacent
    | ResourceFlowMode'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ResourceFlowMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ResourceFlowMode

{-
 - The state of a resource harvester. See <see cref="M:SpaceCenter.ResourceHarvester.State" />.
 -}
data ResourceHarvesterState
    = ResourceHarvesterState'Deploying
    | ResourceHarvesterState'Deployed
    | ResourceHarvesterState'Retracting
    | ResourceHarvesterState'Retracted
    | ResourceHarvesterState'Active
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ResourceHarvesterState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ResourceHarvesterState

{-
 - The behavior of the SAS auto-pilot. See <see cref="M:SpaceCenter.AutoPilot.SASMode" />.
 -}
data SASMode
    = SASMode'StabilityAssist
    | SASMode'Maneuver
    | SASMode'Prograde
    | SASMode'Retrograde
    | SASMode'Normal
    | SASMode'AntiNormal
    | SASMode'Radial
    | SASMode'AntiRadial
    | SASMode'Target
    | SASMode'AntiTarget
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable SASMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable SASMode

{-
 - The state of a solar panel. See <see cref="M:SpaceCenter.SolarPanel.State" />.
 -}
data SolarPanelState
    = SolarPanelState'Extended
    | SolarPanelState'Retracted
    | SolarPanelState'Extending
    | SolarPanelState'Retracting
    | SolarPanelState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable SolarPanelState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable SolarPanelState

{-
 - The mode of the speed reported in the navball.
 - See <see cref="M:SpaceCenter.Control.SpeedMode" />.
 -}
data SpeedMode
    = SpeedMode'Orbit
    | SpeedMode'Surface
    | SpeedMode'Target
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable SpeedMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable SpeedMode

{-
 - The situation a vessel is in.
 - See <see cref="M:SpaceCenter.Vessel.Situation" />.
 -}
data VesselSituation
    = VesselSituation'PreLaunch
    | VesselSituation'Orbiting
    | VesselSituation'SubOrbital
    | VesselSituation'Escaping
    | VesselSituation'Flying
    | VesselSituation'Landed
    | VesselSituation'Splashed
    | VesselSituation'Docked
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable VesselSituation where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable VesselSituation

{-
 - The type of a vessel.
 - See <see cref="M:SpaceCenter.Vessel.Type" />.
 -}
data VesselType
    = VesselType'Ship
    | VesselType'Station
    | VesselType'Lander
    | VesselType'Probe
    | VesselType'Rover
    | VesselType'Base
    | VesselType'Debris
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable VesselType where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable VesselType

{-
 - The time warp mode.
 - Returned by <see cref="T:SpaceCenter.WarpMode" />
 -}
data WarpMode
    = WarpMode'Rails
    | WarpMode'Physics
    | WarpMode'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable WarpMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable WarpMode

{-
 - The state of a wheel. See <see cref="M:SpaceCenter.Wheel.State" />.
 -}
data WheelState
    = WheelState'Deployed
    | WheelState'Retracted
    | WheelState'Deploying
    | WheelState'Retracting
    | WheelState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable WheelState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable WheelState


{-
 - Cancel current transmission of data.
 -}
antennaCancel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m ()
antennaCancel thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_Cancel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Transmit data.
 -}
antennaTransmit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m ()
antennaTransmit thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_Transmit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether partial data transmission is permitted.
 -}
getAntennaAllowPartial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Bool)
getAntennaAllowPartial thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_AllowPartial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaAllowPartialStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Bool)
getAntennaAllowPartialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_AllowPartial" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaAllowPartialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Bool))
getAntennaAllowPartialStream thisArg = requestStream $ getAntennaAllowPartialStreamReq thisArg 

{-
 - Whether data can be transmitted by this antenna.
 -}
getAntennaCanTransmit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Bool)
getAntennaCanTransmit thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_CanTransmit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaCanTransmitStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Bool)
getAntennaCanTransmitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_CanTransmit" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaCanTransmitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Bool))
getAntennaCanTransmitStream thisArg = requestStream $ getAntennaCanTransmitStreamReq thisArg 

{-
 - Whether the antenna can be combined with other antennae on the vessel to boost the power.
 -}
getAntennaCombinable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Bool)
getAntennaCombinable thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_Combinable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaCombinableStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Bool)
getAntennaCombinableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_Combinable" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaCombinableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Bool))
getAntennaCombinableStream thisArg = requestStream $ getAntennaCombinableStreamReq thisArg 

{-
 - Exponent used to calculate the combined power of multiple antennae on a vessel.
 -}
getAntennaCombinableExponent :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Double)
getAntennaCombinableExponent thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_CombinableExponent" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaCombinableExponentStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Double)
getAntennaCombinableExponentStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_CombinableExponent" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaCombinableExponentStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Double))
getAntennaCombinableExponentStream thisArg = requestStream $ getAntennaCombinableExponentStreamReq thisArg 

{-
 - Whether the antenna is deployable.
 -}
getAntennaDeployable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Bool)
getAntennaDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaDeployableStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Bool)
getAntennaDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaDeployableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Bool))
getAntennaDeployableStream thisArg = requestStream $ getAntennaDeployableStreamReq thisArg 

{-
 - Whether the antenna is deployed.Fixed antennas are always deployed.
 - Returns an error if you try to deploy a fixed antenna.
 -}
getAntennaDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Bool)
getAntennaDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaDeployedStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Bool)
getAntennaDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Bool))
getAntennaDeployedStream thisArg = requestStream $ getAntennaDeployedStreamReq thisArg 

{-
 - Interval between sending packets in seconds.
 -}
getAntennaPacketInterval :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Float)
getAntennaPacketInterval thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_PacketInterval" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaPacketIntervalStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Float)
getAntennaPacketIntervalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_PacketInterval" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaPacketIntervalStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Float))
getAntennaPacketIntervalStream thisArg = requestStream $ getAntennaPacketIntervalStreamReq thisArg 

{-
 - Units of electric charge consumed per packet sent.
 -}
getAntennaPacketResourceCost :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Double)
getAntennaPacketResourceCost thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_PacketResourceCost" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaPacketResourceCostStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Double)
getAntennaPacketResourceCostStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_PacketResourceCost" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaPacketResourceCostStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Double))
getAntennaPacketResourceCostStream thisArg = requestStream $ getAntennaPacketResourceCostStreamReq thisArg 

{-
 - Amount of data sent per packet in Mits.
 -}
getAntennaPacketSize :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Float)
getAntennaPacketSize thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_PacketSize" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaPacketSizeStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Float)
getAntennaPacketSizeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_PacketSize" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaPacketSizeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Float))
getAntennaPacketSizeStream thisArg = requestStream $ getAntennaPacketSizeStreamReq thisArg 

{-
 - The part object for this antenna.
 -}
getAntennaPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCHS.SpaceCenter.Part)
getAntennaPart thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaPartStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getAntennaPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getAntennaPartStream thisArg = requestStream $ getAntennaPartStreamReq thisArg 

{-
 - The power of the antenna.
 -}
getAntennaPower :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (Double)
getAntennaPower thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_Power" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaPowerStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (Double)
getAntennaPowerStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_Power" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaPowerStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (Double))
getAntennaPowerStream thisArg = requestStream $ getAntennaPowerStreamReq thisArg 

{-
 - The current state of the antenna.
 -}
getAntennaState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCHS.SpaceCenter.AntennaState)
getAntennaState thisArg = do
    let r = makeRequest "SpaceCenter" "Antenna_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAntennaStateStreamReq :: KRPCHS.SpaceCenter.Antenna -> KRPCStreamReq (KRPCHS.SpaceCenter.AntennaState)
getAntennaStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Antenna_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getAntennaStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> m (KRPCStream (KRPCHS.SpaceCenter.AntennaState))
getAntennaStateStream thisArg = requestStream $ getAntennaStateStreamReq thisArg 

{-
 - Whether partial data transmission is permitted.
 -}
setAntennaAllowPartial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> Bool -> m ()
setAntennaAllowPartial thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Antenna_set_AllowPartial" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the antenna is deployed.Fixed antennas are always deployed.
 - Returns an error if you try to deploy a fixed antenna.
 -}
setAntennaDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Antenna -> Bool -> m ()
setAntennaDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Antenna_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Disengage the auto-pilot.
 -}
autoPilotDisengage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ()
autoPilotDisengage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Disengage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Engage the auto-pilot.
 -}
autoPilotEngage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ()
autoPilotEngage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Engage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set target pitch and heading angles.<param name="pitch">Target pitch angle, in degrees between -90° and +90°.<param name="heading">Target heading angle, in degrees between 0° and 360°.
 -}
autoPilotTargetPitchAndHeading :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Float -> Float -> m ()
autoPilotTargetPitchAndHeading thisArg pitchArg headingArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_TargetPitchAndHeading" [makeArgument 0 thisArg, makeArgument 1 pitchArg, makeArgument 2 headingArg]
    res <- sendRequest r
    processResponse res 

{-
 - Blocks until the vessel is pointing in the target direction and has the target roll (if set).
 -}
autoPilotWait :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ()
autoPilotWait thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Wait" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The angle at which the autopilot considers the vessel to be pointing close to the target.
 - This determines the midpoint of the target velocity attenuation function.
 - A vector of three angles, in degrees, one for each of the pitch, roll and yaw axes.
 - Defaults to 1° for each axis.
 -}
getAutoPilotAttenuationAngle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotAttenuationAngle thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_AttenuationAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotAttenuationAngleStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotAttenuationAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_AttenuationAngle" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotAttenuationAngleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotAttenuationAngleStream thisArg = requestStream $ getAutoPilotAttenuationAngleStreamReq thisArg 

{-
 - Whether the rotation rate controllers PID parameters should be automatically tuned using the
 - vessels moment of inertia and available torque. Defaults totrue.
 - See <see cref="M:SpaceCenter.AutoPilot.TimeToPeak" /> and <see cref="M:SpaceCenter.AutoPilot.Overshoot" />.
 -}
getAutoPilotAutoTune :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Bool)
getAutoPilotAutoTune thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_AutoTune" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotAutoTuneStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Bool)
getAutoPilotAutoTuneStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_AutoTune" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotAutoTuneStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Bool))
getAutoPilotAutoTuneStream thisArg = requestStream $ getAutoPilotAutoTuneStreamReq thisArg 

{-
 - The time the vessel should take to come to a stop pointing in the target direction.
 - This determines the angular acceleration used to decelerate the vessel.
 - A vector of three times, in seconds, one for each of the pitch, roll and yaw axes.
 - Defaults to 5 seconds for each axis.
 -}
getAutoPilotDecelerationTime :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotDecelerationTime thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_DecelerationTime" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotDecelerationTimeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotDecelerationTimeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_DecelerationTime" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotDecelerationTimeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotDecelerationTimeStream thisArg = requestStream $ getAutoPilotDecelerationTimeStreamReq thisArg 

{-
 - The error, in degrees, between the direction the ship has been asked
 - to point in and the direction it is pointing in. Returns zero if the auto-pilot
 - has not been engaged and SAS is not enabled or is in stability assist mode.
 -}
getAutoPilotError :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotErrorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotErrorStream thisArg = requestStream $ getAutoPilotErrorStreamReq thisArg 

{-
 - The error, in degrees, between the vessels current and target heading.
 - Returns zero if the auto-pilot has not been engaged.
 -}
getAutoPilotHeadingError :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotHeadingError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_HeadingError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotHeadingErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotHeadingErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_HeadingError" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotHeadingErrorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotHeadingErrorStream thisArg = requestStream $ getAutoPilotHeadingErrorStreamReq thisArg 

{-
 - The target overshoot percentage used to autotune the PID controllers.
 - A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
 - Defaults to 0.01 for each axis.
 -}
getAutoPilotOvershoot :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotOvershoot thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Overshoot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotOvershootStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotOvershootStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_Overshoot" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotOvershootStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotOvershootStream thisArg = requestStream $ getAutoPilotOvershootStreamReq thisArg 

{-
 - The error, in degrees, between the vessels current and target pitch.
 - Returns zero if the auto-pilot has not been engaged.
 -}
getAutoPilotPitchError :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotPitchError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_PitchError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotPitchErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotPitchErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_PitchError" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotPitchErrorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotPitchErrorStream thisArg = requestStream $ getAutoPilotPitchErrorStreamReq thisArg 

{-
 - Gains for the pitch PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotPitchPIDGains :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotPitchPIDGains thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_PitchPIDGains" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotPitchPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotPitchPIDGainsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_PitchPIDGains" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotPitchPIDGainsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotPitchPIDGainsStream thisArg = requestStream $ getAutoPilotPitchPIDGainsStreamReq thisArg 

{-
 - The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).An error will be thrown if this property is set to a reference frame that rotates with the vessel being controlled,
 - as it is impossible to rotate the vessel in such a reference frame.
 -}
getAutoPilotReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotReferenceFrameStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getAutoPilotReferenceFrameStream thisArg = requestStream $ getAutoPilotReferenceFrameStreamReq thisArg 

{-
 - The error, in degrees, between the vessels current and target roll.
 - Returns zero if the auto-pilot has not been engaged or no target roll is set.
 -}
getAutoPilotRollError :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotRollError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotRollErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotRollErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotRollErrorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotRollErrorStream thisArg = requestStream $ getAutoPilotRollErrorStreamReq thisArg 

{-
 - Gains for the roll PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotRollPIDGains :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotRollPIDGains thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollPIDGains" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotRollPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotRollPIDGainsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_RollPIDGains" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotRollPIDGainsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotRollPIDGainsStream thisArg = requestStream $ getAutoPilotRollPIDGainsStreamReq thisArg 

{-
 - The threshold at which the autopilot will try to match the target roll angle, if any.
 - Defaults to 5 degrees.
 -}
getAutoPilotRollThreshold :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Double)
getAutoPilotRollThreshold thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollThreshold" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotRollThresholdStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Double)
getAutoPilotRollThresholdStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_RollThreshold" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotRollThresholdStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Double))
getAutoPilotRollThresholdStream thisArg = requestStream $ getAutoPilotRollThresholdStreamReq thisArg 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
getAutoPilotSAS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Bool)
getAutoPilotSAS thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotSASStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Bool)
getAutoPilotSASStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotSASStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Bool))
getAutoPilotSASStream thisArg = requestStream $ getAutoPilotSASStreamReq thisArg 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
getAutoPilotSASMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotSASModeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotSASModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getAutoPilotSASModeStream thisArg = requestStream $ getAutoPilotSASModeStreamReq thisArg 

{-
 - The maximum amount of time that the vessel should need to come to a complete stop.
 - This determines the maximum angular velocity of the vessel.
 - A vector of three stopping times, in seconds, one for each of the pitch, roll and yaw axes.
 - Defaults to 0.5 seconds for each axis.
 -}
getAutoPilotStoppingTime :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotStoppingTime thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_StoppingTime" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotStoppingTimeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotStoppingTimeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_StoppingTime" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotStoppingTimeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotStoppingTimeStream thisArg = requestStream $ getAutoPilotStoppingTimeStreamReq thisArg 

{-
 - Direction vector corresponding to the target pitch and heading.
 -}
getAutoPilotTargetDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotTargetDirection thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetDirectionStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotTargetDirectionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotTargetDirectionStream thisArg = requestStream $ getAutoPilotTargetDirectionStreamReq thisArg 

{-
 - The target heading, in degrees, between 0° and 360°.
 -}
getAutoPilotTargetHeading :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotTargetHeading thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetHeading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetHeadingStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetHeadingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetHeading" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetHeadingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotTargetHeadingStream thisArg = requestStream $ getAutoPilotTargetHeadingStreamReq thisArg 

{-
 - The target pitch, in degrees, between -90° and +90°.
 -}
getAutoPilotTargetPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotTargetPitch thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetPitchStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetPitch" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetPitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotTargetPitchStream thisArg = requestStream $ getAutoPilotTargetPitchStreamReq thisArg 

{-
 - The target roll, in degrees.NaNif no target roll is set.
 -}
getAutoPilotTargetRoll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (Float)
getAutoPilotTargetRoll thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetRollStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetRollStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetRollStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream (Float))
getAutoPilotTargetRollStream thisArg = requestStream $ getAutoPilotTargetRollStreamReq thisArg 

{-
 - The target time to peak used to autotune the PID controllers.
 - A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
 - Defaults to 3 seconds for each axis.
 -}
getAutoPilotTimeToPeak :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotTimeToPeak thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TimeToPeak" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTimeToPeakStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotTimeToPeakStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TimeToPeak" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTimeToPeakStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotTimeToPeakStream thisArg = requestStream $ getAutoPilotTimeToPeakStreamReq thisArg 

{-
 - Gains for the yaw PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotYawPIDGains :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m ((Double, Double, Double))
getAutoPilotYawPIDGains thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_YawPIDGains" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotYawPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotYawPIDGainsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_YawPIDGains" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotYawPIDGainsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> m (KRPCStream ((Double, Double, Double)))
getAutoPilotYawPIDGainsStream thisArg = requestStream $ getAutoPilotYawPIDGainsStreamReq thisArg 

{-
 - The angle at which the autopilot considers the vessel to be pointing close to the target.
 - This determines the midpoint of the target velocity attenuation function.
 - A vector of three angles, in degrees, one for each of the pitch, roll and yaw axes.
 - Defaults to 1° for each axis.
 -}
setAutoPilotAttenuationAngle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotAttenuationAngle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_AttenuationAngle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the rotation rate controllers PID parameters should be automatically tuned using the
 - vessels moment of inertia and available torque. Defaults totrue.
 - See <see cref="M:SpaceCenter.AutoPilot.TimeToPeak" /> and <see cref="M:SpaceCenter.AutoPilot.Overshoot" />.
 -}
setAutoPilotAutoTune :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Bool -> m ()
setAutoPilotAutoTune thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_AutoTune" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time the vessel should take to come to a stop pointing in the target direction.
 - This determines the angular acceleration used to decelerate the vessel.
 - A vector of three times, in seconds, one for each of the pitch, roll and yaw axes.
 - Defaults to 5 seconds for each axis.
 -}
setAutoPilotDecelerationTime :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotDecelerationTime thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_DecelerationTime" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target overshoot percentage used to autotune the PID controllers.
 - A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
 - Defaults to 0.01 for each axis.
 -}
setAutoPilotOvershoot :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotOvershoot thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_Overshoot" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Gains for the pitch PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotPitchPIDGains :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotPitchPIDGains thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_PitchPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).An error will be thrown if this property is set to a reference frame that rotates with the vessel being controlled,
 - as it is impossible to rotate the vessel in such a reference frame.
 -}
setAutoPilotReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.ReferenceFrame -> m ()
setAutoPilotReferenceFrame thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Gains for the roll PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotRollPIDGains :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotRollPIDGains thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RollPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The threshold at which the autopilot will try to match the target roll angle, if any.
 - Defaults to 5 degrees.
 -}
setAutoPilotRollThreshold :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Double -> m ()
setAutoPilotRollThreshold thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RollThreshold" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
setAutoPilotSAS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Bool -> m ()
setAutoPilotSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
setAutoPilotSASMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.SASMode -> m ()
setAutoPilotSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The maximum amount of time that the vessel should need to come to a complete stop.
 - This determines the maximum angular velocity of the vessel.
 - A vector of three stopping times, in seconds, one for each of the pitch, roll and yaw axes.
 - Defaults to 0.5 seconds for each axis.
 -}
setAutoPilotStoppingTime :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotStoppingTime thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_StoppingTime" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Direction vector corresponding to the target pitch and heading.
 -}
setAutoPilotTargetDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotTargetDirection thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetDirection" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target heading, in degrees, between 0° and 360°.
 -}
setAutoPilotTargetHeading :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Float -> m ()
setAutoPilotTargetHeading thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetHeading" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target pitch, in degrees, between -90° and +90°.
 -}
setAutoPilotTargetPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Float -> m ()
setAutoPilotTargetPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetPitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target roll, in degrees.NaNif no target roll is set.
 -}
setAutoPilotTargetRoll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> Float -> m ()
setAutoPilotTargetRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetRoll" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target time to peak used to autotune the PID controllers.
 - A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
 - Defaults to 3 seconds for each axis.
 -}
setAutoPilotTimeToPeak :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotTimeToPeak thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TimeToPeak" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Gains for the yaw PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotYawPIDGains :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> m ()
setAutoPilotYawPIDGains thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_YawPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Default distance from the camera to the subject, in meters.
 -}
getCameraDefaultDistance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraDefaultDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraDefaultDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraDefaultDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraDefaultDistanceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraDefaultDistanceStream thisArg = requestStream $ getCameraDefaultDistanceStreamReq thisArg 

{-
 - The distance from the camera to the subject, in meters.
 - A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
getCameraDistance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraDistanceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraDistanceStream thisArg = requestStream $ getCameraDistanceStreamReq thisArg 

{-
 - In map mode, the celestial body that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a celestial body.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBody thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraFocussedBodyStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBodyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]
    in  makeStream req

getCameraFocussedBodyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getCameraFocussedBodyStream thisArg = requestStream $ getCameraFocussedBodyStreamReq thisArg 

{-
 - In map mode, the maneuver node that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a maneuver node.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedNode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCHS.SpaceCenter.Node)
getCameraFocussedNode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraFocussedNodeStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.Node)
getCameraFocussedNodeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]
    in  makeStream req

getCameraFocussedNodeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (KRPCHS.SpaceCenter.Node))
getCameraFocussedNodeStream thisArg = requestStream $ getCameraFocussedNodeStreamReq thisArg 

{-
 - In map mode, the vessel that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a vessel.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraFocussedVesselStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVesselStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]
    in  makeStream req

getCameraFocussedVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCameraFocussedVesselStream thisArg = requestStream $ getCameraFocussedVesselStreamReq thisArg 

{-
 - The heading of the camera, in degrees.
 -}
getCameraHeading :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraHeadingStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraHeadingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]
    in  makeStream req

getCameraHeadingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraHeadingStream thisArg = requestStream $ getCameraHeadingStreamReq thisArg 

{-
 - Maximum distance from the camera to the subject, in meters.
 -}
getCameraMaxDistance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraMaxDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMaxDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMaxDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMaxDistanceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraMaxDistanceStream thisArg = requestStream $ getCameraMaxDistanceStreamReq thisArg 

{-
 - The maximum pitch of the camera.
 -}
getCameraMaxPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraMaxPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMaxPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMaxPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMaxPitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraMaxPitchStream thisArg = requestStream $ getCameraMaxPitchStreamReq thisArg 

{-
 - Minimum distance from the camera to the subject, in meters.
 -}
getCameraMinDistance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraMinDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMinDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMinDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMinDistanceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraMinDistanceStream thisArg = requestStream $ getCameraMinDistanceStreamReq thisArg 

{-
 - The minimum pitch of the camera.
 -}
getCameraMinPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraMinPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMinPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMinPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMinPitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraMinPitchStream thisArg = requestStream $ getCameraMinPitchStreamReq thisArg 

{-
 - The current mode of the camera.
 -}
getCameraMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCHS.SpaceCenter.CameraMode)
getCameraMode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraModeStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.CameraMode)
getCameraModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]
    in  makeStream req

getCameraModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (KRPCHS.SpaceCenter.CameraMode))
getCameraModeStream thisArg = requestStream $ getCameraModeStreamReq thisArg 

{-
 - The pitch of the camera, in degrees.
 - A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
getCameraPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (Float)
getCameraPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]
    in  makeStream req

getCameraPitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> m (KRPCStream (Float))
getCameraPitchStream thisArg = requestStream $ getCameraPitchStreamReq thisArg 

{-
 - The distance from the camera to the subject, in meters.
 - A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
setCameraDistance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> Float -> m ()
setCameraDistance thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Distance" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - In map mode, the celestial body that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a celestial body.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CelestialBody -> m ()
setCameraFocussedBody thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - In map mode, the maneuver node that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a maneuver node.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedNode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Node -> m ()
setCameraFocussedNode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedNode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - In map mode, the vessel that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a vessel.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Vessel -> m ()
setCameraFocussedVessel thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The heading of the camera, in degrees.
 -}
setCameraHeading :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> Float -> m ()
setCameraHeading thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Heading" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current mode of the camera.
 -}
setCameraMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CameraMode -> m ()
setCameraMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The pitch of the camera, in degrees.
 - A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
setCameraPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Camera -> Float -> m ()
setCameraPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returnstrueif regular "on-rails" time warp can be used, at the specified warp
 - <paramref name="factor" />. The maximum time warp rate is limited by various things,
 - including how close the active vessel is to a planet. See
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">the KSP wikifor details.<param name="factor">The warp factor to check.
 -}
canRailsWarpAt :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Int.Int32 -> m (Bool)
canRailsWarpAt factorArg = do
    let r = makeRequest "SpaceCenter" "CanRailsWarpAt" [makeArgument 0 factorArg]
    res <- sendRequest r
    processResponse res

canRailsWarpAtStreamReq :: Data.Int.Int32 -> KRPCStreamReq (Bool)
canRailsWarpAtStreamReq factorArg =
    let req = makeRequest "SpaceCenter" "CanRailsWarpAt" [makeArgument 0 factorArg]
    in  makeStream req

canRailsWarpAtStream :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Int.Int32 -> m (KRPCStream (Bool))
canRailsWarpAtStream factorArg = requestStream $ canRailsWarpAtStreamReq factorArg 

{-
 - Whether the cargo bay is open.
 -}
getCargoBayOpen :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> m (Bool)
getCargoBayOpen thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCargoBayOpenStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (Bool)
getCargoBayOpenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]
    in  makeStream req

getCargoBayOpenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> m (KRPCStream (Bool))
getCargoBayOpenStream thisArg = requestStream $ getCargoBayOpenStreamReq thisArg 

{-
 - The part object for this cargo bay.
 -}
getCargoBayPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> m (KRPCHS.SpaceCenter.Part)
getCargoBayPart thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCargoBayPartStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getCargoBayPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getCargoBayPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getCargoBayPartStream thisArg = requestStream $ getCargoBayPartStreamReq thisArg 

{-
 - The state of the cargo bay.
 -}
getCargoBayState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> m (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayState thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCargoBayStateStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getCargoBayStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> m (KRPCStream (KRPCHS.SpaceCenter.CargoBayState))
getCargoBayStateStream thisArg = requestStream $ getCargoBayStateStreamReq thisArg 

{-
 - Whether the cargo bay is open.
 -}
setCargoBayOpen :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CargoBay -> Bool -> m ()
setCargoBayOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns the angular velocity of the body in the specified reference
 - frame. The magnitude of the vector is the rotational speed of the body, in
 - radians per second, and the direction of the vector indicates the axis of
 - rotation, using the right-hand rule.<param name="referenceFrame">
 -}
celestialBodyAngularVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodyAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyAngularVelocityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyAngularVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyAngularVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodyAngularVelocityStream thisArg referenceFrameArg = requestStream $ celestialBodyAngularVelocityStreamReq thisArg referenceFrameArg 

{-
 - The height of the surface relative to mean sea level at the given position,
 - in meters. When over water, this is the height of the sea-bed and is therefore a
 - negative value.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodyBedrockHeight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> m (Double)
celestialBodyBedrockHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse res

celestialBodyBedrockHeightStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Double)
celestialBodyBedrockHeightStreamReq thisArg latitudeArg longitudeArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    in  makeStream req

celestialBodyBedrockHeightStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> m (KRPCStream (Double))
celestialBodyBedrockHeightStream thisArg latitudeArg longitudeArg = requestStream $ celestialBodyBedrockHeightStreamReq thisArg latitudeArg longitudeArg 

{-
 - The position of the surface at the given latitude and longitude, in the given
 - reference frame. When over water, this is the position at the bottom of the sea-bed.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyBedrockPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodyBedrockPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyBedrockPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyBedrockPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

celestialBodyBedrockPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodyBedrockPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestStream $ celestialBodyBedrockPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-
 - The biomes at the given latitude and longitude, in degrees.
 -}
celestialBodyBiomeAt :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> m (Data.Text.Text)
celestialBodyBiomeAt thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BiomeAt" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse res

celestialBodyBiomeAtStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Data.Text.Text)
celestialBodyBiomeAtStreamReq thisArg latitudeArg longitudeArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_BiomeAt" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    in  makeStream req

celestialBodyBiomeAtStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> m (KRPCStream (Data.Text.Text))
celestialBodyBiomeAtStream thisArg latitudeArg longitudeArg = requestStream $ celestialBodyBiomeAtStreamReq thisArg latitudeArg longitudeArg 

{-
 - Returns the direction in which the north pole of the celestial body is
 - pointing, as a unit vector, in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodyDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyDirectionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodyDirectionStream thisArg referenceFrameArg = requestStream $ celestialBodyDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position at mean sea level at the given latitude and longitude, in the given reference frame.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyMSLPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodyMSLPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyMSLPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyMSLPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

celestialBodyMSLPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodyMSLPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestStream $ celestialBodyMSLPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-
 - Returns the position vector of the center of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodyPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodyPositionStream thisArg referenceFrameArg = requestStream $ celestialBodyPositionStreamReq thisArg referenceFrameArg 

{-
 - Returns the rotation of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double, Double))
celestialBodyRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyRotationStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
celestialBodyRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double, Double)))
celestialBodyRotationStream thisArg referenceFrameArg = requestStream $ celestialBodyRotationStreamReq thisArg referenceFrameArg 

{-
 - The height of the surface relative to mean sea level at the given position,
 - in meters. When over water this is equal to 0.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodySurfaceHeight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> m (Double)
celestialBodySurfaceHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse res

celestialBodySurfaceHeightStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Double)
celestialBodySurfaceHeightStreamReq thisArg latitudeArg longitudeArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    in  makeStream req

celestialBodySurfaceHeightStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> m (KRPCStream (Double))
celestialBodySurfaceHeightStream thisArg latitudeArg longitudeArg = requestStream $ celestialBodySurfaceHeightStreamReq thisArg latitudeArg longitudeArg 

{-
 - The position of the surface at the given latitude and longitude, in the given
 - reference frame. When over water, this is the position of the surface of the water.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodySurfacePosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodySurfacePosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodySurfacePositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodySurfacePositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

celestialBodySurfacePositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodySurfacePositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestStream $ celestialBodySurfacePositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-
 - Returns the velocity vector of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
celestialBodyVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyVelocityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
celestialBodyVelocityStream thisArg referenceFrameArg = requestStream $ celestialBodyVelocityStreamReq thisArg referenceFrameArg 

{-
 - The depth of the atmosphere, in meters.
 -}
getCelestialBodyAtmosphereDepth :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyAtmosphereDepth thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyAtmosphereDepthStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyAtmosphereDepthStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyAtmosphereDepthStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyAtmosphereDepthStream thisArg = requestStream $ getCelestialBodyAtmosphereDepthStreamReq thisArg 

{-
 - The biomes present on this body.
 -}
getCelestialBodyBiomes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Data.Set.Set Data.Text.Text)
getCelestialBodyBiomes thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Biomes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyBiomesStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Data.Set.Set Data.Text.Text)
getCelestialBodyBiomesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Biomes" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyBiomesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Data.Set.Set Data.Text.Text))
getCelestialBodyBiomesStream thisArg = requestStream $ getCelestialBodyBiomesStreamReq thisArg 

{-
 - The equatorial radius of the body, in meters.
 -}
getCelestialBodyEquatorialRadius :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyEquatorialRadius thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyEquatorialRadiusStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyEquatorialRadiusStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyEquatorialRadiusStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyEquatorialRadiusStream thisArg = requestStream $ getCelestialBodyEquatorialRadiusStreamReq thisArg 

{-
 - The altitude, in meters, above which a vessel is considered to be flying "high" when doing science.
 -}
getCelestialBodyFlyingHighAltitudeThreshold :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyFlyingHighAltitudeThreshold thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_FlyingHighAltitudeThreshold" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyFlyingHighAltitudeThresholdStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyFlyingHighAltitudeThresholdStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_FlyingHighAltitudeThreshold" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyFlyingHighAltitudeThresholdStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyFlyingHighAltitudeThresholdStream thisArg = requestStream $ getCelestialBodyFlyingHighAltitudeThresholdStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Standard_gravitational_parameter">standard
 - gravitational parameterof the body inm^3s^{ -2}.
 -}
getCelestialBodyGravitationalParameter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyGravitationalParameter thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyGravitationalParameterStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyGravitationalParameterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyGravitationalParameterStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyGravitationalParameterStream thisArg = requestStream $ getCelestialBodyGravitationalParameterStreamReq thisArg 

{-
 - trueif the body has an atmosphere.
 -}
getCelestialBodyHasAtmosphere :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Bool)
getCelestialBodyHasAtmosphere thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyHasAtmosphereStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Bool)
getCelestialBodyHasAtmosphereStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyHasAtmosphereStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Bool))
getCelestialBodyHasAtmosphereStream thisArg = requestStream $ getCelestialBodyHasAtmosphereStreamReq thisArg 

{-
 - trueif there is oxygen in the atmosphere, required for air-breathing engines.
 -}
getCelestialBodyHasAtmosphericOxygen :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Bool)
getCelestialBodyHasAtmosphericOxygen thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyHasAtmosphericOxygenStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Bool)
getCelestialBodyHasAtmosphericOxygenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyHasAtmosphericOxygenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Bool))
getCelestialBodyHasAtmosphericOxygenStream thisArg = requestStream $ getCelestialBodyHasAtmosphericOxygenStreamReq thisArg 

{-
 - The initial rotation angle of the body (at UT 0), in radians.
 -}
getCelestialBodyInitialRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Double)
getCelestialBodyInitialRotation thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_InitialRotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyInitialRotationStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Double)
getCelestialBodyInitialRotationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_InitialRotation" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyInitialRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Double))
getCelestialBodyInitialRotationStream thisArg = requestStream $ getCelestialBodyInitialRotationStreamReq thisArg 

{-
 - The mass of the body, in kilograms.
 -}
getCelestialBodyMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyMass thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyMassStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyMassStream thisArg = requestStream $ getCelestialBodyMassStreamReq thisArg 

{-
 - The name of the body.
 -}
getCelestialBodyName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Data.Text.Text)
getCelestialBodyName thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyNameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Data.Text.Text)
getCelestialBodyNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Data.Text.Text))
getCelestialBodyNameStream thisArg = requestStream $ getCelestialBodyNameStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this celestial body, and
 - orientated in a fixed direction (it does not rotate with the body).
 - <list type="bullet">The origin is at the center of the body.The axes do not rotate.The x-axis points in an arbitrary direction through the
 - equator.The y-axis points from the center of the body towards
 - the north pole.The z-axis points in an arbitrary direction through the
 - equator.
 -}
getCelestialBodyNonRotatingReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyNonRotatingReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyNonRotatingReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyNonRotatingReferenceFrameStream thisArg = requestStream $ getCelestialBodyNonRotatingReferenceFrameStreamReq thisArg 

{-
 - The orbit of the body.
 -}
getCelestialBodyOrbit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyOrbitStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyOrbitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getCelestialBodyOrbitStream thisArg = requestStream $ getCelestialBodyOrbitStreamReq thisArg 

{-
 - Gets the reference frame that is fixed relative to this celestial body, but
 - orientated with the body's orbital prograde/normal/radial directions.
 - <list type="bullet">The origin is at the center of the body.The axes rotate with the orbital prograde/normal/radial
 - directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.
 -}
getCelestialBodyOrbitalReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyOrbitalReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyOrbitalReferenceFrameStream thisArg = requestStream $ getCelestialBodyOrbitalReferenceFrameStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the celestial body.
 - <list type="bullet">The origin is at the center of the body.The axes rotate with the body.The x-axis points from the center of the body
 - towards the intersection of the prime meridian and equator (the
 - position at 0° longitude, 0° latitude).The y-axis points from the center of the body
 - towards the north pole.The z-axis points from the center of the body
 - towards the equator at 90°E longitude.
 -}
getCelestialBodyReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyReferenceFrameStream thisArg = requestStream $ getCelestialBodyReferenceFrameStreamReq thisArg 

{-
 - The current rotation angle of the body, in radians.
 -}
getCelestialBodyRotationAngle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Double)
getCelestialBodyRotationAngle thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyRotationAngleStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Double)
getCelestialBodyRotationAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_RotationAngle" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyRotationAngleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Double))
getCelestialBodyRotationAngleStream thisArg = requestStream $ getCelestialBodyRotationAngleStreamReq thisArg 

{-
 - The sidereal rotational period of the body, in seconds.
 -}
getCelestialBodyRotationalPeriod :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyRotationalPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyRotationalPeriodStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyRotationalPeriodStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyRotationalPeriodStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyRotationalPeriodStream thisArg = requestStream $ getCelestialBodyRotationalPeriodStreamReq thisArg 

{-
 - The rotational speed of the body, in radians per second.
 -}
getCelestialBodyRotationalSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodyRotationalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyRotationalSpeedStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyRotationalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyRotationalSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodyRotationalSpeedStream thisArg = requestStream $ getCelestialBodyRotationalSpeedStreamReq thisArg 

{-
 - A list of celestial bodies that are in orbit around this celestial body.
 -}
getCelestialBodySatellites :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellites thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySatellitesStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellitesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySatellitesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream ([KRPCHS.SpaceCenter.CelestialBody]))
getCelestialBodySatellitesStream thisArg = requestStream $ getCelestialBodySatellitesStreamReq thisArg 

{-
 - The altitude, in meters, above which a vessel is considered to be in "high" space when doing science.
 -}
getCelestialBodySpaceHighAltitudeThreshold :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodySpaceHighAltitudeThreshold thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SpaceHighAltitudeThreshold" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySpaceHighAltitudeThresholdStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySpaceHighAltitudeThresholdStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_SpaceHighAltitudeThreshold" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySpaceHighAltitudeThresholdStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodySpaceHighAltitudeThresholdStream thisArg = requestStream $ getCelestialBodySpaceHighAltitudeThresholdStreamReq thisArg 

{-
 - The radius of the sphere of influence of the body, in meters.
 -}
getCelestialBodySphereOfInfluence :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodySphereOfInfluence thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySphereOfInfluenceStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySphereOfInfluenceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySphereOfInfluenceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodySphereOfInfluenceStream thisArg = requestStream $ getCelestialBodySphereOfInfluenceStreamReq thisArg 

{-
 - The acceleration due to gravity at sea level (mean altitude) on the body, inm/s^2.
 -}
getCelestialBodySurfaceGravity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (Float)
getCelestialBodySurfaceGravity thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySurfaceGravityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySurfaceGravityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySurfaceGravityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m (KRPCStream (Float))
getCelestialBodySurfaceGravityStream thisArg = requestStream $ getCelestialBodySurfaceGravityStreamReq thisArg 

{-
 - Clears the current target.
 -}
clearTarget :: (MonadIO m, MonadThrow m, MonadRPC m) => m ()
clearTarget  = do
    let r = makeRequest "SpaceCenter" "ClearTarget" []
    res <- sendRequest r
    processResponse res 

{-
 - Start point of the link.
 -}
getCommLinkEnd :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCHS.SpaceCenter.CommNode)
getCommLinkEnd thisArg = do
    let r = makeRequest "SpaceCenter" "CommLink_get_End" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommLinkEndStreamReq :: KRPCHS.SpaceCenter.CommLink -> KRPCStreamReq (KRPCHS.SpaceCenter.CommNode)
getCommLinkEndStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommLink_get_End" [makeArgument 0 thisArg]
    in  makeStream req

getCommLinkEndStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCStream (KRPCHS.SpaceCenter.CommNode))
getCommLinkEndStream thisArg = requestStream $ getCommLinkEndStreamReq thisArg 

{-
 - Signal strength of the link.
 -}
getCommLinkSignalStrength :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (Double)
getCommLinkSignalStrength thisArg = do
    let r = makeRequest "SpaceCenter" "CommLink_get_SignalStrength" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommLinkSignalStrengthStreamReq :: KRPCHS.SpaceCenter.CommLink -> KRPCStreamReq (Double)
getCommLinkSignalStrengthStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommLink_get_SignalStrength" [makeArgument 0 thisArg]
    in  makeStream req

getCommLinkSignalStrengthStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCStream (Double))
getCommLinkSignalStrengthStream thisArg = requestStream $ getCommLinkSignalStrengthStreamReq thisArg 

{-
 - Start point of the link.
 -}
getCommLinkStart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCHS.SpaceCenter.CommNode)
getCommLinkStart thisArg = do
    let r = makeRequest "SpaceCenter" "CommLink_get_Start" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommLinkStartStreamReq :: KRPCHS.SpaceCenter.CommLink -> KRPCStreamReq (KRPCHS.SpaceCenter.CommNode)
getCommLinkStartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommLink_get_Start" [makeArgument 0 thisArg]
    in  makeStream req

getCommLinkStartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCStream (KRPCHS.SpaceCenter.CommNode))
getCommLinkStartStream thisArg = requestStream $ getCommLinkStartStreamReq thisArg 

{-
 - The type of link.
 -}
getCommLinkType :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCHS.SpaceCenter.CommLinkType)
getCommLinkType thisArg = do
    let r = makeRequest "SpaceCenter" "CommLink_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommLinkTypeStreamReq :: KRPCHS.SpaceCenter.CommLink -> KRPCStreamReq (KRPCHS.SpaceCenter.CommLinkType)
getCommLinkTypeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommLink_get_Type" [makeArgument 0 thisArg]
    in  makeStream req

getCommLinkTypeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommLink -> m (KRPCStream (KRPCHS.SpaceCenter.CommLinkType))
getCommLinkTypeStream thisArg = requestStream $ getCommLinkTypeStreamReq thisArg 

{-
 - Whether the communication node is a control point, for example a manned vessel.
 -}
getCommNodeIsControlPoint :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (Bool)
getCommNodeIsControlPoint thisArg = do
    let r = makeRequest "SpaceCenter" "CommNode_get_IsControlPoint" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommNodeIsControlPointStreamReq :: KRPCHS.SpaceCenter.CommNode -> KRPCStreamReq (Bool)
getCommNodeIsControlPointStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommNode_get_IsControlPoint" [makeArgument 0 thisArg]
    in  makeStream req

getCommNodeIsControlPointStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (KRPCStream (Bool))
getCommNodeIsControlPointStream thisArg = requestStream $ getCommNodeIsControlPointStreamReq thisArg 

{-
 - Whether the communication node is on Kerbin.
 -}
getCommNodeIsHome :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (Bool)
getCommNodeIsHome thisArg = do
    let r = makeRequest "SpaceCenter" "CommNode_get_IsHome" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommNodeIsHomeStreamReq :: KRPCHS.SpaceCenter.CommNode -> KRPCStreamReq (Bool)
getCommNodeIsHomeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommNode_get_IsHome" [makeArgument 0 thisArg]
    in  makeStream req

getCommNodeIsHomeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (KRPCStream (Bool))
getCommNodeIsHomeStream thisArg = requestStream $ getCommNodeIsHomeStreamReq thisArg 

{-
 - Whether the communication node is a vessel.
 -}
getCommNodeIsVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (Bool)
getCommNodeIsVessel thisArg = do
    let r = makeRequest "SpaceCenter" "CommNode_get_IsVessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommNodeIsVesselStreamReq :: KRPCHS.SpaceCenter.CommNode -> KRPCStreamReq (Bool)
getCommNodeIsVesselStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommNode_get_IsVessel" [makeArgument 0 thisArg]
    in  makeStream req

getCommNodeIsVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (KRPCStream (Bool))
getCommNodeIsVesselStream thisArg = requestStream $ getCommNodeIsVesselStreamReq thisArg 

{-
 - Name of the communication node.
 -}
getCommNodeName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (Data.Text.Text)
getCommNodeName thisArg = do
    let r = makeRequest "SpaceCenter" "CommNode_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommNodeNameStreamReq :: KRPCHS.SpaceCenter.CommNode -> KRPCStreamReq (Data.Text.Text)
getCommNodeNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommNode_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getCommNodeNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (KRPCStream (Data.Text.Text))
getCommNodeNameStream thisArg = requestStream $ getCommNodeNameStreamReq thisArg 

{-
 - The vessel for this communication node.
 -}
getCommNodeVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (KRPCHS.SpaceCenter.Vessel)
getCommNodeVessel thisArg = do
    let r = makeRequest "SpaceCenter" "CommNode_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommNodeVesselStreamReq :: KRPCHS.SpaceCenter.CommNode -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getCommNodeVesselStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CommNode_get_Vessel" [makeArgument 0 thisArg]
    in  makeStream req

getCommNodeVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CommNode -> m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCommNodeVesselStream thisArg = requestStream $ getCommNodeVesselStreamReq thisArg 

{-
 - Whether the vessel can communicate with KSC.
 -}
getCommsCanCommunicate :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (Bool)
getCommsCanCommunicate thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_CanCommunicate" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsCanCommunicateStreamReq :: KRPCHS.SpaceCenter.Comms -> KRPCStreamReq (Bool)
getCommsCanCommunicateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Comms_get_CanCommunicate" [makeArgument 0 thisArg]
    in  makeStream req

getCommsCanCommunicateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (KRPCStream (Bool))
getCommsCanCommunicateStream thisArg = requestStream $ getCommsCanCommunicateStreamReq thisArg 

{-
 - Whether the vessel can transmit science data to KSC.
 -}
getCommsCanTransmitScience :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (Bool)
getCommsCanTransmitScience thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_CanTransmitScience" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsCanTransmitScienceStreamReq :: KRPCHS.SpaceCenter.Comms -> KRPCStreamReq (Bool)
getCommsCanTransmitScienceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Comms_get_CanTransmitScience" [makeArgument 0 thisArg]
    in  makeStream req

getCommsCanTransmitScienceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (KRPCStream (Bool))
getCommsCanTransmitScienceStream thisArg = requestStream $ getCommsCanTransmitScienceStreamReq thisArg 

{-
 - The communication path used to control the vessel.
 -}
getCommsControlPath :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m ([KRPCHS.SpaceCenter.CommLink])
getCommsControlPath thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_ControlPath" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsControlPathStreamReq :: KRPCHS.SpaceCenter.Comms -> KRPCStreamReq ([KRPCHS.SpaceCenter.CommLink])
getCommsControlPathStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Comms_get_ControlPath" [makeArgument 0 thisArg]
    in  makeStream req

getCommsControlPathStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (KRPCStream ([KRPCHS.SpaceCenter.CommLink]))
getCommsControlPathStream thisArg = requestStream $ getCommsControlPathStreamReq thisArg 

{-
 - The combined power of all active antennae on the vessel.
 -}
getCommsPower :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (Double)
getCommsPower thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_Power" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsPowerStreamReq :: KRPCHS.SpaceCenter.Comms -> KRPCStreamReq (Double)
getCommsPowerStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Comms_get_Power" [makeArgument 0 thisArg]
    in  makeStream req

getCommsPowerStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (KRPCStream (Double))
getCommsPowerStream thisArg = requestStream $ getCommsPowerStreamReq thisArg 

{-
 - Signal delay to KSC in seconds.
 -}
getCommsSignalDelay :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (Double)
getCommsSignalDelay thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_SignalDelay" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsSignalDelayStreamReq :: KRPCHS.SpaceCenter.Comms -> KRPCStreamReq (Double)
getCommsSignalDelayStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Comms_get_SignalDelay" [makeArgument 0 thisArg]
    in  makeStream req

getCommsSignalDelayStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (KRPCStream (Double))
getCommsSignalDelayStream thisArg = requestStream $ getCommsSignalDelayStreamReq thisArg 

{-
 - Signal strength to KSC.
 -}
getCommsSignalStrength :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (Double)
getCommsSignalStrength thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_SignalStrength" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCommsSignalStrengthStreamReq :: KRPCHS.SpaceCenter.Comms -> KRPCStreamReq (Double)
getCommsSignalStrengthStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Comms_get_SignalStrength" [makeArgument 0 thisArg]
    in  makeStream req

getCommsSignalStrengthStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Comms -> m (KRPCStream (Double))
getCommsSignalStrengthStream thisArg = requestStream $ getCommsSignalStrengthStreamReq thisArg 

{-
 - A list of all active contracts.
 -}
getContractManagerActiveContracts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m ([KRPCHS.SpaceCenter.Contract])
getContractManagerActiveContracts thisArg = do
    let r = makeRequest "SpaceCenter" "ContractManager_get_ActiveContracts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractManagerActiveContractsStreamReq :: KRPCHS.SpaceCenter.ContractManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Contract])
getContractManagerActiveContractsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractManager_get_ActiveContracts" [makeArgument 0 thisArg]
    in  makeStream req

getContractManagerActiveContractsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (KRPCStream ([KRPCHS.SpaceCenter.Contract]))
getContractManagerActiveContractsStream thisArg = requestStream $ getContractManagerActiveContractsStreamReq thisArg 

{-
 - A list of all contracts.
 -}
getContractManagerAllContracts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m ([KRPCHS.SpaceCenter.Contract])
getContractManagerAllContracts thisArg = do
    let r = makeRequest "SpaceCenter" "ContractManager_get_AllContracts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractManagerAllContractsStreamReq :: KRPCHS.SpaceCenter.ContractManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Contract])
getContractManagerAllContractsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractManager_get_AllContracts" [makeArgument 0 thisArg]
    in  makeStream req

getContractManagerAllContractsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (KRPCStream ([KRPCHS.SpaceCenter.Contract]))
getContractManagerAllContractsStream thisArg = requestStream $ getContractManagerAllContractsStreamReq thisArg 

{-
 - A list of all completed contracts.
 -}
getContractManagerCompletedContracts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m ([KRPCHS.SpaceCenter.Contract])
getContractManagerCompletedContracts thisArg = do
    let r = makeRequest "SpaceCenter" "ContractManager_get_CompletedContracts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractManagerCompletedContractsStreamReq :: KRPCHS.SpaceCenter.ContractManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Contract])
getContractManagerCompletedContractsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractManager_get_CompletedContracts" [makeArgument 0 thisArg]
    in  makeStream req

getContractManagerCompletedContractsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (KRPCStream ([KRPCHS.SpaceCenter.Contract]))
getContractManagerCompletedContractsStream thisArg = requestStream $ getContractManagerCompletedContractsStreamReq thisArg 

{-
 - A list of all failed contracts.
 -}
getContractManagerFailedContracts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m ([KRPCHS.SpaceCenter.Contract])
getContractManagerFailedContracts thisArg = do
    let r = makeRequest "SpaceCenter" "ContractManager_get_FailedContracts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractManagerFailedContractsStreamReq :: KRPCHS.SpaceCenter.ContractManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Contract])
getContractManagerFailedContractsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractManager_get_FailedContracts" [makeArgument 0 thisArg]
    in  makeStream req

getContractManagerFailedContractsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (KRPCStream ([KRPCHS.SpaceCenter.Contract]))
getContractManagerFailedContractsStream thisArg = requestStream $ getContractManagerFailedContractsStreamReq thisArg 

{-
 - A list of all offered, but unaccepted, contracts.
 -}
getContractManagerOfferedContracts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m ([KRPCHS.SpaceCenter.Contract])
getContractManagerOfferedContracts thisArg = do
    let r = makeRequest "SpaceCenter" "ContractManager_get_OfferedContracts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractManagerOfferedContractsStreamReq :: KRPCHS.SpaceCenter.ContractManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Contract])
getContractManagerOfferedContractsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractManager_get_OfferedContracts" [makeArgument 0 thisArg]
    in  makeStream req

getContractManagerOfferedContractsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (KRPCStream ([KRPCHS.SpaceCenter.Contract]))
getContractManagerOfferedContractsStream thisArg = requestStream $ getContractManagerOfferedContractsStreamReq thisArg 

{-
 - A list of all contract types.
 -}
getContractManagerTypes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (Data.Set.Set Data.Text.Text)
getContractManagerTypes thisArg = do
    let r = makeRequest "SpaceCenter" "ContractManager_get_Types" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractManagerTypesStreamReq :: KRPCHS.SpaceCenter.ContractManager -> KRPCStreamReq (Data.Set.Set Data.Text.Text)
getContractManagerTypesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractManager_get_Types" [makeArgument 0 thisArg]
    in  makeStream req

getContractManagerTypesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractManager -> m (KRPCStream (Data.Set.Set Data.Text.Text))
getContractManagerTypesStream thisArg = requestStream $ getContractManagerTypesStreamReq thisArg 

{-
 - Child contract parameters.
 -}
getContractParameterChildren :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m ([KRPCHS.SpaceCenter.ContractParameter])
getContractParameterChildren thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_Children" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterChildrenStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq ([KRPCHS.SpaceCenter.ContractParameter])
getContractParameterChildrenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_Children" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterChildrenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream ([KRPCHS.SpaceCenter.ContractParameter]))
getContractParameterChildrenStream thisArg = requestStream $ getContractParameterChildrenStreamReq thisArg 

{-
 - Whether the parameter has been completed.
 -}
getContractParameterCompleted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Bool)
getContractParameterCompleted thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_Completed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterCompletedStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Bool)
getContractParameterCompletedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_Completed" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterCompletedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Bool))
getContractParameterCompletedStream thisArg = requestStream $ getContractParameterCompletedStreamReq thisArg 

{-
 - Whether the parameter has been failed.
 -}
getContractParameterFailed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Bool)
getContractParameterFailed thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_Failed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterFailedStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Bool)
getContractParameterFailedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_Failed" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterFailedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Bool))
getContractParameterFailedStream thisArg = requestStream $ getContractParameterFailedStreamReq thisArg 

{-
 - Funds received on completion of the contract parameter.
 -}
getContractParameterFundsCompletion :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Double)
getContractParameterFundsCompletion thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_FundsCompletion" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterFundsCompletionStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Double)
getContractParameterFundsCompletionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_FundsCompletion" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterFundsCompletionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Double))
getContractParameterFundsCompletionStream thisArg = requestStream $ getContractParameterFundsCompletionStreamReq thisArg 

{-
 - Funds lost if the contract parameter is failed.
 -}
getContractParameterFundsFailure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Double)
getContractParameterFundsFailure thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_FundsFailure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterFundsFailureStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Double)
getContractParameterFundsFailureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_FundsFailure" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterFundsFailureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Double))
getContractParameterFundsFailureStream thisArg = requestStream $ getContractParameterFundsFailureStreamReq thisArg 

{-
 - Notes for the parameter.
 -}
getContractParameterNotes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Data.Text.Text)
getContractParameterNotes thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_Notes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterNotesStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Data.Text.Text)
getContractParameterNotesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_Notes" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterNotesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Data.Text.Text))
getContractParameterNotesStream thisArg = requestStream $ getContractParameterNotesStreamReq thisArg 

{-
 - Whether the contract parameter is optional.
 -}
getContractParameterOptional :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Bool)
getContractParameterOptional thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_Optional" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterOptionalStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Bool)
getContractParameterOptionalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_Optional" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterOptionalStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Bool))
getContractParameterOptionalStream thisArg = requestStream $ getContractParameterOptionalStreamReq thisArg 

{-
 - Reputation gained on completion of the contract parameter.
 -}
getContractParameterReputationCompletion :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Double)
getContractParameterReputationCompletion thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_ReputationCompletion" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterReputationCompletionStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Double)
getContractParameterReputationCompletionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_ReputationCompletion" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterReputationCompletionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Double))
getContractParameterReputationCompletionStream thisArg = requestStream $ getContractParameterReputationCompletionStreamReq thisArg 

{-
 - Reputation lost if the contract parameter is failed.
 -}
getContractParameterReputationFailure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Double)
getContractParameterReputationFailure thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_ReputationFailure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterReputationFailureStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Double)
getContractParameterReputationFailureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_ReputationFailure" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterReputationFailureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Double))
getContractParameterReputationFailureStream thisArg = requestStream $ getContractParameterReputationFailureStreamReq thisArg 

{-
 - Science gained on completion of the contract parameter.
 -}
getContractParameterScienceCompletion :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Double)
getContractParameterScienceCompletion thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_ScienceCompletion" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterScienceCompletionStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Double)
getContractParameterScienceCompletionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_ScienceCompletion" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterScienceCompletionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Double))
getContractParameterScienceCompletionStream thisArg = requestStream $ getContractParameterScienceCompletionStreamReq thisArg 

{-
 - Title of the parameter.
 -}
getContractParameterTitle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (Data.Text.Text)
getContractParameterTitle thisArg = do
    let r = makeRequest "SpaceCenter" "ContractParameter_get_Title" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParameterTitleStreamReq :: KRPCHS.SpaceCenter.ContractParameter -> KRPCStreamReq (Data.Text.Text)
getContractParameterTitleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ContractParameter_get_Title" [makeArgument 0 thisArg]
    in  makeStream req

getContractParameterTitleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ContractParameter -> m (KRPCStream (Data.Text.Text))
getContractParameterTitleStream thisArg = requestStream $ getContractParameterTitleStreamReq thisArg 

{-
 - Accept an offered contract.
 -}
contractAccept :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m ()
contractAccept thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_Accept" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Cancel an active contract.
 -}
contractCancel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m ()
contractCancel thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_Cancel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Decline an offered contract.
 -}
contractDecline :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m ()
contractDecline thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_Decline" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the contract is active.
 -}
getContractActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractActive thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractActiveStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getContractActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractActiveStream thisArg = requestStream $ getContractActiveStreamReq thisArg 

{-
 - Whether the contract can be canceled.
 -}
getContractCanBeCanceled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractCanBeCanceled thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_CanBeCanceled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractCanBeCanceledStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractCanBeCanceledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_CanBeCanceled" [makeArgument 0 thisArg]
    in  makeStream req

getContractCanBeCanceledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractCanBeCanceledStream thisArg = requestStream $ getContractCanBeCanceledStreamReq thisArg 

{-
 - Whether the contract can be declined.
 -}
getContractCanBeDeclined :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractCanBeDeclined thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_CanBeDeclined" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractCanBeDeclinedStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractCanBeDeclinedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_CanBeDeclined" [makeArgument 0 thisArg]
    in  makeStream req

getContractCanBeDeclinedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractCanBeDeclinedStream thisArg = requestStream $ getContractCanBeDeclinedStreamReq thisArg 

{-
 - Whether the contract can be failed.
 -}
getContractCanBeFailed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractCanBeFailed thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_CanBeFailed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractCanBeFailedStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractCanBeFailedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_CanBeFailed" [makeArgument 0 thisArg]
    in  makeStream req

getContractCanBeFailedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractCanBeFailedStream thisArg = requestStream $ getContractCanBeFailedStreamReq thisArg 

{-
 - Description of the contract.
 -}
getContractDescription :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Data.Text.Text)
getContractDescription thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Description" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractDescriptionStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Data.Text.Text)
getContractDescriptionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Description" [makeArgument 0 thisArg]
    in  makeStream req

getContractDescriptionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Data.Text.Text))
getContractDescriptionStream thisArg = requestStream $ getContractDescriptionStreamReq thisArg 

{-
 - Whether the contract has been failed.
 -}
getContractFailed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractFailed thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Failed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractFailedStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractFailedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Failed" [makeArgument 0 thisArg]
    in  makeStream req

getContractFailedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractFailedStream thisArg = requestStream $ getContractFailedStreamReq thisArg 

{-
 - Funds received when accepting the contract.
 -}
getContractFundsAdvance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Double)
getContractFundsAdvance thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_FundsAdvance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractFundsAdvanceStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Double)
getContractFundsAdvanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_FundsAdvance" [makeArgument 0 thisArg]
    in  makeStream req

getContractFundsAdvanceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Double))
getContractFundsAdvanceStream thisArg = requestStream $ getContractFundsAdvanceStreamReq thisArg 

{-
 - Funds received on completion of the contract.
 -}
getContractFundsCompletion :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Double)
getContractFundsCompletion thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_FundsCompletion" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractFundsCompletionStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Double)
getContractFundsCompletionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_FundsCompletion" [makeArgument 0 thisArg]
    in  makeStream req

getContractFundsCompletionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Double))
getContractFundsCompletionStream thisArg = requestStream $ getContractFundsCompletionStreamReq thisArg 

{-
 - Funds lost if the contract is failed.
 -}
getContractFundsFailure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Double)
getContractFundsFailure thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_FundsFailure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractFundsFailureStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Double)
getContractFundsFailureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_FundsFailure" [makeArgument 0 thisArg]
    in  makeStream req

getContractFundsFailureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Double))
getContractFundsFailureStream thisArg = requestStream $ getContractFundsFailureStreamReq thisArg 

{-
 - Keywords for the contract.
 -}
getContractKeywords :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m ([Data.Text.Text])
getContractKeywords thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Keywords" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractKeywordsStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq ([Data.Text.Text])
getContractKeywordsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Keywords" [makeArgument 0 thisArg]
    in  makeStream req

getContractKeywordsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream ([Data.Text.Text]))
getContractKeywordsStream thisArg = requestStream $ getContractKeywordsStreamReq thisArg 

{-
 - Notes for the contract.
 -}
getContractNotes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Data.Text.Text)
getContractNotes thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Notes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractNotesStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Data.Text.Text)
getContractNotesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Notes" [makeArgument 0 thisArg]
    in  makeStream req

getContractNotesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Data.Text.Text))
getContractNotesStream thisArg = requestStream $ getContractNotesStreamReq thisArg 

{-
 - Parameters for the contract.
 -}
getContractParameters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m ([KRPCHS.SpaceCenter.ContractParameter])
getContractParameters thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Parameters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractParametersStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq ([KRPCHS.SpaceCenter.ContractParameter])
getContractParametersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Parameters" [makeArgument 0 thisArg]
    in  makeStream req

getContractParametersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream ([KRPCHS.SpaceCenter.ContractParameter]))
getContractParametersStream thisArg = requestStream $ getContractParametersStreamReq thisArg 

{-
 - Whether the contract has been read.
 -}
getContractRead :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractRead thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Read" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractReadStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractReadStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Read" [makeArgument 0 thisArg]
    in  makeStream req

getContractReadStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractReadStream thisArg = requestStream $ getContractReadStreamReq thisArg 

{-
 - Reputation gained on completion of the contract.
 -}
getContractReputationCompletion :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Double)
getContractReputationCompletion thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_ReputationCompletion" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractReputationCompletionStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Double)
getContractReputationCompletionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_ReputationCompletion" [makeArgument 0 thisArg]
    in  makeStream req

getContractReputationCompletionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Double))
getContractReputationCompletionStream thisArg = requestStream $ getContractReputationCompletionStreamReq thisArg 

{-
 - Reputation lost if the contract is failed.
 -}
getContractReputationFailure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Double)
getContractReputationFailure thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_ReputationFailure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractReputationFailureStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Double)
getContractReputationFailureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_ReputationFailure" [makeArgument 0 thisArg]
    in  makeStream req

getContractReputationFailureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Double))
getContractReputationFailureStream thisArg = requestStream $ getContractReputationFailureStreamReq thisArg 

{-
 - Science gained on completion of the contract.
 -}
getContractScienceCompletion :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Double)
getContractScienceCompletion thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_ScienceCompletion" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractScienceCompletionStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Double)
getContractScienceCompletionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_ScienceCompletion" [makeArgument 0 thisArg]
    in  makeStream req

getContractScienceCompletionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Double))
getContractScienceCompletionStream thisArg = requestStream $ getContractScienceCompletionStreamReq thisArg 

{-
 - Whether the contract has been seen.
 -}
getContractSeen :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Bool)
getContractSeen thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Seen" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractSeenStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Bool)
getContractSeenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Seen" [makeArgument 0 thisArg]
    in  makeStream req

getContractSeenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Bool))
getContractSeenStream thisArg = requestStream $ getContractSeenStreamReq thisArg 

{-
 - State of the contract.
 -}
getContractState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCHS.SpaceCenter.ContractState)
getContractState thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractStateStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (KRPCHS.SpaceCenter.ContractState)
getContractStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getContractStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (KRPCHS.SpaceCenter.ContractState))
getContractStateStream thisArg = requestStream $ getContractStateStreamReq thisArg 

{-
 - Synopsis for the contract.
 -}
getContractSynopsis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Data.Text.Text)
getContractSynopsis thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Synopsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractSynopsisStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Data.Text.Text)
getContractSynopsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Synopsis" [makeArgument 0 thisArg]
    in  makeStream req

getContractSynopsisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Data.Text.Text))
getContractSynopsisStream thisArg = requestStream $ getContractSynopsisStreamReq thisArg 

{-
 - Title of the contract.
 -}
getContractTitle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Data.Text.Text)
getContractTitle thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Title" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractTitleStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Data.Text.Text)
getContractTitleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Title" [makeArgument 0 thisArg]
    in  makeStream req

getContractTitleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Data.Text.Text))
getContractTitleStream thisArg = requestStream $ getContractTitleStreamReq thisArg 

{-
 - Type of the contract.
 -}
getContractType :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (Data.Text.Text)
getContractType thisArg = do
    let r = makeRequest "SpaceCenter" "Contract_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getContractTypeStreamReq :: KRPCHS.SpaceCenter.Contract -> KRPCStreamReq (Data.Text.Text)
getContractTypeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Contract_get_Type" [makeArgument 0 thisArg]
    in  makeStream req

getContractTypeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Contract -> m (KRPCStream (Data.Text.Text))
getContractTypeStream thisArg = requestStream $ getContractTypeStreamReq thisArg 

{-
 - The authority limiter for the control surface, which controls how far the control surface will move.
 -}
getControlSurfaceAuthorityLimiter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Float)
getControlSurfaceAuthorityLimiter thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_AuthorityLimiter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceAuthorityLimiterStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Float)
getControlSurfaceAuthorityLimiterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_AuthorityLimiter" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceAuthorityLimiterStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Float))
getControlSurfaceAuthorityLimiterStream thisArg = requestStream $ getControlSurfaceAuthorityLimiterStreamReq thisArg 

{-
 - The available torque in the positive pitch, roll and yaw axes and
 - negative pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getControlSurfaceAvailableTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (((Double, Double, Double), (Double, Double, Double)))
getControlSurfaceAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getControlSurfaceAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceAvailableTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getControlSurfaceAvailableTorqueStream thisArg = requestStream $ getControlSurfaceAvailableTorqueStreamReq thisArg 

{-
 - Whether the control surface has been fully deployed.
 -}
getControlSurfaceDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Bool)
getControlSurfaceDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceDeployedStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Bool))
getControlSurfaceDeployedStream thisArg = requestStream $ getControlSurfaceDeployedStreamReq thisArg 

{-
 - Whether the control surface movement is inverted.
 -}
getControlSurfaceInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Bool)
getControlSurfaceInverted thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceInvertedStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceInvertedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceInvertedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Bool))
getControlSurfaceInvertedStream thisArg = requestStream $ getControlSurfaceInvertedStreamReq thisArg 

{-
 - The part object for this control surface.
 -}
getControlSurfacePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCHS.SpaceCenter.Part)
getControlSurfacePart thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfacePartStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getControlSurfacePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfacePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getControlSurfacePartStream thisArg = requestStream $ getControlSurfacePartStreamReq thisArg 

{-
 - Whether the control surface has pitch control enabled.
 -}
getControlSurfacePitchEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Bool)
getControlSurfacePitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfacePitchEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfacePitchEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfacePitchEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Bool))
getControlSurfacePitchEnabledStream thisArg = requestStream $ getControlSurfacePitchEnabledStreamReq thisArg 

{-
 - Whether the control surface has roll control enabled.
 -}
getControlSurfaceRollEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Bool)
getControlSurfaceRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceRollEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceRollEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceRollEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Bool))
getControlSurfaceRollEnabledStream thisArg = requestStream $ getControlSurfaceRollEnabledStreamReq thisArg 

{-
 - Surface area of the control surface inm^2.
 -}
getControlSurfaceSurfaceArea :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Float)
getControlSurfaceSurfaceArea thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceSurfaceAreaStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Float)
getControlSurfaceSurfaceAreaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceSurfaceAreaStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Float))
getControlSurfaceSurfaceAreaStream thisArg = requestStream $ getControlSurfaceSurfaceAreaStreamReq thisArg 

{-
 - Whether the control surface has yaw control enabled.
 -}
getControlSurfaceYawEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (Bool)
getControlSurfaceYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceYawEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceYawEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceYawEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> m (KRPCStream (Bool))
getControlSurfaceYawEnabledStream thisArg = requestStream $ getControlSurfaceYawEnabledStreamReq thisArg 

{-
 - The authority limiter for the control surface, which controls how far the control surface will move.
 -}
setControlSurfaceAuthorityLimiter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> Float -> m ()
setControlSurfaceAuthorityLimiter thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_AuthorityLimiter" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has been fully deployed.
 -}
setControlSurfaceDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> Bool -> m ()
setControlSurfaceDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface movement is inverted.
 -}
setControlSurfaceInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> Bool -> m ()
setControlSurfaceInverted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Inverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has pitch control enabled.
 -}
setControlSurfacePitchEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> Bool -> m ()
setControlSurfacePitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has roll control enabled.
 -}
setControlSurfaceRollEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> Bool -> m ()
setControlSurfaceRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has yaw control enabled.
 -}
setControlSurfaceYawEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ControlSurface -> Bool -> m ()
setControlSurfaceYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Activates the next stage. Equivalent to pressing the space bar in-game.A list of vessel objects that are jettisoned from the active vessel.When called, the active vessel may change. It is therefore possible that,
 - after calling this function, the object(s) returned by previous call(s) to
 - <see cref="M:SpaceCenter.ActiveVessel" /> no longer refer to the active vessel.
 -}
controlActivateNextStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

controlActivateNextStageStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]
    in  makeStream req

controlActivateNextStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
controlActivateNextStageStream thisArg = requestStream $ controlActivateNextStageStreamReq thisArg 

{-
 - Creates a maneuver node at the given universal time, and returns a
 - <see cref="T:SpaceCenter.Node" /> object that can be used to modify it.
 - Optionally sets the magnitude of the delta-v for the maneuver node
 - in the prograde, normal and radial directions.<param name="ut">Universal time of the maneuver node.<param name="prograde">Delta-v in the prograde direction.<param name="normal">Delta-v in the normal direction.<param name="radial">Delta-v in the radial direction.
 -}
controlAddNode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> m (KRPCHS.SpaceCenter.Node)
controlAddNode thisArg utArg progradeArg normalArg radialArg = do
    let r = makeRequest "SpaceCenter" "Control_AddNode" [makeArgument 0 thisArg, makeArgument 1 utArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg]
    res <- sendRequest r
    processResponse res

controlAddNodeStreamReq :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> KRPCStreamReq (KRPCHS.SpaceCenter.Node)
controlAddNodeStreamReq thisArg utArg progradeArg normalArg radialArg =
    let req = makeRequest "SpaceCenter" "Control_AddNode" [makeArgument 0 thisArg, makeArgument 1 utArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg]
    in  makeStream req

controlAddNodeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> m (KRPCStream (KRPCHS.SpaceCenter.Node))
controlAddNodeStream thisArg utArg progradeArg normalArg radialArg = requestStream $ controlAddNodeStreamReq thisArg utArg progradeArg normalArg radialArg 

{-
 - Returnstrueif the given action group is enabled.<param name="group">
 - A number between 0 and 9 inclusive,
 - or between 0 and 250 inclusive when the <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/67235-12oct3116-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech">Extended Action Groups modis installed.
 -}
controlGetActionGroup :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> m (Bool)
controlGetActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    res <- sendRequest r
    processResponse res

controlGetActionGroupStreamReq :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> KRPCStreamReq (Bool)
controlGetActionGroupStreamReq thisArg groupArg =
    let req = makeRequest "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    in  makeStream req

controlGetActionGroupStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> m (KRPCStream (Bool))
controlGetActionGroupStream thisArg groupArg = requestStream $ controlGetActionGroupStreamReq thisArg groupArg 

{-
 - Remove all maneuver nodes.
 -}
controlRemoveNodes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m ()
controlRemoveNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_RemoveNodes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Sets the state of the given action group.<param name="group">
 - A number between 0 and 9 inclusive,
 - or between 0 and 250 inclusive when the <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/67235-12oct3116-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech">Extended Action Groups modis installed.<param name="state">
 -}
controlSetActionGroup :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> Bool -> m ()
controlSetActionGroup thisArg groupArg stateArg = do
    let r = makeRequest "SpaceCenter" "Control_SetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg, makeArgument 2 stateArg]
    res <- sendRequest r
    processResponse res 

{-
 - Toggles the state of the given action group.<param name="group">
 - A number between 0 and 9 inclusive,
 - or between 0 and 250 inclusive when the <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/67235-12oct3116-action-groups-extended-250-action-groups-in-flight-editing-now-kosremotetech">Extended Action Groups modis installed.
 -}
controlToggleActionGroup :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> m ()
controlToggleActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_ToggleActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the abort action group.
 -}
getControlAbort :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlAbort thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlAbortStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlAbortStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]
    in  makeStream req

getControlAbortStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlAbortStream thisArg = requestStream $ getControlAbortStreamReq thisArg 

{-
 - Returns whether all antennas on the vessel are deployed,
 - and sets the deployment state of all antennas.
 - See <see cref="M:SpaceCenter.Antenna.Deployed" />.
 -}
getControlAntennas :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlAntennas thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Antennas" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlAntennasStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlAntennasStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Antennas" [makeArgument 0 thisArg]
    in  makeStream req

getControlAntennasStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlAntennasStream thisArg = requestStream $ getControlAntennasStreamReq thisArg 

{-
 - The state of the wheel brakes.
 -}
getControlBrakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlBrakes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlBrakesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlBrakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]
    in  makeStream req

getControlBrakesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlBrakesStream thisArg = requestStream $ getControlBrakesStreamReq thisArg 

{-
 - Returns whether any of the cargo bays on the vessel are open,
 - and sets the open state of all cargo bays.
 - See <see cref="M:SpaceCenter.CargoBay.Open" />.
 -}
getControlCargoBays :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlCargoBays thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CargoBays" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlCargoBaysStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlCargoBaysStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_CargoBays" [makeArgument 0 thisArg]
    in  makeStream req

getControlCargoBaysStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlCargoBaysStream thisArg = requestStream $ getControlCargoBaysStreamReq thisArg 

{-
 - The current stage of the vessel. Corresponds to the stage number in
 - the in-game UI.
 -}
getControlCurrentStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Data.Int.Int32)
getControlCurrentStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlCurrentStageStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Data.Int.Int32)
getControlCurrentStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]
    in  makeStream req

getControlCurrentStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Data.Int.Int32))
getControlCurrentStageStream thisArg = requestStream $ getControlCurrentStageStreamReq thisArg 

{-
 - The state of the forward translational control.
 - A value between -1 and 1.
 - Equivalent to the h and n keys.
 -}
getControlForward :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlForward thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlForwardStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlForwardStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]
    in  makeStream req

getControlForwardStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlForwardStream thisArg = requestStream $ getControlForwardStreamReq thisArg 

{-
 - The state of the landing gear/legs.
 -}
getControlGear :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlGear thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlGearStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlGearStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]
    in  makeStream req

getControlGearStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlGearStream thisArg = requestStream $ getControlGearStreamReq thisArg 

{-
 - Returns whether all of the air intakes on the vessel are open,
 - and sets the open state of all air intakes.
 - See <see cref="M:SpaceCenter.Intake.Open" />.
 -}
getControlIntakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlIntakes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Intakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlIntakesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlIntakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Intakes" [makeArgument 0 thisArg]
    in  makeStream req

getControlIntakesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlIntakesStream thisArg = requestStream $ getControlIntakesStreamReq thisArg 

{-
 - Returns whether all landing legs on the vessel are deployed,
 - and sets the deployment state of all landing legs.
 - Does not include wheels (for example landing gear).
 - See <see cref="M:SpaceCenter.Leg.Deployed" />.
 -}
getControlLegs :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlLegs thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Legs" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlLegsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlLegsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Legs" [makeArgument 0 thisArg]
    in  makeStream req

getControlLegsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlLegsStream thisArg = requestStream $ getControlLegsStreamReq thisArg 

{-
 - The state of the lights.
 -}
getControlLights :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlLights thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlLightsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlLightsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]
    in  makeStream req

getControlLightsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlLightsStream thisArg = requestStream $ getControlLightsStreamReq thisArg 

{-
 - Returns a list of all existing maneuver nodes, ordered by time from first to last.
 -}
getControlNodes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m ([KRPCHS.SpaceCenter.Node])
getControlNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlNodesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq ([KRPCHS.SpaceCenter.Node])
getControlNodesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]
    in  makeStream req

getControlNodesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream ([KRPCHS.SpaceCenter.Node]))
getControlNodesStream thisArg = requestStream $ getControlNodesStreamReq thisArg 

{-
 - Returns whether all parachutes on the vessel are deployed,
 - and sets the deployment state of all parachutes.
 - Cannot be set tofalse.
 - See <see cref="M:SpaceCenter.Parachute.Deployed" />.
 -}
getControlParachutes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlParachutes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Parachutes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlParachutesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlParachutesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Parachutes" [makeArgument 0 thisArg]
    in  makeStream req

getControlParachutesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlParachutesStream thisArg = requestStream $ getControlParachutesStreamReq thisArg 

{-
 - The state of the pitch control.
 - A value between -1 and 1.
 - Equivalent to the w and s keys.
 -}
getControlPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlPitchStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]
    in  makeStream req

getControlPitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlPitchStream thisArg = requestStream $ getControlPitchStreamReq thisArg 

{-
 - The state of RCS.
 -}
getControlRCS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRCSStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlRCSStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]
    in  makeStream req

getControlRCSStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlRCSStream thisArg = requestStream $ getControlRCSStreamReq thisArg 

{-
 - Returns whether all radiators on the vessel are deployed,
 - and sets the deployment state of all radiators.
 - See <see cref="M:SpaceCenter.Radiator.Deployed" />.
 -}
getControlRadiators :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlRadiators thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Radiators" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRadiatorsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlRadiatorsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Radiators" [makeArgument 0 thisArg]
    in  makeStream req

getControlRadiatorsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlRadiatorsStream thisArg = requestStream $ getControlRadiatorsStreamReq thisArg 

{-
 - Returns whether all reactive wheels on the vessel are active,
 - and sets the active state of all reaction wheels.
 - See <see cref="M:SpaceCenter.ReactionWheel.Active" />.
 -}
getControlReactionWheels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlReactionWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_ReactionWheels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlReactionWheelsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlReactionWheelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_ReactionWheels" [makeArgument 0 thisArg]
    in  makeStream req

getControlReactionWheelsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlReactionWheelsStream thisArg = requestStream $ getControlReactionWheelsStreamReq thisArg 

{-
 - Returns whether all of the resource harvesters on the vessel are deployed,
 - and sets the deployment state of all resource harvesters.
 - See <see cref="M:SpaceCenter.ResourceHarvester.Deployed" />.
 -}
getControlResourceHarvesters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlResourceHarvesters thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_ResourceHarvesters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlResourceHarvestersStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlResourceHarvestersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_ResourceHarvesters" [makeArgument 0 thisArg]
    in  makeStream req

getControlResourceHarvestersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlResourceHarvestersStream thisArg = requestStream $ getControlResourceHarvestersStreamReq thisArg 

{-
 - Returns whether any of the resource harvesters on the vessel are active,
 - and sets the active state of all resource harvesters.
 - See <see cref="M:SpaceCenter.ResourceHarvester.Active" />.
 -}
getControlResourceHarvestersActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlResourceHarvestersActive thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_ResourceHarvestersActive" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlResourceHarvestersActiveStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlResourceHarvestersActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_ResourceHarvestersActive" [makeArgument 0 thisArg]
    in  makeStream req

getControlResourceHarvestersActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlResourceHarvestersActiveStream thisArg = requestStream $ getControlResourceHarvestersActiveStreamReq thisArg 

{-
 - The state of the right translational control.
 - A value between -1 and 1.
 - Equivalent to the j and l keys.
 -}
getControlRight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlRight thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRightStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlRightStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]
    in  makeStream req

getControlRightStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlRightStream thisArg = requestStream $ getControlRightStreamReq thisArg 

{-
 - The state of the roll control.
 - A value between -1 and 1.
 - Equivalent to the q and e keys.
 -}
getControlRoll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRollStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlRollStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]
    in  makeStream req

getControlRollStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlRollStream thisArg = requestStream $ getControlRollStreamReq thisArg 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
getControlSAS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlSAS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSASStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlSASStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]
    in  makeStream req

getControlSASStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlSASStream thisArg = requestStream $ getControlSASStreamReq thisArg 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
getControlSASMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCHS.SpaceCenter.SASMode)
getControlSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSASModeStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.SASMode)
getControlSASModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]
    in  makeStream req

getControlSASModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getControlSASModeStream thisArg = requestStream $ getControlSASModeStreamReq thisArg 

{-
 - Returns whether all solar panels on the vessel are deployed,
 - and sets the deployment state of all solar panels.
 - See <see cref="M:SpaceCenter.SolarPanel.Deployed" />.
 -}
getControlSolarPanels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlSolarPanels thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SolarPanels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSolarPanelsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlSolarPanelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SolarPanels" [makeArgument 0 thisArg]
    in  makeStream req

getControlSolarPanelsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlSolarPanelsStream thisArg = requestStream $ getControlSolarPanelsStreamReq thisArg 

{-
 - The source of the vessels control, for example by a kerbal or a probe core.
 -}
getControlSource :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCHS.SpaceCenter.ControlSource)
getControlSource thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Source" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSourceStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.ControlSource)
getControlSourceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Source" [makeArgument 0 thisArg]
    in  makeStream req

getControlSourceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (KRPCHS.SpaceCenter.ControlSource))
getControlSourceStream thisArg = requestStream $ getControlSourceStreamReq thisArg 

{-
 - The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
 - This is the mode displayed next to the speed at the top of the navball.
 -}
getControlSpeedMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSpeedModeStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]
    in  makeStream req

getControlSpeedModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (KRPCHS.SpaceCenter.SpeedMode))
getControlSpeedModeStream thisArg = requestStream $ getControlSpeedModeStreamReq thisArg 

{-
 - The control state of the vessel.
 -}
getControlState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCHS.SpaceCenter.ControlState)
getControlState thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlStateStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.ControlState)
getControlStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getControlStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (KRPCHS.SpaceCenter.ControlState))
getControlStateStream thisArg = requestStream $ getControlStateStreamReq thisArg 

{-
 - The state of the throttle. A value between 0 and 1.
 -}
getControlThrottle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlThrottleStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlThrottleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]
    in  makeStream req

getControlThrottleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlThrottleStream thisArg = requestStream $ getControlThrottleStreamReq thisArg 

{-
 - The state of the up translational control.
 - A value between -1 and 1.
 - Equivalent to the i and k keys.
 -}
getControlUp :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlUp thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlUpStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlUpStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]
    in  makeStream req

getControlUpStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlUpStream thisArg = requestStream $ getControlUpStreamReq thisArg 

{-
 - The state of the wheel steering.
 - A value between -1 and 1.
 - A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
getControlWheelSteering :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlWheelSteering thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlWheelSteeringStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlWheelSteeringStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]
    in  makeStream req

getControlWheelSteeringStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlWheelSteeringStream thisArg = requestStream $ getControlWheelSteeringStreamReq thisArg 

{-
 - The state of the wheel throttle.
 - A value between -1 and 1.
 - A value of 1 rotates the wheels forwards, a value of -1 rotates
 - the wheels backwards.
 -}
getControlWheelThrottle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlWheelThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlWheelThrottleStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlWheelThrottleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [makeArgument 0 thisArg]
    in  makeStream req

getControlWheelThrottleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlWheelThrottleStream thisArg = requestStream $ getControlWheelThrottleStreamReq thisArg 

{-
 - Returns whether all wheels on the vessel are deployed,
 - and sets the deployment state of all wheels.
 - Does not include landing legs.
 - See <see cref="M:SpaceCenter.Wheel.Deployed" />.
 -}
getControlWheels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Bool)
getControlWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Wheels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlWheelsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlWheelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Wheels" [makeArgument 0 thisArg]
    in  makeStream req

getControlWheelsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Bool))
getControlWheelsStream thisArg = requestStream $ getControlWheelsStreamReq thisArg 

{-
 - The state of the yaw control.
 - A value between -1 and 1.
 - Equivalent to the a and d keys.
 -}
getControlYaw :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (Float)
getControlYaw thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlYawStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlYawStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]
    in  makeStream req

getControlYawStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> m (KRPCStream (Float))
getControlYawStream thisArg = requestStream $ getControlYawStreamReq thisArg 

{-
 - The state of the abort action group.
 -}
setControlAbort :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlAbort thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Abort" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all antennas on the vessel are deployed,
 - and sets the deployment state of all antennas.
 - See <see cref="M:SpaceCenter.Antenna.Deployed" />.
 -}
setControlAntennas :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlAntennas thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Antennas" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the wheel brakes.
 -}
setControlBrakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlBrakes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Brakes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether any of the cargo bays on the vessel are open,
 - and sets the open state of all cargo bays.
 - See <see cref="M:SpaceCenter.CargoBay.Open" />.
 -}
setControlCargoBays :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlCargoBays thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_CargoBays" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the forward translational control.
 - A value between -1 and 1.
 - Equivalent to the h and n keys.
 -}
setControlForward :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlForward thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Forward" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the landing gear/legs.
 -}
setControlGear :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlGear thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Gear" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all of the air intakes on the vessel are open,
 - and sets the open state of all air intakes.
 - See <see cref="M:SpaceCenter.Intake.Open" />.
 -}
setControlIntakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlIntakes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Intakes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all landing legs on the vessel are deployed,
 - and sets the deployment state of all landing legs.
 - Does not include wheels (for example landing gear).
 - See <see cref="M:SpaceCenter.Leg.Deployed" />.
 -}
setControlLegs :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlLegs thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Legs" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the lights.
 -}
setControlLights :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlLights thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Lights" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all parachutes on the vessel are deployed,
 - and sets the deployment state of all parachutes.
 - Cannot be set tofalse.
 - See <see cref="M:SpaceCenter.Parachute.Deployed" />.
 -}
setControlParachutes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlParachutes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Parachutes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the pitch control.
 - A value between -1 and 1.
 - Equivalent to the w and s keys.
 -}
setControlPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of RCS.
 -}
setControlRCS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlRCS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_RCS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all radiators on the vessel are deployed,
 - and sets the deployment state of all radiators.
 - See <see cref="M:SpaceCenter.Radiator.Deployed" />.
 -}
setControlRadiators :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlRadiators thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Radiators" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all reactive wheels on the vessel are active,
 - and sets the active state of all reaction wheels.
 - See <see cref="M:SpaceCenter.ReactionWheel.Active" />.
 -}
setControlReactionWheels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlReactionWheels thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_ReactionWheels" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all of the resource harvesters on the vessel are deployed,
 - and sets the deployment state of all resource harvesters.
 - See <see cref="M:SpaceCenter.ResourceHarvester.Deployed" />.
 -}
setControlResourceHarvesters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlResourceHarvesters thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_ResourceHarvesters" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether any of the resource harvesters on the vessel are active,
 - and sets the active state of all resource harvesters.
 - See <see cref="M:SpaceCenter.ResourceHarvester.Active" />.
 -}
setControlResourceHarvestersActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlResourceHarvestersActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_ResourceHarvestersActive" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the right translational control.
 - A value between -1 and 1.
 - Equivalent to the j and l keys.
 -}
setControlRight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlRight thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Right" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the roll control.
 - A value between -1 and 1.
 - Equivalent to the q and e keys.
 -}
setControlRoll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Roll" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
setControlSAS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
setControlSASMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SASMode -> m ()
setControlSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all solar panels on the vessel are deployed,
 - and sets the deployment state of all solar panels.
 - See <see cref="M:SpaceCenter.SolarPanel.Deployed" />.
 -}
setControlSolarPanels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlSolarPanels thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SolarPanels" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
 - This is the mode displayed next to the speed at the top of the navball.
 -}
setControlSpeedMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SpeedMode -> m ()
setControlSpeedMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SpeedMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the throttle. A value between 0 and 1.
 -}
setControlThrottle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Throttle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the up translational control.
 - A value between -1 and 1.
 - Equivalent to the i and k keys.
 -}
setControlUp :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlUp thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Up" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the wheel steering.
 - A value between -1 and 1.
 - A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
setControlWheelSteering :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlWheelSteering thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelSteering" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the wheel throttle.
 - A value between -1 and 1.
 - A value of 1 rotates the wheels forwards, a value of -1 rotates
 - the wheels backwards.
 -}
setControlWheelThrottle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlWheelThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelThrottle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns whether all wheels on the vessel are deployed,
 - and sets the deployment state of all wheels.
 - Does not include landing legs.
 - See <see cref="M:SpaceCenter.Wheel.Deployed" />.
 -}
setControlWheels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Bool -> m ()
setControlWheels thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Wheels" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the yaw control.
 - A value between -1 and 1.
 - Equivalent to the a and d keys.
 -}
setControlYaw :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Control -> Float -> m ()
setControlYaw thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Yaw" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Fires the decoupler. Returns the new vessel created when the decoupler fires.
 - Throws an exception if the decoupler has already fired.When called, the active vessel may change. It is therefore possible that,
 - after calling this function, the object(s) returned by previous call(s) to
 - <see cref="M:SpaceCenter.ActiveVessel" /> no longer refer to the active vessel.
 -}
decouplerDecouple :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCHS.SpaceCenter.Vessel)
decouplerDecouple thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

decouplerDecoupleStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
decouplerDecoupleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]
    in  makeStream req

decouplerDecoupleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
decouplerDecoupleStream thisArg = requestStream $ decouplerDecoupleStreamReq thisArg 

{-
 - Whether the decoupler has fired.
 -}
getDecouplerDecoupled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (Bool)
getDecouplerDecoupled thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerDecoupledStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Bool)
getDecouplerDecoupledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerDecoupledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCStream (Bool))
getDecouplerDecoupledStream thisArg = requestStream $ getDecouplerDecoupledStreamReq thisArg 

{-
 - The impulse that the decoupler imparts when it is fired, in Newton seconds.
 -}
getDecouplerImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (Float)
getDecouplerImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerImpulseStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Float)
getDecouplerImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCStream (Float))
getDecouplerImpulseStream thisArg = requestStream $ getDecouplerImpulseStreamReq thisArg 

{-
 - The part object for this decoupler.
 -}
getDecouplerPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCHS.SpaceCenter.Part)
getDecouplerPart thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerPartStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDecouplerPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getDecouplerPartStream thisArg = requestStream $ getDecouplerPartStreamReq thisArg 

{-
 - Whether the decoupler is enabled in the staging sequence.
 -}
getDecouplerStaged :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (Bool)
getDecouplerStaged thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Staged" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerStagedStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Bool)
getDecouplerStagedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Staged" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerStagedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Decoupler -> m (KRPCStream (Bool))
getDecouplerStagedStream thisArg = requestStream $ getDecouplerStagedStreamReq thisArg 

{-
 - The direction that docking port points in, in the given reference frame.
 -}
dockingPortDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
dockingPortDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

dockingPortDirectionStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
dockingPortDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

dockingPortDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
dockingPortDirectionStream thisArg referenceFrameArg = requestStream $ dockingPortDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position of the docking port in the given reference frame.
 -}
dockingPortPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
dockingPortPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

dockingPortPositionStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
dockingPortPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

dockingPortPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
dockingPortPositionStream thisArg referenceFrameArg = requestStream $ dockingPortPositionStreamReq thisArg referenceFrameArg 

{-
 - The rotation of the docking port, in the given reference frame.
 -}
dockingPortRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double, Double))
dockingPortRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

dockingPortRotationStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
dockingPortRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

dockingPortRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double, Double)))
dockingPortRotationStream thisArg referenceFrameArg = requestStream $ dockingPortRotationStreamReq thisArg referenceFrameArg 

{-
 - Undocks the docking port and returns the new <see cref="T:SpaceCenter.Vessel" /> that is created.
 - This method can be called for either docking port in a docked pair.
 - Throws an exception if the docking port is not docked to anything.When called, the active vessel may change. It is therefore possible that,
 - after calling this function, the object(s) returned by previous call(s) to
 - <see cref="M:SpaceCenter.ActiveVessel" /> no longer refer to the active vessel.
 -}
dockingPortUndock :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCHS.SpaceCenter.Vessel)
dockingPortUndock thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

dockingPortUndockStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
dockingPortUndockStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]
    in  makeStream req

dockingPortUndockStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
dockingPortUndockStream thisArg = requestStream $ dockingPortUndockStreamReq thisArg 

{-
 - The part that this docking port is docked to. Returnsnullif this
 - docking port is not docked to anything.
 -}
getDockingPortDockedPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortDockedPartStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortDockedPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortDockedPartStream thisArg = requestStream $ getDockingPortDockedPartStreamReq thisArg 

{-
 - Whether the docking port has a shield.
 -}
getDockingPortHasShield :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (Bool)
getDockingPortHasShield thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortHasShieldStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Bool)
getDockingPortHasShieldStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortHasShieldStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (Bool))
getDockingPortHasShieldStream thisArg = requestStream $ getDockingPortHasShieldStreamReq thisArg 

{-
 - The part object for this docking port.
 -}
getDockingPortPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCHS.SpaceCenter.Part)
getDockingPortPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortPartStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDockingPortPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortPartStream thisArg = requestStream $ getDockingPortPartStreamReq thisArg 

{-
 - The distance a docking port must move away when it undocks before it
 - becomes ready to dock with another port, in meters.
 -}
getDockingPortReengageDistance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (Float)
getDockingPortReengageDistance thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortReengageDistanceStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Float)
getDockingPortReengageDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortReengageDistanceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (Float))
getDockingPortReengageDistanceStream thisArg = requestStream $ getDockingPortReengageDistanceStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this docking port, and
 - oriented with the port.
 - <list type="bullet">The origin is at the position of the docking port.The axes rotate with the docking port.The x-axis points out to the right side of the docking port.The y-axis points in the direction the docking port is facing.The z-axis points out of the bottom off the docking port.This reference frame is not necessarily equivalent to the reference frame
 - for the part, returned by <see cref="M:SpaceCenter.Part.ReferenceFrame" />.
 -}
getDockingPortReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortReferenceFrameStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getDockingPortReferenceFrameStream thisArg = requestStream $ getDockingPortReferenceFrameStreamReq thisArg 

{-
 - The state of the docking ports shield, if it has one.
 - 
 - Returnstrueif the docking port has a shield, and the shield is
 - closed. Otherwise returnsfalse. When set totrue, the shield is
 - closed, and when set tofalsethe shield is opened. If the docking
 - port does not have a shield, setting this attribute has no effect.
 -}
getDockingPortShielded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (Bool)
getDockingPortShielded thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortShieldedStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Bool)
getDockingPortShieldedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortShieldedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (Bool))
getDockingPortShieldedStream thisArg = requestStream $ getDockingPortShieldedStreamReq thisArg 

{-
 - The current state of the docking port.
 -}
getDockingPortState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortState thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortStateStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m (KRPCStream (KRPCHS.SpaceCenter.DockingPortState))
getDockingPortStateStream thisArg = requestStream $ getDockingPortStateStreamReq thisArg 

{-
 - The state of the docking ports shield, if it has one.
 - 
 - Returnstrueif the docking port has a shield, and the shield is
 - closed. Otherwise returnsfalse. When set totrue, the shield is
 - closed, and when set tofalsethe shield is opened. If the docking
 - port does not have a shield, setting this attribute has no effect.
 -}
setDockingPortShielded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> Bool -> m ()
setDockingPortShielded thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Shielded" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Toggle the current engine mode.
 -}
engineToggleMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m ()
engineToggleMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_ToggleMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the engine is active. Setting this attribute may have no effect,
 - depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
getEngineActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineActive thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineActiveStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getEngineActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineActiveStream thisArg = requestStream $ getEngineActiveStreamReq thisArg 

{-
 - Whether the engine will automatically switch modes.
 -}
getEngineAutoModeSwitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineAutoModeSwitch thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineAutoModeSwitchStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineAutoModeSwitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]
    in  makeStream req

getEngineAutoModeSwitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineAutoModeSwitchStream thisArg = requestStream $ getEngineAutoModeSwitchStreamReq thisArg 

{-
 - The amount of thrust, in Newtons, that would be produced by the engine
 - when activated and with its throttle set to 100%.
 - Returns zero if the engine does not have any fuel.
 - Takes the engine's current <see cref="M:SpaceCenter.Engine.ThrustLimit" /> and atmospheric conditions into account.
 -}
getEngineAvailableThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineAvailableThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineAvailableThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineAvailableThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineAvailableThrustStream thisArg = requestStream $ getEngineAvailableThrustStreamReq thisArg 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the engine is inactive, or not gimballed.
 -}
getEngineAvailableTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (((Double, Double, Double), (Double, Double, Double)))
getEngineAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getEngineAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getEngineAvailableTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getEngineAvailableTorqueStream thisArg = requestStream $ getEngineAvailableTorqueStreamReq thisArg 

{-
 - Whether the engine can be restarted once shutdown. If the engine cannot be shutdown,
 - returnsfalse. For example, this istruefor liquid fueled rockets
 - andfalsefor solid rocket boosters.
 -}
getEngineCanRestart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineCanRestart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineCanRestartStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineCanRestartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]
    in  makeStream req

getEngineCanRestartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineCanRestartStream thisArg = requestStream $ getEngineCanRestartStreamReq thisArg 

{-
 - Whether the engine can be shutdown once activated. For example, this istruefor liquid fueled rockets andfalsefor solid rocket boosters.
 -}
getEngineCanShutdown :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineCanShutdown thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineCanShutdownStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineCanShutdownStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]
    in  makeStream req

getEngineCanShutdownStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineCanShutdownStream thisArg = requestStream $ getEngineCanShutdownStreamReq thisArg 

{-
 - The gimbal limiter of the engine. A value between 0 and 1.
 - Returns 0 if the gimbal is locked.
 -}
getEngineGimbalLimit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineGimbalLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimbalLimitStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineGimbalLimitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimbalLimitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineGimbalLimitStream thisArg = requestStream $ getEngineGimbalLimitStreamReq thisArg 

{-
 - Whether the engines gimbal is locked in place. Setting this attribute has
 - no effect if the engine is not gimballed.
 -}
getEngineGimbalLocked :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineGimbalLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimbalLockedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineGimbalLockedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimbalLockedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineGimbalLockedStream thisArg = requestStream $ getEngineGimbalLockedStreamReq thisArg 

{-
 - The range over which the gimbal can move, in degrees.
 - Returns 0 if the engine is not gimballed.
 -}
getEngineGimbalRange :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineGimbalRange thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimbalRangeStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineGimbalRangeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimbalRangeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineGimbalRangeStream thisArg = requestStream $ getEngineGimbalRangeStreamReq thisArg 

{-
 - Whether the engine is gimballed.
 -}
getEngineGimballed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimballedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineGimballedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimballedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineGimballedStream thisArg = requestStream $ getEngineGimballedStreamReq thisArg 

{-
 - Whether the engine has any fuel available.The engine must be activated for this property to update correctly.
 -}
getEngineHasFuel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineHasFuelStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineHasFuelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]
    in  makeStream req

getEngineHasFuelStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineHasFuelStream thisArg = requestStream $ getEngineHasFuelStreamReq thisArg 

{-
 - Whether the engine has multiple modes of operation.
 -}
getEngineHasModes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineHasModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineHasModesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineHasModesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]
    in  makeStream req

getEngineHasModesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineHasModesStream thisArg = requestStream $ getEngineHasModesStreamReq thisArg 

{-
 - The specific impulse of the engine at sea level on Kerbin, in seconds.
 -}
getEngineKerbinSeaLevelSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineKerbinSeaLevelSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getEngineKerbinSeaLevelSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineKerbinSeaLevelSpecificImpulseStream thisArg = requestStream $ getEngineKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-
 - The amount of thrust, in Newtons, that would be produced by the engine
 - when activated and fueled, with its throttle and throttle limiter set to 100%.
 -}
getEngineMaxThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineMaxThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineMaxThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineMaxThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineMaxThrustStream thisArg = requestStream $ getEngineMaxThrustStreamReq thisArg 

{-
 - The maximum amount of thrust that can be produced by the engine in a
 - vacuum, in Newtons. This is the amount of thrust produced by the engine
 - when activated, <see cref="M:SpaceCenter.Engine.ThrustLimit" /> is set to 100%, the main
 - vessel's throttle is set to 100% and the engine is in a vacuum.
 -}
getEngineMaxVacuumThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineMaxVacuumThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineMaxVacuumThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineMaxVacuumThrustStream thisArg = requestStream $ getEngineMaxVacuumThrustStreamReq thisArg 

{-
 - The name of the current engine mode.
 -}
getEngineMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Data.Text.Text)
getEngineMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineModeStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Text.Text)
getEngineModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]
    in  makeStream req

getEngineModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Data.Text.Text))
getEngineModeStream thisArg = requestStream $ getEngineModeStreamReq thisArg 

{-
 - The available modes for the engine.
 - A dictionary mapping mode names to <see cref="T:SpaceCenter.Engine" /> objects.
 -}
getEngineModes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineModesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]
    in  makeStream req

getEngineModesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine)))
getEngineModesStream thisArg = requestStream $ getEngineModesStreamReq thisArg 

{-
 - The part object for this engine.
 -}
getEnginePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCHS.SpaceCenter.Part)
getEnginePart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePartStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getEnginePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getEnginePartStream thisArg = requestStream $ getEnginePartStreamReq thisArg 

{-
 - The names of the propellants that the engine consumes.
 -}
getEnginePropellantNames :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m ([Data.Text.Text])
getEnginePropellantNames thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantNames" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePropellantNamesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([Data.Text.Text])
getEnginePropellantNamesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_PropellantNames" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePropellantNamesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream ([Data.Text.Text]))
getEnginePropellantNamesStream thisArg = requestStream $ getEnginePropellantNamesStreamReq thisArg 

{-
 - The ratio of resources that the engine consumes. A dictionary mapping resource names
 - to the ratio at which they are consumed by the engine.For example, if the ratios are 0.6 for LiquidFuel and 0.4 for Oxidizer, then for every 0.6 units of
 - LiquidFuel that the engine burns, it will burn 0.4 units of Oxidizer.
 -}
getEnginePropellantRatios :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePropellantRatiosStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatiosStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePropellantRatiosStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getEnginePropellantRatiosStream thisArg = requestStream $ getEnginePropellantRatiosStreamReq thisArg 

{-
 - The propellants that the engine consumes.
 -}
getEnginePropellants :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellants thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePropellantsStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellantsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePropellantsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream ([KRPCHS.SpaceCenter.Propellant]))
getEnginePropellantsStream thisArg = requestStream $ getEnginePropellantsStreamReq thisArg 

{-
 - The current specific impulse of the engine, in seconds. Returns zero
 - if the engine is not active.
 -}
getEngineSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getEngineSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineSpecificImpulseStream thisArg = requestStream $ getEngineSpecificImpulseStreamReq thisArg 

{-
 - The current throttle setting for the engine. A value between 0 and 1.
 - This is not necessarily the same as the vessel's main throttle
 - setting, as some engines take time to adjust their throttle
 - (such as jet engines).
 -}
getEngineThrottle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Throttle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrottleStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrottleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Throttle" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrottleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineThrottleStream thisArg = requestStream $ getEngineThrottleStreamReq thisArg 

{-
 - Whether the <see cref="M:SpaceCenter.Control.Throttle" /> affects the engine. For example,
 - this istruefor liquid fueled rockets, andfalsefor solid rocket
 - boosters.
 -}
getEngineThrottleLocked :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Bool)
getEngineThrottleLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrottleLockedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineThrottleLockedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrottleLockedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Bool))
getEngineThrottleLockedStream thisArg = requestStream $ getEngineThrottleLockedStreamReq thisArg 

{-
 - The current amount of thrust being produced by the engine, in Newtons.
 -}
getEngineThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineThrustStream thisArg = requestStream $ getEngineThrustStreamReq thisArg 

{-
 - The thrust limiter of the engine. A value between 0 and 1. Setting this
 - attribute may have no effect, for example the thrust limit for a solid
 - rocket booster cannot be changed in flight.
 -}
getEngineThrustLimit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineThrustLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrustLimitStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrustLimitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrustLimitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineThrustLimitStream thisArg = requestStream $ getEngineThrustLimitStreamReq thisArg 

{-
 - The components of the engine that generate thrust.For example, this corresponds to the rocket nozzel on a solid rocket booster,
 - or the individual nozzels on a RAPIER engine.
 - The overall thrust produced by the engine, as reported by <see cref="M:SpaceCenter.Engine.AvailableThrust" />,
 - <see cref="M:SpaceCenter.Engine.MaxThrust" /> and others, is the sum of the thrust generated by each thruster.
 -}
getEngineThrusters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m ([KRPCHS.SpaceCenter.Thruster])
getEngineThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrusters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrustersStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([KRPCHS.SpaceCenter.Thruster])
getEngineThrustersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Thrusters" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrustersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getEngineThrustersStream thisArg = requestStream $ getEngineThrustersStreamReq thisArg 

{-
 - The vacuum specific impulse of the engine, in seconds.
 -}
getEngineVacuumSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (Float)
getEngineVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineVacuumSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getEngineVacuumSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> m (KRPCStream (Float))
getEngineVacuumSpecificImpulseStream thisArg = requestStream $ getEngineVacuumSpecificImpulseStreamReq thisArg 

{-
 - Whether the engine is active. Setting this attribute may have no effect,
 - depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
setEngineActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> Bool -> m ()
setEngineActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the engine will automatically switch modes.
 -}
setEngineAutoModeSwitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> Bool -> m ()
setEngineAutoModeSwitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_AutoModeSwitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The gimbal limiter of the engine. A value between 0 and 1.
 - Returns 0 if the gimbal is locked.
 -}
setEngineGimbalLimit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> Float -> m ()
setEngineGimbalLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the engines gimbal is locked in place. Setting this attribute has
 - no effect if the engine is not gimballed.
 -}
setEngineGimbalLocked :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> Bool -> m ()
setEngineGimbalLocked thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The name of the current engine mode.
 -}
setEngineMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> Data.Text.Text -> m ()
setEngineMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The thrust limiter of the engine. A value between 0 and 1. Setting this
 - attribute may have no effect, for example the thrust limit for a solid
 - rocket booster cannot be changed in flight.
 -}
setEngineThrustLimit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Engine -> Float -> m ()
setEngineThrustLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_ThrustLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Dump the experimental data contained by the experiment.
 -}
experimentDump :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m ()
experimentDump thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Dump" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reset the experiment.
 -}
experimentReset :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m ()
experimentReset thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Reset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Run the experiment.
 -}
experimentRun :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m ()
experimentRun thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Run" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Transmit all experimental data contained by this part.
 -}
experimentTransmit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m ()
experimentTransmit thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Transmit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Determines if the experiment is available given the current conditions.
 -}
getExperimentAvailable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (Bool)
getExperimentAvailable thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Available" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentAvailableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentAvailableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Available" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentAvailableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (Bool))
getExperimentAvailableStream thisArg = requestStream $ getExperimentAvailableStreamReq thisArg 

{-
 - The name of the biome the experiment is currently in.
 -}
getExperimentBiome :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (Data.Text.Text)
getExperimentBiome thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Biome" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentBiomeStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Data.Text.Text)
getExperimentBiomeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Biome" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentBiomeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (Data.Text.Text))
getExperimentBiomeStream thisArg = requestStream $ getExperimentBiomeStreamReq thisArg 

{-
 - The data contained in this experiment.
 -}
getExperimentData :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m ([KRPCHS.SpaceCenter.ScienceData])
getExperimentData thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Data" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentDataStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq ([KRPCHS.SpaceCenter.ScienceData])
getExperimentDataStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Data" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentDataStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream ([KRPCHS.SpaceCenter.ScienceData]))
getExperimentDataStream thisArg = requestStream $ getExperimentDataStreamReq thisArg 

{-
 - Whether the experiment has been deployed.
 -}
getExperimentDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (Bool)
getExperimentDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentDeployedStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (Bool))
getExperimentDeployedStream thisArg = requestStream $ getExperimentDeployedStreamReq thisArg 

{-
 - Whether the experiment contains data.
 -}
getExperimentHasData :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (Bool)
getExperimentHasData thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_HasData" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentHasDataStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentHasDataStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_HasData" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentHasDataStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (Bool))
getExperimentHasDataStream thisArg = requestStream $ getExperimentHasDataStreamReq thisArg 

{-
 - Whether the experiment is inoperable.
 -}
getExperimentInoperable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (Bool)
getExperimentInoperable thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Inoperable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentInoperableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentInoperableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Inoperable" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentInoperableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (Bool))
getExperimentInoperableStream thisArg = requestStream $ getExperimentInoperableStreamReq thisArg 

{-
 - The part object for this experiment.
 -}
getExperimentPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCHS.SpaceCenter.Part)
getExperimentPart thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentPartStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getExperimentPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getExperimentPartStream thisArg = requestStream $ getExperimentPartStreamReq thisArg 

{-
 - Whether the experiment can be re-run.
 -}
getExperimentRerunnable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (Bool)
getExperimentRerunnable thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Rerunnable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentRerunnableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentRerunnableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Rerunnable" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentRerunnableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (Bool))
getExperimentRerunnableStream thisArg = requestStream $ getExperimentRerunnableStreamReq thisArg 

{-
 - Containing information on the corresponding specific science result for the current conditions.
 - Returns null if experiment is unavailable.
 -}
getExperimentScienceSubject :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCHS.SpaceCenter.ScienceSubject)
getExperimentScienceSubject thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_ScienceSubject" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentScienceSubjectStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (KRPCHS.SpaceCenter.ScienceSubject)
getExperimentScienceSubjectStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_ScienceSubject" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentScienceSubjectStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Experiment -> m (KRPCStream (KRPCHS.SpaceCenter.ScienceSubject))
getExperimentScienceSubjectStream thisArg = requestStream $ getExperimentScienceSubjectStreamReq thisArg 

{-
 - Jettison the fairing. Has no effect if it has already been jettisoned.
 -}
fairingJettison :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Fairing -> m ()
fairingJettison thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_Jettison" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the fairing has been jettisoned.
 -}
getFairingJettisoned :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Fairing -> m (Bool)
getFairingJettisoned thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFairingJettisonedStreamReq :: KRPCHS.SpaceCenter.Fairing -> KRPCStreamReq (Bool)
getFairingJettisonedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]
    in  makeStream req

getFairingJettisonedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Fairing -> m (KRPCStream (Bool))
getFairingJettisonedStream thisArg = requestStream $ getFairingJettisonedStreamReq thisArg 

{-
 - The part object for this fairing.
 -}
getFairingPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Fairing -> m (KRPCHS.SpaceCenter.Part)
getFairingPart thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFairingPartStreamReq :: KRPCHS.SpaceCenter.Fairing -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getFairingPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getFairingPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Fairing -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getFairingPartStream thisArg = requestStream $ getFairingPartStreamReq thisArg 

{-
 - The total aerodynamic forces acting on the vessel, as a vector pointing in the direction of the force, with its
 - magnitude equal to the strength of the force in Newtons.
 -}
getFlightAerodynamicForce :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightAerodynamicForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAerodynamicForceStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAerodynamicForceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAerodynamicForceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightAerodynamicForceStream thisArg = requestStream $ getFlightAerodynamicForceStreamReq thisArg 

{-
 - Gets the pitch angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightAngleOfAttack :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightAngleOfAttack thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAngleOfAttackStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightAngleOfAttackStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAngleOfAttackStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightAngleOfAttackStream thisArg = requestStream $ getFlightAngleOfAttackStreamReq thisArg 

{-
 - The unit direction vector pointing in the anti-normal direction.
 -}
getFlightAntiNormal :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightAntiNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAntiNormalStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAntiNormalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAntiNormalStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightAntiNormalStream thisArg = requestStream $ getFlightAntiNormalStreamReq thisArg 

{-
 - The unit direction vector pointing in the anti-radial direction.
 -}
getFlightAntiRadial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightAntiRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAntiRadialStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAntiRadialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAntiRadialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightAntiRadialStream thisArg = requestStream $ getFlightAntiRadialStreamReq thisArg 

{-
 - The current density of the atmosphere around the vessel, inkg/m^3.
 -}
getFlightAtmosphereDensity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightAtmosphereDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAtmosphereDensityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightAtmosphereDensityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAtmosphereDensityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightAtmosphereDensityStream thisArg = requestStream $ getFlightAtmosphereDensityStreamReq thisArg 

{-
 - Gets the <a href="https://en.wikipedia.org/wiki/Ballistic_coefficient">ballistic coefficient.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightBallisticCoefficient :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightBallisticCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightBallisticCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightBallisticCoefficientStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]
    in  makeStream req

getFlightBallisticCoefficientStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightBallisticCoefficientStream thisArg = requestStream $ getFlightBallisticCoefficientStreamReq thisArg 

{-
 - The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 - Measured from the center of mass of the vessel.
 -}
getFlightBedrockAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightBedrockAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightBedrockAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightBedrockAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightBedrockAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightBedrockAltitudeStream thisArg = requestStream $ getFlightBedrockAltitudeStreamReq thisArg 

{-
 - The position of the center of mass of the vessel.
 -}
getFlightCenterOfMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightCenterOfMass thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightCenterOfMassStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightCenterOfMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]
    in  makeStream req

getFlightCenterOfMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightCenterOfMassStream thisArg = requestStream $ getFlightCenterOfMassStreamReq thisArg 

{-
 - The direction vector that the vessel is pointing in.
 -}
getFlightDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightDirection thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDirectionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightDirectionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightDirectionStream thisArg = requestStream $ getFlightDirectionStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic dragcurrently acting on the vessel,
 - as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.
 -}
getFlightDrag :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightDrag thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDragStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightDragStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDragStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightDragStream thisArg = requestStream $ getFlightDragStreamReq thisArg 

{-
 - Gets the coefficient of drag. This is the amount of drag produced by the vessel. It depends on air speed,
 - air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightDragCoefficient :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightDragCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDragCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightDragCoefficientStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDragCoefficientStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightDragCoefficientStream thisArg = requestStream $ getFlightDragCoefficientStreamReq thisArg 

{-
 - The dynamic pressure acting on the vessel, in Pascals. This is a measure of the strength of the
 - aerodynamic forces. It is equal to\frac{1}{2} . \mbox{air density} . \mbox{velocity}^2.
 - It is commonly denotedQ.
 -}
getFlightDynamicPressure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDynamicPressureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightDynamicPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDynamicPressureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightDynamicPressureStream thisArg = requestStream $ getFlightDynamicPressureStreamReq thisArg 

{-
 - The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level,
 - and is negative when the vessel is over the sea.
 -}
getFlightElevation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightElevation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightElevationStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightElevationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]
    in  makeStream req

getFlightElevationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightElevationStream thisArg = requestStream $ getFlightElevationStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Equivalent_airspeed">equivalent air speedof the vessel, inm/s.
 -}
getFlightEquivalentAirSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightEquivalentAirSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightEquivalentAirSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightEquivalentAirSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightEquivalentAirSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightEquivalentAirSpeedStream thisArg = requestStream $ getFlightEquivalentAirSpeedStreamReq thisArg 

{-
 - The current G force acting on the vessel inm/s^2.
 -}
getFlightGForce :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightGForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightGForceStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightGForceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]
    in  makeStream req

getFlightGForceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightGForceStream thisArg = requestStream $ getFlightGForceStreamReq thisArg 

{-
 - The heading angle of the vessel relative to north, in degrees. A value between 0° and 360°.
 -}
getFlightHeading :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightHeadingStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightHeadingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]
    in  makeStream req

getFlightHeadingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightHeadingStream thisArg = requestStream $ getFlightHeadingStreamReq thisArg 

{-
 - The horizontal speed of the vessel in meters per second.
 -}
getFlightHorizontalSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightHorizontalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightHorizontalSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightHorizontalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightHorizontalSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightHorizontalSpeedStream thisArg = requestStream $ getFlightHorizontalSpeedStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Latitude">latitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLatitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightLatitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLatitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightLatitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLatitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightLatitudeStream thisArg = requestStream $ getFlightLatitudeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic liftcurrently acting on the vessel,
 - as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.
 -}
getFlightLift :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightLift thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLiftStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightLiftStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLiftStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightLiftStream thisArg = requestStream $ getFlightLiftStreamReq thisArg 

{-
 - Gets the coefficient of lift. This is the amount of lift produced by the vessel, and depends on air speed, air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightLiftCoefficient :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightLiftCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLiftCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightLiftCoefficientStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLiftCoefficientStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightLiftCoefficientStream thisArg = requestStream $ getFlightLiftCoefficientStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Longitude">longitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLongitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightLongitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLongitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightLongitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLongitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightLongitudeStream thisArg = requestStream $ getFlightLongitudeStreamReq thisArg 

{-
 - The speed of the vessel, in multiples of the speed of sound.
 -}
getFlightMach :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightMach thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightMachStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightMachStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]
    in  makeStream req

getFlightMachStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightMachStream thisArg = requestStream $ getFlightMachStreamReq thisArg 

{-
 - The altitude above sea level, in meters.
 - Measured from the center of mass of the vessel.
 -}
getFlightMeanAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightMeanAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightMeanAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightMeanAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightMeanAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightMeanAltitudeStream thisArg = requestStream $ getFlightMeanAltitudeStreamReq thisArg 

{-
 - The unit direction vector pointing in the normal direction.
 -}
getFlightNormal :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightNormalStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightNormalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]
    in  makeStream req

getFlightNormalStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightNormalStream thisArg = requestStream $ getFlightNormalStreamReq thisArg 

{-
 - The pitch angle of the vessel relative to the horizon, in degrees. A value between -90° and +90°.
 -}
getFlightPitch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightPitchStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]
    in  makeStream req

getFlightPitchStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightPitchStream thisArg = requestStream $ getFlightPitchStreamReq thisArg 

{-
 - The unit direction vector pointing in the prograde direction.
 -}
getFlightPrograde :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightPrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightProgradeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightProgradeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]
    in  makeStream req

getFlightProgradeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightProgradeStream thisArg = requestStream $ getFlightProgradeStreamReq thisArg 

{-
 - The unit direction vector pointing in the radial direction.
 -}
getFlightRadial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRadialStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightRadialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRadialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightRadialStream thisArg = requestStream $ getFlightRadialStreamReq thisArg 

{-
 - The unit direction vector pointing in the retrograde direction.
 -}
getFlightRetrograde :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightRetrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRetrogradeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightRetrogradeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRetrogradeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightRetrogradeStream thisArg = requestStream $ getFlightRetrogradeStreamReq thisArg 

{-
 - The vessels Reynolds number.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightReynoldsNumber :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightReynoldsNumber thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ReynoldsNumber" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightReynoldsNumberStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightReynoldsNumberStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_ReynoldsNumber" [makeArgument 0 thisArg]
    in  makeStream req

getFlightReynoldsNumberStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightReynoldsNumberStream thisArg = requestStream $ getFlightReynoldsNumberStreamReq thisArg 

{-
 - The roll angle of the vessel relative to the horizon, in degrees. A value between -180° and +180°.
 -}
getFlightRoll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRollStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightRollStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRollStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightRollStream thisArg = requestStream $ getFlightRollStreamReq thisArg 

{-
 - The rotation of the vessel.
 -}
getFlightRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double, Double))
getFlightRotation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRotationStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double, Double))
getFlightRotationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double, Double)))
getFlightRotationStream thisArg = requestStream $ getFlightRotationStreamReq thisArg 

{-
 - Gets the yaw angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightSideslipAngle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightSideslipAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSideslipAngleStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightSideslipAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSideslipAngleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightSideslipAngleStream thisArg = requestStream $ getFlightSideslipAngleStreamReq thisArg 

{-
 - The speed of the vessel in meters per second.
 -}
getFlightSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightSpeedStream thisArg = requestStream $ getFlightSpeedStreamReq thisArg 

{-
 - The speed of sound, in the atmosphere around the vessel, inm/s.
 -}
getFlightSpeedOfSound :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightSpeedOfSound thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSpeedOfSoundStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightSpeedOfSoundStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSpeedOfSoundStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightSpeedOfSoundStream thisArg = requestStream $ getFlightSpeedOfSoundStreamReq thisArg 

{-
 - Gets the current amount of stall, between 0 and 1. A value greater than 0.005 indicates a minor stall
 - and a value greater than 0.5 indicates a large-scale stall.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightStallFraction :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightStallFraction thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStallFractionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStallFractionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStallFractionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightStallFractionStream thisArg = requestStream $ getFlightStallFractionStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">static (ambient) temperatureof the
 - atmosphere around the vessel, in Kelvin.
 -}
getFlightStaticAirTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightStaticAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStaticAirTemperatureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticAirTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStaticAirTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightStaticAirTemperatureStream thisArg = requestStream $ getFlightStaticAirTemperatureStreamReq thisArg 

{-
 - The static atmospheric pressure acting on the vessel, in Pascals.
 -}
getFlightStaticPressure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightStaticPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStaticPressureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStaticPressureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightStaticPressureStream thisArg = requestStream $ getFlightStaticPressureStreamReq thisArg 

{-
 - The static atmospheric pressure at mean sea level, in Pascals.
 -}
getFlightStaticPressureAtMSL :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightStaticPressureAtMSL thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressureAtMSL" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStaticPressureAtMSLStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticPressureAtMSLStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StaticPressureAtMSL" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStaticPressureAtMSLStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightStaticPressureAtMSLStream thisArg = requestStream $ getFlightStaticPressureAtMSLStreamReq thisArg 

{-
 - The altitude above the surface of the body or sea level, whichever is closer, in meters.
 - Measured from the center of mass of the vessel.
 -}
getFlightSurfaceAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightSurfaceAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSurfaceAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightSurfaceAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSurfaceAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightSurfaceAltitudeStream thisArg = requestStream $ getFlightSurfaceAltitudeStreamReq thisArg 

{-
 - An estimate of the current terminal velocity of the vessel, inm/s.
 - This is the speed at which the drag forces cancel out the force of gravity.
 -}
getFlightTerminalVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightTerminalVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightTerminalVelocityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTerminalVelocityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]
    in  makeStream req

getFlightTerminalVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightTerminalVelocityStream thisArg = requestStream $ getFlightTerminalVelocityStreamReq thisArg 

{-
 - Gets the thrust specific fuel consumption for the jet engines on the vessel. This is a measure of the
 - efficiency of the engines, with a lower value indicating a more efficient vessel. This value is the
 - number of Newtons of fuel that are burned, per hour, to produce one newton of thrust.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightThrustSpecificFuelConsumption :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightThrustSpecificFuelConsumption thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightThrustSpecificFuelConsumptionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightThrustSpecificFuelConsumptionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]
    in  makeStream req

getFlightThrustSpecificFuelConsumptionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightThrustSpecificFuelConsumptionStream thisArg = requestStream $ getFlightThrustSpecificFuelConsumptionStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">total air temperatureof the atmosphere
 - around the vessel, in Kelvin. This temperature includes the <see cref="M:SpaceCenter.Flight.StaticAirTemperature" /> and the vessel's kinetic energy.
 -}
getFlightTotalAirTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightTotalAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightTotalAirTemperatureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTotalAirTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getFlightTotalAirTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightTotalAirTemperatureStream thisArg = requestStream $ getFlightTotalAirTemperatureStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/True_airspeed">true air speedof the vessel, inm/s.
 -}
getFlightTrueAirSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Float)
getFlightTrueAirSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TrueAirSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightTrueAirSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTrueAirSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_TrueAirSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightTrueAirSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Float))
getFlightTrueAirSpeedStream thisArg = requestStream $ getFlightTrueAirSpeedStreamReq thisArg 

{-
 - The velocity vector of the vessel. The magnitude of the vector is the speed of the vessel in meters per second.
 - The direction of the vector is the direction of the vessels motion.
 -}
getFlightVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m ((Double, Double, Double))
getFlightVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightVelocityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightVelocityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]
    in  makeStream req

getFlightVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream ((Double, Double, Double)))
getFlightVelocityStream thisArg = requestStream $ getFlightVelocityStreamReq thisArg 

{-
 - The vertical speed of the vessel in meters per second.
 -}
getFlightVerticalSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (Double)
getFlightVerticalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightVerticalSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightVerticalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightVerticalSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Flight -> m (KRPCStream (Double))
getFlightVerticalSpeedStream thisArg = requestStream $ getFlightVerticalSpeedStreamReq thisArg 

{-
 - Remove the force.
 -}
forceRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m ()
forceRemove thisArg = do
    let r = makeRequest "SpaceCenter" "Force_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The force vector. The magnitude of the vector is the strength of the force in Newtons.
 -}
getForceForceVector :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m ((Double, Double, Double))
getForceForceVector thisArg = do
    let r = makeRequest "SpaceCenter" "Force_get_ForceVector" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getForceForceVectorStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq ((Double, Double, Double))
getForceForceVectorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Force_get_ForceVector" [makeArgument 0 thisArg]
    in  makeStream req

getForceForceVectorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m (KRPCStream ((Double, Double, Double)))
getForceForceVectorStream thisArg = requestStream $ getForceForceVectorStreamReq thisArg 

{-
 - The part that this force is applied to.
 -}
getForcePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m (KRPCHS.SpaceCenter.Part)
getForcePart thisArg = do
    let r = makeRequest "SpaceCenter" "Force_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getForcePartStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getForcePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Force_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getForcePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getForcePartStream thisArg = requestStream $ getForcePartStreamReq thisArg 

{-
 - The position at which the force acts.
 -}
getForcePosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m ((Double, Double, Double))
getForcePosition thisArg = do
    let r = makeRequest "SpaceCenter" "Force_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getForcePositionStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq ((Double, Double, Double))
getForcePositionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Force_get_Position" [makeArgument 0 thisArg]
    in  makeStream req

getForcePositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m (KRPCStream ((Double, Double, Double)))
getForcePositionStream thisArg = requestStream $ getForcePositionStreamReq thisArg 

{-
 - The reference frame of the force vector and position.
 -}
getForceReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getForceReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Force_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getForceReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getForceReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Force_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getForceReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getForceReferenceFrameStream thisArg = requestStream $ getForceReferenceFrameStreamReq thisArg 

{-
 - The force vector. The magnitude of the vector is the strength of the force in Newtons.
 -}
setForceForceVector :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> (Double, Double, Double) -> m ()
setForceForceVector thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Force_set_ForceVector" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The position at which the force acts.
 -}
setForcePosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> (Double, Double, Double) -> m ()
setForcePosition thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Force_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The reference frame of the force vector and position.
 -}
setForceReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Force -> KRPCHS.SpaceCenter.ReferenceFrame -> m ()
setForceReferenceFrame thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Force_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The area of the intake's opening, in square meters.
 -}
getIntakeArea :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (Float)
getIntakeArea thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeAreaStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeAreaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeAreaStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (KRPCStream (Float))
getIntakeAreaStream thisArg = requestStream $ getIntakeAreaStreamReq thisArg 

{-
 - The rate of flow into the intake, in units of resource per second.
 -}
getIntakeFlow :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (Float)
getIntakeFlow thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeFlowStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeFlowStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeFlowStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (KRPCStream (Float))
getIntakeFlowStream thisArg = requestStream $ getIntakeFlowStreamReq thisArg 

{-
 - Whether the intake is open.
 -}
getIntakeOpen :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (Bool)
getIntakeOpen thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeOpenStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Bool)
getIntakeOpenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeOpenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (KRPCStream (Bool))
getIntakeOpenStream thisArg = requestStream $ getIntakeOpenStreamReq thisArg 

{-
 - The part object for this intake.
 -}
getIntakePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (KRPCHS.SpaceCenter.Part)
getIntakePart thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakePartStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getIntakePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getIntakePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getIntakePartStream thisArg = requestStream $ getIntakePartStreamReq thisArg 

{-
 - Speed of the flow into the intake, inm/s.
 -}
getIntakeSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (Float)
getIntakeSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeSpeedStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> m (KRPCStream (Float))
getIntakeSpeedStream thisArg = requestStream $ getIntakeSpeedStreamReq thisArg 

{-
 - Whether the intake is open.
 -}
setIntakeOpen :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Intake -> Bool -> m ()
setIntakeOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Intake_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Releases the docking clamp. Has no effect if the clamp has already been released.
 -}
launchClampRelease :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.LaunchClamp -> m ()
launchClampRelease thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_Release" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The part object for this launch clamp.
 -}
getLaunchClampPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.LaunchClamp -> m (KRPCHS.SpaceCenter.Part)
getLaunchClampPart thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLaunchClampPartStreamReq :: KRPCHS.SpaceCenter.LaunchClamp -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLaunchClampPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLaunchClampPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.LaunchClamp -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getLaunchClampPartStream thisArg = requestStream $ getLaunchClampPartStreamReq thisArg 

{-
 - Launch a vessel.<param name="craftDirectory">Name of the directory in the current saves "Ships" directory, that contains the craft file. For example"VAB"or"SPH".<param name="name">Name of the vessel to launch. This is the name of the ".craft" file in the save directory, without the ".craft" file extension.<param name="launchSite">Name of the launch site. For example"LaunchPad"or"Runway".
 -}
launchVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> Data.Text.Text -> Data.Text.Text -> m ()
launchVessel craftDirectoryArg nameArg launchSiteArg = do
    let r = makeRequest "SpaceCenter" "LaunchVessel" [makeArgument 0 craftDirectoryArg, makeArgument 1 nameArg, makeArgument 2 launchSiteArg]
    res <- sendRequest r
    processResponse res 

{-
 - Launch a new vessel from the SPH onto the runway.<param name="name">Name of the vessel to launch.This is equivalent to calling <see cref="M:SpaceCenter.LaunchVessel" /> with the craft directory set to "SPH" and the launch site set to "Runway".
 -}
launchVesselFromSPH :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m ()
launchVesselFromSPH nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromSPH" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Launch a new vessel from the VAB onto the launchpad.<param name="name">Name of the vessel to launch.This is equivalent to calling <see cref="M:SpaceCenter.LaunchVessel" /> with the craft directory set to "VAB" and the launch site set to "LaunchPad".
 -}
launchVesselFromVAB :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m ()
launchVesselFromVAB nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromVAB" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns a list of vessels from the given <paramref name="craftDirectory" /> that can be launched.<param name="craftDirectory">Name of the directory in the current saves "Ships" directory. For example"VAB"or"SPH".
 -}
launchableVessels :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m ([Data.Text.Text])
launchableVessels craftDirectoryArg = do
    let r = makeRequest "SpaceCenter" "LaunchableVessels" [makeArgument 0 craftDirectoryArg]
    res <- sendRequest r
    processResponse res

launchableVesselsStreamReq :: Data.Text.Text -> KRPCStreamReq ([Data.Text.Text])
launchableVesselsStreamReq craftDirectoryArg =
    let req = makeRequest "SpaceCenter" "LaunchableVessels" [makeArgument 0 craftDirectoryArg]
    in  makeStream req

launchableVesselsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (KRPCStream ([Data.Text.Text]))
launchableVesselsStream craftDirectoryArg = requestStream $ launchableVesselsStreamReq craftDirectoryArg 

{-
 - Whether the leg is deployable.
 -}
getLegDeployable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (Bool)
getLegDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Leg_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLegDeployableStreamReq :: KRPCHS.SpaceCenter.Leg -> KRPCStreamReq (Bool)
getLegDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Leg_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getLegDeployableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCStream (Bool))
getLegDeployableStream thisArg = requestStream $ getLegDeployableStreamReq thisArg 

{-
 - Whether the landing leg is deployed.Fixed landing legs are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
getLegDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (Bool)
getLegDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Leg_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLegDeployedStreamReq :: KRPCHS.SpaceCenter.Leg -> KRPCStreamReq (Bool)
getLegDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Leg_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getLegDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCStream (Bool))
getLegDeployedStream thisArg = requestStream $ getLegDeployedStreamReq thisArg 

{-
 - Returns whether the leg is touching the ground.
 -}
getLegIsGrounded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (Bool)
getLegIsGrounded thisArg = do
    let r = makeRequest "SpaceCenter" "Leg_get_IsGrounded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLegIsGroundedStreamReq :: KRPCHS.SpaceCenter.Leg -> KRPCStreamReq (Bool)
getLegIsGroundedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Leg_get_IsGrounded" [makeArgument 0 thisArg]
    in  makeStream req

getLegIsGroundedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCStream (Bool))
getLegIsGroundedStream thisArg = requestStream $ getLegIsGroundedStreamReq thisArg 

{-
 - The part object for this landing leg.
 -}
getLegPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCHS.SpaceCenter.Part)
getLegPart thisArg = do
    let r = makeRequest "SpaceCenter" "Leg_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLegPartStreamReq :: KRPCHS.SpaceCenter.Leg -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLegPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Leg_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLegPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getLegPartStream thisArg = requestStream $ getLegPartStreamReq thisArg 

{-
 - The current state of the landing leg.
 -}
getLegState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCHS.SpaceCenter.LegState)
getLegState thisArg = do
    let r = makeRequest "SpaceCenter" "Leg_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLegStateStreamReq :: KRPCHS.SpaceCenter.Leg -> KRPCStreamReq (KRPCHS.SpaceCenter.LegState)
getLegStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Leg_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getLegStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> m (KRPCStream (KRPCHS.SpaceCenter.LegState))
getLegStateStream thisArg = requestStream $ getLegStateStreamReq thisArg 

{-
 - Whether the landing leg is deployed.Fixed landing legs are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
setLegDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Leg -> Bool -> m ()
setLegDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Leg_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the light is switched on.
 -}
getLightActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (Bool)
getLightActive thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightActiveStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (Bool)
getLightActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getLightActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (KRPCStream (Bool))
getLightActiveStream thisArg = requestStream $ getLightActiveStreamReq thisArg 

{-
 - The color of the light, as an RGB triple.
 -}
getLightColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m ((Float, Float, Float))
getLightColor thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightColorStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq ((Float, Float, Float))
getLightColorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]
    in  makeStream req

getLightColorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (KRPCStream ((Float, Float, Float)))
getLightColorStream thisArg = requestStream $ getLightColorStreamReq thisArg 

{-
 - The part object for this light.
 -}
getLightPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (KRPCHS.SpaceCenter.Part)
getLightPart thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightPartStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLightPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLightPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getLightPartStream thisArg = requestStream $ getLightPartStreamReq thisArg 

{-
 - The current power usage, in units of charge per second.
 -}
getLightPowerUsage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (Float)
getLightPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightPowerUsageStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (Float)
getLightPowerUsageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]
    in  makeStream req

getLightPowerUsageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> m (KRPCStream (Float))
getLightPowerUsageStream thisArg = requestStream $ getLightPowerUsageStreamReq thisArg 

{-
 - Whether the light is switched on.
 -}
setLightActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> Bool -> m ()
setLightActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The color of the light, as an RGB triple.
 -}
setLightColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Light -> (Float, Float, Float) -> m ()
setLightColor thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Load the game with the given name.
 - This will create a load a save file calledname.sfsfrom the folder of the current save game.
 -}
load :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m ()
load nameArg = do
    let r = makeRequest "SpaceCenter" "Load" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns the value of a field.<param name="name">Name of the field.
 -}
moduleGetField :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (Data.Text.Text)
moduleGetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleGetFieldStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Data.Text.Text)
moduleGetFieldStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleGetFieldStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (KRPCStream (Data.Text.Text))
moduleGetFieldStream thisArg nameArg = requestStream $ moduleGetFieldStreamReq thisArg nameArg 

{-
 - trueif the part has an action with the given name.<param name="name">
 -}
moduleHasAction :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (Bool)
moduleHasAction thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleHasActionStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasActionStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleHasActionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (KRPCStream (Bool))
moduleHasActionStream thisArg nameArg = requestStream $ moduleHasActionStreamReq thisArg nameArg 

{-
 - trueif the module has an event with the given name.<param name="name">
 -}
moduleHasEvent :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (Bool)
moduleHasEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleHasEventStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasEventStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleHasEventStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (KRPCStream (Bool))
moduleHasEventStream thisArg nameArg = requestStream $ moduleHasEventStreamReq thisArg nameArg 

{-
 - Returnstrueif the module has a field with the given name.<param name="name">Name of the field.
 -}
moduleHasField :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (Bool)
moduleHasField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleHasFieldStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasFieldStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleHasFieldStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m (KRPCStream (Bool))
moduleHasFieldStream thisArg nameArg = requestStream $ moduleHasFieldStreamReq thisArg nameArg 

{-
 - Set the value of a field to its original value.<param name="name">Name of the field.
 -}
moduleResetField :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m ()
moduleResetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_ResetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of an action with the given name.<param name="name"><param name="value">
 -}
moduleSetAction :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Bool -> m ()
moduleSetAction thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetAction" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of a field to the given floating point number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldFloat :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Float -> m ()
moduleSetFieldFloat thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldFloat" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of a field to the given integer number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldInt :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Int.Int32 -> m ()
moduleSetFieldInt thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldInt" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of a field to the given string.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldString :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Text.Text -> m ()
moduleSetFieldString thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldString" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Trigger the named event. Equivalent to clicking the button in the right-click menu of the part.<param name="name">
 -}
moduleTriggerEvent :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> Data.Text.Text -> m ()
moduleTriggerEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_TriggerEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - A list of all the names of the modules actions. These are the parts actions that can be assigned
 - to action groups in the in-game editor.
 -}
getModuleActions :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m ([Data.Text.Text])
getModuleActions thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleActionsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq ([Data.Text.Text])
getModuleActionsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]
    in  makeStream req

getModuleActionsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (KRPCStream ([Data.Text.Text]))
getModuleActionsStream thisArg = requestStream $ getModuleActionsStreamReq thisArg 

{-
 - A list of the names of all of the modules events. Events are the clickable buttons
 - visible in the right-click menu of the part.
 -}
getModuleEvents :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m ([Data.Text.Text])
getModuleEvents thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleEventsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq ([Data.Text.Text])
getModuleEventsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]
    in  makeStream req

getModuleEventsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (KRPCStream ([Data.Text.Text]))
getModuleEventsStream thisArg = requestStream $ getModuleEventsStreamReq thisArg 

{-
 - The modules field names and their associated values, as a dictionary.
 - These are the values visible in the right-click menu of the part.
 -}
getModuleFields :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFields thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleFieldsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFieldsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]
    in  makeStream req

getModuleFieldsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (KRPCStream (Data.Map.Map (Data.Text.Text) (Data.Text.Text)))
getModuleFieldsStream thisArg = requestStream $ getModuleFieldsStreamReq thisArg 

{-
 - Name of the PartModule. For example, "ModuleEngines".
 -}
getModuleName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (Data.Text.Text)
getModuleName thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleNameStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (Data.Text.Text)
getModuleNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getModuleNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (KRPCStream (Data.Text.Text))
getModuleNameStream thisArg = requestStream $ getModuleNameStreamReq thisArg 

{-
 - The part that contains this module.
 -}
getModulePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (KRPCHS.SpaceCenter.Part)
getModulePart thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModulePartStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getModulePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getModulePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Module -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getModulePartStream thisArg = requestStream $ getModulePartStreamReq thisArg 

{-
 - Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
 - is the delta-v of the burn in m/s.<param name="referenceFrame">Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingBurnVector" />.
 -}
nodeBurnVector :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
nodeBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodeBurnVectorStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeBurnVectorStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodeBurnVectorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
nodeBurnVectorStream thisArg referenceFrameArg = requestStream $ nodeBurnVectorStreamReq thisArg referenceFrameArg 

{-
 - Returns the unit direction vector of the maneuver nodes burn in the given reference frame.<param name="referenceFrame">
 -}
nodeDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
nodeDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodeDirectionStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodeDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
nodeDirectionStream thisArg referenceFrameArg = requestStream $ nodeDirectionStreamReq thisArg referenceFrameArg 

{-
 - Returns the position vector of the maneuver node in the given reference frame.<param name="referenceFrame">
 -}
nodePosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
nodePosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodePositionStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodePositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodePositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
nodePositionStream thisArg referenceFrameArg = requestStream $ nodePositionStreamReq thisArg referenceFrameArg 

{-
 - Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
 - is the delta-v of the burn in m/s. The direction and magnitude change as the burn is executed.<param name="referenceFrame">
 -}
nodeRemainingBurnVector :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
nodeRemainingBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodeRemainingBurnVectorStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeRemainingBurnVectorStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodeRemainingBurnVectorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
nodeRemainingBurnVectorStream thisArg referenceFrameArg = requestStream $ nodeRemainingBurnVectorStreamReq thisArg referenceFrameArg 

{-
 - Removes the maneuver node.
 -}
nodeRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m ()
nodeRemove thisArg = do
    let r = makeRequest "SpaceCenter" "Node_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
getNodeDeltaV :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodeDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeDeltaVStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeDeltaVStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]
    in  makeStream req

getNodeDeltaVStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeDeltaVStream thisArg = requestStream $ getNodeDeltaVStreamReq thisArg 

{-
 - The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
getNodeNormal :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodeNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeNormalStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeNormalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]
    in  makeStream req

getNodeNormalStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeNormalStream thisArg = requestStream $ getNodeNormalStreamReq thisArg 

{-
 - The orbit that results from executing the maneuver node.
 -}
getNodeOrbit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCHS.SpaceCenter.Orbit)
getNodeOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeOrbitStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getNodeOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]
    in  makeStream req

getNodeOrbitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getNodeOrbitStream thisArg = requestStream $ getNodeOrbitStreamReq thisArg 

{-
 - Gets the reference frame that is fixed relative to the maneuver node, and
 - orientated with the orbital prograde/normal/radial directions of the
 - original orbit at the maneuver node's position.
 - <list type="bullet">The origin is at the position of the maneuver node.The x-axis points in the orbital anti-radial direction of the original
 - orbit, at the position of the maneuver node.The y-axis points in the orbital prograde direction of the original
 - orbit, at the position of the maneuver node.The z-axis points in the orbital normal direction of the original orbit,
 - at the position of the maneuver node.
 -}
getNodeOrbitalReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getNodeOrbitalReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeOrbitalReferenceFrameStream thisArg = requestStream $ getNodeOrbitalReferenceFrameStreamReq thisArg 

{-
 - The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
getNodePrograde :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodePrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeProgradeStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeProgradeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]
    in  makeStream req

getNodeProgradeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeProgradeStream thisArg = requestStream $ getNodeProgradeStreamReq thisArg 

{-
 - The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
getNodeRadial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodeRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeRadialStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeRadialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]
    in  makeStream req

getNodeRadialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeRadialStream thisArg = requestStream $ getNodeRadialStreamReq thisArg 

{-
 - Gets the reference frame that is fixed relative to the maneuver node's burn.
 - <list type="bullet">The origin is at the position of the maneuver node.The y-axis points in the direction of the burn.The x-axis and z-axis point in arbitrary but fixed directions.
 -}
getNodeReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getNodeReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeReferenceFrameStream thisArg = requestStream $ getNodeReferenceFrameStreamReq thisArg 

{-
 - Gets the remaining delta-v of the maneuver node, in meters per second. Changes as the node
 - is executed. This is equivalent to the delta-v reported in-game.
 -}
getNodeRemainingDeltaV :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodeRemainingDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeRemainingDeltaVStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeRemainingDeltaVStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]
    in  makeStream req

getNodeRemainingDeltaVStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeRemainingDeltaVStream thisArg = requestStream $ getNodeRemainingDeltaVStreamReq thisArg 

{-
 - The time until the maneuver node will be encountered, in seconds.
 -}
getNodeTimeTo :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodeTimeTo thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeTimeToStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeTimeToStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]
    in  makeStream req

getNodeTimeToStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeTimeToStream thisArg = requestStream $ getNodeTimeToStreamReq thisArg 

{-
 - The universal time at which the maneuver will occur, in seconds.
 -}
getNodeUT :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (Double)
getNodeUT thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeUTStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeUTStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]
    in  makeStream req

getNodeUTStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> m (KRPCStream (Double))
getNodeUTStream thisArg = requestStream $ getNodeUTStreamReq thisArg 

{-
 - The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
setNodeDeltaV :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> Double -> m ()
setNodeDeltaV thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_DeltaV" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
setNodeNormal :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> Double -> m ()
setNodeNormal thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Normal" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
setNodePrograde :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> Double -> m ()
setNodePrograde thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Prograde" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
setNodeRadial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> Double -> m ()
setNodeRadial thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Radial" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The universal time at which the maneuver will occur, in seconds.
 -}
setNodeUT :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Node -> Double -> m ()
setNodeUT thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_UT" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The eccentric anomaly at the given universal time.<param name="ut">The universal time, in seconds.
 -}
orbitEccentricAnomalyAtUT :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (Double)
orbitEccentricAnomalyAtUT thisArg utArg = do
    let r = makeRequest "SpaceCenter" "Orbit_EccentricAnomalyAtUT" [makeArgument 0 thisArg, makeArgument 1 utArg]
    res <- sendRequest r
    processResponse res

orbitEccentricAnomalyAtUTStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitEccentricAnomalyAtUTStreamReq thisArg utArg =
    let req = makeRequest "SpaceCenter" "Orbit_EccentricAnomalyAtUT" [makeArgument 0 thisArg, makeArgument 1 utArg]
    in  makeStream req

orbitEccentricAnomalyAtUTStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (KRPCStream (Double))
orbitEccentricAnomalyAtUTStream thisArg utArg = requestStream $ orbitEccentricAnomalyAtUTStreamReq thisArg utArg 

{-
 - The orbital speed at the given time, in meters per second.<param name="time">Time from now, in seconds.
 -}
orbitOrbitalSpeedAt :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (Double)
orbitOrbitalSpeedAt thisArg timeArg = do
    let r = makeRequest "SpaceCenter" "Orbit_OrbitalSpeedAt" [makeArgument 0 thisArg, makeArgument 1 timeArg]
    res <- sendRequest r
    processResponse res

orbitOrbitalSpeedAtStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitOrbitalSpeedAtStreamReq thisArg timeArg =
    let req = makeRequest "SpaceCenter" "Orbit_OrbitalSpeedAt" [makeArgument 0 thisArg, makeArgument 1 timeArg]
    in  makeStream req

orbitOrbitalSpeedAtStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (KRPCStream (Double))
orbitOrbitalSpeedAtStream thisArg timeArg = requestStream $ orbitOrbitalSpeedAtStreamReq thisArg timeArg 

{-
 - The orbital radius at the point in the orbit given by the true anomaly.<param name="trueAnomaly">The true anomaly.
 -}
orbitRadiusAtTrueAnomaly :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (Double)
orbitRadiusAtTrueAnomaly thisArg trueAnomalyArg = do
    let r = makeRequest "SpaceCenter" "Orbit_RadiusAtTrueAnomaly" [makeArgument 0 thisArg, makeArgument 1 trueAnomalyArg]
    res <- sendRequest r
    processResponse res

orbitRadiusAtTrueAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitRadiusAtTrueAnomalyStreamReq thisArg trueAnomalyArg =
    let req = makeRequest "SpaceCenter" "Orbit_RadiusAtTrueAnomaly" [makeArgument 0 thisArg, makeArgument 1 trueAnomalyArg]
    in  makeStream req

orbitRadiusAtTrueAnomalyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (KRPCStream (Double))
orbitRadiusAtTrueAnomalyStream thisArg trueAnomalyArg = requestStream $ orbitRadiusAtTrueAnomalyStreamReq thisArg trueAnomalyArg 

{-
 - The unit direction vector from which the orbits longitude of ascending node is measured,
 - in the given reference frame.<param name="referenceFrame">
 -}
orbitReferencePlaneDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
orbitReferencePlaneDirection referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]
    res <- sendRequest r
    processResponse res

orbitReferencePlaneDirectionStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
orbitReferencePlaneDirectionStreamReq referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]
    in  makeStream req

orbitReferencePlaneDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneDirectionStream referenceFrameArg = requestStream $ orbitReferencePlaneDirectionStreamReq referenceFrameArg 

{-
 - The unit direction vector that is normal to the orbits reference plane, in the given
 - reference frame. The reference plane is the plane from which the orbits inclination is measured.<param name="referenceFrame">
 -}
orbitReferencePlaneNormal :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
orbitReferencePlaneNormal referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]
    res <- sendRequest r
    processResponse res

orbitReferencePlaneNormalStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
orbitReferencePlaneNormalStreamReq referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]
    in  makeStream req

orbitReferencePlaneNormalStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneNormalStream referenceFrameArg = requestStream $ orbitReferencePlaneNormalStreamReq referenceFrameArg 

{-
 - The true anomaly at the given orbital radius.<param name="radius">The orbital radius in meters.
 -}
orbitTrueAnomalyAtRadius :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (Double)
orbitTrueAnomalyAtRadius thisArg radiusArg = do
    let r = makeRequest "SpaceCenter" "Orbit_TrueAnomalyAtRadius" [makeArgument 0 thisArg, makeArgument 1 radiusArg]
    res <- sendRequest r
    processResponse res

orbitTrueAnomalyAtRadiusStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitTrueAnomalyAtRadiusStreamReq thisArg radiusArg =
    let req = makeRequest "SpaceCenter" "Orbit_TrueAnomalyAtRadius" [makeArgument 0 thisArg, makeArgument 1 radiusArg]
    in  makeStream req

orbitTrueAnomalyAtRadiusStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (KRPCStream (Double))
orbitTrueAnomalyAtRadiusStream thisArg radiusArg = requestStream $ orbitTrueAnomalyAtRadiusStreamReq thisArg radiusArg 

{-
 - The true anomaly at the given time.<param name="ut">The universal time in seconds.
 -}
orbitTrueAnomalyAtUT :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (Double)
orbitTrueAnomalyAtUT thisArg utArg = do
    let r = makeRequest "SpaceCenter" "Orbit_TrueAnomalyAtUT" [makeArgument 0 thisArg, makeArgument 1 utArg]
    res <- sendRequest r
    processResponse res

orbitTrueAnomalyAtUTStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitTrueAnomalyAtUTStreamReq thisArg utArg =
    let req = makeRequest "SpaceCenter" "Orbit_TrueAnomalyAtUT" [makeArgument 0 thisArg, makeArgument 1 utArg]
    in  makeStream req

orbitTrueAnomalyAtUTStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (KRPCStream (Double))
orbitTrueAnomalyAtUTStream thisArg utArg = requestStream $ orbitTrueAnomalyAtUTStreamReq thisArg utArg 

{-
 - The universal time, in seconds, corresponding to the given true anomaly.<param name="trueAnomaly">True anomaly.
 -}
orbitUTAtTrueAnomaly :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (Double)
orbitUTAtTrueAnomaly thisArg trueAnomalyArg = do
    let r = makeRequest "SpaceCenter" "Orbit_UTAtTrueAnomaly" [makeArgument 0 thisArg, makeArgument 1 trueAnomalyArg]
    res <- sendRequest r
    processResponse res

orbitUTAtTrueAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitUTAtTrueAnomalyStreamReq thisArg trueAnomalyArg =
    let req = makeRequest "SpaceCenter" "Orbit_UTAtTrueAnomaly" [makeArgument 0 thisArg, makeArgument 1 trueAnomalyArg]
    in  makeStream req

orbitUTAtTrueAnomalyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> Double -> m (KRPCStream (Double))
orbitUTAtTrueAnomalyStream thisArg trueAnomalyArg = requestStream $ orbitUTAtTrueAnomalyStreamReq thisArg trueAnomalyArg 

{-
 - Gets the apoapsis of the orbit, in meters, from the center of mass of the body being orbited.For the apoapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.ApoapsisAltitude" />.
 -}
getOrbitApoapsis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitApoapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitApoapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitApoapsisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitApoapsisStream thisArg = requestStream $ getOrbitApoapsisStreamReq thisArg 

{-
 - The apoapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Apoapsis" /> minus the equatorial radius of the body.
 -}
getOrbitApoapsisAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitApoapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitApoapsisAltitudeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitApoapsisAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitApoapsisAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitApoapsisAltitudeStream thisArg = requestStream $ getOrbitApoapsisAltitudeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Argument_of_periapsis">argument of periapsis, in radians.
 -}
getOrbitArgumentOfPeriapsis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitArgumentOfPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitArgumentOfPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitArgumentOfPeriapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitArgumentOfPeriapsisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitArgumentOfPeriapsisStream thisArg = requestStream $ getOrbitArgumentOfPeriapsisStreamReq thisArg 

{-
 - The celestial body (e.g. planet or moon) around which the object is orbiting.
 -}
getOrbitBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBody thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitBodyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBodyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitBodyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getOrbitBodyStream thisArg = requestStream $ getOrbitBodyStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Eccentric_anomaly">eccentric anomaly.
 -}
getOrbitEccentricAnomaly :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitEccentricAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitEccentricAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEccentricAnomalyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitEccentricAnomalyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitEccentricAnomalyStream thisArg = requestStream $ getOrbitEccentricAnomalyStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Orbital_eccentricity">eccentricityof the orbit.
 -}
getOrbitEccentricity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitEccentricity thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitEccentricityStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEccentricityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitEccentricityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitEccentricityStream thisArg = requestStream $ getOrbitEccentricityStreamReq thisArg 

{-
 - The time since the epoch (the point at which the
 - <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epochwas measured, in seconds.
 -}
getOrbitEpoch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitEpochStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEpochStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitEpochStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitEpochStream thisArg = requestStream $ getOrbitEpochStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Orbital_inclination">inclinationof the orbit,
 - in radians.
 -}
getOrbitInclination :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitInclination thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitInclinationStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitInclinationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitInclinationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitInclinationStream thisArg = requestStream $ getOrbitInclinationStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Longitude_of_the_ascending_node">longitude of the
 - ascending node, in radians.
 -}
getOrbitLongitudeOfAscendingNode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitLongitudeOfAscendingNode thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitLongitudeOfAscendingNodeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitLongitudeOfAscendingNodeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitLongitudeOfAscendingNodeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitLongitudeOfAscendingNodeStream thisArg = requestStream $ getOrbitLongitudeOfAscendingNodeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly.
 -}
getOrbitMeanAnomaly :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitMeanAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitMeanAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitMeanAnomalyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitMeanAnomalyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitMeanAnomalyStream thisArg = requestStream $ getOrbitMeanAnomalyStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epoch.
 -}
getOrbitMeanAnomalyAtEpoch :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitMeanAnomalyAtEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitMeanAnomalyAtEpochStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitMeanAnomalyAtEpochStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitMeanAnomalyAtEpochStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitMeanAnomalyAtEpochStream thisArg = requestStream $ getOrbitMeanAnomalyAtEpochStreamReq thisArg 

{-
 - If the object is going to change sphere of influence in the future, returns the new orbit
 - after the change. Otherwise returnsnull.
 -}
getOrbitNextOrbit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitNextOrbitStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitNextOrbitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getOrbitNextOrbitStream thisArg = requestStream $ getOrbitNextOrbitStreamReq thisArg 

{-
 - The current orbital speed in meters per second.
 -}
getOrbitOrbitalSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitOrbitalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_OrbitalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitOrbitalSpeedStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitOrbitalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_OrbitalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitOrbitalSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitOrbitalSpeedStream thisArg = requestStream $ getOrbitOrbitalSpeedStreamReq thisArg 

{-
 - The periapsis of the orbit, in meters, from the center of mass of the body being orbited.For the periapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.PeriapsisAltitude" />.
 -}
getOrbitPeriapsis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitPeriapsisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitPeriapsisStream thisArg = requestStream $ getOrbitPeriapsisStreamReq thisArg 

{-
 - The periapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Periapsis" /> minus the equatorial radius of the body.
 -}
getOrbitPeriapsisAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitPeriapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitPeriapsisAltitudeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriapsisAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitPeriapsisAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitPeriapsisAltitudeStream thisArg = requestStream $ getOrbitPeriapsisAltitudeStreamReq thisArg 

{-
 - The orbital period, in seconds.
 -}
getOrbitPeriod :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitPeriodStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriodStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitPeriodStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitPeriodStream thisArg = requestStream $ getOrbitPeriodStreamReq thisArg 

{-
 - The current radius of the orbit, in meters. This is the distance between the center
 - of mass of the object in orbit, and the center of mass of the body around which it is orbiting.This value will change over time if the orbit is elliptical.
 -}
getOrbitRadius :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitRadius thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitRadiusStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitRadiusStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitRadiusStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitRadiusStream thisArg = requestStream $ getOrbitRadiusStreamReq thisArg 

{-
 - The semi-major axis of the orbit, in meters.
 -}
getOrbitSemiMajorAxis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitSemiMajorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitSemiMajorAxisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSemiMajorAxisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitSemiMajorAxisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitSemiMajorAxisStream thisArg = requestStream $ getOrbitSemiMajorAxisStreamReq thisArg 

{-
 - The semi-minor axis of the orbit, in meters.
 -}
getOrbitSemiMinorAxis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitSemiMinorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitSemiMinorAxisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSemiMinorAxisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitSemiMinorAxisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitSemiMinorAxisStream thisArg = requestStream $ getOrbitSemiMinorAxisStreamReq thisArg 

{-
 - The current orbital speed of the object in meters per second.This value will change over time if the orbit is elliptical.
 -}
getOrbitSpeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitSpeedStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitSpeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitSpeedStream thisArg = requestStream $ getOrbitSpeedStreamReq thisArg 

{-
 - The time until the object reaches apoapsis, in seconds.
 -}
getOrbitTimeToApoapsis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitTimeToApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTimeToApoapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToApoapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTimeToApoapsisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitTimeToApoapsisStream thisArg = requestStream $ getOrbitTimeToApoapsisStreamReq thisArg 

{-
 - The time until the object reaches periapsis, in seconds.
 -}
getOrbitTimeToPeriapsis :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitTimeToPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTimeToPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToPeriapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTimeToPeriapsisStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitTimeToPeriapsisStream thisArg = requestStream $ getOrbitTimeToPeriapsisStreamReq thisArg 

{-
 - The time until the object changes sphere of influence, in seconds. ReturnsNaNif the
 - object is not going to change sphere of influence.
 -}
getOrbitTimeToSOIChange :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitTimeToSOIChange thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTimeToSOIChangeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToSOIChangeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTimeToSOIChangeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitTimeToSOIChangeStream thisArg = requestStream $ getOrbitTimeToSOIChangeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/True_anomaly">true anomaly.
 -}
getOrbitTrueAnomaly :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (Double)
getOrbitTrueAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TrueAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTrueAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTrueAnomalyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TrueAnomaly" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTrueAnomalyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Orbit -> m (KRPCStream (Double))
getOrbitTrueAnomalyStream thisArg = requestStream $ getOrbitTrueAnomalyStreamReq thisArg 

{-
 - Deploys the parachute. This has no effect if the parachute has already
 - been armed or deployed. Only applicable to RealChutes parachutes.
 -}
parachuteArm :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m ()
parachuteArm thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_Arm" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Deploys the parachute. This has no effect if the parachute has already
 - been deployed.
 -}
parachuteDeploy :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m ()
parachuteDeploy thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_Deploy" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the parachute has been armed or deployed. Only applicable to RealChutes parachutes.
 -}
getParachuteArmed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (Bool)
getParachuteArmed thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Armed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteArmedStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Bool)
getParachuteArmedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_Armed" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteArmedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCStream (Bool))
getParachuteArmedStream thisArg = requestStream $ getParachuteArmedStreamReq thisArg 

{-
 - The altitude at which the parachute will full deploy, in meters.
 - Only applicable to stock parachutes.
 -}
getParachuteDeployAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (Float)
getParachuteDeployAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteDeployAltitudeStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Float)
getParachuteDeployAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteDeployAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCStream (Float))
getParachuteDeployAltitudeStream thisArg = requestStream $ getParachuteDeployAltitudeStreamReq thisArg 

{-
 - The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 - Only applicable to stock parachutes.
 -}
getParachuteDeployMinPressure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (Float)
getParachuteDeployMinPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteDeployMinPressureStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Float)
getParachuteDeployMinPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteDeployMinPressureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCStream (Float))
getParachuteDeployMinPressureStream thisArg = requestStream $ getParachuteDeployMinPressureStreamReq thisArg 

{-
 - Whether the parachute has been deployed.
 -}
getParachuteDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (Bool)
getParachuteDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteDeployedStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Bool)
getParachuteDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCStream (Bool))
getParachuteDeployedStream thisArg = requestStream $ getParachuteDeployedStreamReq thisArg 

{-
 - The part object for this parachute.
 -}
getParachutePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCHS.SpaceCenter.Part)
getParachutePart thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachutePartStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getParachutePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getParachutePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getParachutePartStream thisArg = requestStream $ getParachutePartStreamReq thisArg 

{-
 - The current state of the parachute.
 -}
getParachuteState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCHS.SpaceCenter.ParachuteState)
getParachuteState thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteStateStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (KRPCHS.SpaceCenter.ParachuteState)
getParachuteStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> m (KRPCStream (KRPCHS.SpaceCenter.ParachuteState))
getParachuteStateStream thisArg = requestStream $ getParachuteStateStreamReq thisArg 

{-
 - The altitude at which the parachute will full deploy, in meters.
 - Only applicable to stock parachutes.
 -}
setParachuteDeployAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> Float -> m ()
setParachuteDeployAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 - Only applicable to stock parachutes.
 -}
setParachuteDeployMinPressure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parachute -> Float -> m ()
setParachuteDeployMinPressure thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployMinPressure" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Exert a constant force on the part, acting at the given position.
 - Returns an object that can be used to remove or modify the force.
 -}
partAddForce :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCHS.SpaceCenter.Force)
partAddForce thisArg forceArg positionArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_AddForce" [makeArgument 0 thisArg, makeArgument 1 forceArg, makeArgument 2 positionArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partAddForceStreamReq :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (KRPCHS.SpaceCenter.Force)
partAddForceStreamReq thisArg forceArg positionArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_AddForce" [makeArgument 0 thisArg, makeArgument 1 forceArg, makeArgument 2 positionArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

partAddForceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream (KRPCHS.SpaceCenter.Force))
partAddForceStream thisArg forceArg positionArg referenceFrameArg = requestStream $ partAddForceStreamReq thisArg forceArg positionArg referenceFrameArg 

{-
 - The axis-aligned bounding box of the vessel in the given reference frame.
 - Returns the minimum and maximum vertices of the box.<param name="referenceFrame">This is computed from the collision meshes of the part.
 - If the part is not collidable, the box has zero volume and is centered on
 - the <see cref="M:SpaceCenter.Part.Position" /> of the part.
 -}
partBoundingBox :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (((Double, Double, Double), (Double, Double, Double)))
partBoundingBox thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_BoundingBox" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partBoundingBoxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
partBoundingBoxStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_BoundingBox" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partBoundingBoxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
partBoundingBoxStream thisArg referenceFrameArg = requestStream $ partBoundingBoxStreamReq thisArg referenceFrameArg 

{-
 - The position of the parts center of mass in the given reference frame.
 - If the part is physicsless, this is equivalent to <see cref="M:SpaceCenter.Part.Position" />.<param name="referenceFrame">
 -}
partCenterOfMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
partCenterOfMass thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partCenterOfMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partCenterOfMassStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partCenterOfMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
partCenterOfMassStream thisArg referenceFrameArg = requestStream $ partCenterOfMassStreamReq thisArg referenceFrameArg 

{-
 - The direction of the part in the given reference frame.<param name="referenceFrame">
 -}
partDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
partDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partDirectionStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
partDirectionStream thisArg referenceFrameArg = requestStream $ partDirectionStreamReq thisArg referenceFrameArg 

{-
 - Exert an instantaneous force on the part, acting at the given position.The force is applied instantaneously in a single physics update.
 -}
partInstantaneousForce :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> m ()
partInstantaneousForce thisArg forceArg positionArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_InstantaneousForce" [makeArgument 0 thisArg, makeArgument 1 forceArg, makeArgument 2 positionArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res 

{-
 - The position of the part in the given reference frame.This is a fixed position in the part, defined by the parts model.
 - It s not necessarily the same as the parts center of mass.
 - Use <see cref="M:SpaceCenter.Part.CenterOfMass" /> to get the parts center of mass.<param name="referenceFrame">
 -}
partPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
partPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partPositionStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
partPositionStream thisArg referenceFrameArg = requestStream $ partPositionStreamReq thisArg referenceFrameArg 

{-
 - The rotation of the part in the given reference frame.<param name="referenceFrame">
 -}
partRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double, Double))
partRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partRotationStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
partRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double, Double)))
partRotationStream thisArg referenceFrameArg = requestStream $ partRotationStreamReq thisArg referenceFrameArg 

{-
 - The velocity of the part in the given reference frame.<param name="referenceFrame">
 -}
partVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
partVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partVelocityStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
partVelocityStream thisArg referenceFrameArg = requestStream $ partVelocityStreamReq thisArg referenceFrameArg 

{-
 - A <see cref="T:SpaceCenter.Antenna" /> if the part is an antenna, otherwisenull.
 -}
getPartAntenna :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Antenna)
getPartAntenna thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Antenna" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartAntennaStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Antenna)
getPartAntennaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Antenna" [makeArgument 0 thisArg]
    in  makeStream req

getPartAntennaStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Antenna))
getPartAntennaStream thisArg = requestStream $ getPartAntennaStreamReq thisArg 

{-
 - Whether the part is axially attached to its parent, i.e. on the top
 - or bottom of its parent. If the part has no parent, returnsfalse.
 -}
getPartAxiallyAttached :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartAxiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartAxiallyAttachedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartAxiallyAttachedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]
    in  makeStream req

getPartAxiallyAttachedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartAxiallyAttachedStream thisArg = requestStream $ getPartAxiallyAttachedStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.CargoBay" /> if the part is a cargo bay, otherwisenull.
 -}
getPartCargoBay :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBay thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCargoBayStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBayStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]
    in  makeStream req

getPartCargoBayStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.CargoBay))
getPartCargoBayStream thisArg = requestStream $ getPartCargoBayStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this part, and centered on its center of mass.
 - <list type="bullet">The origin is at the center of mass of the part, as returned by <see cref="M:SpaceCenter.Part.CenterOfMass" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
 - for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartCenterOfMassReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCenterOfMassReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getPartCenterOfMassReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartCenterOfMassReferenceFrameStream thisArg = requestStream $ getPartCenterOfMassReferenceFrameStreamReq thisArg 

{-
 - The parts children. Returns an empty list if the part has no children.
 - This, in combination with <see cref="M:SpaceCenter.Part.Parent" />, can be used to traverse the vessels parts tree.
 -}
getPartChildren :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ([KRPCHS.SpaceCenter.Part])
getPartChildren thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartChildrenStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartChildrenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]
    in  makeStream req

getPartChildrenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartChildrenStream thisArg = requestStream $ getPartChildrenStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ControlSurface" /> if the part is an aerodynamic control surface, otherwisenull.
 -}
getPartControlSurface :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurface thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartControlSurfaceStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurfaceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]
    in  makeStream req

getPartControlSurfaceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.ControlSurface))
getPartControlSurfaceStream thisArg = requestStream $ getPartControlSurfaceStreamReq thisArg 

{-
 - The cost of the part, in units of funds.
 -}
getPartCost :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartCost thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCostStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartCostStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]
    in  makeStream req

getPartCostStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartCostStream thisArg = requestStream $ getPartCostStreamReq thisArg 

{-
 - Whether this part is crossfeed capable.
 -}
getPartCrossfeed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartCrossfeed thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCrossfeedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartCrossfeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]
    in  makeStream req

getPartCrossfeedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartCrossfeedStream thisArg = requestStream $ getPartCrossfeedStreamReq thisArg 

{-
 - The stage in which this part will be decoupled. Returns -1 if the part is never decoupled from the vessel.
 -}
getPartDecoupleStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Data.Int.Int32)
getPartDecoupleStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Int.Int32)
getPartDecoupleStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]
    in  makeStream req

getPartDecoupleStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Data.Int.Int32))
getPartDecoupleStageStream thisArg = requestStream $ getPartDecoupleStageStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Decoupler" /> if the part is a decoupler, otherwisenull.
 -}
getPartDecoupler :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Decoupler)
getPartDecoupler thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDecouplerStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Decoupler)
getPartDecouplerStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]
    in  makeStream req

getPartDecouplerStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Decoupler))
getPartDecouplerStream thisArg = requestStream $ getPartDecouplerStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.DockingPort" /> if the part is a docking port, otherwisenull.
 -}
getPartDockingPort :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPort thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDockingPortStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPortStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]
    in  makeStream req

getPartDockingPortStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getPartDockingPortStream thisArg = requestStream $ getPartDockingPortStreamReq thisArg 

{-
 - The mass of the part, not including any resources it contains, in kilograms. Returns zero if the part is massless.
 -}
getPartDryMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDryMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartDryMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartDryMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartDryMassStream thisArg = requestStream $ getPartDryMassStreamReq thisArg 

{-
 - The dynamic pressure acting on the part, in Pascals.
 -}
getPartDynamicPressure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDynamicPressureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartDynamicPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]
    in  makeStream req

getPartDynamicPressureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartDynamicPressureStream thisArg = requestStream $ getPartDynamicPressureStreamReq thisArg 

{-
 - An <see cref="T:SpaceCenter.Engine" /> if the part is an engine, otherwisenull.
 -}
getPartEngine :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Engine)
getPartEngine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartEngineStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Engine)
getPartEngineStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]
    in  makeStream req

getPartEngineStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Engine))
getPartEngineStream thisArg = requestStream $ getPartEngineStreamReq thisArg 

{-
 - An <see cref="T:SpaceCenter.Experiment" /> if the part is a science experiment, otherwisenull.
 -}
getPartExperiment :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Experiment)
getPartExperiment thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Experiment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartExperimentStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Experiment)
getPartExperimentStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Experiment" [makeArgument 0 thisArg]
    in  makeStream req

getPartExperimentStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Experiment))
getPartExperimentStream thisArg = requestStream $ getPartExperimentStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Fairing" /> if the part is a fairing, otherwisenull.
 -}
getPartFairing :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Fairing)
getPartFairing thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartFairingStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Fairing)
getPartFairingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]
    in  makeStream req

getPartFairingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Fairing))
getPartFairingStream thisArg = requestStream $ getPartFairingStreamReq thisArg 

{-
 - The parts that are connected to this part via fuel lines, where the direction of the fuel line is into this part.
 -}
getPartFuelLinesFrom :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFrom thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartFuelLinesFromStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFromStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]
    in  makeStream req

getPartFuelLinesFromStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesFromStream thisArg = requestStream $ getPartFuelLinesFromStreamReq thisArg 

{-
 - The parts that are connected to this part via fuel lines, where the direction of the fuel line is out of this part.
 -}
getPartFuelLinesTo :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesTo thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartFuelLinesToStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesToStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]
    in  makeStream req

getPartFuelLinesToStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesToStream thisArg = requestStream $ getPartFuelLinesToStreamReq thisArg 

{-
 - The color used to highlight the part.
 -}
getPartHighlightColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ((Double, Double, Double))
getPartHighlightColor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_HighlightColor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartHighlightColorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ((Double, Double, Double))
getPartHighlightColorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_HighlightColor" [makeArgument 0 thisArg]
    in  makeStream req

getPartHighlightColorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ((Double, Double, Double)))
getPartHighlightColorStream thisArg = requestStream $ getPartHighlightColorStreamReq thisArg 

{-
 - Whether the part is highlighted.
 -}
getPartHighlighted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartHighlighted thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Highlighted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartHighlightedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartHighlightedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Highlighted" [makeArgument 0 thisArg]
    in  makeStream req

getPartHighlightedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartHighlightedStream thisArg = requestStream $ getPartHighlightedStreamReq thisArg 

{-
 - The impact tolerance of the part, in meters per second.
 -}
getPartImpactTolerance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartImpactTolerance thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartImpactToleranceStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartImpactToleranceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]
    in  makeStream req

getPartImpactToleranceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartImpactToleranceStream thisArg = requestStream $ getPartImpactToleranceStreamReq thisArg 

{-
 - The inertia tensor of the part in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getPartInertiaTensor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ([Double])
getPartInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartInertiaTensorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([Double])
getPartInertiaTensorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]
    in  makeStream req

getPartInertiaTensorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ([Double]))
getPartInertiaTensorStream thisArg = requestStream $ getPartInertiaTensorStreamReq thisArg 

{-
 - An <see cref="T:SpaceCenter.Intake" /> if the part is an intake, otherwisenull.This includes any part that generates thrust. This covers many different types of engine,
 - including liquid fuel rockets, solid rocket boosters and jet engines.
 - For RCS thrusters see <see cref="T:SpaceCenter.RCS" />.
 -}
getPartIntake :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Intake)
getPartIntake thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartIntakeStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Intake)
getPartIntakeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]
    in  makeStream req

getPartIntakeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Intake))
getPartIntakeStream thisArg = requestStream $ getPartIntakeStreamReq thisArg 

{-
 - Whether this part is a fuel line.
 -}
getPartIsFuelLine :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartIsFuelLine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartIsFuelLineStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartIsFuelLineStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]
    in  makeStream req

getPartIsFuelLineStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartIsFuelLineStream thisArg = requestStream $ getPartIsFuelLineStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.LaunchClamp" /> if the part is a launch clamp, otherwisenull.
 -}
getPartLaunchClamp :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClamp thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLaunchClampStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClampStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]
    in  makeStream req

getPartLaunchClampStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.LaunchClamp))
getPartLaunchClampStream thisArg = requestStream $ getPartLaunchClampStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Leg" /> if the part is a landing leg, otherwisenull.
 -}
getPartLeg :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Leg)
getPartLeg thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Leg" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLegStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Leg)
getPartLegStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Leg" [makeArgument 0 thisArg]
    in  makeStream req

getPartLegStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Leg))
getPartLegStream thisArg = requestStream $ getPartLegStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Light" /> if the part is a light, otherwisenull.
 -}
getPartLight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Light)
getPartLight thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLightStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Light)
getPartLightStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]
    in  makeStream req

getPartLightStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Light))
getPartLightStream thisArg = requestStream $ getPartLightStreamReq thisArg 

{-
 - The current mass of the part, including resources it contains, in kilograms.
 - Returns zero if the part is massless.
 -}
getPartMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]
    in  makeStream req

getPartMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartMassStream thisArg = requestStream $ getPartMassStreamReq thisArg 

{-
 - Whether the part is <a href="http://wiki.kerbalspaceprogram.com/wiki/Massless_part">massless.
 -}
getPartMassless :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartMassless thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMasslessStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartMasslessStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]
    in  makeStream req

getPartMasslessStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartMasslessStream thisArg = requestStream $ getPartMasslessStreamReq thisArg 

{-
 - Maximum temperature that the skin of the part can survive, in Kelvin.
 -}
getPartMaxSkinTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartMaxSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMaxSkinTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMaxSkinTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartMaxSkinTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartMaxSkinTemperatureStream thisArg = requestStream $ getPartMaxSkinTemperatureStreamReq thisArg 

{-
 - Maximum temperature that the part can survive, in Kelvin.
 -}
getPartMaxTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartMaxTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMaxTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMaxTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartMaxTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartMaxTemperatureStream thisArg = requestStream $ getPartMaxTemperatureStreamReq thisArg 

{-
 - The modules for this part.
 -}
getPartModules :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ([KRPCHS.SpaceCenter.Module])
getPartModules thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartModulesStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Module])
getPartModulesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]
    in  makeStream req

getPartModulesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ([KRPCHS.SpaceCenter.Module]))
getPartModulesStream thisArg = requestStream $ getPartModulesStreamReq thisArg 

{-
 - The moment of inertia of the part inkg.m^2around its center of mass
 - in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 -}
getPartMomentOfInertia :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m ((Double, Double, Double))
getPartMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMomentOfInertiaStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ((Double, Double, Double))
getPartMomentOfInertiaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]
    in  makeStream req

getPartMomentOfInertiaStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream ((Double, Double, Double)))
getPartMomentOfInertiaStream thisArg = requestStream $ getPartMomentOfInertiaStreamReq thisArg 

{-
 - Internal name of the part, as used in
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation">part cfg files.
 - For example "Mark1-2Pod".
 -}
getPartName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Data.Text.Text)
getPartName thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartNameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getPartNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Data.Text.Text))
getPartNameStream thisArg = requestStream $ getPartNameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Parachute" /> if the part is a parachute, otherwisenull.
 -}
getPartParachute :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Parachute)
getPartParachute thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartParachuteStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Parachute)
getPartParachuteStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]
    in  makeStream req

getPartParachuteStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Parachute))
getPartParachuteStream thisArg = requestStream $ getPartParachuteStreamReq thisArg 

{-
 - The parts parent. Returnsnullif the part does not have a parent.
 - This, in combination with <see cref="M:SpaceCenter.Part.Children" />, can be used to traverse the vessels parts tree.
 -}
getPartParent :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Part)
getPartParent thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartParentStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartParentStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]
    in  makeStream req

getPartParentStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartParentStream thisArg = requestStream $ getPartParentStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.RCS" /> if the part is an RCS block/thruster, otherwisenull.
 -}
getPartRCS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.RCS)
getPartRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartRCSStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.RCS)
getPartRCSStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]
    in  makeStream req

getPartRCSStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.RCS))
getPartRCSStream thisArg = requestStream $ getPartRCSStreamReq thisArg 

{-
 - Whether the part is radially attached to its parent, i.e. on the side of its parent.
 - If the part has no parent, returnsfalse.
 -}
getPartRadiallyAttached :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartRadiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartRadiallyAttachedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartRadiallyAttachedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]
    in  makeStream req

getPartRadiallyAttachedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartRadiallyAttachedStream thisArg = requestStream $ getPartRadiallyAttachedStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Radiator" /> if the part is a radiator, otherwisenull.
 -}
getPartRadiator :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Radiator)
getPartRadiator thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartRadiatorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Radiator)
getPartRadiatorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]
    in  makeStream req

getPartRadiatorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Radiator))
getPartRadiatorStream thisArg = requestStream $ getPartRadiatorStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ReactionWheel" /> if the part is a reaction wheel, otherwisenull.
 -}
getPartReactionWheel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartReactionWheelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]
    in  makeStream req

getPartReactionWheelStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.ReactionWheel))
getPartReactionWheelStream thisArg = requestStream $ getPartReactionWheelStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this part, and centered on a fixed position within the part, defined by the parts model.
 - <list type="bullet">The origin is at the position of the part, as returned by <see cref="M:SpaceCenter.Part.Position" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
 - for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getPartReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartReferenceFrameStream thisArg = requestStream $ getPartReferenceFrameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ResourceConverter" /> if the part is a resource converter, otherwisenull.
 -}
getPartResourceConverter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverter thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartResourceConverterStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]
    in  makeStream req

getPartResourceConverterStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceConverter))
getPartResourceConverterStream thisArg = requestStream $ getPartResourceConverterStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ResourceHarvester" /> if the part is a resource harvester, otherwisenull.
 -}
getPartResourceHarvester :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvester thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartResourceHarvesterStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvesterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]
    in  makeStream req

getPartResourceHarvesterStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvester))
getPartResourceHarvesterStream thisArg = requestStream $ getPartResourceHarvesterStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Resources" /> object for the part.
 -}
getPartResources :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Resources)
getPartResources thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartResourcesStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
getPartResourcesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]
    in  makeStream req

getPartResourcesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Resources))
getPartResourcesStream thisArg = requestStream $ getPartResourcesStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Sensor" /> if the part is a sensor, otherwisenull.
 -}
getPartSensor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Sensor)
getPartSensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartSensorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Sensor)
getPartSensorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]
    in  makeStream req

getPartSensorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Sensor))
getPartSensorStream thisArg = requestStream $ getPartSensorStreamReq thisArg 

{-
 - Whether the part is shielded from the exterior of the vessel, for example by a fairing.
 -}
getPartShielded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Bool)
getPartShielded thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartShieldedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartShieldedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]
    in  makeStream req

getPartShieldedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Bool))
getPartShieldedStream thisArg = requestStream $ getPartShieldedStreamReq thisArg 

{-
 - Temperature of the skin of the part, in Kelvin.
 -}
getPartSkinTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartSkinTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartSkinTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartSkinTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartSkinTemperatureStream thisArg = requestStream $ getPartSkinTemperatureStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.SolarPanel" /> if the part is a solar panel, otherwisenull.
 -}
getPartSolarPanel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartSolarPanelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]
    in  makeStream req

getPartSolarPanelStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.SolarPanel))
getPartSolarPanelStream thisArg = requestStream $ getPartSolarPanelStreamReq thisArg 

{-
 - The stage in which this part will be activated. Returns -1 if the part is not activated by staging.
 -}
getPartStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Data.Int.Int32)
getPartStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartStageStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Int.Int32)
getPartStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]
    in  makeStream req

getPartStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Data.Int.Int32))
getPartStageStream thisArg = requestStream $ getPartStageStreamReq thisArg 

{-
 - The name tag for the part. Can be set to a custom string using the in-game user interface.This requires either the <a href="http://github.com/krpc/NameTag/releases/latest">NameTagor
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/61827-/">kOSmods to be installed.
 -}
getPartTag :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Data.Text.Text)
getPartTag thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Tag" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartTagStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartTagStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Tag" [makeArgument 0 thisArg]
    in  makeStream req

getPartTagStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Data.Text.Text))
getPartTagStream thisArg = requestStream $ getPartTagStreamReq thisArg 

{-
 - Temperature of the part, in Kelvin.
 -}
getPartTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Double)
getPartTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Double))
getPartTemperatureStream thisArg = requestStream $ getPartTemperatureStreamReq thisArg 

{-
 - The rate at which heat energy is conducting into or out of the part via contact with other parts.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConductionFlux :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalConductionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalConductionFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalConductionFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalConductionFluxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalConductionFluxStream thisArg = requestStream $ getPartThermalConductionFluxStreamReq thisArg 

{-
 - The rate at which heat energy is convecting into or out of the part from the surrounding atmosphere.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConvectionFlux :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalConvectionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalConvectionFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalConvectionFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalConvectionFluxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalConvectionFluxStream thisArg = requestStream $ getPartThermalConvectionFluxStreamReq thisArg 

{-
 - The rate at which heat energy is begin generated by the part.
 - For example, some engines generate heat by combusting fuel.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalInternalFlux :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalInternalFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalInternalFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalInternalFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalInternalFluxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalInternalFluxStream thisArg = requestStream $ getPartThermalInternalFluxStreamReq thisArg 

{-
 - A measure of how much energy it takes to increase the internal temperature of the part, in Joules per Kelvin.
 -}
getPartThermalMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalMassStream thisArg = requestStream $ getPartThermalMassStreamReq thisArg 

{-
 - The rate at which heat energy is radiating into or out of the part from the surrounding environment.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalRadiationFlux :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalRadiationFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalRadiationFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalRadiationFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalRadiationFluxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalRadiationFluxStream thisArg = requestStream $ getPartThermalRadiationFluxStreamReq thisArg 

{-
 - A measure of how much energy it takes to increase the temperature of the resources contained in the part, in Joules per Kelvin.
 -}
getPartThermalResourceMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalResourceMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalResourceMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalResourceMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalResourceMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalResourceMassStream thisArg = requestStream $ getPartThermalResourceMassStreamReq thisArg 

{-
 - A measure of how much energy it takes to increase the skin temperature of the part, in Joules per Kelvin.
 -}
getPartThermalSkinMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalSkinMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalSkinMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalSkinMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalSkinMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalSkinMassStream thisArg = requestStream $ getPartThermalSkinMassStreamReq thisArg 

{-
 - The rate at which heat energy is transferring between the part's skin and its internals.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part's internals are gaining heat energy,
 - and negative means its skin is gaining heat energy.
 -}
getPartThermalSkinToInternalFlux :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Float)
getPartThermalSkinToInternalFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalSkinToInternalFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalSkinToInternalFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalSkinToInternalFluxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Float))
getPartThermalSkinToInternalFluxStream thisArg = requestStream $ getPartThermalSkinToInternalFluxStreamReq thisArg 

{-
 - Title of the part, as shown when the part is right clicked in-game. For example "Mk1-2 Command Pod".
 -}
getPartTitle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (Data.Text.Text)
getPartTitle thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartTitleStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartTitleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]
    in  makeStream req

getPartTitleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (Data.Text.Text))
getPartTitleStream thisArg = requestStream $ getPartTitleStreamReq thisArg 

{-
 - The vessel that contains this part.
 -}
getPartVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Vessel)
getPartVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartVesselStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getPartVesselStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]
    in  makeStream req

getPartVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getPartVesselStream thisArg = requestStream $ getPartVesselStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Wheel" /> if the part is a wheel, otherwisenull.
 -}
getPartWheel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCHS.SpaceCenter.Wheel)
getPartWheel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Wheel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartWheelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Wheel)
getPartWheelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Wheel" [makeArgument 0 thisArg]
    in  makeStream req

getPartWheelStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> m (KRPCStream (KRPCHS.SpaceCenter.Wheel))
getPartWheelStream thisArg = requestStream $ getPartWheelStreamReq thisArg 

{-
 - The color used to highlight the part.
 -}
setPartHighlightColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> m ()
setPartHighlightColor thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Part_set_HighlightColor" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the part is highlighted.
 -}
setPartHighlighted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> Bool -> m ()
setPartHighlighted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Part_set_Highlighted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The name tag for the part. Can be set to a custom string using the in-game user interface.This requires either the <a href="http://github.com/krpc/NameTag/releases/latest">NameTagor
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/61827-/">kOSmods to be installed.
 -}
setPartTag :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> Data.Text.Text -> m ()
setPartTag thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Part_set_Tag" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - A list of all parts that are decoupled in the given <paramref name="stage" />.<param name="stage">
 -}
partsInDecoupleStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> m ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    res <- sendRequest r
    processResponse res

partsInDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStageStreamReq thisArg stageArg =
    let req = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    in  makeStream req

partsInDecoupleStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInDecoupleStageStream thisArg stageArg = requestStream $ partsInDecoupleStageStreamReq thisArg stageArg 

{-
 - A list of all parts that are activated in the given <paramref name="stage" />.<param name="stage">
 -}
partsInStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> m ([KRPCHS.SpaceCenter.Part])
partsInStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    res <- sendRequest r
    processResponse res

partsInStageStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsInStageStreamReq thisArg stageArg =
    let req = makeRequest "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    in  makeStream req

partsInStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInStageStream thisArg stageArg = requestStream $ partsInStageStreamReq thisArg stageArg 

{-
 - A list of modules (combined across all parts in the vessel) whose
 - <see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsModulesWithName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m ([KRPCHS.SpaceCenter.Module])
partsModulesWithName thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    res <- sendRequest r
    processResponse res

partsModulesWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Module])
partsModulesWithNameStreamReq thisArg moduleNameArg =
    let req = makeRequest "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    in  makeStream req

partsModulesWithNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m (KRPCStream ([KRPCHS.SpaceCenter.Module]))
partsModulesWithNameStream thisArg moduleNameArg = requestStream $ partsModulesWithNameStreamReq thisArg moduleNameArg 

{-
 - A list of all parts that contain a <see cref="T:SpaceCenter.Module" /> whose
 - <see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsWithModule :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m ([KRPCHS.SpaceCenter.Part])
partsWithModule thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    res <- sendRequest r
    processResponse res

partsWithModuleStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithModuleStreamReq thisArg moduleNameArg =
    let req = makeRequest "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    in  makeStream req

partsWithModuleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithModuleStream thisArg moduleNameArg = requestStream $ partsWithModuleStreamReq thisArg moduleNameArg 

{-
 - A list of parts whose <see cref="M:SpaceCenter.Part.Name" /> is <paramref name="name" />.<param name="name">
 -}
partsWithName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m ([KRPCHS.SpaceCenter.Part])
partsWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

partsWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithNameStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

partsWithNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithNameStream thisArg nameArg = requestStream $ partsWithNameStreamReq thisArg nameArg 

{-
 - A list of all parts whose <see cref="M:SpaceCenter.Part.Tag" /> is <paramref name="tag" />.<param name="tag">
 -}
partsWithTag :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m ([KRPCHS.SpaceCenter.Part])
partsWithTag thisArg tagArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTag" [makeArgument 0 thisArg, makeArgument 1 tagArg]
    res <- sendRequest r
    processResponse res

partsWithTagStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithTagStreamReq thisArg tagArg =
    let req = makeRequest "SpaceCenter" "Parts_WithTag" [makeArgument 0 thisArg, makeArgument 1 tagArg]
    in  makeStream req

partsWithTagStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithTagStream thisArg tagArg = requestStream $ partsWithTagStreamReq thisArg tagArg 

{-
 - A list of all parts whose <see cref="M:SpaceCenter.Part.Title" /> is <paramref name="title" />.<param name="title">
 -}
partsWithTitle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m ([KRPCHS.SpaceCenter.Part])
partsWithTitle thisArg titleArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]
    res <- sendRequest r
    processResponse res

partsWithTitleStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithTitleStreamReq thisArg titleArg =
    let req = makeRequest "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]
    in  makeStream req

partsWithTitleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithTitleStream thisArg titleArg = requestStream $ partsWithTitleStreamReq thisArg titleArg 

{-
 - A list of all of the vessels parts.
 -}
getPartsAll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Part])
getPartsAll thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsAllStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartsAllStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]
    in  makeStream req

getPartsAllStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartsAllStream thisArg = requestStream $ getPartsAllStreamReq thisArg 

{-
 - A list of all antennas in the vessel.
 -}
getPartsAntennas :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Antenna])
getPartsAntennas thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Antennas" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsAntennasStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Antenna])
getPartsAntennasStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Antennas" [makeArgument 0 thisArg]
    in  makeStream req

getPartsAntennasStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Antenna]))
getPartsAntennasStream thisArg = requestStream $ getPartsAntennasStreamReq thisArg 

{-
 - A list of all cargo bays in the vessel.
 -}
getPartsCargoBays :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBays thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsCargoBaysStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBaysStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]
    in  makeStream req

getPartsCargoBaysStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.CargoBay]))
getPartsCargoBaysStream thisArg = requestStream $ getPartsCargoBaysStreamReq thisArg 

{-
 - A list of all control surfaces in the vessel.
 -}
getPartsControlSurfaces :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfaces thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsControlSurfacesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfacesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]
    in  makeStream req

getPartsControlSurfacesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.ControlSurface]))
getPartsControlSurfacesStream thisArg = requestStream $ getPartsControlSurfacesStreamReq thisArg 

{-
 - The part from which the vessel is controlled.
 -}
getPartsControlling :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCHS.SpaceCenter.Part)
getPartsControlling thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsControllingStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartsControllingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]
    in  makeStream req

getPartsControllingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsControllingStream thisArg = requestStream $ getPartsControllingStreamReq thisArg 

{-
 - A list of all decouplers in the vessel.
 -}
getPartsDecouplers :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplers thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsDecouplersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]
    in  makeStream req

getPartsDecouplersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Decoupler]))
getPartsDecouplersStream thisArg = requestStream $ getPartsDecouplersStreamReq thisArg 

{-
 - A list of all docking ports in the vessel.
 -}
getPartsDockingPorts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPorts thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsDockingPortsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPortsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]
    in  makeStream req

getPartsDockingPortsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.DockingPort]))
getPartsDockingPortsStream thisArg = requestStream $ getPartsDockingPortsStreamReq thisArg 

{-
 - A list of all engines in the vessel.This includes any part that generates thrust. This covers many different types of engine,
 - including liquid fuel rockets, solid rocket boosters, jet engines and RCS thrusters.
 -}
getPartsEngines :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Engine])
getPartsEngines thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsEnginesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Engine])
getPartsEnginesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]
    in  makeStream req

getPartsEnginesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Engine]))
getPartsEnginesStream thisArg = requestStream $ getPartsEnginesStreamReq thisArg 

{-
 - A list of all science experiments in the vessel.
 -}
getPartsExperiments :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Experiment])
getPartsExperiments thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Experiments" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsExperimentsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Experiment])
getPartsExperimentsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Experiments" [makeArgument 0 thisArg]
    in  makeStream req

getPartsExperimentsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Experiment]))
getPartsExperimentsStream thisArg = requestStream $ getPartsExperimentsStreamReq thisArg 

{-
 - A list of all fairings in the vessel.
 -}
getPartsFairings :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Fairing])
getPartsFairings thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsFairingsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Fairing])
getPartsFairingsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]
    in  makeStream req

getPartsFairingsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Fairing]))
getPartsFairingsStream thisArg = requestStream $ getPartsFairingsStreamReq thisArg 

{-
 - A list of all intakes in the vessel.
 -}
getPartsIntakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Intake])
getPartsIntakes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsIntakesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Intake])
getPartsIntakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]
    in  makeStream req

getPartsIntakesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Intake]))
getPartsIntakesStream thisArg = requestStream $ getPartsIntakesStreamReq thisArg 

{-
 - A list of all launch clamps attached to the vessel.
 -}
getPartsLaunchClamps :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClamps thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLaunchClampsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClampsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLaunchClampsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.LaunchClamp]))
getPartsLaunchClampsStream thisArg = requestStream $ getPartsLaunchClampsStreamReq thisArg 

{-
 - A list of all landing legs attached to the vessel.
 -}
getPartsLegs :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Leg])
getPartsLegs thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Legs" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLegsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Leg])
getPartsLegsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Legs" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLegsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Leg]))
getPartsLegsStream thisArg = requestStream $ getPartsLegsStreamReq thisArg 

{-
 - A list of all lights in the vessel.
 -}
getPartsLights :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Light])
getPartsLights thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLightsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Light])
getPartsLightsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLightsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Light]))
getPartsLightsStream thisArg = requestStream $ getPartsLightsStreamReq thisArg 

{-
 - A list of all parachutes in the vessel.
 -}
getPartsParachutes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsParachutesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]
    in  makeStream req

getPartsParachutesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Parachute]))
getPartsParachutesStream thisArg = requestStream $ getPartsParachutesStreamReq thisArg 

{-
 - A list of all RCS blocks/thrusters in the vessel.
 -}
getPartsRCS :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.RCS])
getPartsRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsRCSStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.RCS])
getPartsRCSStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]
    in  makeStream req

getPartsRCSStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.RCS]))
getPartsRCSStream thisArg = requestStream $ getPartsRCSStreamReq thisArg 

{-
 - A list of all radiators in the vessel.
 -}
getPartsRadiators :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiators thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsRadiatorsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiatorsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]
    in  makeStream req

getPartsRadiatorsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Radiator]))
getPartsRadiatorsStream thisArg = requestStream $ getPartsRadiatorsStreamReq thisArg 

{-
 - A list of all reaction wheels in the vessel.
 -}
getPartsReactionWheels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsReactionWheelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]
    in  makeStream req

getPartsReactionWheelsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.ReactionWheel]))
getPartsReactionWheelsStream thisArg = requestStream $ getPartsReactionWheelsStreamReq thisArg 

{-
 - A list of all resource converters in the vessel.
 -}
getPartsResourceConverters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConverters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsResourceConvertersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConvertersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]
    in  makeStream req

getPartsResourceConvertersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.ResourceConverter]))
getPartsResourceConvertersStream thisArg = requestStream $ getPartsResourceConvertersStreamReq thisArg 

{-
 - A list of all resource harvesters in the vessel.
 -}
getPartsResourceHarvesters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvesters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsResourceHarvestersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvestersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]
    in  makeStream req

getPartsResourceHarvestersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.ResourceHarvester]))
getPartsResourceHarvestersStream thisArg = requestStream $ getPartsResourceHarvestersStreamReq thisArg 

{-
 - The vessels root part.
 -}
getPartsRoot :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCHS.SpaceCenter.Part)
getPartsRoot thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsRootStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartsRootStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]
    in  makeStream req

getPartsRootStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsRootStream thisArg = requestStream $ getPartsRootStreamReq thisArg 

{-
 - A list of all sensors in the vessel.
 -}
getPartsSensors :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Sensor])
getPartsSensors thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsSensorsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Sensor])
getPartsSensorsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]
    in  makeStream req

getPartsSensorsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Sensor]))
getPartsSensorsStream thisArg = requestStream $ getPartsSensorsStreamReq thisArg 

{-
 - A list of all solar panels in the vessel.
 -}
getPartsSolarPanels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsSolarPanelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]
    in  makeStream req

getPartsSolarPanelsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.SolarPanel]))
getPartsSolarPanelsStream thisArg = requestStream $ getPartsSolarPanelsStreamReq thisArg 

{-
 - A list of all wheels in the vessel.
 -}
getPartsWheels :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m ([KRPCHS.SpaceCenter.Wheel])
getPartsWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Wheels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsWheelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Wheel])
getPartsWheelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Wheels" [makeArgument 0 thisArg]
    in  makeStream req

getPartsWheelsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> m (KRPCStream ([KRPCHS.SpaceCenter.Wheel]))
getPartsWheelsStream thisArg = requestStream $ getPartsWheelsStreamReq thisArg 

{-
 - The part from which the vessel is controlled.
 -}
setPartsControlling :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Parts -> KRPCHS.SpaceCenter.Part -> m ()
setPartsControlling thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parts_set_Controlling" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current amount of propellant.
 -}
getPropellantCurrentAmount :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Double)
getPropellantCurrentAmount thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_CurrentAmount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantCurrentAmountStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantCurrentAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_CurrentAmount" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantCurrentAmountStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Double))
getPropellantCurrentAmountStream thisArg = requestStream $ getPropellantCurrentAmountStreamReq thisArg 

{-
 - The required amount of propellant.
 -}
getPropellantCurrentRequirement :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Double)
getPropellantCurrentRequirement thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_CurrentRequirement" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantCurrentRequirementStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantCurrentRequirementStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_CurrentRequirement" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantCurrentRequirementStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Double))
getPropellantCurrentRequirementStream thisArg = requestStream $ getPropellantCurrentRequirementStreamReq thisArg 

{-
 - If this propellant has a stack gauge or not.
 -}
getPropellantDrawStackGauge :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Bool)
getPropellantDrawStackGauge thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_DrawStackGauge" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantDrawStackGaugeStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantDrawStackGaugeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_DrawStackGauge" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantDrawStackGaugeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Bool))
getPropellantDrawStackGaugeStream thisArg = requestStream $ getPropellantDrawStackGaugeStreamReq thisArg 

{-
 - If this propellant should be ignored when calculating required mass flow given specific impulse.
 -}
getPropellantIgnoreForIsp :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Bool)
getPropellantIgnoreForIsp thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_IgnoreForIsp" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantIgnoreForIspStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIgnoreForIspStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_IgnoreForIsp" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantIgnoreForIspStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Bool))
getPropellantIgnoreForIspStream thisArg = requestStream $ getPropellantIgnoreForIspStreamReq thisArg 

{-
 - If this propellant should be ignored for thrust curve calculations.
 -}
getPropellantIgnoreForThrustCurve :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Bool)
getPropellantIgnoreForThrustCurve thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_IgnoreForThrustCurve" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantIgnoreForThrustCurveStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIgnoreForThrustCurveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_IgnoreForThrustCurve" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantIgnoreForThrustCurveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Bool))
getPropellantIgnoreForThrustCurveStream thisArg = requestStream $ getPropellantIgnoreForThrustCurveStreamReq thisArg 

{-
 - If this propellant is deprived.
 -}
getPropellantIsDeprived :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Bool)
getPropellantIsDeprived thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_IsDeprived" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantIsDeprivedStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIsDeprivedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_IsDeprived" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantIsDeprivedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Bool))
getPropellantIsDeprivedStream thisArg = requestStream $ getPropellantIsDeprivedStreamReq thisArg 

{-
 - The name of the propellant.
 -}
getPropellantName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Data.Text.Text)
getPropellantName thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantNameStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Data.Text.Text)
getPropellantNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Data.Text.Text))
getPropellantNameStream thisArg = requestStream $ getPropellantNameStreamReq thisArg 

{-
 - The propellant ratio.
 -}
getPropellantRatio :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Float)
getPropellantRatio thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_Ratio" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantRatioStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Float)
getPropellantRatioStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_Ratio" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantRatioStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Float))
getPropellantRatioStream thisArg = requestStream $ getPropellantRatioStreamReq thisArg 

{-
 - The total amount of the underlying resource currently reachable given resource flow rules.
 -}
getPropellantTotalResourceAvailable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Double)
getPropellantTotalResourceAvailable thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_TotalResourceAvailable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantTotalResourceAvailableStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantTotalResourceAvailableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_TotalResourceAvailable" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantTotalResourceAvailableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Double))
getPropellantTotalResourceAvailableStream thisArg = requestStream $ getPropellantTotalResourceAvailableStreamReq thisArg 

{-
 - The total vehicle capacity for the underlying propellant resource, restricted by resource flow rules.
 -}
getPropellantTotalResourceCapacity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (Double)
getPropellantTotalResourceCapacity thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_TotalResourceCapacity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantTotalResourceCapacityStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantTotalResourceCapacityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_TotalResourceCapacity" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantTotalResourceCapacityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Propellant -> m (KRPCStream (Double))
getPropellantTotalResourceCapacityStream thisArg = requestStream $ getPropellantTotalResourceCapacityStreamReq thisArg 

{-
 - Load a quicksave.This is the same as calling <see cref="M:SpaceCenter.Load" /> with the name "quicksave".
 -}
quickload :: (MonadIO m, MonadThrow m, MonadRPC m) => m ()
quickload  = do
    let r = makeRequest "SpaceCenter" "Quickload" []
    res <- sendRequest r
    processResponse res 

{-
 - Save a quicksave.This is the same as calling <see cref="M:SpaceCenter.Save" /> with the name "quicksave".
 -}
quicksave :: (MonadIO m, MonadThrow m, MonadRPC m) => m ()
quicksave  = do
    let r = makeRequest "SpaceCenter" "Quicksave" []
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thrusters are active.
 - An RCS thruster is inactive if the RCS action group is disabled (<see cref="M:SpaceCenter.Control.RCS" />),
 - the RCS thruster itself is not enabled (<see cref="M:SpaceCenter.RCS.Enabled" />) or
 - it is covered by a fairing (<see cref="M:SpaceCenter.Part.Shielded" />).
 -}
getRCSActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSActive thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSActiveStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getRCSActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSActiveStream thisArg = requestStream $ getRCSActiveStreamReq thisArg 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the RCS is inactive.
 -}
getRCSAvailableTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (((Double, Double, Double), (Double, Double, Double)))
getRCSAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getRCSAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getRCSAvailableTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getRCSAvailableTorqueStream thisArg = requestStream $ getRCSAvailableTorqueStreamReq thisArg 

{-
 - Whether the RCS thrusters are enabled.
 -}
getRCSEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSEnabledStream thisArg = requestStream $ getRCSEnabledStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSForwardEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSForwardEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSForwardEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSForwardEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSForwardEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSForwardEnabledStream thisArg = requestStream $ getRCSForwardEnabledStreamReq thisArg 

{-
 - Whether the RCS has fuel available.The RCS thruster must be activated for this property to update correctly.
 -}
getRCSHasFuel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSHasFuelStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSHasFuelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]
    in  makeStream req

getRCSHasFuelStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSHasFuelStream thisArg = requestStream $ getRCSHasFuelStreamReq thisArg 

{-
 - The specific impulse of the RCS at sea level on Kerbin, in seconds.
 -}
getRCSKerbinSeaLevelSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Float)
getRCSKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSKerbinSeaLevelSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getRCSKerbinSeaLevelSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Float))
getRCSKerbinSeaLevelSpecificImpulseStream thisArg = requestStream $ getRCSKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-
 - The maximum amount of thrust that can be produced by the RCS thrusters when active, in Newtons.
 -}
getRCSMaxThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Float)
getRCSMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSMaxThrustStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSMaxThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]
    in  makeStream req

getRCSMaxThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Float))
getRCSMaxThrustStream thisArg = requestStream $ getRCSMaxThrustStreamReq thisArg 

{-
 - The maximum amount of thrust that can be produced by the RCS thrusters when active in a vacuum, in Newtons.
 -}
getRCSMaxVacuumThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Float)
getRCSMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSMaxVacuumThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    in  makeStream req

getRCSMaxVacuumThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Float))
getRCSMaxVacuumThrustStream thisArg = requestStream $ getRCSMaxVacuumThrustStreamReq thisArg 

{-
 - The part object for this RCS.
 -}
getRCSPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCHS.SpaceCenter.Part)
getRCSPart thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPartStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getRCSPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getRCSPartStream thisArg = requestStream $ getRCSPartStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSPitchEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSPitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPitchEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSPitchEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPitchEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSPitchEnabledStream thisArg = requestStream $ getRCSPitchEnabledStreamReq thisArg 

{-
 - The ratios of resources that the RCS consumes. A dictionary mapping resource names
 - to the ratios at which they are consumed by the RCS.
 -}
getRCSPropellantRatios :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPropellantRatiosStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatiosStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPropellantRatiosStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getRCSPropellantRatiosStream thisArg = requestStream $ getRCSPropellantRatiosStreamReq thisArg 

{-
 - The names of resources that the RCS consumes.
 -}
getRCSPropellants :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m ([Data.Text.Text])
getRCSPropellants thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPropellantsStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ([Data.Text.Text])
getRCSPropellantsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPropellantsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream ([Data.Text.Text]))
getRCSPropellantsStream thisArg = requestStream $ getRCSPropellantsStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRightEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSRightEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSRightEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSRightEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSRightEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSRightEnabledStream thisArg = requestStream $ getRCSRightEnabledStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRollEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSRollEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSRollEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSRollEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSRollEnabledStream thisArg = requestStream $ getRCSRollEnabledStreamReq thisArg 

{-
 - The current specific impulse of the RCS, in seconds. Returns zero
 - if the RCS is not active.
 -}
getRCSSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Float)
getRCSSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getRCSSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Float))
getRCSSpecificImpulseStream thisArg = requestStream $ getRCSSpecificImpulseStreamReq thisArg 

{-
 - A list of thrusters, one of each nozzel in the RCS part.
 -}
getRCSThrusters :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m ([KRPCHS.SpaceCenter.Thruster])
getRCSThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSThrustersStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ([KRPCHS.SpaceCenter.Thruster])
getRCSThrustersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]
    in  makeStream req

getRCSThrustersStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getRCSThrustersStream thisArg = requestStream $ getRCSThrustersStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSUpEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSUpEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSUpEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSUpEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSUpEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSUpEnabledStream thisArg = requestStream $ getRCSUpEnabledStreamReq thisArg 

{-
 - The vacuum specific impulse of the RCS, in seconds.
 -}
getRCSVacuumSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Float)
getRCSVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSVacuumSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getRCSVacuumSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Float))
getRCSVacuumSpecificImpulseStream thisArg = requestStream $ getRCSVacuumSpecificImpulseStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSYawEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (Bool)
getRCSYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSYawEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSYawEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSYawEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> m (KRPCStream (Bool))
getRCSYawEnabledStream thisArg = requestStream $ getRCSYawEnabledStreamReq thisArg 

{-
 - Whether the RCS thrusters are enabled.
 -}
setRCSEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSForwardEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSForwardEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_ForwardEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSPitchEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSPitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRightEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSRightEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RightEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRollEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSUpEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSUpEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_UpEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSYawEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.RCS -> Bool -> m ()
setRCSYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the radiator is deployable.
 -}
getRadiatorDeployable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (Bool)
getRadiatorDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorDeployableStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (Bool)
getRadiatorDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorDeployableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (KRPCStream (Bool))
getRadiatorDeployableStream thisArg = requestStream $ getRadiatorDeployableStreamReq thisArg 

{-
 - For a deployable radiator,trueif the radiator is extended.
 - If the radiator is not deployable, this is alwaystrue.
 -}
getRadiatorDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (Bool)
getRadiatorDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorDeployedStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (Bool)
getRadiatorDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (KRPCStream (Bool))
getRadiatorDeployedStream thisArg = requestStream $ getRadiatorDeployedStreamReq thisArg 

{-
 - The part object for this radiator.
 -}
getRadiatorPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (KRPCHS.SpaceCenter.Part)
getRadiatorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorPartStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getRadiatorPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getRadiatorPartStream thisArg = requestStream $ getRadiatorPartStreamReq thisArg 

{-
 - The current state of the radiator.A fixed radiator is always <see cref="M:SpaceCenter.RadiatorState.Extended" />.
 -}
getRadiatorState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorState thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorStateStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> m (KRPCStream (KRPCHS.SpaceCenter.RadiatorState))
getRadiatorStateStream thisArg = requestStream $ getRadiatorStateStreamReq thisArg 

{-
 - For a deployable radiator,trueif the radiator is extended.
 - If the radiator is not deployable, this is alwaystrue.
 -}
setRadiatorDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Radiator -> Bool -> m ()
setRadiatorDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Radiator_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the reaction wheel is active.
 -}
getReactionWheelActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (Bool)
getReactionWheelActive thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelActiveStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (Bool)
getReactionWheelActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (KRPCStream (Bool))
getReactionWheelActiveStream thisArg = requestStream $ getReactionWheelActiveStreamReq thisArg 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the reaction wheel is inactive or broken.
 -}
getReactionWheelAvailableTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (((Double, Double, Double), (Double, Double, Double)))
getReactionWheelAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getReactionWheelAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelAvailableTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getReactionWheelAvailableTorqueStream thisArg = requestStream $ getReactionWheelAvailableTorqueStreamReq thisArg 

{-
 - Whether the reaction wheel is broken.
 -}
getReactionWheelBroken :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (Bool)
getReactionWheelBroken thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelBrokenStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (Bool)
getReactionWheelBrokenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelBrokenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (KRPCStream (Bool))
getReactionWheelBrokenStream thisArg = requestStream $ getReactionWheelBrokenStreamReq thisArg 

{-
 - The maximum torque the reaction wheel can provide, is it active,
 - in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getReactionWheelMaxTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (((Double, Double, Double), (Double, Double, Double)))
getReactionWheelMaxTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelMaxTorqueStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getReactionWheelMaxTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelMaxTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getReactionWheelMaxTorqueStream thisArg = requestStream $ getReactionWheelMaxTorqueStreamReq thisArg 

{-
 - The part object for this reaction wheel.
 -}
getReactionWheelPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (KRPCHS.SpaceCenter.Part)
getReactionWheelPart thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelPartStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getReactionWheelPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getReactionWheelPartStream thisArg = requestStream $ getReactionWheelPartStreamReq thisArg 

{-
 - Whether the reaction wheel is active.
 -}
setReactionWheelActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReactionWheel -> Bool -> m ()
setReactionWheelActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Create a hybrid reference frame, which is a custom reference frame
 - whose components are inherited from other reference frames.<param name="position">The reference frame providing the position of the origin.<param name="rotation">The reference frame providing the orientation of the frame.<param name="velocity">The reference frame providing the linear velocity of the frame.<param name="angularVelocity">The reference frame providing the angular velocity of the frame.The <paramref name="position" /> is required but all other reference frames are optional.
 - If omitted, they are set to the <paramref name="position" /> reference frame.
 -}
referenceFrameCreateHybrid :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCHS.SpaceCenter.ReferenceFrame)
referenceFrameCreateHybrid positionArg rotationArg velocityArg angularVelocityArg = do
    let r = makeRequest "SpaceCenter" "ReferenceFrame_CreateHybrid" [makeArgument 0 positionArg, makeArgument 1 rotationArg, makeArgument 2 velocityArg, makeArgument 3 angularVelocityArg]
    res <- sendRequest r
    processResponse res

referenceFrameCreateHybridStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
referenceFrameCreateHybridStreamReq positionArg rotationArg velocityArg angularVelocityArg =
    let req = makeRequest "SpaceCenter" "ReferenceFrame_CreateHybrid" [makeArgument 0 positionArg, makeArgument 1 rotationArg, makeArgument 2 velocityArg, makeArgument 3 angularVelocityArg]
    in  makeStream req

referenceFrameCreateHybridStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
referenceFrameCreateHybridStream positionArg rotationArg velocityArg angularVelocityArg = requestStream $ referenceFrameCreateHybridStreamReq positionArg rotationArg velocityArg angularVelocityArg 

{-
 - Create a relative reference frame.<param name="referenceFrame">The parent reference frame.<param name="position">The offset of the position of the origin.<param name="rotation">The rotation to apply to the parent frames rotation, as a quaternion. Defaults to zero.<param name="velocity">The linear velocity to offset the parent frame by. Defaults to zero.<param name="angularVelocity">The angular velocity to offset the parent frame by. Defaults to zero.
 -}
referenceFrameCreateRelative :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> (Double, Double, Double) -> (Double, Double, Double) -> m (KRPCHS.SpaceCenter.ReferenceFrame)
referenceFrameCreateRelative referenceFrameArg positionArg rotationArg velocityArg angularVelocityArg = do
    let r = makeRequest "SpaceCenter" "ReferenceFrame_CreateRelative" [makeArgument 0 referenceFrameArg, makeArgument 1 positionArg, makeArgument 2 rotationArg, makeArgument 3 velocityArg, makeArgument 4 angularVelocityArg]
    res <- sendRequest r
    processResponse res

referenceFrameCreateRelativeStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
referenceFrameCreateRelativeStreamReq referenceFrameArg positionArg rotationArg velocityArg angularVelocityArg =
    let req = makeRequest "SpaceCenter" "ReferenceFrame_CreateRelative" [makeArgument 0 referenceFrameArg, makeArgument 1 positionArg, makeArgument 2 rotationArg, makeArgument 3 velocityArg, makeArgument 4 angularVelocityArg]
    in  makeStream req

referenceFrameCreateRelativeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> (Double, Double, Double) -> (Double, Double, Double) -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
referenceFrameCreateRelativeStream referenceFrameArg positionArg rotationArg velocityArg angularVelocityArg = requestStream $ referenceFrameCreateRelativeStreamReq referenceFrameArg positionArg rotationArg velocityArg angularVelocityArg 

{-
 - True if the specified converter is active.<param name="index">Index of the converter.
 -}
resourceConverterActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (Bool)
resourceConverterActive thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterActiveStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Bool)
resourceConverterActiveStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCStream (Bool))
resourceConverterActiveStream thisArg indexArg = requestStream $ resourceConverterActiveStreamReq thisArg indexArg 

{-
 - List of the names of resources consumed by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterInputs :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m ([Data.Text.Text])
resourceConverterInputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterInputsStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq ([Data.Text.Text])
resourceConverterInputsStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterInputsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCStream ([Data.Text.Text]))
resourceConverterInputsStream thisArg indexArg = requestStream $ resourceConverterInputsStreamReq thisArg indexArg 

{-
 - The name of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (Data.Text.Text)
resourceConverterName thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterNameStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Data.Text.Text)
resourceConverterNameStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCStream (Data.Text.Text))
resourceConverterNameStream thisArg indexArg = requestStream $ resourceConverterNameStreamReq thisArg indexArg 

{-
 - List of the names of resources produced by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterOutputs :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m ([Data.Text.Text])
resourceConverterOutputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterOutputsStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq ([Data.Text.Text])
resourceConverterOutputsStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterOutputsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCStream ([Data.Text.Text]))
resourceConverterOutputsStream thisArg indexArg = requestStream $ resourceConverterOutputsStreamReq thisArg indexArg 

{-
 - Start the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m ()
resourceConverterStart thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Start" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterState thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterStateStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterStateStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceConverterState))
resourceConverterStateStream thisArg indexArg = requestStream $ resourceConverterStateStreamReq thisArg indexArg 

{-
 - Status information for the specified converter.
 - This is the full status message shown in the in-game UI.<param name="index">Index of the converter.
 -}
resourceConverterStatusInfo :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (Data.Text.Text)
resourceConverterStatusInfo thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterStatusInfoStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Data.Text.Text)
resourceConverterStatusInfoStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterStatusInfoStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m (KRPCStream (Data.Text.Text))
resourceConverterStatusInfoStream thisArg indexArg = requestStream $ resourceConverterStatusInfoStreamReq thisArg indexArg 

{-
 - Stop the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStop :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> m ()
resourceConverterStop thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Stop" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res 

{-
 - The number of converters in the part.
 -}
getResourceConverterCount :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> m (Data.Int.Int32)
getResourceConverterCount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceConverterCountStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCStreamReq (Data.Int.Int32)
getResourceConverterCountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]
    in  makeStream req

getResourceConverterCountStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> m (KRPCStream (Data.Int.Int32))
getResourceConverterCountStream thisArg = requestStream $ getResourceConverterCountStreamReq thisArg 

{-
 - The part object for this converter.
 -}
getResourceConverterPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> m (KRPCHS.SpaceCenter.Part)
getResourceConverterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceConverterPartStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourceConverterPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getResourceConverterPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceConverter -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceConverterPartStream thisArg = requestStream $ getResourceConverterPartStreamReq thisArg 

{-
 - Whether the harvester is actively drilling.
 -}
getResourceHarvesterActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (Bool)
getResourceHarvesterActive thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterActiveStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Bool)
getResourceHarvesterActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (Bool))
getResourceHarvesterActiveStream thisArg = requestStream $ getResourceHarvesterActiveStreamReq thisArg 

{-
 - The core temperature of the drill, in Kelvin.
 -}
getResourceHarvesterCoreTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (Float)
getResourceHarvesterCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterCoreTemperatureStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterCoreTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterCoreTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (Float))
getResourceHarvesterCoreTemperatureStream thisArg = requestStream $ getResourceHarvesterCoreTemperatureStreamReq thisArg 

{-
 - Whether the harvester is deployed.
 -}
getResourceHarvesterDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (Bool)
getResourceHarvesterDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterDeployedStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Bool)
getResourceHarvesterDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (Bool))
getResourceHarvesterDeployedStream thisArg = requestStream $ getResourceHarvesterDeployedStreamReq thisArg 

{-
 - The rate at which the drill is extracting ore, in units per second.
 -}
getResourceHarvesterExtractionRate :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (Float)
getResourceHarvesterExtractionRate thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterExtractionRateStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterExtractionRateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterExtractionRateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (Float))
getResourceHarvesterExtractionRateStream thisArg = requestStream $ getResourceHarvesterExtractionRateStreamReq thisArg 

{-
 - The core temperature at which the drill will operate with peak efficiency, in Kelvin.
 -}
getResourceHarvesterOptimumCoreTemperature :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (Float)
getResourceHarvesterOptimumCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterOptimumCoreTemperatureStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterOptimumCoreTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterOptimumCoreTemperatureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (Float))
getResourceHarvesterOptimumCoreTemperatureStream thisArg = requestStream $ getResourceHarvesterOptimumCoreTemperatureStreamReq thisArg 

{-
 - The part object for this harvester.
 -}
getResourceHarvesterPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterPartStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceHarvesterPartStream thisArg = requestStream $ getResourceHarvesterPartStreamReq thisArg 

{-
 - The state of the harvester.
 -}
getResourceHarvesterState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterState thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterStateStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvesterState))
getResourceHarvesterStateStream thisArg = requestStream $ getResourceHarvesterStateStreamReq thisArg 

{-
 - The thermal efficiency of the drill, as a percentage of its maximum.
 -}
getResourceHarvesterThermalEfficiency :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (Float)
getResourceHarvesterThermalEfficiency thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterThermalEfficiencyStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterThermalEfficiencyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterThermalEfficiencyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> m (KRPCStream (Float))
getResourceHarvesterThermalEfficiencyStream thisArg = requestStream $ getResourceHarvesterThermalEfficiencyStreamReq thisArg 

{-
 - Whether the harvester is actively drilling.
 -}
setResourceHarvesterActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> m ()
setResourceHarvesterActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the harvester is deployed.
 -}
setResourceHarvesterDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> m ()
setResourceHarvesterDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Start transferring a resource transfer between a pair of parts. The transfer will move at most
 - <paramref name="maxAmount" /> units of the resource, depending on how much of the resource is
 - available in the source part and how much storage is available in the destination part.
 - Use <see cref="M:SpaceCenter.ResourceTransfer.Complete" /> to check if the transfer is complete.
 - Use <see cref="M:SpaceCenter.ResourceTransfer.Amount" /> to see how much of the resource has been transferred.<param name="fromPart">The part to transfer to.<param name="toPart">The part to transfer from.<param name="resource">The name of the resource to transfer.<param name="maxAmount">The maximum amount of resource to transfer.
 -}
resourceTransferStart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> m (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStart fromPartArg toPartArg resourceArg maxAmountArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_Start" [makeArgument 0 fromPartArg, makeArgument 1 toPartArg, makeArgument 2 resourceArg, makeArgument 3 maxAmountArg]
    res <- sendRequest r
    processResponse res

resourceTransferStartStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStartStreamReq fromPartArg toPartArg resourceArg maxAmountArg =
    let req = makeRequest "SpaceCenter" "ResourceTransfer_Start" [makeArgument 0 fromPartArg, makeArgument 1 toPartArg, makeArgument 2 resourceArg, makeArgument 3 maxAmountArg]
    in  makeStream req

resourceTransferStartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceTransfer))
resourceTransferStartStream fromPartArg toPartArg resourceArg maxAmountArg = requestStream $ resourceTransferStartStreamReq fromPartArg toPartArg resourceArg maxAmountArg 

{-
 - The amount of the resource that has been transferred.
 -}
getResourceTransferAmount :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceTransfer -> m (Float)
getResourceTransferAmount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceTransferAmountStreamReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCStreamReq (Float)
getResourceTransferAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]
    in  makeStream req

getResourceTransferAmountStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceTransfer -> m (KRPCStream (Float))
getResourceTransferAmountStream thisArg = requestStream $ getResourceTransferAmountStreamReq thisArg 

{-
 - Whether the transfer has completed.
 -}
getResourceTransferComplete :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceTransfer -> m (Bool)
getResourceTransferComplete thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceTransferCompleteStreamReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCStreamReq (Bool)
getResourceTransferCompleteStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]
    in  makeStream req

getResourceTransferCompleteStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ResourceTransfer -> m (KRPCStream (Bool))
getResourceTransferCompleteStream thisArg = requestStream $ getResourceTransferCompleteStreamReq thisArg 

{-
 - The amount of the resource that is currently stored in the part.
 -}
getResourceAmount :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (Float)
getResourceAmount thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceAmountStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]
    in  makeStream req

getResourceAmountStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (Float))
getResourceAmountStream thisArg = requestStream $ getResourceAmountStreamReq thisArg 

{-
 - The density of the resource, inkg/l.
 -}
getResourceDensity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (Float)
getResourceDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceDensityStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceDensityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]
    in  makeStream req

getResourceDensityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (Float))
getResourceDensityStream thisArg = requestStream $ getResourceDensityStreamReq thisArg 

{-
 - Whether use of this resource is enabled.
 -}
getResourceEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (Bool)
getResourceEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceEnabledStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Bool)
getResourceEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]
    in  makeStream req

getResourceEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (Bool))
getResourceEnabledStream thisArg = requestStream $ getResourceEnabledStreamReq thisArg 

{-
 - The flow mode of the resource.
 -}
getResourceFlowMode :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowMode thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceFlowModeStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]
    in  makeStream req

getResourceFlowModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
getResourceFlowModeStream thisArg = requestStream $ getResourceFlowModeStreamReq thisArg 

{-
 - The total amount of the resource that can be stored in the part.
 -}
getResourceMax :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (Float)
getResourceMax thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceMaxStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceMaxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]
    in  makeStream req

getResourceMaxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (Float))
getResourceMaxStream thisArg = requestStream $ getResourceMaxStreamReq thisArg 

{-
 - The name of the resource.
 -}
getResourceName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (Data.Text.Text)
getResourceName thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceNameStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Data.Text.Text)
getResourceNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getResourceNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (Data.Text.Text))
getResourceNameStream thisArg = requestStream $ getResourceNameStreamReq thisArg 

{-
 - The part containing the resource.
 -}
getResourcePart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCHS.SpaceCenter.Part)
getResourcePart thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcePartStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourcePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getResourcePartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourcePartStream thisArg = requestStream $ getResourcePartStreamReq thisArg 

{-
 - Whether use of this resource is enabled.
 -}
setResourceEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resource -> Bool -> m ()
setResourceEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Resource_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns the amount of a resource that is currently stored.<param name="name">The name of the resource.
 -}
resourcesAmount :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (Float)
resourcesAmount thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesAmountStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Float)
resourcesAmountStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesAmountStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (KRPCStream (Float))
resourcesAmountStream thisArg nameArg = requestStream $ resourcesAmountStreamReq thisArg nameArg 

{-
 - Returns the density of a resource, in kg/l.<param name="name">The name of the resource.
 -}
resourcesDensity :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (Float)
resourcesDensity nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Density" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res

resourcesDensityStreamReq :: Data.Text.Text -> KRPCStreamReq (Float)
resourcesDensityStreamReq nameArg =
    let req = makeRequest "SpaceCenter" "Resources_Density" [makeArgument 0 nameArg]
    in  makeStream req

resourcesDensityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (KRPCStream (Float))
resourcesDensityStream nameArg = requestStream $ resourcesDensityStreamReq nameArg 

{-
 - Returns the flow mode of a resource.<param name="name">The name of the resource.
 -}
resourcesFlowMode :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesFlowMode nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_FlowMode" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res

resourcesFlowModeStreamReq :: Data.Text.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesFlowModeStreamReq nameArg =
    let req = makeRequest "SpaceCenter" "Resources_FlowMode" [makeArgument 0 nameArg]
    in  makeStream req

resourcesFlowModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
resourcesFlowModeStream nameArg = requestStream $ resourcesFlowModeStreamReq nameArg 

{-
 - Check whether the named resource can be stored.<param name="name">The name of the resource.
 -}
resourcesHasResource :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (Bool)
resourcesHasResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesHasResourceStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Bool)
resourcesHasResourceStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesHasResourceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (KRPCStream (Bool))
resourcesHasResourceStream thisArg nameArg = requestStream $ resourcesHasResourceStreamReq thisArg nameArg 

{-
 - Returns the amount of a resource that can be stored.<param name="name">The name of the resource.
 -}
resourcesMax :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (Float)
resourcesMax thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesMaxStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Float)
resourcesMaxStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesMaxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (KRPCStream (Float))
resourcesMaxStream thisArg nameArg = requestStream $ resourcesMaxStreamReq thisArg nameArg 

{-
 - All the individual resources with the given name that can be stored.
 -}
resourcesWithResource :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m ([KRPCHS.SpaceCenter.Resource])
resourcesWithResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesWithResourceStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
resourcesWithResourceStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesWithResourceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> m (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
resourcesWithResourceStream thisArg nameArg = requestStream $ resourcesWithResourceStreamReq thisArg nameArg 

{-
 - All the individual resources that can be stored.
 -}
getResourcesAll :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> m ([KRPCHS.SpaceCenter.Resource])
getResourcesAll thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcesAllStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
getResourcesAllStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]
    in  makeStream req

getResourcesAllStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> m (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
getResourcesAllStream thisArg = requestStream $ getResourcesAllStreamReq thisArg 

{-
 - Whether use of all the resources are enabled.This is true if all of the resources are enabled. If any of the resources are not enabled, this is false.
 -}
getResourcesEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> m (Bool)
getResourcesEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcesEnabledStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq (Bool)
getResourcesEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resources_get_Enabled" [makeArgument 0 thisArg]
    in  makeStream req

getResourcesEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> m (KRPCStream (Bool))
getResourcesEnabledStream thisArg = requestStream $ getResourcesEnabledStreamReq thisArg 

{-
 - A list of resource names that can be stored.
 -}
getResourcesNames :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> m ([Data.Text.Text])
getResourcesNames thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcesNamesStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq ([Data.Text.Text])
getResourcesNamesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]
    in  makeStream req

getResourcesNamesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> m (KRPCStream ([Data.Text.Text]))
getResourcesNamesStream thisArg = requestStream $ getResourcesNamesStreamReq thisArg 

{-
 - Whether use of all the resources are enabled.This is true if all of the resources are enabled. If any of the resources are not enabled, this is false.
 -}
setResourcesEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Resources -> Bool -> m ()
setResourcesEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Resources_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Save the game with a given name.
 - This will create a save file calledname.sfsin the folder of the current save game.
 -}
save :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> m ()
save nameArg = do
    let r = makeRequest "SpaceCenter" "Save" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Data amount.
 -}
getScienceDataDataAmount :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceData -> m (Float)
getScienceDataDataAmount thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceData_get_DataAmount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceDataDataAmountStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataDataAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceData_get_DataAmount" [makeArgument 0 thisArg]
    in  makeStream req

getScienceDataDataAmountStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceData -> m (KRPCStream (Float))
getScienceDataDataAmountStream thisArg = requestStream $ getScienceDataDataAmountStreamReq thisArg 

{-
 - Science value.
 -}
getScienceDataScienceValue :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceData -> m (Float)
getScienceDataScienceValue thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceData_get_ScienceValue" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceDataScienceValueStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataScienceValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceData_get_ScienceValue" [makeArgument 0 thisArg]
    in  makeStream req

getScienceDataScienceValueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceData -> m (KRPCStream (Float))
getScienceDataScienceValueStream thisArg = requestStream $ getScienceDataScienceValueStreamReq thisArg 

{-
 - Transmit value.
 -}
getScienceDataTransmitValue :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceData -> m (Float)
getScienceDataTransmitValue thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceData_get_TransmitValue" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceDataTransmitValueStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataTransmitValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceData_get_TransmitValue" [makeArgument 0 thisArg]
    in  makeStream req

getScienceDataTransmitValueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceData -> m (KRPCStream (Float))
getScienceDataTransmitValueStream thisArg = requestStream $ getScienceDataTransmitValueStreamReq thisArg 

{-
 - Multiply science value by this to determine data amount in mits.
 -}
getScienceSubjectDataScale :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Float)
getScienceSubjectDataScale thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_DataScale" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectDataScaleStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectDataScaleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_DataScale" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectDataScaleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Float))
getScienceSubjectDataScaleStream thisArg = requestStream $ getScienceSubjectDataScaleStreamReq thisArg 

{-
 - Whether the experiment has been completed.
 -}
getScienceSubjectIsComplete :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Bool)
getScienceSubjectIsComplete thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_IsComplete" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectIsCompleteStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Bool)
getScienceSubjectIsCompleteStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_IsComplete" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectIsCompleteStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Bool))
getScienceSubjectIsCompleteStream thisArg = requestStream $ getScienceSubjectIsCompleteStreamReq thisArg 

{-
 - Amount of science already earned from this subject, not updated until after transmission/recovery.
 -}
getScienceSubjectScience :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Float)
getScienceSubjectScience thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_Science" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectScienceStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectScienceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_Science" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectScienceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Float))
getScienceSubjectScienceStream thisArg = requestStream $ getScienceSubjectScienceStreamReq thisArg 

{-
 - Total science allowable for this subject.
 -}
getScienceSubjectScienceCap :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Float)
getScienceSubjectScienceCap thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_ScienceCap" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectScienceCapStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectScienceCapStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_ScienceCap" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectScienceCapStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Float))
getScienceSubjectScienceCapStream thisArg = requestStream $ getScienceSubjectScienceCapStreamReq thisArg 

{-
 - Diminishing value multiplier for decreasing the science value returned from repeated experiments.
 -}
getScienceSubjectScientificValue :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Float)
getScienceSubjectScientificValue thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_ScientificValue" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectScientificValueStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectScientificValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_ScientificValue" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectScientificValueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Float))
getScienceSubjectScientificValueStream thisArg = requestStream $ getScienceSubjectScientificValueStreamReq thisArg 

{-
 - Multiplier for specific Celestial Body/Experiment Situation combination.
 -}
getScienceSubjectSubjectValue :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Float)
getScienceSubjectSubjectValue thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_SubjectValue" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectSubjectValueStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectSubjectValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_SubjectValue" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectSubjectValueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Float))
getScienceSubjectSubjectValueStream thisArg = requestStream $ getScienceSubjectSubjectValueStreamReq thisArg 

{-
 - Title of science subject, displayed in science archives
 -}
getScienceSubjectTitle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (Data.Text.Text)
getScienceSubjectTitle thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceSubject_get_Title" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceSubjectTitleStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Data.Text.Text)
getScienceSubjectTitleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceSubject_get_Title" [makeArgument 0 thisArg]
    in  makeStream req

getScienceSubjectTitleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.ScienceSubject -> m (KRPCStream (Data.Text.Text))
getScienceSubjectTitleStream thisArg = requestStream $ getScienceSubjectTitleStreamReq thisArg 

{-
 - Whether the sensor is active.
 -}
getSensorActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> m (Bool)
getSensorActive thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorActiveStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Bool)
getSensorActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getSensorActiveStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> m (KRPCStream (Bool))
getSensorActiveStream thisArg = requestStream $ getSensorActiveStreamReq thisArg 

{-
 - The part object for this sensor.
 -}
getSensorPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> m (KRPCHS.SpaceCenter.Part)
getSensorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorPartStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getSensorPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getSensorPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getSensorPartStream thisArg = requestStream $ getSensorPartStreamReq thisArg 

{-
 - The current value of the sensor.
 -}
getSensorValue :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> m (Data.Text.Text)
getSensorValue thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorValueStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Data.Text.Text)
getSensorValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]
    in  makeStream req

getSensorValueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> m (KRPCStream (Data.Text.Text))
getSensorValueStream thisArg = requestStream $ getSensorValueStreamReq thisArg 

{-
 - Whether the sensor is active.
 -}
setSensorActive :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Sensor -> Bool -> m ()
setSensorActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Sensor_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the solar panel is deployable.
 -}
getSolarPanelDeployable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (Bool)
getSolarPanelDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelDeployableStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Bool)
getSolarPanelDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelDeployableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCStream (Bool))
getSolarPanelDeployableStream thisArg = requestStream $ getSolarPanelDeployableStreamReq thisArg 

{-
 - Whether the solar panel is extended.
 -}
getSolarPanelDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (Bool)
getSolarPanelDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelDeployedStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Bool)
getSolarPanelDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCStream (Bool))
getSolarPanelDeployedStream thisArg = requestStream $ getSolarPanelDeployedStreamReq thisArg 

{-
 - The current amount of energy being generated by the solar panel, in
 - units of charge per second.
 -}
getSolarPanelEnergyFlow :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (Float)
getSolarPanelEnergyFlow thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelEnergyFlowStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Float)
getSolarPanelEnergyFlowStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelEnergyFlowStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCStream (Float))
getSolarPanelEnergyFlowStream thisArg = requestStream $ getSolarPanelEnergyFlowStreamReq thisArg 

{-
 - The part object for this solar panel.
 -}
getSolarPanelPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCHS.SpaceCenter.Part)
getSolarPanelPart thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelPartStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getSolarPanelPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getSolarPanelPartStream thisArg = requestStream $ getSolarPanelPartStreamReq thisArg 

{-
 - The current state of the solar panel.
 -}
getSolarPanelState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelState thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelStateStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCStream (KRPCHS.SpaceCenter.SolarPanelState))
getSolarPanelStateStream thisArg = requestStream $ getSolarPanelStateStreamReq thisArg 

{-
 - The current amount of sunlight that is incident on the solar panel,
 - as a percentage. A value between 0 and 1.
 -}
getSolarPanelSunExposure :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (Float)
getSolarPanelSunExposure thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelSunExposureStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Float)
getSolarPanelSunExposureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelSunExposureStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> m (KRPCStream (Float))
getSolarPanelSunExposureStream thisArg = requestStream $ getSolarPanelSunExposureStreamReq thisArg 

{-
 - Whether the solar panel is extended.
 -}
setSolarPanelDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.SolarPanel -> Bool -> m ()
setSolarPanelDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position around which the gimbal pivots.
 -}
thrusterGimbalPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
thrusterGimbalPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterGimbalPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterGimbalPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterGimbalPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
thrusterGimbalPositionStream thisArg referenceFrameArg = requestStream $ thrusterGimbalPositionStreamReq thisArg referenceFrameArg 

{-
 - The direction of the force generated by the thruster, when the engine is in its
 - initial position (no gimballing), in the given reference frame.
 - This is opposite to the direction in which the thruster expels propellant.<param name="referenceFrame">
 -}
thrusterInitialThrustDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
thrusterInitialThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterInitialThrustDirectionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterInitialThrustDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterInitialThrustDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustDirectionStream thisArg referenceFrameArg = requestStream $ thrusterInitialThrustDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position at which the thruster generates thrust, when the engine is in its
 - initial position (no gimballing), in the given reference frame.<param name="referenceFrame">This position can move when the gimbal rotates. This is because the thrust position and
 - gimbal position are not necessarily the same.
 -}
thrusterInitialThrustPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
thrusterInitialThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterInitialThrustPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterInitialThrustPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterInitialThrustPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustPositionStream thisArg referenceFrameArg = requestStream $ thrusterInitialThrustPositionStreamReq thisArg referenceFrameArg 

{-
 - The direction of the force generated by the thruster, in the given reference frame.
 - This is opposite to the direction in which the thruster expels propellant.
 - For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
thrusterThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterThrustDirectionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterThrustDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterThrustDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
thrusterThrustDirectionStream thisArg referenceFrameArg = requestStream $ thrusterThrustDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position at which the thruster generates thrust, in the given reference frame.
 - For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
thrusterThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterThrustPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterThrustPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterThrustPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
thrusterThrustPositionStream thisArg referenceFrameArg = requestStream $ thrusterThrustPositionStreamReq thisArg referenceFrameArg 

{-
 - The current gimbal angle in the pitch, roll and yaw axes.
 -}
getThrusterGimbalAngle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m ((Double, Double, Double))
getThrusterGimbalAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterGimbalAngleStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq ((Double, Double, Double))
getThrusterGimbalAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterGimbalAngleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (KRPCStream ((Double, Double, Double)))
getThrusterGimbalAngleStream thisArg = requestStream $ getThrusterGimbalAngleStreamReq thisArg 

{-
 - Whether the thruster is gimballed.
 -}
getThrusterGimballed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (Bool)
getThrusterGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterGimballedStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (Bool)
getThrusterGimballedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterGimballedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (KRPCStream (Bool))
getThrusterGimballedStream thisArg = requestStream $ getThrusterGimballedStreamReq thisArg 

{-
 - The <see cref="T:SpaceCenter.Part" /> that contains this thruster.
 -}
getThrusterPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (KRPCHS.SpaceCenter.Part)
getThrusterPart thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterPartStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getThrusterPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getThrusterPartStream thisArg = requestStream $ getThrusterPartStreamReq thisArg 

{-
 - A reference frame that is fixed relative to the thruster and orientated with
 - its thrust direction (<see cref="M:SpaceCenter.Thruster.ThrustDirection" />).
 - For gimballed engines, this takes into account the current rotation of the gimbal.
 - <list type="bullet">The origin is at the position of thrust for this thruster (<see cref="M:SpaceCenter.Thruster.ThrustPosition" />).The axes rotate with the thrust direction.
 - This is the direction in which the thruster expels propellant, including any gimballing.The y-axis points along the thrust direction.The x-axis and z-axis are perpendicular to the thrust direction.
 -}
getThrusterThrustReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterThrustReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterThrustReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Thruster -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getThrusterThrustReferenceFrameStream thisArg = requestStream $ getThrusterThrustReferenceFrameStreamReq thisArg 

{-
 - Converts a direction vector from one reference frame to another.<param name="direction">Direction vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the direction vector is in.<param name="to">The reference frame to covert the direction vector to.The corresponding direction vector in reference frame <paramref name="to" />.
 -}
transformDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
transformDirection directionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse res

transformDirectionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformDirectionStreamReq directionArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    in  makeStream req

transformDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
transformDirectionStream directionArg fromArg toArg = requestStream $ transformDirectionStreamReq directionArg fromArg toArg 

{-
 - Converts a position vector from one reference frame to another.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position vector is in.<param name="to">The reference frame to covert the position vector to.The corresponding position vector in reference frame <paramref name="to" />.
 -}
transformPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
transformPosition positionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse res

transformPositionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformPositionStreamReq positionArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    in  makeStream req

transformPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
transformPositionStream positionArg fromArg toArg = requestStream $ transformPositionStreamReq positionArg fromArg toArg 

{-
 - Converts a rotation from one reference frame to another.<param name="rotation">Rotation in reference frame <paramref name="from" />.<param name="from">The reference frame that the rotation is in.<param name="to">The corresponding rotation in reference frame <paramref name="to" />.The corresponding rotation in reference frame <paramref name="to" />.
 -}
transformRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double, Double))
transformRotation rotationArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse res

transformRotationStreamReq :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
transformRotationStreamReq rotationArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    in  makeStream req

transformRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double, Double)))
transformRotationStream rotationArg fromArg toArg = requestStream $ transformRotationStreamReq rotationArg fromArg toArg 

{-
 - Converts a velocity vector (acting at the specified position vector) from one
 - reference frame to another. The position vector is required to take the
 - relative angular velocity of the reference frames into account.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="velocity">Velocity vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position and velocity vectors are in.<param name="to">The reference frame to covert the velocity vector to.The corresponding velocity in reference frame <paramref name="to" />.
 -}
transformVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
transformVelocity positionArg velocityArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]
    res <- sendRequest r
    processResponse res

transformVelocityStreamReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformVelocityStreamReq positionArg velocityArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]
    in  makeStream req

transformVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
transformVelocityStream positionArg velocityArg fromArg toArg = requestStream $ transformVelocityStreamReq positionArg velocityArg fromArg toArg 

{-
 - Returns the angular velocity of the vessel in the given reference frame. The magnitude of the returned
 - vector is the rotational speed in radians per second, and the direction of the vector indicates the
 - axis of rotation (using the right hand rule).<param name="referenceFrame">
 -}
vesselAngularVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
vesselAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselAngularVelocityStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselAngularVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselAngularVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
vesselAngularVelocityStream thisArg referenceFrameArg = requestStream $ vesselAngularVelocityStreamReq thisArg referenceFrameArg 

{-
 - The axis-aligned bounding box of the vessel in the given reference frame.
 - Returns the minimum and maximum vertices of the box.<param name="referenceFrame">
 -}
vesselBoundingBox :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (((Double, Double, Double), (Double, Double, Double)))
vesselBoundingBox thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_BoundingBox" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselBoundingBoxStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
vesselBoundingBoxStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_BoundingBox" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselBoundingBoxStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
vesselBoundingBoxStream thisArg referenceFrameArg = requestStream $ vesselBoundingBoxStreamReq thisArg referenceFrameArg 

{-
 - Returns the direction in which the vessel is pointing, as a unit vector, in the given reference frame.<param name="referenceFrame">
 -}
vesselDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
vesselDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselDirectionStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
vesselDirectionStream thisArg referenceFrameArg = requestStream $ vesselDirectionStreamReq thisArg referenceFrameArg 

{-
 - Returns a <see cref="T:SpaceCenter.Flight" /> object that can be used to get flight
 - telemetry for the vessel, in the specified reference frame.<param name="referenceFrame">
 - Reference frame. Defaults to the vessel's surface reference frame (<see cref="M:SpaceCenter.Vessel.SurfaceReferenceFrame" />).
 -}
vesselFlight :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCHS.SpaceCenter.Flight)
vesselFlight thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselFlightStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (KRPCHS.SpaceCenter.Flight)
vesselFlightStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselFlightStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream (KRPCHS.SpaceCenter.Flight))
vesselFlightStream thisArg referenceFrameArg = requestStream $ vesselFlightStreamReq thisArg referenceFrameArg 

{-
 - Returns the position vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
vesselPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselPositionStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
vesselPositionStream thisArg referenceFrameArg = requestStream $ vesselPositionStreamReq thisArg referenceFrameArg 

{-
 - Recover the vessel.
 -}
vesselRecover :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m ()
vesselRecover thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Recover" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns a <see cref="T:SpaceCenter.Resources" /> object, that can used to get
 - information about resources stored in a given <paramref name="stage" />.<param name="stage">Get resources for parts that are decoupled in this stage.<param name="cumulative">Whenfalse, returns the resources for parts
 - decoupled in just the given stage. Whentruereturns the resources decoupled in
 - the given stage and all subsequent stages combined.
 -}
vesselResourcesInDecoupleStage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> m (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStage thisArg stageArg cumulativeArg = do
    let r = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg, makeArgument 2 cumulativeArg]
    res <- sendRequest r
    processResponse res

vesselResourcesInDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStageStreamReq thisArg stageArg cumulativeArg =
    let req = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg, makeArgument 2 cumulativeArg]
    in  makeStream req

vesselResourcesInDecoupleStageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> m (KRPCStream (KRPCHS.SpaceCenter.Resources))
vesselResourcesInDecoupleStageStream thisArg stageArg cumulativeArg = requestStream $ vesselResourcesInDecoupleStageStreamReq thisArg stageArg cumulativeArg 

{-
 - Returns the rotation of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double, Double))
vesselRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselRotationStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
vesselRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double, Double)))
vesselRotationStream thisArg referenceFrameArg = requestStream $ vesselRotationStreamReq thisArg referenceFrameArg 

{-
 - Returns the velocity vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselVelocity :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m ((Double, Double, Double))
vesselVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselVelocityStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselVelocityStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> m (KRPCStream ((Double, Double, Double)))
vesselVelocityStream thisArg referenceFrameArg = requestStream $ vesselVelocityStreamReq thisArg referenceFrameArg 

{-
 - An <see cref="T:SpaceCenter.AutoPilot" /> object, that can be used to perform
 - simple auto-piloting of the vessel.
 -}
getVesselAutoPilot :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilot thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAutoPilotStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilotStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAutoPilotStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.AutoPilot))
getVesselAutoPilotStream thisArg = requestStream $ getVesselAutoPilotStreamReq thisArg 

{-
 - The maximum torque that the aerodynamic control surfaces can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableControlSurfaceTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableControlSurfaceTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableControlSurfaceTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableControlSurfaceTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableControlSurfaceTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableControlSurfaceTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableControlSurfaceTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getVesselAvailableControlSurfaceTorqueStream thisArg = requestStream $ getVesselAvailableControlSurfaceTorqueStreamReq thisArg 

{-
 - The maximum torque that the currently active and gimballed engines can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableEngineTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableEngineTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableEngineTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableEngineTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableEngineTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableEngineTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableEngineTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getVesselAvailableEngineTorqueStream thisArg = requestStream $ getVesselAvailableEngineTorqueStreamReq thisArg 

{-
 - The maximum torque that parts (excluding reaction wheels, gimballed engines, RCS and control surfaces) can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableOtherTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableOtherTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableOtherTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableOtherTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableOtherTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableOtherTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableOtherTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getVesselAvailableOtherTorqueStream thisArg = requestStream $ getVesselAvailableOtherTorqueStreamReq thisArg 

{-
 - The maximum torque that the currently active RCS thrusters can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableRCSTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableRCSTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableRCSTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableRCSTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableRCSTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableRCSTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableRCSTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getVesselAvailableRCSTorqueStream thisArg = requestStream $ getVesselAvailableRCSTorqueStreamReq thisArg 

{-
 - The maximum torque that the currently active and powered reaction wheels can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableReactionWheelTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableReactionWheelTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableReactionWheelTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableReactionWheelTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableReactionWheelTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableReactionWheelTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableReactionWheelTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getVesselAvailableReactionWheelTorqueStream thisArg = requestStream $ getVesselAvailableReactionWheelTorqueStreamReq thisArg 

{-
 - Gets the total available thrust that can be produced by the vessel's
 - active engines, in Newtons. This is computed by summing
 - <see cref="M:SpaceCenter.Engine.AvailableThrust" /> for every active engine in the vessel.
 -}
getVesselAvailableThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselAvailableThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselAvailableThrustStream thisArg = requestStream $ getVesselAvailableThrustStreamReq thisArg 

{-
 - The maximum torque that the vessel generate. Includes contributions from reaction wheels,
 - RCS, gimballed engines and aerodynamic control surfaces.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableTorque :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (((Double, Double, Double), (Double, Double, Double)))
getVesselAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableTorqueStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (((Double, Double, Double), (Double, Double, Double))))
getVesselAvailableTorqueStream thisArg = requestStream $ getVesselAvailableTorqueStreamReq thisArg 

{-
 - The name of the biome the vessel is currently in.
 -}
getVesselBiome :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Data.Text.Text)
getVesselBiome thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Biome" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselBiomeStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Data.Text.Text)
getVesselBiomeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Biome" [makeArgument 0 thisArg]
    in  makeStream req

getVesselBiomeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Data.Text.Text))
getVesselBiomeStream thisArg = requestStream $ getVesselBiomeStreamReq thisArg 

{-
 - Returns a <see cref="T:SpaceCenter.Comms" /> object that can be used to interact
 - with CommNet for this vessel.
 -}
getVesselComms :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.Comms)
getVesselComms thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Comms" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselCommsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Comms)
getVesselCommsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Comms" [makeArgument 0 thisArg]
    in  makeStream req

getVesselCommsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.Comms))
getVesselCommsStream thisArg = requestStream $ getVesselCommsStreamReq thisArg 

{-
 - Returns a <see cref="T:SpaceCenter.Control" /> object that can be used to manipulate
 - the vessel's control inputs. For example, its pitch/yaw/roll controls,
 - RCS and thrust.
 -}
getVesselControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.Control)
getVesselControl thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselControlStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Control)
getVesselControlStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]
    in  makeStream req

getVesselControlStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.Control))
getVesselControlStream thisArg = requestStream $ getVesselControlStreamReq thisArg 

{-
 - The total mass of the vessel, excluding resources, in kg.
 -}
getVesselDryMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselDryMassStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselDryMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]
    in  makeStream req

getVesselDryMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselDryMassStream thisArg = requestStream $ getVesselDryMassStreamReq thisArg 

{-
 - The inertia tensor of the vessel around its center of mass, in the vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getVesselInertiaTensor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m ([Double])
getVesselInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselInertiaTensorStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ([Double])
getVesselInertiaTensorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]
    in  makeStream req

getVesselInertiaTensorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream ([Double]))
getVesselInertiaTensorStream thisArg = requestStream $ getVesselInertiaTensorStreamReq thisArg 

{-
 - The combined specific impulse of all active engines at sea level on Kerbin, in seconds.
 - This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselKerbinSeaLevelSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselKerbinSeaLevelSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getVesselKerbinSeaLevelSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselKerbinSeaLevelSpecificImpulseStream thisArg = requestStream $ getVesselKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-
 - The mission elapsed time in seconds.
 -}
getVesselMET :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Double)
getVesselMET thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMETStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Double)
getVesselMETStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMETStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Double))
getVesselMETStream thisArg = requestStream $ getVesselMETStreamReq thisArg 

{-
 - The total mass of the vessel, including resources, in kg.
 -}
getVesselMass :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMassStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMassStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselMassStream thisArg = requestStream $ getVesselMassStreamReq thisArg 

{-
 - The total maximum thrust that can be produced by the vessel's active
 - engines, in Newtons. This is computed by summing
 - <see cref="M:SpaceCenter.Engine.MaxThrust" /> for every active engine.
 -}
getVesselMaxThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMaxThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMaxThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMaxThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselMaxThrustStream thisArg = requestStream $ getVesselMaxThrustStreamReq thisArg 

{-
 - The total maximum thrust that can be produced by the vessel's active
 - engines when the vessel is in a vacuum, in Newtons. This is computed by
 - summing <see cref="M:SpaceCenter.Engine.MaxVacuumThrust" /> for every active engine.
 -}
getVesselMaxVacuumThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMaxVacuumThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMaxVacuumThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselMaxVacuumThrustStream thisArg = requestStream $ getVesselMaxVacuumThrustStreamReq thisArg 

{-
 - The moment of inertia of the vessel around its center of mass inkg.m^2.
 - The inertia values are around the pitch, roll and yaw directions respectively.
 - This corresponds to the vessels reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 -}
getVesselMomentOfInertia :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m ((Double, Double, Double))
getVesselMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMomentOfInertiaStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselMomentOfInertiaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMomentOfInertiaStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream ((Double, Double, Double)))
getVesselMomentOfInertiaStream thisArg = requestStream $ getVesselMomentOfInertiaStreamReq thisArg 

{-
 - The name of the vessel.
 -}
getVesselName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Data.Text.Text)
getVesselName thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Data.Text.Text)
getVesselNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getVesselNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Data.Text.Text))
getVesselNameStream thisArg = requestStream $ getVesselNameStreamReq thisArg 

{-
 - The current orbit of the vessel.
 -}
getVesselOrbit :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.Orbit)
getVesselOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselOrbitStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getVesselOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]
    in  makeStream req

getVesselOrbitStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getVesselOrbitStream thisArg = requestStream $ getVesselOrbitStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the vessels
 - orbital prograde/normal/radial directions.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the orbital prograde/normal/radial directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.Be careful not to confuse this with 'orbit' mode on the navball.
 -}
getVesselOrbitalReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselOrbitalReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselOrbitalReferenceFrameStream thisArg = requestStream $ getVesselOrbitalReferenceFrameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Parts" /> object, that can used to interact with the parts that make up this vessel.
 -}
getVesselParts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.Parts)
getVesselParts thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselPartsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Parts)
getVesselPartsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]
    in  makeStream req

getVesselPartsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.Parts))
getVesselPartsStream thisArg = requestStream $ getVesselPartsStreamReq thisArg 

{-
 - Whether the vessel is recoverable.
 -}
getVesselRecoverable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Bool)
getVesselRecoverable thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Recoverable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselRecoverableStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Bool)
getVesselRecoverableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Recoverable" [makeArgument 0 thisArg]
    in  makeStream req

getVesselRecoverableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Bool))
getVesselRecoverableStream thisArg = requestStream $ getVesselRecoverableStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the vessel.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel.The x-axis points out to the right of the vessel.The y-axis points in the forward direction of the vessel.The z-axis points out of the bottom off the vessel.
 -}
getVesselReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselReferenceFrameStream thisArg = requestStream $ getVesselReferenceFrameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Resources" /> object, that can used to get information
 - about resources stored in the vessel.
 -}
getVesselResources :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.Resources)
getVesselResources thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselResourcesStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
getVesselResourcesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]
    in  makeStream req

getVesselResourcesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.Resources))
getVesselResourcesStream thisArg = requestStream $ getVesselResourcesStreamReq thisArg 

{-
 - The situation the vessel is in.
 -}
getVesselSituation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituation thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSituationStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSituationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.VesselSituation))
getVesselSituationStream thisArg = requestStream $ getVesselSituationStreamReq thisArg 

{-
 - The combined specific impulse of all active engines, in seconds. This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselSpecificImpulseStream thisArg = requestStream $ getVesselSpecificImpulseStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the surface
 - of the body being orbited.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the north and up directions on the surface of the body.The x-axis points in the <a href="https://en.wikipedia.org/wiki/Zenith">zenithdirection (upwards, normal to the body being orbited, from the center of the body towards the center of
 - mass of the vessel).The y-axis points northwards towards the
 - <a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon(north, and tangential to the
 - surface of the body -- the direction in which a compass would point when on the surface).The z-axis points eastwards towards the
 - <a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon(east, and tangential to the
 - surface of the body -- east on a compass when on the surface).Be careful not to confuse this with 'surface' mode on the navball.
 -}
getVesselSurfaceReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSurfaceReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSurfaceReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceReferenceFrameStream thisArg = requestStream $ getVesselSurfaceReferenceFrameStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the velocity
 - vector of the vessel relative to the surface of the body being orbited.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel's velocity vector.The y-axis points in the direction of the vessel's velocity vector,
 - relative to the surface of the body being orbited.The z-axis is in the plane of the
 - <a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon.The x-axis is orthogonal to the other two axes.
 -}
getVesselSurfaceVelocityReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSurfaceVelocityReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSurfaceVelocityReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceVelocityReferenceFrameStream thisArg = requestStream $ getVesselSurfaceVelocityReferenceFrameStreamReq thisArg 

{-
 - The total thrust currently being produced by the vessel's engines, in
 - Newtons. This is computed by summing <see cref="M:SpaceCenter.Engine.Thrust" /> for
 - every engine in the vessel.
 -}
getVesselThrust :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselThrustStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselThrustStream thisArg = requestStream $ getVesselThrustStreamReq thisArg 

{-
 - The type of the vessel.
 -}
getVesselType :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCHS.SpaceCenter.VesselType)
getVesselType thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselTypeStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.VesselType)
getVesselTypeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]
    in  makeStream req

getVesselTypeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (KRPCHS.SpaceCenter.VesselType))
getVesselTypeStream thisArg = requestStream $ getVesselTypeStreamReq thisArg 

{-
 - The combined vacuum specific impulse of all active engines, in seconds. This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselVacuumSpecificImpulse :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (Float)
getVesselVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselVacuumSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getVesselVacuumSpecificImpulseStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m (KRPCStream (Float))
getVesselVacuumSpecificImpulseStream thisArg = requestStream $ getVesselVacuumSpecificImpulseStreamReq thisArg 

{-
 - The name of the vessel.
 -}
setVesselName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> m ()
setVesselName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The type of the vessel.
 -}
setVesselType :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.VesselType -> m ()
setVesselType thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Type" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Uses time acceleration to warp forward to a time in the future, specified
 - by universal time <paramref name="ut" />. This call blocks until the desired
 - time is reached. Uses regular "on-rails" or physical time warp as appropriate.
 - For example, physical time warp is used when the active vessel is traveling
 - through an atmosphere. When using regular "on-rails" time warp, the warp
 - rate is limited by <paramref name="maxRailsRate" />, and when using physical
 - time warp, the warp rate is limited by <paramref name="maxPhysicsRate" />.<param name="ut">The universal time to warp to, in seconds.<param name="maxRailsRate">The maximum warp rate in regular "on-rails" time warp.<param name="maxPhysicsRate">The maximum warp rate in physical time warp.When the time warp is complete.
 -}
warpTo :: (MonadIO m, MonadThrow m, MonadRPC m) => Double -> Float -> Float -> m ()
warpTo utArg maxRailsRateArg maxPhysicsRateArg = do
    let r = makeRequest "SpaceCenter" "WarpTo" [makeArgument 0 utArg, makeArgument 1 maxRailsRateArg, makeArgument 2 maxPhysicsRateArg]
    res <- sendRequest r
    processResponse res 

{-
 - Creates a waypoint at the given position at ground level, and returns a
 - <see cref="T:SpaceCenter.Waypoint" /> object that can be used to modify it.<param name="latitude">Latitude of the waypoint.<param name="longitude">Longitude of the waypoint.<param name="body">Celestial body the waypoint is attached to.<param name="name">Name of the waypoint.
 -}
waypointManagerAddWaypoint :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> m (KRPCHS.SpaceCenter.Waypoint)
waypointManagerAddWaypoint thisArg latitudeArg longitudeArg bodyArg nameArg = do
    let r = makeRequest "SpaceCenter" "WaypointManager_AddWaypoint" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 bodyArg, makeArgument 4 nameArg]
    res <- sendRequest r
    processResponse res

waypointManagerAddWaypointStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.Waypoint)
waypointManagerAddWaypointStreamReq thisArg latitudeArg longitudeArg bodyArg nameArg =
    let req = makeRequest "SpaceCenter" "WaypointManager_AddWaypoint" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 bodyArg, makeArgument 4 nameArg]
    in  makeStream req

waypointManagerAddWaypointStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> m (KRPCStream (KRPCHS.SpaceCenter.Waypoint))
waypointManagerAddWaypointStream thisArg latitudeArg longitudeArg bodyArg nameArg = requestStream $ waypointManagerAddWaypointStreamReq thisArg latitudeArg longitudeArg bodyArg nameArg 

{-
 - An example map of known color - seed pairs.
 - Any other integers may be used as seed.
 -}
getWaypointManagerColors :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> m (Data.Map.Map (Data.Text.Text) (Data.Int.Int32))
getWaypointManagerColors thisArg = do
    let r = makeRequest "SpaceCenter" "WaypointManager_get_Colors" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointManagerColorsStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Data.Int.Int32))
getWaypointManagerColorsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "WaypointManager_get_Colors" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointManagerColorsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> m (KRPCStream (Data.Map.Map (Data.Text.Text) (Data.Int.Int32)))
getWaypointManagerColorsStream thisArg = requestStream $ getWaypointManagerColorsStreamReq thisArg 

{-
 - Returns all available icons (from "GameData/Squad/Contracts/Icons/").
 -}
getWaypointManagerIcons :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> m ([Data.Text.Text])
getWaypointManagerIcons thisArg = do
    let r = makeRequest "SpaceCenter" "WaypointManager_get_Icons" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointManagerIconsStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCStreamReq ([Data.Text.Text])
getWaypointManagerIconsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "WaypointManager_get_Icons" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointManagerIconsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> m (KRPCStream ([Data.Text.Text]))
getWaypointManagerIconsStream thisArg = requestStream $ getWaypointManagerIconsStreamReq thisArg 

{-
 - A list of all existing waypoints.
 -}
getWaypointManagerWaypoints :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> m ([KRPCHS.SpaceCenter.Waypoint])
getWaypointManagerWaypoints thisArg = do
    let r = makeRequest "SpaceCenter" "WaypointManager_get_Waypoints" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointManagerWaypointsStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Waypoint])
getWaypointManagerWaypointsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "WaypointManager_get_Waypoints" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointManagerWaypointsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.WaypointManager -> m (KRPCStream ([KRPCHS.SpaceCenter.Waypoint]))
getWaypointManagerWaypointsStream thisArg = requestStream $ getWaypointManagerWaypointsStreamReq thisArg 

{-
 - Removes the waypoint.
 -}
waypointRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m ()
waypointRemove thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The altitude of the waypoint above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 -}
getWaypointBedrockAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Double)
getWaypointBedrockAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_BedrockAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointBedrockAltitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointBedrockAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_BedrockAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointBedrockAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Double))
getWaypointBedrockAltitudeStream thisArg = requestStream $ getWaypointBedrockAltitudeStreamReq thisArg 

{-
 - Celestial body the waypoint is attached to.
 -}
getWaypointBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCHS.SpaceCenter.CelestialBody)
getWaypointBody thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Body" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointBodyStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getWaypointBodyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Body" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointBodyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getWaypointBodyStream thisArg = requestStream $ getWaypointBodyStreamReq thisArg 

{-
 - True if this waypoint is part of a set of clustered waypoints with greek letter names appended (Alpha, Beta, Gamma, etc).
 - If true, there is a one-to-one correspondence with the greek letter name and the <see cref="M:SpaceCenter.Waypoint.Index" />.
 -}
getWaypointClustered :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Bool)
getWaypointClustered thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Clustered" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointClusteredStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointClusteredStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Clustered" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointClusteredStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Bool))
getWaypointClusteredStream thisArg = requestStream $ getWaypointClusteredStreamReq thisArg 

{-
 - The seed of the icon color. See <see cref="M:SpaceCenter.WaypointManager.Colors" /> for example colors.
 -}
getWaypointColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Data.Int.Int32)
getWaypointColor thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointColorStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Int.Int32)
getWaypointColorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Color" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointColorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Data.Int.Int32))
getWaypointColorStream thisArg = requestStream $ getWaypointColorStreamReq thisArg 

{-
 - The associated contract.
 -}
getWaypointContract :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCHS.SpaceCenter.Contract)
getWaypointContract thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Contract" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointContractStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (KRPCHS.SpaceCenter.Contract)
getWaypointContractStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Contract" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointContractStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (KRPCHS.SpaceCenter.Contract))
getWaypointContractStream thisArg = requestStream $ getWaypointContractStreamReq thisArg 

{-
 - True if waypoint is actually glued to the ground.
 -}
getWaypointGrounded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Bool)
getWaypointGrounded thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Grounded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointGroundedStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointGroundedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Grounded" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointGroundedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Bool))
getWaypointGroundedStream thisArg = requestStream $ getWaypointGroundedStreamReq thisArg 

{-
 - Whether the waypoint belongs to a contract.
 -}
getWaypointHasContract :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Bool)
getWaypointHasContract thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_HasContract" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointHasContractStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointHasContractStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_HasContract" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointHasContractStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Bool))
getWaypointHasContractStream thisArg = requestStream $ getWaypointHasContractStreamReq thisArg 

{-
 - The icon of the waypoint.
 -}
getWaypointIcon :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Data.Text.Text)
getWaypointIcon thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Icon" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointIconStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Text.Text)
getWaypointIconStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Icon" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointIconStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Data.Text.Text))
getWaypointIconStream thisArg = requestStream $ getWaypointIconStreamReq thisArg 

{-
 - The integer index of this waypoint amongst its cluster of sibling waypoints.
 - In other words, when you have a cluster of waypoints called "Somewhere Alpha", "Somewhere Beta", and "Somewhere Gamma",
 - then the alpha site has index 0, the beta site has index 1 and the gamma site has index 2.
 - When <see cref="M:SpaceCenter.Waypoint.Clustered" /> is false, this value is zero but meaningless.
 -}
getWaypointIndex :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Data.Int.Int32)
getWaypointIndex thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Index" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointIndexStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Int.Int32)
getWaypointIndexStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Index" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointIndexStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Data.Int.Int32))
getWaypointIndexStream thisArg = requestStream $ getWaypointIndexStreamReq thisArg 

{-
 - The latitude of the waypoint.
 -}
getWaypointLatitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Double)
getWaypointLatitude thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Latitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointLatitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointLatitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Latitude" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointLatitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Double))
getWaypointLatitudeStream thisArg = requestStream $ getWaypointLatitudeStreamReq thisArg 

{-
 - The longitude of the waypoint.
 -}
getWaypointLongitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Double)
getWaypointLongitude thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Longitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointLongitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointLongitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Longitude" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointLongitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Double))
getWaypointLongitudeStream thisArg = requestStream $ getWaypointLongitudeStreamReq thisArg 

{-
 - The altitude of the waypoint above sea level, in meters.
 -}
getWaypointMeanAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Double)
getWaypointMeanAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_MeanAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointMeanAltitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointMeanAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_MeanAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointMeanAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Double))
getWaypointMeanAltitudeStream thisArg = requestStream $ getWaypointMeanAltitudeStreamReq thisArg 

{-
 - Name of the waypoint as it appears on the map and the contract.
 -}
getWaypointName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Data.Text.Text)
getWaypointName thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointNameStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Text.Text)
getWaypointNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointNameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Data.Text.Text))
getWaypointNameStream thisArg = requestStream $ getWaypointNameStreamReq thisArg 

{-
 - True if waypoint is a point near or on the body rather than high in orbit.
 -}
getWaypointNearSurface :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Bool)
getWaypointNearSurface thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_NearSurface" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointNearSurfaceStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointNearSurfaceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_NearSurface" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointNearSurfaceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Bool))
getWaypointNearSurfaceStream thisArg = requestStream $ getWaypointNearSurfaceStreamReq thisArg 

{-
 - The altitude of the waypoint above the surface of the body or sea level, whichever is closer, in meters.
 -}
getWaypointSurfaceAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (Double)
getWaypointSurfaceAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_get_SurfaceAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWaypointSurfaceAltitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointSurfaceAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Waypoint_get_SurfaceAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getWaypointSurfaceAltitudeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> m (KRPCStream (Double))
getWaypointSurfaceAltitudeStream thisArg = requestStream $ getWaypointSurfaceAltitudeStreamReq thisArg 

{-
 - The altitude of the waypoint above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 -}
setWaypointBedrockAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Double -> m ()
setWaypointBedrockAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_BedrockAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Celestial body the waypoint is attached to.
 -}
setWaypointBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> KRPCHS.SpaceCenter.CelestialBody -> m ()
setWaypointBody thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_Body" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The seed of the icon color. See <see cref="M:SpaceCenter.WaypointManager.Colors" /> for example colors.
 -}
setWaypointColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Data.Int.Int32 -> m ()
setWaypointColor thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The icon of the waypoint.
 -}
setWaypointIcon :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Data.Text.Text -> m ()
setWaypointIcon thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_Icon" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The latitude of the waypoint.
 -}
setWaypointLatitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Double -> m ()
setWaypointLatitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_Latitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The longitude of the waypoint.
 -}
setWaypointLongitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Double -> m ()
setWaypointLongitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_Longitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The altitude of the waypoint above sea level, in meters.
 -}
setWaypointMeanAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Double -> m ()
setWaypointMeanAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_MeanAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Name of the waypoint as it appears on the map and the contract.
 -}
setWaypointName :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Data.Text.Text -> m ()
setWaypointName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The altitude of the waypoint above the surface of the body or sea level, whichever is closer, in meters.
 -}
setWaypointSurfaceAltitude :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Waypoint -> Double -> m ()
setWaypointSurfaceAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Waypoint_set_SurfaceAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether automatic friction control is enabled.
 -}
getWheelAutoFrictionControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelAutoFrictionControl thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_AutoFrictionControl" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelAutoFrictionControlStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelAutoFrictionControlStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_AutoFrictionControl" [makeArgument 0 thisArg]
    in  makeStream req

getWheelAutoFrictionControlStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelAutoFrictionControlStream thisArg = requestStream $ getWheelAutoFrictionControlStreamReq thisArg 

{-
 - The braking force, as a percentage of maximum, when the brakes are applied.
 -}
getWheelBrakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelBrakes thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Brakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelBrakesStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelBrakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Brakes" [makeArgument 0 thisArg]
    in  makeStream req

getWheelBrakesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelBrakesStream thisArg = requestStream $ getWheelBrakesStreamReq thisArg 

{-
 - Whether the wheel is broken.
 -}
getWheelBroken :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelBroken thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Broken" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelBrokenStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelBrokenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Broken" [makeArgument 0 thisArg]
    in  makeStream req

getWheelBrokenStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelBrokenStream thisArg = requestStream $ getWheelBrokenStreamReq thisArg 

{-
 - Current deflection of the wheel.
 -}
getWheelDeflection :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelDeflection thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Deflection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelDeflectionStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelDeflectionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Deflection" [makeArgument 0 thisArg]
    in  makeStream req

getWheelDeflectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelDeflectionStream thisArg = requestStream $ getWheelDeflectionStreamReq thisArg 

{-
 - Whether the wheel is deployable.
 -}
getWheelDeployable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelDeployableStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getWheelDeployableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelDeployableStream thisArg = requestStream $ getWheelDeployableStreamReq thisArg 

{-
 - Whether the wheel is deployed.
 -}
getWheelDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelDeployedStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getWheelDeployedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelDeployedStream thisArg = requestStream $ getWheelDeployedStreamReq thisArg 

{-
 - Manual setting for the motor limiter.
 - Only takes effect if the wheel has automatic traction control disabled.
 - A value between 0 and 100 inclusive.
 -}
getWheelDriveLimiter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelDriveLimiter thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_DriveLimiter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelDriveLimiterStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelDriveLimiterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_DriveLimiter" [makeArgument 0 thisArg]
    in  makeStream req

getWheelDriveLimiterStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelDriveLimiterStream thisArg = requestStream $ getWheelDriveLimiterStreamReq thisArg 

{-
 - Whether the wheel is touching the ground.
 -}
getWheelGrounded :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelGrounded thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Grounded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelGroundedStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelGroundedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Grounded" [makeArgument 0 thisArg]
    in  makeStream req

getWheelGroundedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelGroundedStream thisArg = requestStream $ getWheelGroundedStreamReq thisArg 

{-
 - Whether the wheel has brakes.
 -}
getWheelHasBrakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelHasBrakes thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_HasBrakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelHasBrakesStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelHasBrakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_HasBrakes" [makeArgument 0 thisArg]
    in  makeStream req

getWheelHasBrakesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelHasBrakesStream thisArg = requestStream $ getWheelHasBrakesStreamReq thisArg 

{-
 - Whether the wheel has suspension.
 -}
getWheelHasSuspension :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelHasSuspension thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_HasSuspension" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelHasSuspensionStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelHasSuspensionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_HasSuspension" [makeArgument 0 thisArg]
    in  makeStream req

getWheelHasSuspensionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelHasSuspensionStream thisArg = requestStream $ getWheelHasSuspensionStreamReq thisArg 

{-
 - Manual friction control value. Only has an effect if automatic friction control is disabled.
 - A value between 0 and 5 inclusive.
 -}
getWheelManualFrictionControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelManualFrictionControl thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_ManualFrictionControl" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelManualFrictionControlStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelManualFrictionControlStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_ManualFrictionControl" [makeArgument 0 thisArg]
    in  makeStream req

getWheelManualFrictionControlStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelManualFrictionControlStream thisArg = requestStream $ getWheelManualFrictionControlStreamReq thisArg 

{-
 - Whether the motor is enabled.
 -}
getWheelMotorEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelMotorEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_MotorEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelMotorEnabledStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelMotorEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_MotorEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getWheelMotorEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelMotorEnabledStream thisArg = requestStream $ getWheelMotorEnabledStreamReq thisArg 

{-
 - Whether the direction of the motor is inverted.
 -}
getWheelMotorInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelMotorInverted thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_MotorInverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelMotorInvertedStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelMotorInvertedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_MotorInverted" [makeArgument 0 thisArg]
    in  makeStream req

getWheelMotorInvertedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelMotorInvertedStream thisArg = requestStream $ getWheelMotorInvertedStreamReq thisArg 

{-
 - The output of the motor. This is the torque currently being generated, in Newton meters.
 -}
getWheelMotorOutput :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelMotorOutput thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_MotorOutput" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelMotorOutputStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelMotorOutputStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_MotorOutput" [makeArgument 0 thisArg]
    in  makeStream req

getWheelMotorOutputStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelMotorOutputStream thisArg = requestStream $ getWheelMotorOutputStreamReq thisArg 

{-
 - Whether the direction of the motor is inverted.
 -}
getWheelMotorState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCHS.SpaceCenter.MotorState)
getWheelMotorState thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_MotorState" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelMotorStateStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (KRPCHS.SpaceCenter.MotorState)
getWheelMotorStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_MotorState" [makeArgument 0 thisArg]
    in  makeStream req

getWheelMotorStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (KRPCHS.SpaceCenter.MotorState))
getWheelMotorStateStream thisArg = requestStream $ getWheelMotorStateStreamReq thisArg 

{-
 - The part object for this wheel.
 -}
getWheelPart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCHS.SpaceCenter.Part)
getWheelPart thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelPartStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getWheelPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getWheelPartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (KRPCHS.SpaceCenter.Part))
getWheelPartStream thisArg = requestStream $ getWheelPartStreamReq thisArg 

{-
 - Whether the wheel is powered by a motor.
 -}
getWheelPowered :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelPowered thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Powered" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelPoweredStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelPoweredStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Powered" [makeArgument 0 thisArg]
    in  makeStream req

getWheelPoweredStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelPoweredStream thisArg = requestStream $ getWheelPoweredStreamReq thisArg 

{-
 - Radius of the wheel, in meters.
 -}
getWheelRadius :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelRadius thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Radius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelRadiusStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelRadiusStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Radius" [makeArgument 0 thisArg]
    in  makeStream req

getWheelRadiusStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelRadiusStream thisArg = requestStream $ getWheelRadiusStreamReq thisArg 

{-
 - Whether the wheel is repairable.
 -}
getWheelRepairable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelRepairable thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Repairable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelRepairableStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelRepairableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Repairable" [makeArgument 0 thisArg]
    in  makeStream req

getWheelRepairableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelRepairableStream thisArg = requestStream $ getWheelRepairableStreamReq thisArg 

{-
 - Current slip of the wheel.
 -}
getWheelSlip :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelSlip thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Slip" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelSlipStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelSlipStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Slip" [makeArgument 0 thisArg]
    in  makeStream req

getWheelSlipStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelSlipStream thisArg = requestStream $ getWheelSlipStreamReq thisArg 

{-
 - The current state of the wheel.
 -}
getWheelState :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCHS.SpaceCenter.WheelState)
getWheelState thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelStateStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (KRPCHS.SpaceCenter.WheelState)
getWheelStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getWheelStateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (KRPCHS.SpaceCenter.WheelState))
getWheelStateStream thisArg = requestStream $ getWheelStateStreamReq thisArg 

{-
 - Whether the wheel has steering.
 -}
getWheelSteerable :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelSteerable thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Steerable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelSteerableStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelSteerableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Steerable" [makeArgument 0 thisArg]
    in  makeStream req

getWheelSteerableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelSteerableStream thisArg = requestStream $ getWheelSteerableStreamReq thisArg 

{-
 - Whether the wheel steering is enabled.
 -}
getWheelSteeringEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelSteeringEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_SteeringEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelSteeringEnabledStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelSteeringEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_SteeringEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getWheelSteeringEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelSteeringEnabledStream thisArg = requestStream $ getWheelSteeringEnabledStreamReq thisArg 

{-
 - Whether the wheel steering is inverted.
 -}
getWheelSteeringInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelSteeringInverted thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_SteeringInverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelSteeringInvertedStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelSteeringInvertedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_SteeringInverted" [makeArgument 0 thisArg]
    in  makeStream req

getWheelSteeringInvertedStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelSteeringInvertedStream thisArg = requestStream $ getWheelSteeringInvertedStreamReq thisArg 

{-
 - Current stress on the wheel.
 -}
getWheelStress :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelStress thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_Stress" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelStressStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelStressStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_Stress" [makeArgument 0 thisArg]
    in  makeStream req

getWheelStressStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelStressStream thisArg = requestStream $ getWheelStressStreamReq thisArg 

{-
 - Current stress on the wheel as a percentage of its stress tolerance.
 -}
getWheelStressPercentage :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelStressPercentage thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_StressPercentage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelStressPercentageStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelStressPercentageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_StressPercentage" [makeArgument 0 thisArg]
    in  makeStream req

getWheelStressPercentageStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelStressPercentageStream thisArg = requestStream $ getWheelStressPercentageStreamReq thisArg 

{-
 - Stress tolerance of the wheel.
 -}
getWheelStressTolerance :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelStressTolerance thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_StressTolerance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelStressToleranceStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelStressToleranceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_StressTolerance" [makeArgument 0 thisArg]
    in  makeStream req

getWheelStressToleranceStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelStressToleranceStream thisArg = requestStream $ getWheelStressToleranceStreamReq thisArg 

{-
 - Suspension damper strength, as set in the editor.
 -}
getWheelSuspensionDamperStrength :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelSuspensionDamperStrength thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_SuspensionDamperStrength" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelSuspensionDamperStrengthStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelSuspensionDamperStrengthStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_SuspensionDamperStrength" [makeArgument 0 thisArg]
    in  makeStream req

getWheelSuspensionDamperStrengthStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelSuspensionDamperStrengthStream thisArg = requestStream $ getWheelSuspensionDamperStrengthStreamReq thisArg 

{-
 - Suspension spring strength, as set in the editor.
 -}
getWheelSuspensionSpringStrength :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelSuspensionSpringStrength thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_SuspensionSpringStrength" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelSuspensionSpringStrengthStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelSuspensionSpringStrengthStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_SuspensionSpringStrength" [makeArgument 0 thisArg]
    in  makeStream req

getWheelSuspensionSpringStrengthStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelSuspensionSpringStrengthStream thisArg = requestStream $ getWheelSuspensionSpringStrengthStreamReq thisArg 

{-
 - Setting for the traction control.
 - Only takes effect if the wheel has automatic traction control enabled.
 - A value between 0 and 5 inclusive.
 -}
getWheelTractionControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Float)
getWheelTractionControl thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_TractionControl" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelTractionControlStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Float)
getWheelTractionControlStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_TractionControl" [makeArgument 0 thisArg]
    in  makeStream req

getWheelTractionControlStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Float))
getWheelTractionControlStream thisArg = requestStream $ getWheelTractionControlStreamReq thisArg 

{-
 - Whether automatic traction control is enabled.
 - A wheel only has traction control if it is powered.
 -}
getWheelTractionControlEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (Bool)
getWheelTractionControlEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Wheel_get_TractionControlEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getWheelTractionControlEnabledStreamReq :: KRPCHS.SpaceCenter.Wheel -> KRPCStreamReq (Bool)
getWheelTractionControlEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Wheel_get_TractionControlEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getWheelTractionControlEnabledStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> m (KRPCStream (Bool))
getWheelTractionControlEnabledStream thisArg = requestStream $ getWheelTractionControlEnabledStreamReq thisArg 

{-
 - Whether automatic friction control is enabled.
 -}
setWheelAutoFrictionControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelAutoFrictionControl thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_AutoFrictionControl" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The braking force, as a percentage of maximum, when the brakes are applied.
 -}
setWheelBrakes :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Float -> m ()
setWheelBrakes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_Brakes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the wheel is deployed.
 -}
setWheelDeployed :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Manual setting for the motor limiter.
 - Only takes effect if the wheel has automatic traction control disabled.
 - A value between 0 and 100 inclusive.
 -}
setWheelDriveLimiter :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Float -> m ()
setWheelDriveLimiter thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_DriveLimiter" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Manual friction control value. Only has an effect if automatic friction control is disabled.
 - A value between 0 and 5 inclusive.
 -}
setWheelManualFrictionControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Float -> m ()
setWheelManualFrictionControl thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_ManualFrictionControl" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the motor is enabled.
 -}
setWheelMotorEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelMotorEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_MotorEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the direction of the motor is inverted.
 -}
setWheelMotorInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelMotorInverted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_MotorInverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the wheel steering is enabled.
 -}
setWheelSteeringEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelSteeringEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_SteeringEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the wheel steering is inverted.
 -}
setWheelSteeringInverted :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelSteeringInverted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_SteeringInverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Setting for the traction control.
 - Only takes effect if the wheel has automatic traction control enabled.
 - A value between 0 and 5 inclusive.
 -}
setWheelTractionControl :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Float -> m ()
setWheelTractionControl thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_TractionControl" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether automatic traction control is enabled.
 - A wheel only has traction control if it is powered.
 -}
setWheelTractionControlEnabled :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Wheel -> Bool -> m ()
setWheelTractionControlEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Wheel_set_TractionControlEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently active vessel.
 -}
getActiveVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.Vessel)
getActiveVessel  = do
    let r = makeRequest "SpaceCenter" "get_ActiveVessel" []
    res <- sendRequest r
    processResponse res

getActiveVesselStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getActiveVesselStreamReq  =
    let req = makeRequest "SpaceCenter" "get_ActiveVessel" []
    in  makeStream req

getActiveVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getActiveVesselStream  = requestStream $ getActiveVesselStreamReq  

{-
 - A dictionary of all celestial bodies (planets, moons, etc.) in the game,
 - keyed by the name of the body.
 -}
getBodies :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodies  = do
    let r = makeRequest "SpaceCenter" "get_Bodies" []
    res <- sendRequest r
    processResponse res

getBodiesStreamReq :: KRPCStreamReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodiesStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Bodies" []
    in  makeStream req

getBodiesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody)))
getBodiesStream  = requestStream $ getBodiesStreamReq  

{-
 - An object that can be used to control the camera.
 -}
getCamera :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.Camera)
getCamera  = do
    let r = makeRequest "SpaceCenter" "get_Camera" []
    res <- sendRequest r
    processResponse res

getCameraStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Camera)
getCameraStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Camera" []
    in  makeStream req

getCameraStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.Camera))
getCameraStream  = requestStream $ getCameraStreamReq  

{-
 - The contract manager.
 -}
getContractManager :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.ContractManager)
getContractManager  = do
    let r = makeRequest "SpaceCenter" "get_ContractManager" []
    res <- sendRequest r
    processResponse res

getContractManagerStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.ContractManager)
getContractManagerStreamReq  =
    let req = makeRequest "SpaceCenter" "get_ContractManager" []
    in  makeStream req

getContractManagerStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.ContractManager))
getContractManagerStream  = requestStream $ getContractManagerStreamReq  

{-
 - Whether <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFARAvailable :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Bool)
getFARAvailable  = do
    let r = makeRequest "SpaceCenter" "get_FARAvailable" []
    res <- sendRequest r
    processResponse res

getFARAvailableStreamReq :: KRPCStreamReq (Bool)
getFARAvailableStreamReq  =
    let req = makeRequest "SpaceCenter" "get_FARAvailable" []
    in  makeStream req

getFARAvailableStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Bool))
getFARAvailableStream  = requestStream $ getFARAvailableStreamReq  

{-
 - The value of the <a href="https://en.wikipedia.org/wiki/Gravitational_constant">gravitational constantG inN(m/kg)^2.
 -}
getG :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Double)
getG  = do
    let r = makeRequest "SpaceCenter" "get_G" []
    res <- sendRequest r
    processResponse res

getGStreamReq :: KRPCStreamReq (Double)
getGStreamReq  =
    let req = makeRequest "SpaceCenter" "get_G" []
    in  makeStream req

getGStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Double))
getGStream  = requestStream $ getGStreamReq  

{-
 - The current maximum regular "on-rails" warp factor that can be set.
 - A value between 0 and 7 inclusive. See
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">the KSP wikifor details.
 -}
getMaximumRailsWarpFactor :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Data.Int.Int32)
getMaximumRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" []
    res <- sendRequest r
    processResponse res

getMaximumRailsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getMaximumRailsWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" []
    in  makeStream req

getMaximumRailsWarpFactorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Data.Int.Int32))
getMaximumRailsWarpFactorStream  = requestStream $ getMaximumRailsWarpFactorStreamReq  

{-
 - Whether the navball is visible.
 -}
getNavball :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Bool)
getNavball  = do
    let r = makeRequest "SpaceCenter" "get_Navball" []
    res <- sendRequest r
    processResponse res

getNavballStreamReq :: KRPCStreamReq (Bool)
getNavballStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Navball" []
    in  makeStream req

getNavballStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Bool))
getNavballStream  = requestStream $ getNavballStreamReq  

{-
 - The physical time warp rate. A value between 0 and 3 inclusive. 0 means
 - no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
getPhysicsWarpFactor :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Data.Int.Int32)
getPhysicsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" []
    res <- sendRequest r
    processResponse res

getPhysicsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getPhysicsWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" []
    in  makeStream req

getPhysicsWarpFactorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Data.Int.Int32))
getPhysicsWarpFactorStream  = requestStream $ getPhysicsWarpFactorStreamReq  

{-
 - The time warp rate, using regular "on-rails" time warp. A value between
 - 0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
 - is active.
 - 
 - If requested time warp factor cannot be set, it will be set to the next
 - lowest possible value. For example, if the vessel is too close to a
 - planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
 - the KSP wikifor details.
 -}
getRailsWarpFactor :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Data.Int.Int32)
getRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_RailsWarpFactor" []
    res <- sendRequest r
    processResponse res

getRailsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getRailsWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_RailsWarpFactor" []
    in  makeStream req

getRailsWarpFactorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Data.Int.Int32))
getRailsWarpFactorStream  = requestStream $ getRailsWarpFactorStreamReq  

{-
 - The currently targeted celestial body.
 -}
getTargetBody :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.CelestialBody)
getTargetBody  = do
    let r = makeRequest "SpaceCenter" "get_TargetBody" []
    res <- sendRequest r
    processResponse res

getTargetBodyStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getTargetBodyStreamReq  =
    let req = makeRequest "SpaceCenter" "get_TargetBody" []
    in  makeStream req

getTargetBodyStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getTargetBodyStream  = requestStream $ getTargetBodyStreamReq  

{-
 - The currently targeted docking port.
 -}
getTargetDockingPort :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPort  = do
    let r = makeRequest "SpaceCenter" "get_TargetDockingPort" []
    res <- sendRequest r
    processResponse res

getTargetDockingPortStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPortStreamReq  =
    let req = makeRequest "SpaceCenter" "get_TargetDockingPort" []
    in  makeStream req

getTargetDockingPortStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getTargetDockingPortStream  = requestStream $ getTargetDockingPortStreamReq  

{-
 - The currently targeted vessel.
 -}
getTargetVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.Vessel)
getTargetVessel  = do
    let r = makeRequest "SpaceCenter" "get_TargetVessel" []
    res <- sendRequest r
    processResponse res

getTargetVesselStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getTargetVesselStreamReq  =
    let req = makeRequest "SpaceCenter" "get_TargetVessel" []
    in  makeStream req

getTargetVesselStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getTargetVesselStream  = requestStream $ getTargetVesselStreamReq  

{-
 - Whether the UI is visible.
 -}
getUIVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Bool)
getUIVisible  = do
    let r = makeRequest "SpaceCenter" "get_UIVisible" []
    res <- sendRequest r
    processResponse res

getUIVisibleStreamReq :: KRPCStreamReq (Bool)
getUIVisibleStreamReq  =
    let req = makeRequest "SpaceCenter" "get_UIVisible" []
    in  makeStream req

getUIVisibleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Bool))
getUIVisibleStream  = requestStream $ getUIVisibleStreamReq  

{-
 - The current universal time in seconds.
 -}
getUT :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Double)
getUT  = do
    let r = makeRequest "SpaceCenter" "get_UT" []
    res <- sendRequest r
    processResponse res

getUTStreamReq :: KRPCStreamReq (Double)
getUTStreamReq  =
    let req = makeRequest "SpaceCenter" "get_UT" []
    in  makeStream req

getUTStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Double))
getUTStream  = requestStream $ getUTStreamReq  

{-
 - A list of all the vessels in the game.
 -}
getVessels :: (MonadIO m, MonadThrow m, MonadRPC m) => m ([KRPCHS.SpaceCenter.Vessel])
getVessels  = do
    let r = makeRequest "SpaceCenter" "get_Vessels" []
    res <- sendRequest r
    processResponse res

getVesselsStreamReq :: KRPCStreamReq ([KRPCHS.SpaceCenter.Vessel])
getVesselsStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Vessels" []
    in  makeStream req

getVesselsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
getVesselsStream  = requestStream $ getVesselsStreamReq  

{-
 - The current warp factor. This is the index of the rate at which time
 - is passing for either regular "on-rails" or physical time warp. Returns 0
 - if time warp is not active. When in on-rails time warp, this is equal to
 - <see cref="M:SpaceCenter.RailsWarpFactor" />, and in physics time warp, this is equal to
 - <see cref="M:SpaceCenter.PhysicsWarpFactor" />.
 -}
getWarpFactor :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Float)
getWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_WarpFactor" []
    res <- sendRequest r
    processResponse res

getWarpFactorStreamReq :: KRPCStreamReq (Float)
getWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WarpFactor" []
    in  makeStream req

getWarpFactorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Float))
getWarpFactorStream  = requestStream $ getWarpFactorStreamReq  

{-
 - The current time warp mode. Returns <see cref="M:SpaceCenter.WarpMode.None" /> if time
 - warp is not active, <see cref="M:SpaceCenter.WarpMode.Rails" /> if regular "on-rails" time warp
 - is active, or <see cref="M:SpaceCenter.WarpMode.Physics" /> if physical time warp is active.
 -}
getWarpMode :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.WarpMode)
getWarpMode  = do
    let r = makeRequest "SpaceCenter" "get_WarpMode" []
    res <- sendRequest r
    processResponse res

getWarpModeStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.WarpMode)
getWarpModeStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WarpMode" []
    in  makeStream req

getWarpModeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.WarpMode))
getWarpModeStream  = requestStream $ getWarpModeStreamReq  

{-
 - The current warp rate. This is the rate at which time is passing for
 - either on-rails or physical time warp. For example, a value of 10 means
 - time is passing 10x faster than normal. Returns 1 if time warp is not
 - active.
 -}
getWarpRate :: (MonadIO m, MonadThrow m, MonadRPC m) => m (Float)
getWarpRate  = do
    let r = makeRequest "SpaceCenter" "get_WarpRate" []
    res <- sendRequest r
    processResponse res

getWarpRateStreamReq :: KRPCStreamReq (Float)
getWarpRateStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WarpRate" []
    in  makeStream req

getWarpRateStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (Float))
getWarpRateStream  = requestStream $ getWarpRateStreamReq  

{-
 - The waypoint manager.
 -}
getWaypointManager :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCHS.SpaceCenter.WaypointManager)
getWaypointManager  = do
    let r = makeRequest "SpaceCenter" "get_WaypointManager" []
    res <- sendRequest r
    processResponse res

getWaypointManagerStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.WaypointManager)
getWaypointManagerStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WaypointManager" []
    in  makeStream req

getWaypointManagerStream :: (MonadIO m, MonadThrow m, MonadRPC m) => m (KRPCStream (KRPCHS.SpaceCenter.WaypointManager))
getWaypointManagerStream  = requestStream $ getWaypointManagerStreamReq  

{-
 - The currently active vessel.
 -}
setActiveVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m ()
setActiveVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_ActiveVessel" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the navball is visible.
 -}
setNavball :: (MonadIO m, MonadThrow m, MonadRPC m) => Bool -> m ()
setNavball valueArg = do
    let r = makeRequest "SpaceCenter" "set_Navball" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The physical time warp rate. A value between 0 and 3 inclusive. 0 means
 - no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
setPhysicsWarpFactor :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Int.Int32 -> m ()
setPhysicsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_PhysicsWarpFactor" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time warp rate, using regular "on-rails" time warp. A value between
 - 0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
 - is active.
 - 
 - If requested time warp factor cannot be set, it will be set to the next
 - lowest possible value. For example, if the vessel is too close to a
 - planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
 - the KSP wikifor details.
 -}
setRailsWarpFactor :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Int.Int32 -> m ()
setRailsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_RailsWarpFactor" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently targeted celestial body.
 -}
setTargetBody :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.CelestialBody -> m ()
setTargetBody valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetBody" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently targeted docking port.
 -}
setTargetDockingPort :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.DockingPort -> m ()
setTargetDockingPort valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetDockingPort" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently targeted vessel.
 -}
setTargetVessel :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.SpaceCenter.Vessel -> m ()
setTargetVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetVessel" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the UI is visible.
 -}
setUIVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => Bool -> m ()
setUIVisible valueArg = do
    let r = makeRequest "SpaceCenter" "set_UIVisible" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

