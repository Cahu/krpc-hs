module KRPCHS.SpaceCenter
( CameraMode(..)
, CargoBayState(..)
, DockingPortState(..)
, LandingGearState(..)
, LandingLegState(..)
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
, AutoPilot
, Camera
, CargoBay
, CelestialBody
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
, LandingGear
, LandingLeg
, LaunchClamp
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
, autoPilotDisengage
, autoPilotDisengageReq
, autoPilotEngage
, autoPilotEngageReq
, autoPilotTargetPitchAndHeading
, autoPilotTargetPitchAndHeadingReq
, autoPilotWait
, autoPilotWaitReq
, getAutoPilotAttenuationAngle
, getAutoPilotAttenuationAngleReq
, getAutoPilotAttenuationAngleStream
, getAutoPilotAttenuationAngleStreamReq
, getAutoPilotAutoTune
, getAutoPilotAutoTuneReq
, getAutoPilotAutoTuneStream
, getAutoPilotAutoTuneStreamReq
, getAutoPilotDecelerationTime
, getAutoPilotDecelerationTimeReq
, getAutoPilotDecelerationTimeStream
, getAutoPilotDecelerationTimeStreamReq
, getAutoPilotError
, getAutoPilotErrorReq
, getAutoPilotErrorStream
, getAutoPilotErrorStreamReq
, getAutoPilotHeadingError
, getAutoPilotHeadingErrorReq
, getAutoPilotHeadingErrorStream
, getAutoPilotHeadingErrorStreamReq
, getAutoPilotOvershoot
, getAutoPilotOvershootReq
, getAutoPilotOvershootStream
, getAutoPilotOvershootStreamReq
, getAutoPilotPitchError
, getAutoPilotPitchErrorReq
, getAutoPilotPitchErrorStream
, getAutoPilotPitchErrorStreamReq
, getAutoPilotPitchPIDGains
, getAutoPilotPitchPIDGainsReq
, getAutoPilotPitchPIDGainsStream
, getAutoPilotPitchPIDGainsStreamReq
, getAutoPilotReferenceFrame
, getAutoPilotReferenceFrameReq
, getAutoPilotReferenceFrameStream
, getAutoPilotReferenceFrameStreamReq
, getAutoPilotRollError
, getAutoPilotRollErrorReq
, getAutoPilotRollErrorStream
, getAutoPilotRollErrorStreamReq
, getAutoPilotRollPIDGains
, getAutoPilotRollPIDGainsReq
, getAutoPilotRollPIDGainsStream
, getAutoPilotRollPIDGainsStreamReq
, getAutoPilotRollThreshold
, getAutoPilotRollThresholdReq
, getAutoPilotRollThresholdStream
, getAutoPilotRollThresholdStreamReq
, getAutoPilotSAS
, getAutoPilotSASReq
, getAutoPilotSASStream
, getAutoPilotSASStreamReq
, getAutoPilotSASMode
, getAutoPilotSASModeReq
, getAutoPilotSASModeStream
, getAutoPilotSASModeStreamReq
, getAutoPilotStoppingTime
, getAutoPilotStoppingTimeReq
, getAutoPilotStoppingTimeStream
, getAutoPilotStoppingTimeStreamReq
, getAutoPilotTargetDirection
, getAutoPilotTargetDirectionReq
, getAutoPilotTargetDirectionStream
, getAutoPilotTargetDirectionStreamReq
, getAutoPilotTargetHeading
, getAutoPilotTargetHeadingReq
, getAutoPilotTargetHeadingStream
, getAutoPilotTargetHeadingStreamReq
, getAutoPilotTargetPitch
, getAutoPilotTargetPitchReq
, getAutoPilotTargetPitchStream
, getAutoPilotTargetPitchStreamReq
, getAutoPilotTargetRoll
, getAutoPilotTargetRollReq
, getAutoPilotTargetRollStream
, getAutoPilotTargetRollStreamReq
, getAutoPilotTimeToPeak
, getAutoPilotTimeToPeakReq
, getAutoPilotTimeToPeakStream
, getAutoPilotTimeToPeakStreamReq
, getAutoPilotYawPIDGains
, getAutoPilotYawPIDGainsReq
, getAutoPilotYawPIDGainsStream
, getAutoPilotYawPIDGainsStreamReq
, setAutoPilotAttenuationAngle
, setAutoPilotAttenuationAngleReq
, setAutoPilotAutoTune
, setAutoPilotAutoTuneReq
, setAutoPilotDecelerationTime
, setAutoPilotDecelerationTimeReq
, setAutoPilotOvershoot
, setAutoPilotOvershootReq
, setAutoPilotPitchPIDGains
, setAutoPilotPitchPIDGainsReq
, setAutoPilotReferenceFrame
, setAutoPilotReferenceFrameReq
, setAutoPilotRollPIDGains
, setAutoPilotRollPIDGainsReq
, setAutoPilotRollThreshold
, setAutoPilotRollThresholdReq
, setAutoPilotSAS
, setAutoPilotSASReq
, setAutoPilotSASMode
, setAutoPilotSASModeReq
, setAutoPilotStoppingTime
, setAutoPilotStoppingTimeReq
, setAutoPilotTargetDirection
, setAutoPilotTargetDirectionReq
, setAutoPilotTargetHeading
, setAutoPilotTargetHeadingReq
, setAutoPilotTargetPitch
, setAutoPilotTargetPitchReq
, setAutoPilotTargetRoll
, setAutoPilotTargetRollReq
, setAutoPilotTimeToPeak
, setAutoPilotTimeToPeakReq
, setAutoPilotYawPIDGains
, setAutoPilotYawPIDGainsReq
, getCameraDefaultDistance
, getCameraDefaultDistanceReq
, getCameraDefaultDistanceStream
, getCameraDefaultDistanceStreamReq
, getCameraDistance
, getCameraDistanceReq
, getCameraDistanceStream
, getCameraDistanceStreamReq
, getCameraFocussedBody
, getCameraFocussedBodyReq
, getCameraFocussedBodyStream
, getCameraFocussedBodyStreamReq
, getCameraFocussedNode
, getCameraFocussedNodeReq
, getCameraFocussedNodeStream
, getCameraFocussedNodeStreamReq
, getCameraFocussedVessel
, getCameraFocussedVesselReq
, getCameraFocussedVesselStream
, getCameraFocussedVesselStreamReq
, getCameraHeading
, getCameraHeadingReq
, getCameraHeadingStream
, getCameraHeadingStreamReq
, getCameraMaxDistance
, getCameraMaxDistanceReq
, getCameraMaxDistanceStream
, getCameraMaxDistanceStreamReq
, getCameraMaxPitch
, getCameraMaxPitchReq
, getCameraMaxPitchStream
, getCameraMaxPitchStreamReq
, getCameraMinDistance
, getCameraMinDistanceReq
, getCameraMinDistanceStream
, getCameraMinDistanceStreamReq
, getCameraMinPitch
, getCameraMinPitchReq
, getCameraMinPitchStream
, getCameraMinPitchStreamReq
, getCameraMode
, getCameraModeReq
, getCameraModeStream
, getCameraModeStreamReq
, getCameraPitch
, getCameraPitchReq
, getCameraPitchStream
, getCameraPitchStreamReq
, setCameraDistance
, setCameraDistanceReq
, setCameraFocussedBody
, setCameraFocussedBodyReq
, setCameraFocussedNode
, setCameraFocussedNodeReq
, setCameraFocussedVessel
, setCameraFocussedVesselReq
, setCameraHeading
, setCameraHeadingReq
, setCameraMode
, setCameraModeReq
, setCameraPitch
, setCameraPitchReq
, canRailsWarpAt
, canRailsWarpAtReq
, canRailsWarpAtStream
, canRailsWarpAtStreamReq
, getCargoBayOpen
, getCargoBayOpenReq
, getCargoBayOpenStream
, getCargoBayOpenStreamReq
, getCargoBayPart
, getCargoBayPartReq
, getCargoBayPartStream
, getCargoBayPartStreamReq
, getCargoBayState
, getCargoBayStateReq
, getCargoBayStateStream
, getCargoBayStateStreamReq
, setCargoBayOpen
, setCargoBayOpenReq
, celestialBodyAngularVelocity
, celestialBodyAngularVelocityReq
, celestialBodyAngularVelocityStream
, celestialBodyAngularVelocityStreamReq
, celestialBodyBedrockHeight
, celestialBodyBedrockHeightReq
, celestialBodyBedrockHeightStream
, celestialBodyBedrockHeightStreamReq
, celestialBodyBedrockPosition
, celestialBodyBedrockPositionReq
, celestialBodyBedrockPositionStream
, celestialBodyBedrockPositionStreamReq
, celestialBodyBiomeAt
, celestialBodyBiomeAtReq
, celestialBodyBiomeAtStream
, celestialBodyBiomeAtStreamReq
, celestialBodyDirection
, celestialBodyDirectionReq
, celestialBodyDirectionStream
, celestialBodyDirectionStreamReq
, celestialBodyMSLPosition
, celestialBodyMSLPositionReq
, celestialBodyMSLPositionStream
, celestialBodyMSLPositionStreamReq
, celestialBodyPosition
, celestialBodyPositionReq
, celestialBodyPositionStream
, celestialBodyPositionStreamReq
, celestialBodyRotation
, celestialBodyRotationReq
, celestialBodyRotationStream
, celestialBodyRotationStreamReq
, celestialBodySurfaceHeight
, celestialBodySurfaceHeightReq
, celestialBodySurfaceHeightStream
, celestialBodySurfaceHeightStreamReq
, celestialBodySurfacePosition
, celestialBodySurfacePositionReq
, celestialBodySurfacePositionStream
, celestialBodySurfacePositionStreamReq
, celestialBodyVelocity
, celestialBodyVelocityReq
, celestialBodyVelocityStream
, celestialBodyVelocityStreamReq
, getCelestialBodyAtmosphereDepth
, getCelestialBodyAtmosphereDepthReq
, getCelestialBodyAtmosphereDepthStream
, getCelestialBodyAtmosphereDepthStreamReq
, getCelestialBodyBiomes
, getCelestialBodyBiomesReq
, getCelestialBodyBiomesStream
, getCelestialBodyBiomesStreamReq
, getCelestialBodyEquatorialRadius
, getCelestialBodyEquatorialRadiusReq
, getCelestialBodyEquatorialRadiusStream
, getCelestialBodyEquatorialRadiusStreamReq
, getCelestialBodyFlyingHighAltitudeThreshold
, getCelestialBodyFlyingHighAltitudeThresholdReq
, getCelestialBodyFlyingHighAltitudeThresholdStream
, getCelestialBodyFlyingHighAltitudeThresholdStreamReq
, getCelestialBodyGravitationalParameter
, getCelestialBodyGravitationalParameterReq
, getCelestialBodyGravitationalParameterStream
, getCelestialBodyGravitationalParameterStreamReq
, getCelestialBodyHasAtmosphere
, getCelestialBodyHasAtmosphereReq
, getCelestialBodyHasAtmosphereStream
, getCelestialBodyHasAtmosphereStreamReq
, getCelestialBodyHasAtmosphericOxygen
, getCelestialBodyHasAtmosphericOxygenReq
, getCelestialBodyHasAtmosphericOxygenStream
, getCelestialBodyHasAtmosphericOxygenStreamReq
, getCelestialBodyMass
, getCelestialBodyMassReq
, getCelestialBodyMassStream
, getCelestialBodyMassStreamReq
, getCelestialBodyName
, getCelestialBodyNameReq
, getCelestialBodyNameStream
, getCelestialBodyNameStreamReq
, getCelestialBodyNonRotatingReferenceFrame
, getCelestialBodyNonRotatingReferenceFrameReq
, getCelestialBodyNonRotatingReferenceFrameStream
, getCelestialBodyNonRotatingReferenceFrameStreamReq
, getCelestialBodyOrbit
, getCelestialBodyOrbitReq
, getCelestialBodyOrbitStream
, getCelestialBodyOrbitStreamReq
, getCelestialBodyOrbitalReferenceFrame
, getCelestialBodyOrbitalReferenceFrameReq
, getCelestialBodyOrbitalReferenceFrameStream
, getCelestialBodyOrbitalReferenceFrameStreamReq
, getCelestialBodyReferenceFrame
, getCelestialBodyReferenceFrameReq
, getCelestialBodyReferenceFrameStream
, getCelestialBodyReferenceFrameStreamReq
, getCelestialBodyRotationalPeriod
, getCelestialBodyRotationalPeriodReq
, getCelestialBodyRotationalPeriodStream
, getCelestialBodyRotationalPeriodStreamReq
, getCelestialBodyRotationalSpeed
, getCelestialBodyRotationalSpeedReq
, getCelestialBodyRotationalSpeedStream
, getCelestialBodyRotationalSpeedStreamReq
, getCelestialBodySatellites
, getCelestialBodySatellitesReq
, getCelestialBodySatellitesStream
, getCelestialBodySatellitesStreamReq
, getCelestialBodySpaceHighAltitudeThreshold
, getCelestialBodySpaceHighAltitudeThresholdReq
, getCelestialBodySpaceHighAltitudeThresholdStream
, getCelestialBodySpaceHighAltitudeThresholdStreamReq
, getCelestialBodySphereOfInfluence
, getCelestialBodySphereOfInfluenceReq
, getCelestialBodySphereOfInfluenceStream
, getCelestialBodySphereOfInfluenceStreamReq
, getCelestialBodySurfaceGravity
, getCelestialBodySurfaceGravityReq
, getCelestialBodySurfaceGravityStream
, getCelestialBodySurfaceGravityStreamReq
, clearTarget
, clearTargetReq
, getControlSurfaceAvailableTorque
, getControlSurfaceAvailableTorqueReq
, getControlSurfaceAvailableTorqueStream
, getControlSurfaceAvailableTorqueStreamReq
, getControlSurfaceDeployed
, getControlSurfaceDeployedReq
, getControlSurfaceDeployedStream
, getControlSurfaceDeployedStreamReq
, getControlSurfaceInverted
, getControlSurfaceInvertedReq
, getControlSurfaceInvertedStream
, getControlSurfaceInvertedStreamReq
, getControlSurfacePart
, getControlSurfacePartReq
, getControlSurfacePartStream
, getControlSurfacePartStreamReq
, getControlSurfacePitchEnabled
, getControlSurfacePitchEnabledReq
, getControlSurfacePitchEnabledStream
, getControlSurfacePitchEnabledStreamReq
, getControlSurfaceRollEnabled
, getControlSurfaceRollEnabledReq
, getControlSurfaceRollEnabledStream
, getControlSurfaceRollEnabledStreamReq
, getControlSurfaceSurfaceArea
, getControlSurfaceSurfaceAreaReq
, getControlSurfaceSurfaceAreaStream
, getControlSurfaceSurfaceAreaStreamReq
, getControlSurfaceYawEnabled
, getControlSurfaceYawEnabledReq
, getControlSurfaceYawEnabledStream
, getControlSurfaceYawEnabledStreamReq
, setControlSurfaceDeployed
, setControlSurfaceDeployedReq
, setControlSurfaceInverted
, setControlSurfaceInvertedReq
, setControlSurfacePitchEnabled
, setControlSurfacePitchEnabledReq
, setControlSurfaceRollEnabled
, setControlSurfaceRollEnabledReq
, setControlSurfaceYawEnabled
, setControlSurfaceYawEnabledReq
, controlActivateNextStage
, controlActivateNextStageReq
, controlActivateNextStageStream
, controlActivateNextStageStreamReq
, controlAddNode
, controlAddNodeReq
, controlAddNodeStream
, controlAddNodeStreamReq
, controlGetActionGroup
, controlGetActionGroupReq
, controlGetActionGroupStream
, controlGetActionGroupStreamReq
, controlRemoveNodes
, controlRemoveNodesReq
, controlSetActionGroup
, controlSetActionGroupReq
, controlToggleActionGroup
, controlToggleActionGroupReq
, getControlAbort
, getControlAbortReq
, getControlAbortStream
, getControlAbortStreamReq
, getControlBrakes
, getControlBrakesReq
, getControlBrakesStream
, getControlBrakesStreamReq
, getControlCurrentStage
, getControlCurrentStageReq
, getControlCurrentStageStream
, getControlCurrentStageStreamReq
, getControlForward
, getControlForwardReq
, getControlForwardStream
, getControlForwardStreamReq
, getControlGear
, getControlGearReq
, getControlGearStream
, getControlGearStreamReq
, getControlLights
, getControlLightsReq
, getControlLightsStream
, getControlLightsStreamReq
, getControlNodes
, getControlNodesReq
, getControlNodesStream
, getControlNodesStreamReq
, getControlPitch
, getControlPitchReq
, getControlPitchStream
, getControlPitchStreamReq
, getControlRCS
, getControlRCSReq
, getControlRCSStream
, getControlRCSStreamReq
, getControlRight
, getControlRightReq
, getControlRightStream
, getControlRightStreamReq
, getControlRoll
, getControlRollReq
, getControlRollStream
, getControlRollStreamReq
, getControlSAS
, getControlSASReq
, getControlSASStream
, getControlSASStreamReq
, getControlSASMode
, getControlSASModeReq
, getControlSASModeStream
, getControlSASModeStreamReq
, getControlSpeedMode
, getControlSpeedModeReq
, getControlSpeedModeStream
, getControlSpeedModeStreamReq
, getControlThrottle
, getControlThrottleReq
, getControlThrottleStream
, getControlThrottleStreamReq
, getControlUp
, getControlUpReq
, getControlUpStream
, getControlUpStreamReq
, getControlWheelSteering
, getControlWheelSteeringReq
, getControlWheelSteeringStream
, getControlWheelSteeringStreamReq
, getControlWheelThrottle
, getControlWheelThrottleReq
, getControlWheelThrottleStream
, getControlWheelThrottleStreamReq
, getControlYaw
, getControlYawReq
, getControlYawStream
, getControlYawStreamReq
, setControlAbort
, setControlAbortReq
, setControlBrakes
, setControlBrakesReq
, setControlForward
, setControlForwardReq
, setControlGear
, setControlGearReq
, setControlLights
, setControlLightsReq
, setControlPitch
, setControlPitchReq
, setControlRCS
, setControlRCSReq
, setControlRight
, setControlRightReq
, setControlRoll
, setControlRollReq
, setControlSAS
, setControlSASReq
, setControlSASMode
, setControlSASModeReq
, setControlSpeedMode
, setControlSpeedModeReq
, setControlThrottle
, setControlThrottleReq
, setControlUp
, setControlUpReq
, setControlWheelSteering
, setControlWheelSteeringReq
, setControlWheelThrottle
, setControlWheelThrottleReq
, setControlYaw
, setControlYawReq
, decouplerDecouple
, decouplerDecoupleReq
, decouplerDecoupleStream
, decouplerDecoupleStreamReq
, getDecouplerDecoupled
, getDecouplerDecoupledReq
, getDecouplerDecoupledStream
, getDecouplerDecoupledStreamReq
, getDecouplerImpulse
, getDecouplerImpulseReq
, getDecouplerImpulseStream
, getDecouplerImpulseStreamReq
, getDecouplerPart
, getDecouplerPartReq
, getDecouplerPartStream
, getDecouplerPartStreamReq
, getDecouplerStaged
, getDecouplerStagedReq
, getDecouplerStagedStream
, getDecouplerStagedStreamReq
, dockingPortDirection
, dockingPortDirectionReq
, dockingPortDirectionStream
, dockingPortDirectionStreamReq
, dockingPortPosition
, dockingPortPositionReq
, dockingPortPositionStream
, dockingPortPositionStreamReq
, dockingPortRotation
, dockingPortRotationReq
, dockingPortRotationStream
, dockingPortRotationStreamReq
, dockingPortUndock
, dockingPortUndockReq
, dockingPortUndockStream
, dockingPortUndockStreamReq
, getDockingPortDockedPart
, getDockingPortDockedPartReq
, getDockingPortDockedPartStream
, getDockingPortDockedPartStreamReq
, getDockingPortHasShield
, getDockingPortHasShieldReq
, getDockingPortHasShieldStream
, getDockingPortHasShieldStreamReq
, getDockingPortPart
, getDockingPortPartReq
, getDockingPortPartStream
, getDockingPortPartStreamReq
, getDockingPortReengageDistance
, getDockingPortReengageDistanceReq
, getDockingPortReengageDistanceStream
, getDockingPortReengageDistanceStreamReq
, getDockingPortReferenceFrame
, getDockingPortReferenceFrameReq
, getDockingPortReferenceFrameStream
, getDockingPortReferenceFrameStreamReq
, getDockingPortShielded
, getDockingPortShieldedReq
, getDockingPortShieldedStream
, getDockingPortShieldedStreamReq
, getDockingPortState
, getDockingPortStateReq
, getDockingPortStateStream
, getDockingPortStateStreamReq
, setDockingPortShielded
, setDockingPortShieldedReq
, engineToggleMode
, engineToggleModeReq
, getEngineActive
, getEngineActiveReq
, getEngineActiveStream
, getEngineActiveStreamReq
, getEngineAutoModeSwitch
, getEngineAutoModeSwitchReq
, getEngineAutoModeSwitchStream
, getEngineAutoModeSwitchStreamReq
, getEngineAvailableThrust
, getEngineAvailableThrustReq
, getEngineAvailableThrustStream
, getEngineAvailableThrustStreamReq
, getEngineAvailableTorque
, getEngineAvailableTorqueReq
, getEngineAvailableTorqueStream
, getEngineAvailableTorqueStreamReq
, getEngineCanRestart
, getEngineCanRestartReq
, getEngineCanRestartStream
, getEngineCanRestartStreamReq
, getEngineCanShutdown
, getEngineCanShutdownReq
, getEngineCanShutdownStream
, getEngineCanShutdownStreamReq
, getEngineGimbalLimit
, getEngineGimbalLimitReq
, getEngineGimbalLimitStream
, getEngineGimbalLimitStreamReq
, getEngineGimbalLocked
, getEngineGimbalLockedReq
, getEngineGimbalLockedStream
, getEngineGimbalLockedStreamReq
, getEngineGimbalRange
, getEngineGimbalRangeReq
, getEngineGimbalRangeStream
, getEngineGimbalRangeStreamReq
, getEngineGimballed
, getEngineGimballedReq
, getEngineGimballedStream
, getEngineGimballedStreamReq
, getEngineHasFuel
, getEngineHasFuelReq
, getEngineHasFuelStream
, getEngineHasFuelStreamReq
, getEngineHasModes
, getEngineHasModesReq
, getEngineHasModesStream
, getEngineHasModesStreamReq
, getEngineKerbinSeaLevelSpecificImpulse
, getEngineKerbinSeaLevelSpecificImpulseReq
, getEngineKerbinSeaLevelSpecificImpulseStream
, getEngineKerbinSeaLevelSpecificImpulseStreamReq
, getEngineMaxThrust
, getEngineMaxThrustReq
, getEngineMaxThrustStream
, getEngineMaxThrustStreamReq
, getEngineMaxVacuumThrust
, getEngineMaxVacuumThrustReq
, getEngineMaxVacuumThrustStream
, getEngineMaxVacuumThrustStreamReq
, getEngineMode
, getEngineModeReq
, getEngineModeStream
, getEngineModeStreamReq
, getEngineModes
, getEngineModesReq
, getEngineModesStream
, getEngineModesStreamReq
, getEnginePart
, getEnginePartReq
, getEnginePartStream
, getEnginePartStreamReq
, getEnginePropellantNames
, getEnginePropellantNamesReq
, getEnginePropellantNamesStream
, getEnginePropellantNamesStreamReq
, getEnginePropellantRatios
, getEnginePropellantRatiosReq
, getEnginePropellantRatiosStream
, getEnginePropellantRatiosStreamReq
, getEnginePropellants
, getEnginePropellantsReq
, getEnginePropellantsStream
, getEnginePropellantsStreamReq
, getEngineSpecificImpulse
, getEngineSpecificImpulseReq
, getEngineSpecificImpulseStream
, getEngineSpecificImpulseStreamReq
, getEngineThrottle
, getEngineThrottleReq
, getEngineThrottleStream
, getEngineThrottleStreamReq
, getEngineThrottleLocked
, getEngineThrottleLockedReq
, getEngineThrottleLockedStream
, getEngineThrottleLockedStreamReq
, getEngineThrust
, getEngineThrustReq
, getEngineThrustStream
, getEngineThrustStreamReq
, getEngineThrustLimit
, getEngineThrustLimitReq
, getEngineThrustLimitStream
, getEngineThrustLimitStreamReq
, getEngineThrusters
, getEngineThrustersReq
, getEngineThrustersStream
, getEngineThrustersStreamReq
, getEngineVacuumSpecificImpulse
, getEngineVacuumSpecificImpulseReq
, getEngineVacuumSpecificImpulseStream
, getEngineVacuumSpecificImpulseStreamReq
, setEngineActive
, setEngineActiveReq
, setEngineAutoModeSwitch
, setEngineAutoModeSwitchReq
, setEngineGimbalLimit
, setEngineGimbalLimitReq
, setEngineGimbalLocked
, setEngineGimbalLockedReq
, setEngineMode
, setEngineModeReq
, setEngineThrustLimit
, setEngineThrustLimitReq
, experimentDump
, experimentDumpReq
, experimentReset
, experimentResetReq
, experimentRun
, experimentRunReq
, experimentTransmit
, experimentTransmitReq
, getExperimentAvailable
, getExperimentAvailableReq
, getExperimentAvailableStream
, getExperimentAvailableStreamReq
, getExperimentBiome
, getExperimentBiomeReq
, getExperimentBiomeStream
, getExperimentBiomeStreamReq
, getExperimentData
, getExperimentDataReq
, getExperimentDataStream
, getExperimentDataStreamReq
, getExperimentDeployed
, getExperimentDeployedReq
, getExperimentDeployedStream
, getExperimentDeployedStreamReq
, getExperimentHasData
, getExperimentHasDataReq
, getExperimentHasDataStream
, getExperimentHasDataStreamReq
, getExperimentInoperable
, getExperimentInoperableReq
, getExperimentInoperableStream
, getExperimentInoperableStreamReq
, getExperimentPart
, getExperimentPartReq
, getExperimentPartStream
, getExperimentPartStreamReq
, getExperimentRerunnable
, getExperimentRerunnableReq
, getExperimentRerunnableStream
, getExperimentRerunnableStreamReq
, getExperimentScienceSubject
, getExperimentScienceSubjectReq
, getExperimentScienceSubjectStream
, getExperimentScienceSubjectStreamReq
, fairingJettison
, fairingJettisonReq
, getFairingJettisoned
, getFairingJettisonedReq
, getFairingJettisonedStream
, getFairingJettisonedStreamReq
, getFairingPart
, getFairingPartReq
, getFairingPartStream
, getFairingPartStreamReq
, getFlightAerodynamicForce
, getFlightAerodynamicForceReq
, getFlightAerodynamicForceStream
, getFlightAerodynamicForceStreamReq
, getFlightAngleOfAttack
, getFlightAngleOfAttackReq
, getFlightAngleOfAttackStream
, getFlightAngleOfAttackStreamReq
, getFlightAntiNormal
, getFlightAntiNormalReq
, getFlightAntiNormalStream
, getFlightAntiNormalStreamReq
, getFlightAntiRadial
, getFlightAntiRadialReq
, getFlightAntiRadialStream
, getFlightAntiRadialStreamReq
, getFlightAtmosphereDensity
, getFlightAtmosphereDensityReq
, getFlightAtmosphereDensityStream
, getFlightAtmosphereDensityStreamReq
, getFlightBallisticCoefficient
, getFlightBallisticCoefficientReq
, getFlightBallisticCoefficientStream
, getFlightBallisticCoefficientStreamReq
, getFlightBedrockAltitude
, getFlightBedrockAltitudeReq
, getFlightBedrockAltitudeStream
, getFlightBedrockAltitudeStreamReq
, getFlightCenterOfMass
, getFlightCenterOfMassReq
, getFlightCenterOfMassStream
, getFlightCenterOfMassStreamReq
, getFlightDirection
, getFlightDirectionReq
, getFlightDirectionStream
, getFlightDirectionStreamReq
, getFlightDrag
, getFlightDragReq
, getFlightDragStream
, getFlightDragStreamReq
, getFlightDragCoefficient
, getFlightDragCoefficientReq
, getFlightDragCoefficientStream
, getFlightDragCoefficientStreamReq
, getFlightDynamicPressure
, getFlightDynamicPressureReq
, getFlightDynamicPressureStream
, getFlightDynamicPressureStreamReq
, getFlightElevation
, getFlightElevationReq
, getFlightElevationStream
, getFlightElevationStreamReq
, getFlightEquivalentAirSpeed
, getFlightEquivalentAirSpeedReq
, getFlightEquivalentAirSpeedStream
, getFlightEquivalentAirSpeedStreamReq
, getFlightGForce
, getFlightGForceReq
, getFlightGForceStream
, getFlightGForceStreamReq
, getFlightHeading
, getFlightHeadingReq
, getFlightHeadingStream
, getFlightHeadingStreamReq
, getFlightHorizontalSpeed
, getFlightHorizontalSpeedReq
, getFlightHorizontalSpeedStream
, getFlightHorizontalSpeedStreamReq
, getFlightLatitude
, getFlightLatitudeReq
, getFlightLatitudeStream
, getFlightLatitudeStreamReq
, getFlightLift
, getFlightLiftReq
, getFlightLiftStream
, getFlightLiftStreamReq
, getFlightLiftCoefficient
, getFlightLiftCoefficientReq
, getFlightLiftCoefficientStream
, getFlightLiftCoefficientStreamReq
, getFlightLongitude
, getFlightLongitudeReq
, getFlightLongitudeStream
, getFlightLongitudeStreamReq
, getFlightMach
, getFlightMachReq
, getFlightMachStream
, getFlightMachStreamReq
, getFlightMeanAltitude
, getFlightMeanAltitudeReq
, getFlightMeanAltitudeStream
, getFlightMeanAltitudeStreamReq
, getFlightNormal
, getFlightNormalReq
, getFlightNormalStream
, getFlightNormalStreamReq
, getFlightPitch
, getFlightPitchReq
, getFlightPitchStream
, getFlightPitchStreamReq
, getFlightPrograde
, getFlightProgradeReq
, getFlightProgradeStream
, getFlightProgradeStreamReq
, getFlightRadial
, getFlightRadialReq
, getFlightRadialStream
, getFlightRadialStreamReq
, getFlightRetrograde
, getFlightRetrogradeReq
, getFlightRetrogradeStream
, getFlightRetrogradeStreamReq
, getFlightReynoldsNumber
, getFlightReynoldsNumberReq
, getFlightReynoldsNumberStream
, getFlightReynoldsNumberStreamReq
, getFlightRoll
, getFlightRollReq
, getFlightRollStream
, getFlightRollStreamReq
, getFlightRotation
, getFlightRotationReq
, getFlightRotationStream
, getFlightRotationStreamReq
, getFlightSideslipAngle
, getFlightSideslipAngleReq
, getFlightSideslipAngleStream
, getFlightSideslipAngleStreamReq
, getFlightSpeed
, getFlightSpeedReq
, getFlightSpeedStream
, getFlightSpeedStreamReq
, getFlightSpeedOfSound
, getFlightSpeedOfSoundReq
, getFlightSpeedOfSoundStream
, getFlightSpeedOfSoundStreamReq
, getFlightStallFraction
, getFlightStallFractionReq
, getFlightStallFractionStream
, getFlightStallFractionStreamReq
, getFlightStaticAirTemperature
, getFlightStaticAirTemperatureReq
, getFlightStaticAirTemperatureStream
, getFlightStaticAirTemperatureStreamReq
, getFlightStaticPressure
, getFlightStaticPressureReq
, getFlightStaticPressureStream
, getFlightStaticPressureStreamReq
, getFlightStaticPressureAtMSL
, getFlightStaticPressureAtMSLReq
, getFlightStaticPressureAtMSLStream
, getFlightStaticPressureAtMSLStreamReq
, getFlightSurfaceAltitude
, getFlightSurfaceAltitudeReq
, getFlightSurfaceAltitudeStream
, getFlightSurfaceAltitudeStreamReq
, getFlightTerminalVelocity
, getFlightTerminalVelocityReq
, getFlightTerminalVelocityStream
, getFlightTerminalVelocityStreamReq
, getFlightThrustSpecificFuelConsumption
, getFlightThrustSpecificFuelConsumptionReq
, getFlightThrustSpecificFuelConsumptionStream
, getFlightThrustSpecificFuelConsumptionStreamReq
, getFlightTotalAirTemperature
, getFlightTotalAirTemperatureReq
, getFlightTotalAirTemperatureStream
, getFlightTotalAirTemperatureStreamReq
, getFlightTrueAirSpeed
, getFlightTrueAirSpeedReq
, getFlightTrueAirSpeedStream
, getFlightTrueAirSpeedStreamReq
, getFlightVelocity
, getFlightVelocityReq
, getFlightVelocityStream
, getFlightVelocityStreamReq
, getFlightVerticalSpeed
, getFlightVerticalSpeedReq
, getFlightVerticalSpeedStream
, getFlightVerticalSpeedStreamReq
, forceRemove
, forceRemoveReq
, getForceForceVector
, getForceForceVectorReq
, getForceForceVectorStream
, getForceForceVectorStreamReq
, getForcePart
, getForcePartReq
, getForcePartStream
, getForcePartStreamReq
, getForcePosition
, getForcePositionReq
, getForcePositionStream
, getForcePositionStreamReq
, getForceReferenceFrame
, getForceReferenceFrameReq
, getForceReferenceFrameStream
, getForceReferenceFrameStreamReq
, setForceForceVector
, setForceForceVectorReq
, setForcePosition
, setForcePositionReq
, setForceReferenceFrame
, setForceReferenceFrameReq
, getIntakeArea
, getIntakeAreaReq
, getIntakeAreaStream
, getIntakeAreaStreamReq
, getIntakeFlow
, getIntakeFlowReq
, getIntakeFlowStream
, getIntakeFlowStreamReq
, getIntakeOpen
, getIntakeOpenReq
, getIntakeOpenStream
, getIntakeOpenStreamReq
, getIntakePart
, getIntakePartReq
, getIntakePartStream
, getIntakePartStreamReq
, getIntakeSpeed
, getIntakeSpeedReq
, getIntakeSpeedStream
, getIntakeSpeedStreamReq
, setIntakeOpen
, setIntakeOpenReq
, getLandingGearDeployable
, getLandingGearDeployableReq
, getLandingGearDeployableStream
, getLandingGearDeployableStreamReq
, getLandingGearDeployed
, getLandingGearDeployedReq
, getLandingGearDeployedStream
, getLandingGearDeployedStreamReq
, getLandingGearPart
, getLandingGearPartReq
, getLandingGearPartStream
, getLandingGearPartStreamReq
, getLandingGearState
, getLandingGearStateReq
, getLandingGearStateStream
, getLandingGearStateStreamReq
, setLandingGearDeployed
, setLandingGearDeployedReq
, getLandingLegDeployed
, getLandingLegDeployedReq
, getLandingLegDeployedStream
, getLandingLegDeployedStreamReq
, getLandingLegPart
, getLandingLegPartReq
, getLandingLegPartStream
, getLandingLegPartStreamReq
, getLandingLegState
, getLandingLegStateReq
, getLandingLegStateStream
, getLandingLegStateStreamReq
, setLandingLegDeployed
, setLandingLegDeployedReq
, launchClampRelease
, launchClampReleaseReq
, getLaunchClampPart
, getLaunchClampPartReq
, getLaunchClampPartStream
, getLaunchClampPartStreamReq
, launchVessel
, launchVesselReq
, launchVesselFromSPH
, launchVesselFromSPHReq
, launchVesselFromVAB
, launchVesselFromVABReq
, launchableVessels
, launchableVesselsReq
, launchableVesselsStream
, launchableVesselsStreamReq
, getLightActive
, getLightActiveReq
, getLightActiveStream
, getLightActiveStreamReq
, getLightColor
, getLightColorReq
, getLightColorStream
, getLightColorStreamReq
, getLightPart
, getLightPartReq
, getLightPartStream
, getLightPartStreamReq
, getLightPowerUsage
, getLightPowerUsageReq
, getLightPowerUsageStream
, getLightPowerUsageStreamReq
, setLightActive
, setLightActiveReq
, setLightColor
, setLightColorReq
, load
, loadReq
, moduleGetField
, moduleGetFieldReq
, moduleGetFieldStream
, moduleGetFieldStreamReq
, moduleHasAction
, moduleHasActionReq
, moduleHasActionStream
, moduleHasActionStreamReq
, moduleHasEvent
, moduleHasEventReq
, moduleHasEventStream
, moduleHasEventStreamReq
, moduleHasField
, moduleHasFieldReq
, moduleHasFieldStream
, moduleHasFieldStreamReq
, moduleResetField
, moduleResetFieldReq
, moduleSetAction
, moduleSetActionReq
, moduleSetFieldFloat
, moduleSetFieldFloatReq
, moduleSetFieldInt
, moduleSetFieldIntReq
, moduleSetFieldString
, moduleSetFieldStringReq
, moduleTriggerEvent
, moduleTriggerEventReq
, getModuleActions
, getModuleActionsReq
, getModuleActionsStream
, getModuleActionsStreamReq
, getModuleEvents
, getModuleEventsReq
, getModuleEventsStream
, getModuleEventsStreamReq
, getModuleFields
, getModuleFieldsReq
, getModuleFieldsStream
, getModuleFieldsStreamReq
, getModuleName
, getModuleNameReq
, getModuleNameStream
, getModuleNameStreamReq
, getModulePart
, getModulePartReq
, getModulePartStream
, getModulePartStreamReq
, nodeBurnVector
, nodeBurnVectorReq
, nodeBurnVectorStream
, nodeBurnVectorStreamReq
, nodeDirection
, nodeDirectionReq
, nodeDirectionStream
, nodeDirectionStreamReq
, nodePosition
, nodePositionReq
, nodePositionStream
, nodePositionStreamReq
, nodeRemainingBurnVector
, nodeRemainingBurnVectorReq
, nodeRemainingBurnVectorStream
, nodeRemainingBurnVectorStreamReq
, nodeRemove
, nodeRemoveReq
, getNodeDeltaV
, getNodeDeltaVReq
, getNodeDeltaVStream
, getNodeDeltaVStreamReq
, getNodeNormal
, getNodeNormalReq
, getNodeNormalStream
, getNodeNormalStreamReq
, getNodeOrbit
, getNodeOrbitReq
, getNodeOrbitStream
, getNodeOrbitStreamReq
, getNodeOrbitalReferenceFrame
, getNodeOrbitalReferenceFrameReq
, getNodeOrbitalReferenceFrameStream
, getNodeOrbitalReferenceFrameStreamReq
, getNodePrograde
, getNodeProgradeReq
, getNodeProgradeStream
, getNodeProgradeStreamReq
, getNodeRadial
, getNodeRadialReq
, getNodeRadialStream
, getNodeRadialStreamReq
, getNodeReferenceFrame
, getNodeReferenceFrameReq
, getNodeReferenceFrameStream
, getNodeReferenceFrameStreamReq
, getNodeRemainingDeltaV
, getNodeRemainingDeltaVReq
, getNodeRemainingDeltaVStream
, getNodeRemainingDeltaVStreamReq
, getNodeTimeTo
, getNodeTimeToReq
, getNodeTimeToStream
, getNodeTimeToStreamReq
, getNodeUT
, getNodeUTReq
, getNodeUTStream
, getNodeUTStreamReq
, setNodeDeltaV
, setNodeDeltaVReq
, setNodeNormal
, setNodeNormalReq
, setNodePrograde
, setNodeProgradeReq
, setNodeRadial
, setNodeRadialReq
, setNodeUT
, setNodeUTReq
, orbitEccentricAnomalyAtUT
, orbitEccentricAnomalyAtUTReq
, orbitEccentricAnomalyAtUTStream
, orbitEccentricAnomalyAtUTStreamReq
, orbitOrbitalSpeedAt
, orbitOrbitalSpeedAtReq
, orbitOrbitalSpeedAtStream
, orbitOrbitalSpeedAtStreamReq
, orbitRadiusAtTrueAnomaly
, orbitRadiusAtTrueAnomalyReq
, orbitRadiusAtTrueAnomalyStream
, orbitRadiusAtTrueAnomalyStreamReq
, orbitTrueAnomalyAtRadius
, orbitTrueAnomalyAtRadiusReq
, orbitTrueAnomalyAtRadiusStream
, orbitTrueAnomalyAtRadiusStreamReq
, orbitTrueAnomalyAtUT
, orbitTrueAnomalyAtUTReq
, orbitTrueAnomalyAtUTStream
, orbitTrueAnomalyAtUTStreamReq
, orbitUTAtTrueAnomaly
, orbitUTAtTrueAnomalyReq
, orbitUTAtTrueAnomalyStream
, orbitUTAtTrueAnomalyStreamReq
, getOrbitApoapsis
, getOrbitApoapsisReq
, getOrbitApoapsisStream
, getOrbitApoapsisStreamReq
, getOrbitApoapsisAltitude
, getOrbitApoapsisAltitudeReq
, getOrbitApoapsisAltitudeStream
, getOrbitApoapsisAltitudeStreamReq
, getOrbitArgumentOfPeriapsis
, getOrbitArgumentOfPeriapsisReq
, getOrbitArgumentOfPeriapsisStream
, getOrbitArgumentOfPeriapsisStreamReq
, getOrbitBody
, getOrbitBodyReq
, getOrbitBodyStream
, getOrbitBodyStreamReq
, getOrbitEccentricAnomaly
, getOrbitEccentricAnomalyReq
, getOrbitEccentricAnomalyStream
, getOrbitEccentricAnomalyStreamReq
, getOrbitEccentricity
, getOrbitEccentricityReq
, getOrbitEccentricityStream
, getOrbitEccentricityStreamReq
, getOrbitEpoch
, getOrbitEpochReq
, getOrbitEpochStream
, getOrbitEpochStreamReq
, getOrbitInclination
, getOrbitInclinationReq
, getOrbitInclinationStream
, getOrbitInclinationStreamReq
, getOrbitLongitudeOfAscendingNode
, getOrbitLongitudeOfAscendingNodeReq
, getOrbitLongitudeOfAscendingNodeStream
, getOrbitLongitudeOfAscendingNodeStreamReq
, getOrbitMeanAnomaly
, getOrbitMeanAnomalyReq
, getOrbitMeanAnomalyStream
, getOrbitMeanAnomalyStreamReq
, getOrbitMeanAnomalyAtEpoch
, getOrbitMeanAnomalyAtEpochReq
, getOrbitMeanAnomalyAtEpochStream
, getOrbitMeanAnomalyAtEpochStreamReq
, getOrbitNextOrbit
, getOrbitNextOrbitReq
, getOrbitNextOrbitStream
, getOrbitNextOrbitStreamReq
, getOrbitOrbitalSpeed
, getOrbitOrbitalSpeedReq
, getOrbitOrbitalSpeedStream
, getOrbitOrbitalSpeedStreamReq
, getOrbitPeriapsis
, getOrbitPeriapsisReq
, getOrbitPeriapsisStream
, getOrbitPeriapsisStreamReq
, getOrbitPeriapsisAltitude
, getOrbitPeriapsisAltitudeReq
, getOrbitPeriapsisAltitudeStream
, getOrbitPeriapsisAltitudeStreamReq
, getOrbitPeriod
, getOrbitPeriodReq
, getOrbitPeriodStream
, getOrbitPeriodStreamReq
, getOrbitRadius
, getOrbitRadiusReq
, getOrbitRadiusStream
, getOrbitRadiusStreamReq
, getOrbitSemiMajorAxis
, getOrbitSemiMajorAxisReq
, getOrbitSemiMajorAxisStream
, getOrbitSemiMajorAxisStreamReq
, getOrbitSemiMinorAxis
, getOrbitSemiMinorAxisReq
, getOrbitSemiMinorAxisStream
, getOrbitSemiMinorAxisStreamReq
, getOrbitSpeed
, getOrbitSpeedReq
, getOrbitSpeedStream
, getOrbitSpeedStreamReq
, getOrbitTimeToApoapsis
, getOrbitTimeToApoapsisReq
, getOrbitTimeToApoapsisStream
, getOrbitTimeToApoapsisStreamReq
, getOrbitTimeToPeriapsis
, getOrbitTimeToPeriapsisReq
, getOrbitTimeToPeriapsisStream
, getOrbitTimeToPeriapsisStreamReq
, getOrbitTimeToSOIChange
, getOrbitTimeToSOIChangeReq
, getOrbitTimeToSOIChangeStream
, getOrbitTimeToSOIChangeStreamReq
, getOrbitTrueAnomaly
, getOrbitTrueAnomalyReq
, getOrbitTrueAnomalyStream
, getOrbitTrueAnomalyStreamReq
, orbitStaticReferencePlaneDirection
, orbitStaticReferencePlaneDirectionReq
, orbitStaticReferencePlaneDirectionStream
, orbitStaticReferencePlaneDirectionStreamReq
, orbitStaticReferencePlaneNormal
, orbitStaticReferencePlaneNormalReq
, orbitStaticReferencePlaneNormalStream
, orbitStaticReferencePlaneNormalStreamReq
, parachuteDeploy
, parachuteDeployReq
, getParachuteDeployAltitude
, getParachuteDeployAltitudeReq
, getParachuteDeployAltitudeStream
, getParachuteDeployAltitudeStreamReq
, getParachuteDeployMinPressure
, getParachuteDeployMinPressureReq
, getParachuteDeployMinPressureStream
, getParachuteDeployMinPressureStreamReq
, getParachuteDeployed
, getParachuteDeployedReq
, getParachuteDeployedStream
, getParachuteDeployedStreamReq
, getParachutePart
, getParachutePartReq
, getParachutePartStream
, getParachutePartStreamReq
, getParachuteState
, getParachuteStateReq
, getParachuteStateStream
, getParachuteStateStreamReq
, setParachuteDeployAltitude
, setParachuteDeployAltitudeReq
, setParachuteDeployMinPressure
, setParachuteDeployMinPressureReq
, partAddForce
, partAddForceReq
, partAddForceStream
, partAddForceStreamReq
, partCenterOfMass
, partCenterOfMassReq
, partCenterOfMassStream
, partCenterOfMassStreamReq
, partDirection
, partDirectionReq
, partDirectionStream
, partDirectionStreamReq
, partInstantaneousForce
, partInstantaneousForceReq
, partPosition
, partPositionReq
, partPositionStream
, partPositionStreamReq
, partRotation
, partRotationReq
, partRotationStream
, partRotationStreamReq
, partVelocity
, partVelocityReq
, partVelocityStream
, partVelocityStreamReq
, getPartAxiallyAttached
, getPartAxiallyAttachedReq
, getPartAxiallyAttachedStream
, getPartAxiallyAttachedStreamReq
, getPartCargoBay
, getPartCargoBayReq
, getPartCargoBayStream
, getPartCargoBayStreamReq
, getPartCenterOfMassReferenceFrame
, getPartCenterOfMassReferenceFrameReq
, getPartCenterOfMassReferenceFrameStream
, getPartCenterOfMassReferenceFrameStreamReq
, getPartChildren
, getPartChildrenReq
, getPartChildrenStream
, getPartChildrenStreamReq
, getPartControlSurface
, getPartControlSurfaceReq
, getPartControlSurfaceStream
, getPartControlSurfaceStreamReq
, getPartCost
, getPartCostReq
, getPartCostStream
, getPartCostStreamReq
, getPartCrossfeed
, getPartCrossfeedReq
, getPartCrossfeedStream
, getPartCrossfeedStreamReq
, getPartDecoupleStage
, getPartDecoupleStageReq
, getPartDecoupleStageStream
, getPartDecoupleStageStreamReq
, getPartDecoupler
, getPartDecouplerReq
, getPartDecouplerStream
, getPartDecouplerStreamReq
, getPartDockingPort
, getPartDockingPortReq
, getPartDockingPortStream
, getPartDockingPortStreamReq
, getPartDryMass
, getPartDryMassReq
, getPartDryMassStream
, getPartDryMassStreamReq
, getPartDynamicPressure
, getPartDynamicPressureReq
, getPartDynamicPressureStream
, getPartDynamicPressureStreamReq
, getPartEngine
, getPartEngineReq
, getPartEngineStream
, getPartEngineStreamReq
, getPartExperiment
, getPartExperimentReq
, getPartExperimentStream
, getPartExperimentStreamReq
, getPartFairing
, getPartFairingReq
, getPartFairingStream
, getPartFairingStreamReq
, getPartFuelLinesFrom
, getPartFuelLinesFromReq
, getPartFuelLinesFromStream
, getPartFuelLinesFromStreamReq
, getPartFuelLinesTo
, getPartFuelLinesToReq
, getPartFuelLinesToStream
, getPartFuelLinesToStreamReq
, getPartImpactTolerance
, getPartImpactToleranceReq
, getPartImpactToleranceStream
, getPartImpactToleranceStreamReq
, getPartInertiaTensor
, getPartInertiaTensorReq
, getPartInertiaTensorStream
, getPartInertiaTensorStreamReq
, getPartIntake
, getPartIntakeReq
, getPartIntakeStream
, getPartIntakeStreamReq
, getPartIsFuelLine
, getPartIsFuelLineReq
, getPartIsFuelLineStream
, getPartIsFuelLineStreamReq
, getPartLandingGear
, getPartLandingGearReq
, getPartLandingGearStream
, getPartLandingGearStreamReq
, getPartLandingLeg
, getPartLandingLegReq
, getPartLandingLegStream
, getPartLandingLegStreamReq
, getPartLaunchClamp
, getPartLaunchClampReq
, getPartLaunchClampStream
, getPartLaunchClampStreamReq
, getPartLight
, getPartLightReq
, getPartLightStream
, getPartLightStreamReq
, getPartMass
, getPartMassReq
, getPartMassStream
, getPartMassStreamReq
, getPartMassless
, getPartMasslessReq
, getPartMasslessStream
, getPartMasslessStreamReq
, getPartMaxSkinTemperature
, getPartMaxSkinTemperatureReq
, getPartMaxSkinTemperatureStream
, getPartMaxSkinTemperatureStreamReq
, getPartMaxTemperature
, getPartMaxTemperatureReq
, getPartMaxTemperatureStream
, getPartMaxTemperatureStreamReq
, getPartModules
, getPartModulesReq
, getPartModulesStream
, getPartModulesStreamReq
, getPartMomentOfInertia
, getPartMomentOfInertiaReq
, getPartMomentOfInertiaStream
, getPartMomentOfInertiaStreamReq
, getPartName
, getPartNameReq
, getPartNameStream
, getPartNameStreamReq
, getPartParachute
, getPartParachuteReq
, getPartParachuteStream
, getPartParachuteStreamReq
, getPartParent
, getPartParentReq
, getPartParentStream
, getPartParentStreamReq
, getPartRCS
, getPartRCSReq
, getPartRCSStream
, getPartRCSStreamReq
, getPartRadiallyAttached
, getPartRadiallyAttachedReq
, getPartRadiallyAttachedStream
, getPartRadiallyAttachedStreamReq
, getPartRadiator
, getPartRadiatorReq
, getPartRadiatorStream
, getPartRadiatorStreamReq
, getPartReactionWheel
, getPartReactionWheelReq
, getPartReactionWheelStream
, getPartReactionWheelStreamReq
, getPartReferenceFrame
, getPartReferenceFrameReq
, getPartReferenceFrameStream
, getPartReferenceFrameStreamReq
, getPartResourceConverter
, getPartResourceConverterReq
, getPartResourceConverterStream
, getPartResourceConverterStreamReq
, getPartResourceHarvester
, getPartResourceHarvesterReq
, getPartResourceHarvesterStream
, getPartResourceHarvesterStreamReq
, getPartResources
, getPartResourcesReq
, getPartResourcesStream
, getPartResourcesStreamReq
, getPartSensor
, getPartSensorReq
, getPartSensorStream
, getPartSensorStreamReq
, getPartShielded
, getPartShieldedReq
, getPartShieldedStream
, getPartShieldedStreamReq
, getPartSkinTemperature
, getPartSkinTemperatureReq
, getPartSkinTemperatureStream
, getPartSkinTemperatureStreamReq
, getPartSolarPanel
, getPartSolarPanelReq
, getPartSolarPanelStream
, getPartSolarPanelStreamReq
, getPartStage
, getPartStageReq
, getPartStageStream
, getPartStageStreamReq
, getPartTag
, getPartTagReq
, getPartTagStream
, getPartTagStreamReq
, getPartTemperature
, getPartTemperatureReq
, getPartTemperatureStream
, getPartTemperatureStreamReq
, getPartThermalConductionFlux
, getPartThermalConductionFluxReq
, getPartThermalConductionFluxStream
, getPartThermalConductionFluxStreamReq
, getPartThermalConvectionFlux
, getPartThermalConvectionFluxReq
, getPartThermalConvectionFluxStream
, getPartThermalConvectionFluxStreamReq
, getPartThermalInternalFlux
, getPartThermalInternalFluxReq
, getPartThermalInternalFluxStream
, getPartThermalInternalFluxStreamReq
, getPartThermalMass
, getPartThermalMassReq
, getPartThermalMassStream
, getPartThermalMassStreamReq
, getPartThermalRadiationFlux
, getPartThermalRadiationFluxReq
, getPartThermalRadiationFluxStream
, getPartThermalRadiationFluxStreamReq
, getPartThermalResourceMass
, getPartThermalResourceMassReq
, getPartThermalResourceMassStream
, getPartThermalResourceMassStreamReq
, getPartThermalSkinMass
, getPartThermalSkinMassReq
, getPartThermalSkinMassStream
, getPartThermalSkinMassStreamReq
, getPartThermalSkinToInternalFlux
, getPartThermalSkinToInternalFluxReq
, getPartThermalSkinToInternalFluxStream
, getPartThermalSkinToInternalFluxStreamReq
, getPartTitle
, getPartTitleReq
, getPartTitleStream
, getPartTitleStreamReq
, getPartVessel
, getPartVesselReq
, getPartVesselStream
, getPartVesselStreamReq
, setPartTag
, setPartTagReq
, partsInDecoupleStage
, partsInDecoupleStageReq
, partsInDecoupleStageStream
, partsInDecoupleStageStreamReq
, partsInStage
, partsInStageReq
, partsInStageStream
, partsInStageStreamReq
, partsModulesWithName
, partsModulesWithNameReq
, partsModulesWithNameStream
, partsModulesWithNameStreamReq
, partsWithModule
, partsWithModuleReq
, partsWithModuleStream
, partsWithModuleStreamReq
, partsWithName
, partsWithNameReq
, partsWithNameStream
, partsWithNameStreamReq
, partsWithTag
, partsWithTagReq
, partsWithTagStream
, partsWithTagStreamReq
, partsWithTitle
, partsWithTitleReq
, partsWithTitleStream
, partsWithTitleStreamReq
, getPartsAll
, getPartsAllReq
, getPartsAllStream
, getPartsAllStreamReq
, getPartsCargoBays
, getPartsCargoBaysReq
, getPartsCargoBaysStream
, getPartsCargoBaysStreamReq
, getPartsControlSurfaces
, getPartsControlSurfacesReq
, getPartsControlSurfacesStream
, getPartsControlSurfacesStreamReq
, getPartsControlling
, getPartsControllingReq
, getPartsControllingStream
, getPartsControllingStreamReq
, getPartsDecouplers
, getPartsDecouplersReq
, getPartsDecouplersStream
, getPartsDecouplersStreamReq
, getPartsDockingPorts
, getPartsDockingPortsReq
, getPartsDockingPortsStream
, getPartsDockingPortsStreamReq
, getPartsEngines
, getPartsEnginesReq
, getPartsEnginesStream
, getPartsEnginesStreamReq
, getPartsExperiments
, getPartsExperimentsReq
, getPartsExperimentsStream
, getPartsExperimentsStreamReq
, getPartsFairings
, getPartsFairingsReq
, getPartsFairingsStream
, getPartsFairingsStreamReq
, getPartsIntakes
, getPartsIntakesReq
, getPartsIntakesStream
, getPartsIntakesStreamReq
, getPartsLandingGear
, getPartsLandingGearReq
, getPartsLandingGearStream
, getPartsLandingGearStreamReq
, getPartsLandingLegs
, getPartsLandingLegsReq
, getPartsLandingLegsStream
, getPartsLandingLegsStreamReq
, getPartsLaunchClamps
, getPartsLaunchClampsReq
, getPartsLaunchClampsStream
, getPartsLaunchClampsStreamReq
, getPartsLights
, getPartsLightsReq
, getPartsLightsStream
, getPartsLightsStreamReq
, getPartsParachutes
, getPartsParachutesReq
, getPartsParachutesStream
, getPartsParachutesStreamReq
, getPartsRCS
, getPartsRCSReq
, getPartsRCSStream
, getPartsRCSStreamReq
, getPartsRadiators
, getPartsRadiatorsReq
, getPartsRadiatorsStream
, getPartsRadiatorsStreamReq
, getPartsReactionWheels
, getPartsReactionWheelsReq
, getPartsReactionWheelsStream
, getPartsReactionWheelsStreamReq
, getPartsResourceConverters
, getPartsResourceConvertersReq
, getPartsResourceConvertersStream
, getPartsResourceConvertersStreamReq
, getPartsResourceHarvesters
, getPartsResourceHarvestersReq
, getPartsResourceHarvestersStream
, getPartsResourceHarvestersStreamReq
, getPartsRoot
, getPartsRootReq
, getPartsRootStream
, getPartsRootStreamReq
, getPartsSensors
, getPartsSensorsReq
, getPartsSensorsStream
, getPartsSensorsStreamReq
, getPartsSolarPanels
, getPartsSolarPanelsReq
, getPartsSolarPanelsStream
, getPartsSolarPanelsStreamReq
, setPartsControlling
, setPartsControllingReq
, getPropellantConnectedResources
, getPropellantConnectedResourcesReq
, getPropellantConnectedResourcesStream
, getPropellantConnectedResourcesStreamReq
, getPropellantCurrentAmount
, getPropellantCurrentAmountReq
, getPropellantCurrentAmountStream
, getPropellantCurrentAmountStreamReq
, getPropellantCurrentRequirement
, getPropellantCurrentRequirementReq
, getPropellantCurrentRequirementStream
, getPropellantCurrentRequirementStreamReq
, getPropellantDrawStackGauge
, getPropellantDrawStackGaugeReq
, getPropellantDrawStackGaugeStream
, getPropellantDrawStackGaugeStreamReq
, getPropellantIgnoreForIsp
, getPropellantIgnoreForIspReq
, getPropellantIgnoreForIspStream
, getPropellantIgnoreForIspStreamReq
, getPropellantIgnoreForThrustCurve
, getPropellantIgnoreForThrustCurveReq
, getPropellantIgnoreForThrustCurveStream
, getPropellantIgnoreForThrustCurveStreamReq
, getPropellantIsDeprived
, getPropellantIsDeprivedReq
, getPropellantIsDeprivedStream
, getPropellantIsDeprivedStreamReq
, getPropellantName
, getPropellantNameReq
, getPropellantNameStream
, getPropellantNameStreamReq
, getPropellantRatio
, getPropellantRatioReq
, getPropellantRatioStream
, getPropellantRatioStreamReq
, getPropellantTotalResourceAvailable
, getPropellantTotalResourceAvailableReq
, getPropellantTotalResourceAvailableStream
, getPropellantTotalResourceAvailableStreamReq
, getPropellantTotalResourceCapacity
, getPropellantTotalResourceCapacityReq
, getPropellantTotalResourceCapacityStream
, getPropellantTotalResourceCapacityStreamReq
, quickload
, quickloadReq
, quicksave
, quicksaveReq
, getRCSActive
, getRCSActiveReq
, getRCSActiveStream
, getRCSActiveStreamReq
, getRCSAvailableTorque
, getRCSAvailableTorqueReq
, getRCSAvailableTorqueStream
, getRCSAvailableTorqueStreamReq
, getRCSEnabled
, getRCSEnabledReq
, getRCSEnabledStream
, getRCSEnabledStreamReq
, getRCSForwardEnabled
, getRCSForwardEnabledReq
, getRCSForwardEnabledStream
, getRCSForwardEnabledStreamReq
, getRCSHasFuel
, getRCSHasFuelReq
, getRCSHasFuelStream
, getRCSHasFuelStreamReq
, getRCSKerbinSeaLevelSpecificImpulse
, getRCSKerbinSeaLevelSpecificImpulseReq
, getRCSKerbinSeaLevelSpecificImpulseStream
, getRCSKerbinSeaLevelSpecificImpulseStreamReq
, getRCSMaxThrust
, getRCSMaxThrustReq
, getRCSMaxThrustStream
, getRCSMaxThrustStreamReq
, getRCSMaxVacuumThrust
, getRCSMaxVacuumThrustReq
, getRCSMaxVacuumThrustStream
, getRCSMaxVacuumThrustStreamReq
, getRCSPart
, getRCSPartReq
, getRCSPartStream
, getRCSPartStreamReq
, getRCSPitchEnabled
, getRCSPitchEnabledReq
, getRCSPitchEnabledStream
, getRCSPitchEnabledStreamReq
, getRCSPropellantRatios
, getRCSPropellantRatiosReq
, getRCSPropellantRatiosStream
, getRCSPropellantRatiosStreamReq
, getRCSPropellants
, getRCSPropellantsReq
, getRCSPropellantsStream
, getRCSPropellantsStreamReq
, getRCSRightEnabled
, getRCSRightEnabledReq
, getRCSRightEnabledStream
, getRCSRightEnabledStreamReq
, getRCSRollEnabled
, getRCSRollEnabledReq
, getRCSRollEnabledStream
, getRCSRollEnabledStreamReq
, getRCSSpecificImpulse
, getRCSSpecificImpulseReq
, getRCSSpecificImpulseStream
, getRCSSpecificImpulseStreamReq
, getRCSThrusters
, getRCSThrustersReq
, getRCSThrustersStream
, getRCSThrustersStreamReq
, getRCSUpEnabled
, getRCSUpEnabledReq
, getRCSUpEnabledStream
, getRCSUpEnabledStreamReq
, getRCSVacuumSpecificImpulse
, getRCSVacuumSpecificImpulseReq
, getRCSVacuumSpecificImpulseStream
, getRCSVacuumSpecificImpulseStreamReq
, getRCSYawEnabled
, getRCSYawEnabledReq
, getRCSYawEnabledStream
, getRCSYawEnabledStreamReq
, setRCSEnabled
, setRCSEnabledReq
, setRCSForwardEnabled
, setRCSForwardEnabledReq
, setRCSPitchEnabled
, setRCSPitchEnabledReq
, setRCSRightEnabled
, setRCSRightEnabledReq
, setRCSRollEnabled
, setRCSRollEnabledReq
, setRCSUpEnabled
, setRCSUpEnabledReq
, setRCSYawEnabled
, setRCSYawEnabledReq
, getRadiatorDeployable
, getRadiatorDeployableReq
, getRadiatorDeployableStream
, getRadiatorDeployableStreamReq
, getRadiatorDeployed
, getRadiatorDeployedReq
, getRadiatorDeployedStream
, getRadiatorDeployedStreamReq
, getRadiatorPart
, getRadiatorPartReq
, getRadiatorPartStream
, getRadiatorPartStreamReq
, getRadiatorState
, getRadiatorStateReq
, getRadiatorStateStream
, getRadiatorStateStreamReq
, setRadiatorDeployed
, setRadiatorDeployedReq
, getReactionWheelActive
, getReactionWheelActiveReq
, getReactionWheelActiveStream
, getReactionWheelActiveStreamReq
, getReactionWheelAvailableTorque
, getReactionWheelAvailableTorqueReq
, getReactionWheelAvailableTorqueStream
, getReactionWheelAvailableTorqueStreamReq
, getReactionWheelBroken
, getReactionWheelBrokenReq
, getReactionWheelBrokenStream
, getReactionWheelBrokenStreamReq
, getReactionWheelMaxTorque
, getReactionWheelMaxTorqueReq
, getReactionWheelMaxTorqueStream
, getReactionWheelMaxTorqueStreamReq
, getReactionWheelPart
, getReactionWheelPartReq
, getReactionWheelPartStream
, getReactionWheelPartStreamReq
, setReactionWheelActive
, setReactionWheelActiveReq
, resourceConverterActive
, resourceConverterActiveReq
, resourceConverterActiveStream
, resourceConverterActiveStreamReq
, resourceConverterInputs
, resourceConverterInputsReq
, resourceConverterInputsStream
, resourceConverterInputsStreamReq
, resourceConverterName
, resourceConverterNameReq
, resourceConverterNameStream
, resourceConverterNameStreamReq
, resourceConverterOutputs
, resourceConverterOutputsReq
, resourceConverterOutputsStream
, resourceConverterOutputsStreamReq
, resourceConverterStart
, resourceConverterStartReq
, resourceConverterState
, resourceConverterStateReq
, resourceConverterStateStream
, resourceConverterStateStreamReq
, resourceConverterStatusInfo
, resourceConverterStatusInfoReq
, resourceConverterStatusInfoStream
, resourceConverterStatusInfoStreamReq
, resourceConverterStop
, resourceConverterStopReq
, getResourceConverterCount
, getResourceConverterCountReq
, getResourceConverterCountStream
, getResourceConverterCountStreamReq
, getResourceConverterPart
, getResourceConverterPartReq
, getResourceConverterPartStream
, getResourceConverterPartStreamReq
, getResourceHarvesterActive
, getResourceHarvesterActiveReq
, getResourceHarvesterActiveStream
, getResourceHarvesterActiveStreamReq
, getResourceHarvesterCoreTemperature
, getResourceHarvesterCoreTemperatureReq
, getResourceHarvesterCoreTemperatureStream
, getResourceHarvesterCoreTemperatureStreamReq
, getResourceHarvesterDeployed
, getResourceHarvesterDeployedReq
, getResourceHarvesterDeployedStream
, getResourceHarvesterDeployedStreamReq
, getResourceHarvesterExtractionRate
, getResourceHarvesterExtractionRateReq
, getResourceHarvesterExtractionRateStream
, getResourceHarvesterExtractionRateStreamReq
, getResourceHarvesterOptimumCoreTemperature
, getResourceHarvesterOptimumCoreTemperatureReq
, getResourceHarvesterOptimumCoreTemperatureStream
, getResourceHarvesterOptimumCoreTemperatureStreamReq
, getResourceHarvesterPart
, getResourceHarvesterPartReq
, getResourceHarvesterPartStream
, getResourceHarvesterPartStreamReq
, getResourceHarvesterState
, getResourceHarvesterStateReq
, getResourceHarvesterStateStream
, getResourceHarvesterStateStreamReq
, getResourceHarvesterThermalEfficiency
, getResourceHarvesterThermalEfficiencyReq
, getResourceHarvesterThermalEfficiencyStream
, getResourceHarvesterThermalEfficiencyStreamReq
, setResourceHarvesterActive
, setResourceHarvesterActiveReq
, setResourceHarvesterDeployed
, setResourceHarvesterDeployedReq
, getResourceTransferAmount
, getResourceTransferAmountReq
, getResourceTransferAmountStream
, getResourceTransferAmountStreamReq
, getResourceTransferComplete
, getResourceTransferCompleteReq
, getResourceTransferCompleteStream
, getResourceTransferCompleteStreamReq
, resourceTransferStaticStart
, resourceTransferStaticStartReq
, resourceTransferStaticStartStream
, resourceTransferStaticStartStreamReq
, getResourceAmount
, getResourceAmountReq
, getResourceAmountStream
, getResourceAmountStreamReq
, getResourceDensity
, getResourceDensityReq
, getResourceDensityStream
, getResourceDensityStreamReq
, getResourceEnabled
, getResourceEnabledReq
, getResourceEnabledStream
, getResourceEnabledStreamReq
, getResourceFlowMode
, getResourceFlowModeReq
, getResourceFlowModeStream
, getResourceFlowModeStreamReq
, getResourceMax
, getResourceMaxReq
, getResourceMaxStream
, getResourceMaxStreamReq
, getResourceName
, getResourceNameReq
, getResourceNameStream
, getResourceNameStreamReq
, getResourcePart
, getResourcePartReq
, getResourcePartStream
, getResourcePartStreamReq
, setResourceEnabled
, setResourceEnabledReq
, resourcesAmount
, resourcesAmountReq
, resourcesAmountStream
, resourcesAmountStreamReq
, resourcesHasResource
, resourcesHasResourceReq
, resourcesHasResourceStream
, resourcesHasResourceStreamReq
, resourcesMax
, resourcesMaxReq
, resourcesMaxStream
, resourcesMaxStreamReq
, resourcesWithResource
, resourcesWithResourceReq
, resourcesWithResourceStream
, resourcesWithResourceStreamReq
, getResourcesAll
, getResourcesAllReq
, getResourcesAllStream
, getResourcesAllStreamReq
, getResourcesEnabled
, getResourcesEnabledReq
, getResourcesEnabledStream
, getResourcesEnabledStreamReq
, getResourcesNames
, getResourcesNamesReq
, getResourcesNamesStream
, getResourcesNamesStreamReq
, setResourcesEnabled
, setResourcesEnabledReq
, resourcesStaticDensity
, resourcesStaticDensityReq
, resourcesStaticDensityStream
, resourcesStaticDensityStreamReq
, resourcesStaticFlowMode
, resourcesStaticFlowModeReq
, resourcesStaticFlowModeStream
, resourcesStaticFlowModeStreamReq
, save
, saveReq
, getScienceDataDataAmount
, getScienceDataDataAmountReq
, getScienceDataDataAmountStream
, getScienceDataDataAmountStreamReq
, getScienceDataScienceValue
, getScienceDataScienceValueReq
, getScienceDataScienceValueStream
, getScienceDataScienceValueStreamReq
, getScienceDataTransmitValue
, getScienceDataTransmitValueReq
, getScienceDataTransmitValueStream
, getScienceDataTransmitValueStreamReq
, getScienceSubjectDataScale
, getScienceSubjectDataScaleReq
, getScienceSubjectDataScaleStream
, getScienceSubjectDataScaleStreamReq
, getScienceSubjectIsComplete
, getScienceSubjectIsCompleteReq
, getScienceSubjectIsCompleteStream
, getScienceSubjectIsCompleteStreamReq
, getScienceSubjectScience
, getScienceSubjectScienceReq
, getScienceSubjectScienceStream
, getScienceSubjectScienceStreamReq
, getScienceSubjectScienceCap
, getScienceSubjectScienceCapReq
, getScienceSubjectScienceCapStream
, getScienceSubjectScienceCapStreamReq
, getScienceSubjectScientificValue
, getScienceSubjectScientificValueReq
, getScienceSubjectScientificValueStream
, getScienceSubjectScientificValueStreamReq
, getScienceSubjectSubjectValue
, getScienceSubjectSubjectValueReq
, getScienceSubjectSubjectValueStream
, getScienceSubjectSubjectValueStreamReq
, getScienceSubjectTitle
, getScienceSubjectTitleReq
, getScienceSubjectTitleStream
, getScienceSubjectTitleStreamReq
, getSensorActive
, getSensorActiveReq
, getSensorActiveStream
, getSensorActiveStreamReq
, getSensorPart
, getSensorPartReq
, getSensorPartStream
, getSensorPartStreamReq
, getSensorPowerUsage
, getSensorPowerUsageReq
, getSensorPowerUsageStream
, getSensorPowerUsageStreamReq
, getSensorValue
, getSensorValueReq
, getSensorValueStream
, getSensorValueStreamReq
, setSensorActive
, setSensorActiveReq
, getSolarPanelDeployed
, getSolarPanelDeployedReq
, getSolarPanelDeployedStream
, getSolarPanelDeployedStreamReq
, getSolarPanelEnergyFlow
, getSolarPanelEnergyFlowReq
, getSolarPanelEnergyFlowStream
, getSolarPanelEnergyFlowStreamReq
, getSolarPanelPart
, getSolarPanelPartReq
, getSolarPanelPartStream
, getSolarPanelPartStreamReq
, getSolarPanelState
, getSolarPanelStateReq
, getSolarPanelStateStream
, getSolarPanelStateStreamReq
, getSolarPanelSunExposure
, getSolarPanelSunExposureReq
, getSolarPanelSunExposureStream
, getSolarPanelSunExposureStreamReq
, setSolarPanelDeployed
, setSolarPanelDeployedReq
, thrusterGimbalPosition
, thrusterGimbalPositionReq
, thrusterGimbalPositionStream
, thrusterGimbalPositionStreamReq
, thrusterInitialThrustDirection
, thrusterInitialThrustDirectionReq
, thrusterInitialThrustDirectionStream
, thrusterInitialThrustDirectionStreamReq
, thrusterInitialThrustPosition
, thrusterInitialThrustPositionReq
, thrusterInitialThrustPositionStream
, thrusterInitialThrustPositionStreamReq
, thrusterThrustDirection
, thrusterThrustDirectionReq
, thrusterThrustDirectionStream
, thrusterThrustDirectionStreamReq
, thrusterThrustPosition
, thrusterThrustPositionReq
, thrusterThrustPositionStream
, thrusterThrustPositionStreamReq
, getThrusterGimbalAngle
, getThrusterGimbalAngleReq
, getThrusterGimbalAngleStream
, getThrusterGimbalAngleStreamReq
, getThrusterGimballed
, getThrusterGimballedReq
, getThrusterGimballedStream
, getThrusterGimballedStreamReq
, getThrusterPart
, getThrusterPartReq
, getThrusterPartStream
, getThrusterPartStreamReq
, getThrusterThrustReferenceFrame
, getThrusterThrustReferenceFrameReq
, getThrusterThrustReferenceFrameStream
, getThrusterThrustReferenceFrameStreamReq
, transformDirection
, transformDirectionReq
, transformDirectionStream
, transformDirectionStreamReq
, transformPosition
, transformPositionReq
, transformPositionStream
, transformPositionStreamReq
, transformRotation
, transformRotationReq
, transformRotationStream
, transformRotationStreamReq
, transformVelocity
, transformVelocityReq
, transformVelocityStream
, transformVelocityStreamReq
, vesselAngularVelocity
, vesselAngularVelocityReq
, vesselAngularVelocityStream
, vesselAngularVelocityStreamReq
, vesselDirection
, vesselDirectionReq
, vesselDirectionStream
, vesselDirectionStreamReq
, vesselFlight
, vesselFlightReq
, vesselFlightStream
, vesselFlightStreamReq
, vesselPosition
, vesselPositionReq
, vesselPositionStream
, vesselPositionStreamReq
, vesselRecover
, vesselRecoverReq
, vesselResourcesInDecoupleStage
, vesselResourcesInDecoupleStageReq
, vesselResourcesInDecoupleStageStream
, vesselResourcesInDecoupleStageStreamReq
, vesselRotation
, vesselRotationReq
, vesselRotationStream
, vesselRotationStreamReq
, vesselVelocity
, vesselVelocityReq
, vesselVelocityStream
, vesselVelocityStreamReq
, getVesselAutoPilot
, getVesselAutoPilotReq
, getVesselAutoPilotStream
, getVesselAutoPilotStreamReq
, getVesselAvailableControlSurfaceTorque
, getVesselAvailableControlSurfaceTorqueReq
, getVesselAvailableControlSurfaceTorqueStream
, getVesselAvailableControlSurfaceTorqueStreamReq
, getVesselAvailableEngineTorque
, getVesselAvailableEngineTorqueReq
, getVesselAvailableEngineTorqueStream
, getVesselAvailableEngineTorqueStreamReq
, getVesselAvailableRCSTorque
, getVesselAvailableRCSTorqueReq
, getVesselAvailableRCSTorqueStream
, getVesselAvailableRCSTorqueStreamReq
, getVesselAvailableReactionWheelTorque
, getVesselAvailableReactionWheelTorqueReq
, getVesselAvailableReactionWheelTorqueStream
, getVesselAvailableReactionWheelTorqueStreamReq
, getVesselAvailableThrust
, getVesselAvailableThrustReq
, getVesselAvailableThrustStream
, getVesselAvailableThrustStreamReq
, getVesselAvailableTorque
, getVesselAvailableTorqueReq
, getVesselAvailableTorqueStream
, getVesselAvailableTorqueStreamReq
, getVesselBiome
, getVesselBiomeReq
, getVesselBiomeStream
, getVesselBiomeStreamReq
, getVesselControl
, getVesselControlReq
, getVesselControlStream
, getVesselControlStreamReq
, getVesselDryMass
, getVesselDryMassReq
, getVesselDryMassStream
, getVesselDryMassStreamReq
, getVesselInertiaTensor
, getVesselInertiaTensorReq
, getVesselInertiaTensorStream
, getVesselInertiaTensorStreamReq
, getVesselKerbinSeaLevelSpecificImpulse
, getVesselKerbinSeaLevelSpecificImpulseReq
, getVesselKerbinSeaLevelSpecificImpulseStream
, getVesselKerbinSeaLevelSpecificImpulseStreamReq
, getVesselMET
, getVesselMETReq
, getVesselMETStream
, getVesselMETStreamReq
, getVesselMass
, getVesselMassReq
, getVesselMassStream
, getVesselMassStreamReq
, getVesselMaxThrust
, getVesselMaxThrustReq
, getVesselMaxThrustStream
, getVesselMaxThrustStreamReq
, getVesselMaxVacuumThrust
, getVesselMaxVacuumThrustReq
, getVesselMaxVacuumThrustStream
, getVesselMaxVacuumThrustStreamReq
, getVesselMomentOfInertia
, getVesselMomentOfInertiaReq
, getVesselMomentOfInertiaStream
, getVesselMomentOfInertiaStreamReq
, getVesselName
, getVesselNameReq
, getVesselNameStream
, getVesselNameStreamReq
, getVesselOrbit
, getVesselOrbitReq
, getVesselOrbitStream
, getVesselOrbitStreamReq
, getVesselOrbitalReferenceFrame
, getVesselOrbitalReferenceFrameReq
, getVesselOrbitalReferenceFrameStream
, getVesselOrbitalReferenceFrameStreamReq
, getVesselParts
, getVesselPartsReq
, getVesselPartsStream
, getVesselPartsStreamReq
, getVesselRecoverable
, getVesselRecoverableReq
, getVesselRecoverableStream
, getVesselRecoverableStreamReq
, getVesselReferenceFrame
, getVesselReferenceFrameReq
, getVesselReferenceFrameStream
, getVesselReferenceFrameStreamReq
, getVesselResources
, getVesselResourcesReq
, getVesselResourcesStream
, getVesselResourcesStreamReq
, getVesselSituation
, getVesselSituationReq
, getVesselSituationStream
, getVesselSituationStreamReq
, getVesselSpecificImpulse
, getVesselSpecificImpulseReq
, getVesselSpecificImpulseStream
, getVesselSpecificImpulseStreamReq
, getVesselSurfaceReferenceFrame
, getVesselSurfaceReferenceFrameReq
, getVesselSurfaceReferenceFrameStream
, getVesselSurfaceReferenceFrameStreamReq
, getVesselSurfaceVelocityReferenceFrame
, getVesselSurfaceVelocityReferenceFrameReq
, getVesselSurfaceVelocityReferenceFrameStream
, getVesselSurfaceVelocityReferenceFrameStreamReq
, getVesselThrust
, getVesselThrustReq
, getVesselThrustStream
, getVesselThrustStreamReq
, getVesselType
, getVesselTypeReq
, getVesselTypeStream
, getVesselTypeStreamReq
, getVesselVacuumSpecificImpulse
, getVesselVacuumSpecificImpulseReq
, getVesselVacuumSpecificImpulseStream
, getVesselVacuumSpecificImpulseStreamReq
, setVesselName
, setVesselNameReq
, setVesselType
, setVesselTypeReq
, warpTo
, warpToReq
, waypointManagerAddWaypoint
, waypointManagerAddWaypointReq
, waypointManagerAddWaypointStream
, waypointManagerAddWaypointStreamReq
, getWaypointManagerColors
, getWaypointManagerColorsReq
, getWaypointManagerColorsStream
, getWaypointManagerColorsStreamReq
, getWaypointManagerIcons
, getWaypointManagerIconsReq
, getWaypointManagerIconsStream
, getWaypointManagerIconsStreamReq
, getWaypointManagerWaypoints
, getWaypointManagerWaypointsReq
, getWaypointManagerWaypointsStream
, getWaypointManagerWaypointsStreamReq
, waypointRemove
, waypointRemoveReq
, getWaypointBedrockAltitude
, getWaypointBedrockAltitudeReq
, getWaypointBedrockAltitudeStream
, getWaypointBedrockAltitudeStreamReq
, getWaypointBody
, getWaypointBodyReq
, getWaypointBodyStream
, getWaypointBodyStreamReq
, getWaypointClustered
, getWaypointClusteredReq
, getWaypointClusteredStream
, getWaypointClusteredStreamReq
, getWaypointColor
, getWaypointColorReq
, getWaypointColorStream
, getWaypointColorStreamReq
, getWaypointContractId
, getWaypointContractIdReq
, getWaypointContractIdStream
, getWaypointContractIdStreamReq
, getWaypointGrounded
, getWaypointGroundedReq
, getWaypointGroundedStream
, getWaypointGroundedStreamReq
, getWaypointHasContract
, getWaypointHasContractReq
, getWaypointHasContractStream
, getWaypointHasContractStreamReq
, getWaypointIcon
, getWaypointIconReq
, getWaypointIconStream
, getWaypointIconStreamReq
, getWaypointIndex
, getWaypointIndexReq
, getWaypointIndexStream
, getWaypointIndexStreamReq
, getWaypointLatitude
, getWaypointLatitudeReq
, getWaypointLatitudeStream
, getWaypointLatitudeStreamReq
, getWaypointLongitude
, getWaypointLongitudeReq
, getWaypointLongitudeStream
, getWaypointLongitudeStreamReq
, getWaypointMeanAltitude
, getWaypointMeanAltitudeReq
, getWaypointMeanAltitudeStream
, getWaypointMeanAltitudeStreamReq
, getWaypointName
, getWaypointNameReq
, getWaypointNameStream
, getWaypointNameStreamReq
, getWaypointNearSurface
, getWaypointNearSurfaceReq
, getWaypointNearSurfaceStream
, getWaypointNearSurfaceStreamReq
, getWaypointSurfaceAltitude
, getWaypointSurfaceAltitudeReq
, getWaypointSurfaceAltitudeStream
, getWaypointSurfaceAltitudeStreamReq
, setWaypointBedrockAltitude
, setWaypointBedrockAltitudeReq
, setWaypointBody
, setWaypointBodyReq
, setWaypointColor
, setWaypointColorReq
, setWaypointIcon
, setWaypointIconReq
, setWaypointLatitude
, setWaypointLatitudeReq
, setWaypointLongitude
, setWaypointLongitudeReq
, setWaypointMeanAltitude
, setWaypointMeanAltitudeReq
, setWaypointName
, setWaypointNameReq
, setWaypointSurfaceAltitude
, setWaypointSurfaceAltitudeReq
, getActiveVessel
, getActiveVesselReq
, getActiveVesselStream
, getActiveVesselStreamReq
, getBodies
, getBodiesReq
, getBodiesStream
, getBodiesStreamReq
, getCamera
, getCameraReq
, getCameraStream
, getCameraStreamReq
, getFARAvailable
, getFARAvailableReq
, getFARAvailableStream
, getFARAvailableStreamReq
, getG
, getGReq
, getGStream
, getGStreamReq
, getMaximumRailsWarpFactor
, getMaximumRailsWarpFactorReq
, getMaximumRailsWarpFactorStream
, getMaximumRailsWarpFactorStreamReq
, getPhysicsWarpFactor
, getPhysicsWarpFactorReq
, getPhysicsWarpFactorStream
, getPhysicsWarpFactorStreamReq
, getRailsWarpFactor
, getRailsWarpFactorReq
, getRailsWarpFactorStream
, getRailsWarpFactorStreamReq
, getTargetBody
, getTargetBodyReq
, getTargetBodyStream
, getTargetBodyStreamReq
, getTargetDockingPort
, getTargetDockingPortReq
, getTargetDockingPortStream
, getTargetDockingPortStreamReq
, getTargetVessel
, getTargetVesselReq
, getTargetVesselStream
, getTargetVesselStreamReq
, getUT
, getUTReq
, getUTStream
, getUTStreamReq
, getVessels
, getVesselsReq
, getVesselsStream
, getVesselsStreamReq
, getWarpFactor
, getWarpFactorReq
, getWarpFactorStream
, getWarpFactorStreamReq
, getWarpMode
, getWarpModeReq
, getWarpModeStream
, getWarpModeStreamReq
, getWarpRate
, getWarpRateReq
, getWarpRateStream
, getWarpRateStreamReq
, getWaypointManager
, getWaypointManagerReq
, getWaypointManagerStream
, getWaypointManagerStreamReq
, setActiveVessel
, setActiveVesselReq
, setPhysicsWarpFactor
, setPhysicsWarpFactorReq
, setRailsWarpFactor
, setRailsWarpFactorReq
, setTargetBody
, setTargetBodyReq
, setTargetDockingPort
, setTargetDockingPortReq
, setTargetVessel
, setTargetVesselReq
) where

import qualified Data.Int
import qualified Data.Map
import qualified Data.Text
import qualified Data.Word

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


{-|
Provides basic auto-piloting utilities for a vessel.
Created by calling <see cref="M:SpaceCenter.Vessel.AutoPilot" />.If a client engages the auto-pilot and then closes its connection to the server,
the auto-pilot will be disengaged and its target reference frame, direction and roll reset to default.
 -}
newtype AutoPilot = AutoPilot { autoPilotId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable AutoPilot where
    encodePb = encodePb . autoPilotId

instance PbDecodable AutoPilot where
    decodePb b = AutoPilot <$> decodePb b

instance KRPCResponseExtractable AutoPilot

{-|
Controls the game's camera.
Obtained by calling <see cref="M:SpaceCenter.Camera" />.
 -}
newtype Camera = Camera { cameraId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Camera where
    encodePb = encodePb . cameraId

instance PbDecodable Camera where
    decodePb b = Camera <$> decodePb b

instance KRPCResponseExtractable Camera

{-|
A cargo bay. Obtained by calling <see cref="M:SpaceCenter.Part.CargoBay" />.
 -}
newtype CargoBay = CargoBay { cargoBayId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable CargoBay where
    encodePb = encodePb . cargoBayId

instance PbDecodable CargoBay where
    decodePb b = CargoBay <$> decodePb b

instance KRPCResponseExtractable CargoBay

{-|
Represents a celestial body (such as a planet or moon).
See <see cref="M:SpaceCenter.Bodies" />.
 -}
newtype CelestialBody = CelestialBody { celestialBodyId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable CelestialBody where
    encodePb = encodePb . celestialBodyId

instance PbDecodable CelestialBody where
    decodePb b = CelestialBody <$> decodePb b

instance KRPCResponseExtractable CelestialBody

{-|
Used to manipulate the controls of a vessel. This includes adjusting the
throttle, enabling/disabling systems such as SAS and RCS, or altering the
direction in which the vessel is pointing.
Obtained by calling <see cref="M:SpaceCenter.Vessel.Control" />.Control inputs (such as pitch, yaw and roll) are zeroed when all clients
that have set one or more of these inputs are no longer connected.
 -}
newtype Control = Control { controlId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Control where
    encodePb = encodePb . controlId

instance PbDecodable Control where
    decodePb b = Control <$> decodePb b

instance KRPCResponseExtractable Control

{-|
An aerodynamic control surface. Obtained by calling <see cref="M:SpaceCenter.Part.ControlSurface" />.
 -}
newtype ControlSurface = ControlSurface { controlSurfaceId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ControlSurface where
    encodePb = encodePb . controlSurfaceId

instance PbDecodable ControlSurface where
    decodePb b = ControlSurface <$> decodePb b

instance KRPCResponseExtractable ControlSurface

{-|
A decoupler. Obtained by calling <see cref="M:SpaceCenter.Part.Decoupler" />
 -}
newtype Decoupler = Decoupler { decouplerId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Decoupler where
    encodePb = encodePb . decouplerId

instance PbDecodable Decoupler where
    decodePb b = Decoupler <$> decodePb b

instance KRPCResponseExtractable Decoupler

{-|
A docking port. Obtained by calling <see cref="M:SpaceCenter.Part.DockingPort" />
 -}
newtype DockingPort = DockingPort { dockingPortId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable DockingPort where
    encodePb = encodePb . dockingPortId

instance PbDecodable DockingPort where
    decodePb b = DockingPort <$> decodePb b

instance KRPCResponseExtractable DockingPort

{-|
An engine, including ones of various types.
For example liquid fuelled gimballed engines, solid rocket boosters and jet engines.
Obtained by calling <see cref="M:SpaceCenter.Part.Engine" />.For RCS thrusters <see cref="M:SpaceCenter.Part.RCS" />.
 -}
newtype Engine = Engine { engineId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Engine where
    encodePb = encodePb . engineId

instance PbDecodable Engine where
    decodePb b = Engine <$> decodePb b

instance KRPCResponseExtractable Engine

{-|
Obtained by calling <see cref="M:SpaceCenter.Part.Experiment" />.
 -}
newtype Experiment = Experiment { experimentId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Experiment where
    encodePb = encodePb . experimentId

instance PbDecodable Experiment where
    decodePb b = Experiment <$> decodePb b

instance KRPCResponseExtractable Experiment

{-|
A fairing. Obtained by calling <see cref="M:SpaceCenter.Part.Fairing" />.
 -}
newtype Fairing = Fairing { fairingId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Fairing where
    encodePb = encodePb . fairingId

instance PbDecodable Fairing where
    decodePb b = Fairing <$> decodePb b

instance KRPCResponseExtractable Fairing

{-|
Used to get flight telemetry for a vessel, by calling <see cref="M:SpaceCenter.Vessel.Flight" />.
All of the information returned by this class is given in the reference frame
passed to that method.
Obtained by calling <see cref="M:SpaceCenter.Vessel.Flight" />.To get orbital information, such as the apoapsis or inclination, see <see cref="T:SpaceCenter.Orbit" />.
 -}
newtype Flight = Flight { flightId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Flight where
    encodePb = encodePb . flightId

instance PbDecodable Flight where
    decodePb b = Flight <$> decodePb b

instance KRPCResponseExtractable Flight

{-|
Obtained by calling <see cref="M:SpaceCenter.Part.AddForce" />.
 -}
newtype Force = Force { forceId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Force where
    encodePb = encodePb . forceId

instance PbDecodable Force where
    decodePb b = Force <$> decodePb b

instance KRPCResponseExtractable Force

{-|
An air intake. Obtained by calling <see cref="M:SpaceCenter.Part.Intake" />.
 -}
newtype Intake = Intake { intakeId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Intake where
    encodePb = encodePb . intakeId

instance PbDecodable Intake where
    decodePb b = Intake <$> decodePb b

instance KRPCResponseExtractable Intake

{-|
Landing gear with wheels. Obtained by calling <see cref="M:SpaceCenter.Part.LandingGear" />.
 -}
newtype LandingGear = LandingGear { landingGearId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable LandingGear where
    encodePb = encodePb . landingGearId

instance PbDecodable LandingGear where
    decodePb b = LandingGear <$> decodePb b

instance KRPCResponseExtractable LandingGear

{-|
A landing leg. Obtained by calling <see cref="M:SpaceCenter.Part.LandingLeg" />.
 -}
newtype LandingLeg = LandingLeg { landingLegId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable LandingLeg where
    encodePb = encodePb . landingLegId

instance PbDecodable LandingLeg where
    decodePb b = LandingLeg <$> decodePb b

instance KRPCResponseExtractable LandingLeg

{-|
A launch clamp. Obtained by calling <see cref="M:SpaceCenter.Part.LaunchClamp" />.
 -}
newtype LaunchClamp = LaunchClamp { launchClampId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable LaunchClamp where
    encodePb = encodePb . launchClampId

instance PbDecodable LaunchClamp where
    decodePb b = LaunchClamp <$> decodePb b

instance KRPCResponseExtractable LaunchClamp

{-|
A light. Obtained by calling <see cref="M:SpaceCenter.Part.Light" />.
 -}
newtype Light = Light { lightId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Light where
    encodePb = encodePb . lightId

instance PbDecodable Light where
    decodePb b = Light <$> decodePb b

instance KRPCResponseExtractable Light

{-|
This can be used to interact with a specific part module. This includes part modules in stock KSP,
and those added by mods.

In KSP, each part has zero or more
<a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation#MODULES">PartModulesassociated with it. Each one contains some of the functionality of the part.
For example, an engine has a "ModuleEngines" part module that contains all the
functionality of an engine.
 -}
newtype Module = Module { moduleId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Module where
    encodePb = encodePb . moduleId

instance PbDecodable Module where
    decodePb b = Module <$> decodePb b

instance KRPCResponseExtractable Module

{-|
Represents a maneuver node. Can be created using <see cref="M:SpaceCenter.Control.AddNode" />.
 -}
newtype Node = Node { nodeId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Node where
    encodePb = encodePb . nodeId

instance PbDecodable Node where
    decodePb b = Node <$> decodePb b

instance KRPCResponseExtractable Node

{-|
Describes an orbit. For example, the orbit of a vessel, obtained by calling
<see cref="M:SpaceCenter.Vessel.Orbit" />, or a celestial body, obtained by calling
<see cref="M:SpaceCenter.CelestialBody.Orbit" />.
 -}
newtype Orbit = Orbit { orbitId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Orbit where
    encodePb = encodePb . orbitId

instance PbDecodable Orbit where
    decodePb b = Orbit <$> decodePb b

instance KRPCResponseExtractable Orbit

{-|
A parachute. Obtained by calling <see cref="M:SpaceCenter.Part.Parachute" />.
 -}
newtype Parachute = Parachute { parachuteId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Parachute where
    encodePb = encodePb . parachuteId

instance PbDecodable Parachute where
    decodePb b = Parachute <$> decodePb b

instance KRPCResponseExtractable Parachute

{-|
Represents an individual part. Vessels are made up of multiple parts.
Instances of this class can be obtained by several methods in <see cref="T:SpaceCenter.Parts" />.
 -}
newtype Part = Part { partId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Part where
    encodePb = encodePb . partId

instance PbDecodable Part where
    decodePb b = Part <$> decodePb b

instance KRPCResponseExtractable Part

{-|
Instances of this class are used to interact with the parts of a vessel.
An instance can be obtained by calling <see cref="M:SpaceCenter.Vessel.Parts" />.
 -}
newtype Parts = Parts { partsId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Parts where
    encodePb = encodePb . partsId

instance PbDecodable Parts where
    decodePb b = Parts <$> decodePb b

instance KRPCResponseExtractable Parts

{-|
A propellant for an engine. Obtains by calling <see cref="M:SpaceCenter.Engine.Propellants" />.
 -}
newtype Propellant = Propellant { propellantId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Propellant where
    encodePb = encodePb . propellantId

instance PbDecodable Propellant where
    decodePb b = Propellant <$> decodePb b

instance KRPCResponseExtractable Propellant

{-|
An RCS block or thruster. Obtained by calling <see cref="M:SpaceCenter.Part.RCS" />.
 -}
newtype RCS = RCS { rCSId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable RCS where
    encodePb = encodePb . rCSId

instance PbDecodable RCS where
    decodePb b = RCS <$> decodePb b

instance KRPCResponseExtractable RCS

{-|
A radiator. Obtained by calling <see cref="M:SpaceCenter.Part.Radiator" />.
 -}
newtype Radiator = Radiator { radiatorId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Radiator where
    encodePb = encodePb . radiatorId

instance PbDecodable Radiator where
    decodePb b = Radiator <$> decodePb b

instance KRPCResponseExtractable Radiator

{-|
A reaction wheel. Obtained by calling <see cref="M:SpaceCenter.Part.ReactionWheel" />.
 -}
newtype ReactionWheel = ReactionWheel { reactionWheelId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ReactionWheel where
    encodePb = encodePb . reactionWheelId

instance PbDecodable ReactionWheel where
    decodePb b = ReactionWheel <$> decodePb b

instance KRPCResponseExtractable ReactionWheel

{-|
Represents a reference frame for positions, rotations and
velocities. Contains:
<list type="bullet">The position of the origin.The directions of the x, y and z axes.The linear velocity of the frame.The angular velocity of the frame.This class does not contain any properties or methods. It is only
used as a parameter to other functions.
 -}
newtype ReferenceFrame = ReferenceFrame { referenceFrameId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ReferenceFrame where
    encodePb = encodePb . referenceFrameId

instance PbDecodable ReferenceFrame where
    decodePb b = ReferenceFrame <$> decodePb b

instance KRPCResponseExtractable ReferenceFrame

{-|
An individual resource stored within a part.
Created using methods in the <see cref="T:SpaceCenter.Resources" /> class.
 -}
newtype Resource = Resource { resourceId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Resource where
    encodePb = encodePb . resourceId

instance PbDecodable Resource where
    decodePb b = Resource <$> decodePb b

instance KRPCResponseExtractable Resource

{-|
A resource converter. Obtained by calling <see cref="M:SpaceCenter.Part.ResourceConverter" />.
 -}
newtype ResourceConverter = ResourceConverter { resourceConverterId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ResourceConverter where
    encodePb = encodePb . resourceConverterId

instance PbDecodable ResourceConverter where
    decodePb b = ResourceConverter <$> decodePb b

instance KRPCResponseExtractable ResourceConverter

{-|
A resource harvester (drill). Obtained by calling <see cref="M:SpaceCenter.Part.ResourceHarvester" />.
 -}
newtype ResourceHarvester = ResourceHarvester { resourceHarvesterId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ResourceHarvester where
    encodePb = encodePb . resourceHarvesterId

instance PbDecodable ResourceHarvester where
    decodePb b = ResourceHarvester <$> decodePb b

instance KRPCResponseExtractable ResourceHarvester

{-|
Transfer resources between parts.
 -}
newtype ResourceTransfer = ResourceTransfer { resourceTransferId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ResourceTransfer where
    encodePb = encodePb . resourceTransferId

instance PbDecodable ResourceTransfer where
    decodePb b = ResourceTransfer <$> decodePb b

instance KRPCResponseExtractable ResourceTransfer

{-|
Represents the collection of resources stored in a vessel, stage or part.
Created by calling <see cref="M:SpaceCenter.Vessel.Resources" />,
<see cref="M:SpaceCenter.Vessel.ResourcesInDecoupleStage" /> or
<see cref="M:SpaceCenter.Part.Resources" />.
 -}
newtype Resources = Resources { resourcesId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Resources where
    encodePb = encodePb . resourcesId

instance PbDecodable Resources where
    decodePb b = Resources <$> decodePb b

instance KRPCResponseExtractable Resources

{-|
Obtained by calling <see cref="M:SpaceCenter.Experiment.Data" />.
 -}
newtype ScienceData = ScienceData { scienceDataId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ScienceData where
    encodePb = encodePb . scienceDataId

instance PbDecodable ScienceData where
    decodePb b = ScienceData <$> decodePb b

instance KRPCResponseExtractable ScienceData

{-|
Obtained by calling <see cref="M:SpaceCenter.Experiment.ScienceSubject" />.
 -}
newtype ScienceSubject = ScienceSubject { scienceSubjectId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable ScienceSubject where
    encodePb = encodePb . scienceSubjectId

instance PbDecodable ScienceSubject where
    decodePb b = ScienceSubject <$> decodePb b

instance KRPCResponseExtractable ScienceSubject

{-|
A sensor, such as a thermometer. Obtained by calling <see cref="M:SpaceCenter.Part.Sensor" />.
 -}
newtype Sensor = Sensor { sensorId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Sensor where
    encodePb = encodePb . sensorId

instance PbDecodable Sensor where
    decodePb b = Sensor <$> decodePb b

instance KRPCResponseExtractable Sensor

{-|
A solar panel. Obtained by calling <see cref="M:SpaceCenter.Part.SolarPanel" />.
 -}
newtype SolarPanel = SolarPanel { solarPanelId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable SolarPanel where
    encodePb = encodePb . solarPanelId

instance PbDecodable SolarPanel where
    decodePb b = SolarPanel <$> decodePb b

instance KRPCResponseExtractable SolarPanel

{-|
The component of an <see cref="T:SpaceCenter.Engine" /> or <see cref="T:SpaceCenter.RCS" /> part that generates thrust.
Can obtained by calling <see cref="M:SpaceCenter.Engine.Thrusters" /> or <see cref="M:SpaceCenter.RCS.Thrusters" />.Engines can consist of multiple thrusters.
For example, the S3 KS-25x4 "Mammoth" has four rocket nozzels, and so consists of four thrusters.
 -}
newtype Thruster = Thruster { thrusterId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Thruster where
    encodePb = encodePb . thrusterId

instance PbDecodable Thruster where
    decodePb b = Thruster <$> decodePb b

instance KRPCResponseExtractable Thruster

{-|
These objects are used to interact with vessels in KSP. This includes getting
orbital and flight data, manipulating control inputs and managing resources.
Created using <see cref="M:SpaceCenter.ActiveVessel" /> or <see cref="M:SpaceCenter.Vessels" />.
 -}
newtype Vessel = Vessel { vesselId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Vessel where
    encodePb = encodePb . vesselId

instance PbDecodable Vessel where
    decodePb b = Vessel <$> decodePb b

instance KRPCResponseExtractable Vessel

{-|
Represents a waypoint. Can be created using <see cref="M:SpaceCenter.WaypointManager.AddWaypoint" />.
 -}
newtype Waypoint = Waypoint { waypointId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Waypoint where
    encodePb = encodePb . waypointId

instance PbDecodable Waypoint where
    decodePb b = Waypoint <$> decodePb b

instance KRPCResponseExtractable Waypoint

{-|
Waypoints are the location markers you can see on the map view showing you where contracts are targeted for. 
With this structure, you can obtain coordinate data for the locations of these waypoints.
Obtained by calling <see cref="M:SpaceCenter.WaypointManager" />.
 -}
newtype WaypointManager = WaypointManager { waypointManagerId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable WaypointManager where
    encodePb = encodePb . waypointManagerId

instance PbDecodable WaypointManager where
    decodePb b = WaypointManager <$> decodePb b

instance KRPCResponseExtractable WaypointManager


{-|
See <see cref="M:SpaceCenter.Camera.Mode" />.
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

instance PbEncodable CameraMode where
    encodePb = encodePb . fromEnum

instance PbDecodable CameraMode where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable CameraMode

{-|
The state of a cargo bay. See <see cref="M:SpaceCenter.CargoBay.State" />.
 -}
data CargoBayState
    = CargoBayState'Open
    | CargoBayState'Closed
    | CargoBayState'Opening
    | CargoBayState'Closing
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable CargoBayState where
    encodePb = encodePb . fromEnum

instance PbDecodable CargoBayState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable CargoBayState

{-|
The state of a docking port. See <see cref="M:SpaceCenter.DockingPort.State" />.
 -}
data DockingPortState
    = DockingPortState'Ready
    | DockingPortState'Docked
    | DockingPortState'Docking
    | DockingPortState'Undocking
    | DockingPortState'Shielded
    | DockingPortState'Moving
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable DockingPortState where
    encodePb = encodePb . fromEnum

instance PbDecodable DockingPortState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable DockingPortState

{-|
The state of a landing gear. See <see cref="M:SpaceCenter.LandingGear.State" />.
 -}
data LandingGearState
    = LandingGearState'Deployed
    | LandingGearState'Retracted
    | LandingGearState'Deploying
    | LandingGearState'Retracting
    | LandingGearState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable LandingGearState where
    encodePb = encodePb . fromEnum

instance PbDecodable LandingGearState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable LandingGearState

{-|
The state of a landing leg. See <see cref="M:SpaceCenter.LandingLeg.State" />.
 -}
data LandingLegState
    = LandingLegState'Deployed
    | LandingLegState'Retracted
    | LandingLegState'Deploying
    | LandingLegState'Retracting
    | LandingLegState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable LandingLegState where
    encodePb = encodePb . fromEnum

instance PbDecodable LandingLegState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable LandingLegState

{-|
The state of a parachute. See <see cref="M:SpaceCenter.Parachute.State" />.
 -}
data ParachuteState
    = ParachuteState'Active
    | ParachuteState'Cut
    | ParachuteState'Deployed
    | ParachuteState'SemiDeployed
    | ParachuteState'Stowed
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable ParachuteState where
    encodePb = encodePb . fromEnum

instance PbDecodable ParachuteState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ParachuteState

{-|
The state of a radiator. <see cref="T:SpaceCenter.RadiatorState" />
 -}
data RadiatorState
    = RadiatorState'Extended
    | RadiatorState'Retracted
    | RadiatorState'Extending
    | RadiatorState'Retracting
    | RadiatorState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable RadiatorState where
    encodePb = encodePb . fromEnum

instance PbDecodable RadiatorState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable RadiatorState

{-|
The state of a resource converter. See <see cref="M:SpaceCenter.ResourceConverter.State" />.
 -}
data ResourceConverterState
    = ResourceConverterState'Running
    | ResourceConverterState'Idle
    | ResourceConverterState'MissingResource
    | ResourceConverterState'StorageFull
    | ResourceConverterState'Capacity
    | ResourceConverterState'Unknown
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable ResourceConverterState where
    encodePb = encodePb . fromEnum

instance PbDecodable ResourceConverterState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ResourceConverterState

{-|
The way in which a resource flows between parts. See <see cref="M:SpaceCenter.Resources.FlowMode" />.
 -}
data ResourceFlowMode
    = ResourceFlowMode'Vessel
    | ResourceFlowMode'Stage
    | ResourceFlowMode'Adjacent
    | ResourceFlowMode'None
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable ResourceFlowMode where
    encodePb = encodePb . fromEnum

instance PbDecodable ResourceFlowMode where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ResourceFlowMode

{-|
The state of a resource harvester. See <see cref="M:SpaceCenter.ResourceHarvester.State" />.
 -}
data ResourceHarvesterState
    = ResourceHarvesterState'Deploying
    | ResourceHarvesterState'Deployed
    | ResourceHarvesterState'Retracting
    | ResourceHarvesterState'Retracted
    | ResourceHarvesterState'Active
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable ResourceHarvesterState where
    encodePb = encodePb . fromEnum

instance PbDecodable ResourceHarvesterState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable ResourceHarvesterState

{-|
The behavior of the SAS auto-pilot. See <see cref="M:SpaceCenter.AutoPilot.SASMode" />.
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

instance PbEncodable SASMode where
    encodePb = encodePb . fromEnum

instance PbDecodable SASMode where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable SASMode

{-|
The state of a solar panel. See <see cref="M:SpaceCenter.SolarPanel.State" />.
 -}
data SolarPanelState
    = SolarPanelState'Extended
    | SolarPanelState'Retracted
    | SolarPanelState'Extending
    | SolarPanelState'Retracting
    | SolarPanelState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable SolarPanelState where
    encodePb = encodePb . fromEnum

instance PbDecodable SolarPanelState where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable SolarPanelState

{-|
The mode of the speed reported in the navball.
See <see cref="M:SpaceCenter.Control.SpeedMode" />.
 -}
data SpeedMode
    = SpeedMode'Orbit
    | SpeedMode'Surface
    | SpeedMode'Target
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable SpeedMode where
    encodePb = encodePb . fromEnum

instance PbDecodable SpeedMode where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable SpeedMode

{-|
The situation a vessel is in.
See <see cref="M:SpaceCenter.Vessel.Situation" />.
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

instance PbEncodable VesselSituation where
    encodePb = encodePb . fromEnum

instance PbDecodable VesselSituation where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable VesselSituation

{-|
The type of a vessel.
See <see cref="M:SpaceCenter.Vessel.Type" />.
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

instance PbEncodable VesselType where
    encodePb = encodePb . fromEnum

instance PbDecodable VesselType where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable VesselType

{-|
The time warp mode.
Returned by <see cref="T:SpaceCenter.WarpMode" />
 -}
data WarpMode
    = WarpMode'Rails
    | WarpMode'Physics
    | WarpMode'None
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable WarpMode where
    encodePb = encodePb . fromEnum

instance PbDecodable WarpMode where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable WarpMode


{-|
Disengage the auto-pilot.
 -}
autoPilotDisengageReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ()
autoPilotDisengageReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_Disengage" [makeArgument 0 thisArg]

autoPilotDisengage :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ()
autoPilotDisengage thisArg = simpleRequest $ autoPilotDisengageReq thisArg 

{-|
Engage the auto-pilot.
 -}
autoPilotEngageReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ()
autoPilotEngageReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_Engage" [makeArgument 0 thisArg]

autoPilotEngage :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ()
autoPilotEngage thisArg = simpleRequest $ autoPilotEngageReq thisArg 

{-|
Set target pitch and heading angles.<param name="pitch">Target pitch angle, in degrees between -90° and +90°.<param name="heading">Target heading angle, in degrees between 0° and 360°.
 -}
autoPilotTargetPitchAndHeadingReq :: KRPCHS.SpaceCenter.AutoPilot -> Float -> Float -> KRPCCallReq ()
autoPilotTargetPitchAndHeadingReq thisArg pitchArg headingArg = makeCallReq "SpaceCenter" "AutoPilot_TargetPitchAndHeading" [makeArgument 0 thisArg, makeArgument 1 pitchArg, makeArgument 2 headingArg]

autoPilotTargetPitchAndHeading :: KRPCHS.SpaceCenter.AutoPilot -> Float -> Float -> RPCContext ()
autoPilotTargetPitchAndHeading thisArg pitchArg headingArg = simpleRequest $ autoPilotTargetPitchAndHeadingReq thisArg pitchArg headingArg 

{-|
Blocks until the vessel is pointing in the target direction and has the target roll (if set).
 -}
autoPilotWaitReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ()
autoPilotWaitReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_Wait" [makeArgument 0 thisArg]

autoPilotWait :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ()
autoPilotWait thisArg = simpleRequest $ autoPilotWaitReq thisArg 

{-|
The angle at which the autopilot considers the vessel to be pointing close to the target.
This determines the midpoint of the target velocity attenuation function.
A vector of three angles, in degrees, one for each of the pitch, roll and yaw axes.
Defaults to 1° for each axis.
 -}
getAutoPilotAttenuationAngleReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotAttenuationAngleReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_AttenuationAngle" [makeArgument 0 thisArg]

getAutoPilotAttenuationAngle :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotAttenuationAngle thisArg = simpleRequest $ getAutoPilotAttenuationAngleReq thisArg

getAutoPilotAttenuationAngleStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotAttenuationAngleStreamReq thisArg = makeStreamReq $ getAutoPilotAttenuationAngleReq thisArg

getAutoPilotAttenuationAngleStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotAttenuationAngleStream thisArg = requestAddStream $ getAutoPilotAttenuationAngleStreamReq thisArg 

{-|
Whether the rotation rate controllers PID parameters should be automatically tuned using the
vessels moment of inertia and available torque. Defaults totrue.
See <see cref="M:SpaceCenter.AutoPilot.TimeToPeak" /> and  <see cref="M:SpaceCenter.AutoPilot.Overshoot" />.
 -}
getAutoPilotAutoTuneReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Bool)
getAutoPilotAutoTuneReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_AutoTune" [makeArgument 0 thisArg]

getAutoPilotAutoTune :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
getAutoPilotAutoTune thisArg = simpleRequest $ getAutoPilotAutoTuneReq thisArg

getAutoPilotAutoTuneStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Bool)
getAutoPilotAutoTuneStreamReq thisArg = makeStreamReq $ getAutoPilotAutoTuneReq thisArg

getAutoPilotAutoTuneStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Bool))
getAutoPilotAutoTuneStream thisArg = requestAddStream $ getAutoPilotAutoTuneStreamReq thisArg 

{-|
The time the vessel should take to come to a stop pointing in the target direction.
This determines the angular acceleration used to decelerate the vessel.
A vector of three times, in seconds, one for each of the pitch, roll and yaw axes.
Defaults to 5 seconds for each axis.
 -}
getAutoPilotDecelerationTimeReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotDecelerationTimeReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_DecelerationTime" [makeArgument 0 thisArg]

getAutoPilotDecelerationTime :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotDecelerationTime thisArg = simpleRequest $ getAutoPilotDecelerationTimeReq thisArg

getAutoPilotDecelerationTimeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotDecelerationTimeStreamReq thisArg = makeStreamReq $ getAutoPilotDecelerationTimeReq thisArg

getAutoPilotDecelerationTimeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotDecelerationTimeStream thisArg = requestAddStream $ getAutoPilotDecelerationTimeStreamReq thisArg 

{-|
The error, in degrees, between the direction the ship has been asked
to point in and the direction it is pointing in. Returns zero if the auto-pilot
has not been engaged and SAS is not enabled or is in stability assist mode.
 -}
getAutoPilotErrorReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotErrorReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]

getAutoPilotError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotError thisArg = simpleRequest $ getAutoPilotErrorReq thisArg

getAutoPilotErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotErrorStreamReq thisArg = makeStreamReq $ getAutoPilotErrorReq thisArg

getAutoPilotErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotErrorStream thisArg = requestAddStream $ getAutoPilotErrorStreamReq thisArg 

{-|
The error, in degrees, between the vessels current and target heading.
Returns zero if the auto-pilot has not been engaged.
 -}
getAutoPilotHeadingErrorReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotHeadingErrorReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_HeadingError" [makeArgument 0 thisArg]

getAutoPilotHeadingError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotHeadingError thisArg = simpleRequest $ getAutoPilotHeadingErrorReq thisArg

getAutoPilotHeadingErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotHeadingErrorStreamReq thisArg = makeStreamReq $ getAutoPilotHeadingErrorReq thisArg

getAutoPilotHeadingErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotHeadingErrorStream thisArg = requestAddStream $ getAutoPilotHeadingErrorStreamReq thisArg 

{-|
The target overshoot percentage used to autotune the PID controllers.
A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
Defaults to 0.01 for each axis.
 -}
getAutoPilotOvershootReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotOvershootReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_Overshoot" [makeArgument 0 thisArg]

getAutoPilotOvershoot :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotOvershoot thisArg = simpleRequest $ getAutoPilotOvershootReq thisArg

getAutoPilotOvershootStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotOvershootStreamReq thisArg = makeStreamReq $ getAutoPilotOvershootReq thisArg

getAutoPilotOvershootStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotOvershootStream thisArg = requestAddStream $ getAutoPilotOvershootStreamReq thisArg 

{-|
The error, in degrees, between the vessels current and target pitch.
Returns zero if the auto-pilot has not been engaged.
 -}
getAutoPilotPitchErrorReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotPitchErrorReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_PitchError" [makeArgument 0 thisArg]

getAutoPilotPitchError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotPitchError thisArg = simpleRequest $ getAutoPilotPitchErrorReq thisArg

getAutoPilotPitchErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotPitchErrorStreamReq thisArg = makeStreamReq $ getAutoPilotPitchErrorReq thisArg

getAutoPilotPitchErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotPitchErrorStream thisArg = requestAddStream $ getAutoPilotPitchErrorStreamReq thisArg 

{-|
Gains for the pitch PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotPitchPIDGainsReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotPitchPIDGainsReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_PitchPIDGains" [makeArgument 0 thisArg]

getAutoPilotPitchPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotPitchPIDGains thisArg = simpleRequest $ getAutoPilotPitchPIDGainsReq thisArg

getAutoPilotPitchPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotPitchPIDGainsStreamReq thisArg = makeStreamReq $ getAutoPilotPitchPIDGainsReq thisArg

getAutoPilotPitchPIDGainsStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotPitchPIDGainsStream thisArg = requestAddStream $ getAutoPilotPitchPIDGainsStreamReq thisArg 

{-|
The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).An error will be thrown if this property is set to a reference frame that rotates with the vessel being controlled,
as it is impossible to rotate the vessel in such a reference frame.
 -}
getAutoPilotReferenceFrameReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]

getAutoPilotReferenceFrame :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrame thisArg = simpleRequest $ getAutoPilotReferenceFrameReq thisArg

getAutoPilotReferenceFrameStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrameStreamReq thisArg = makeStreamReq $ getAutoPilotReferenceFrameReq thisArg

getAutoPilotReferenceFrameStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getAutoPilotReferenceFrameStream thisArg = requestAddStream $ getAutoPilotReferenceFrameStreamReq thisArg 

{-|
The error, in degrees, between the vessels current and target roll.
Returns zero if the auto-pilot has not been engaged or no target roll is set.
 -}
getAutoPilotRollErrorReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotRollErrorReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]

getAutoPilotRollError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotRollError thisArg = simpleRequest $ getAutoPilotRollErrorReq thisArg

getAutoPilotRollErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotRollErrorStreamReq thisArg = makeStreamReq $ getAutoPilotRollErrorReq thisArg

getAutoPilotRollErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRollErrorStream thisArg = requestAddStream $ getAutoPilotRollErrorStreamReq thisArg 

{-|
Gains for the roll PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotRollPIDGainsReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotRollPIDGainsReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_RollPIDGains" [makeArgument 0 thisArg]

getAutoPilotRollPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotRollPIDGains thisArg = simpleRequest $ getAutoPilotRollPIDGainsReq thisArg

getAutoPilotRollPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotRollPIDGainsStreamReq thisArg = makeStreamReq $ getAutoPilotRollPIDGainsReq thisArg

getAutoPilotRollPIDGainsStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotRollPIDGainsStream thisArg = requestAddStream $ getAutoPilotRollPIDGainsStreamReq thisArg 

{-|
The threshold at which the autopilot will try to match the target roll angle, if any.
Defaults to 5 degrees.
 -}
getAutoPilotRollThresholdReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Double)
getAutoPilotRollThresholdReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_RollThreshold" [makeArgument 0 thisArg]

getAutoPilotRollThreshold :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Double)
getAutoPilotRollThreshold thisArg = simpleRequest $ getAutoPilotRollThresholdReq thisArg

getAutoPilotRollThresholdStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Double)
getAutoPilotRollThresholdStreamReq thisArg = makeStreamReq $ getAutoPilotRollThresholdReq thisArg

getAutoPilotRollThresholdStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Double))
getAutoPilotRollThresholdStream thisArg = requestAddStream $ getAutoPilotRollThresholdStreamReq thisArg 

{-|
The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
getAutoPilotSASReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Bool)
getAutoPilotSASReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]

getAutoPilotSAS :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
getAutoPilotSAS thisArg = simpleRequest $ getAutoPilotSASReq thisArg

getAutoPilotSASStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Bool)
getAutoPilotSASStreamReq thisArg = makeStreamReq $ getAutoPilotSASReq thisArg

getAutoPilotSASStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Bool))
getAutoPilotSASStream thisArg = requestAddStream $ getAutoPilotSASStreamReq thisArg 

{-|
The current <see cref="T:SpaceCenter.SASMode" />.
These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
getAutoPilotSASModeReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASModeReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]

getAutoPilotSASMode :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASMode thisArg = simpleRequest $ getAutoPilotSASModeReq thisArg

getAutoPilotSASModeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASModeStreamReq thisArg = makeStreamReq $ getAutoPilotSASModeReq thisArg

getAutoPilotSASModeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getAutoPilotSASModeStream thisArg = requestAddStream $ getAutoPilotSASModeStreamReq thisArg 

{-|
The maximum amount of time that the vessel should need to come to a complete stop.
This determines the maximum angular velocity of the vessel.
A vector of three stopping times, in seconds, one for each of the pitch, roll and yaw axes.
Defaults to 0.5 seconds for each axis.
 -}
getAutoPilotStoppingTimeReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotStoppingTimeReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_StoppingTime" [makeArgument 0 thisArg]

getAutoPilotStoppingTime :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotStoppingTime thisArg = simpleRequest $ getAutoPilotStoppingTimeReq thisArg

getAutoPilotStoppingTimeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotStoppingTimeStreamReq thisArg = makeStreamReq $ getAutoPilotStoppingTimeReq thisArg

getAutoPilotStoppingTimeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotStoppingTimeStream thisArg = requestAddStream $ getAutoPilotStoppingTimeStreamReq thisArg 

{-|
Direction vector corresponding to the target pitch and heading.
 -}
getAutoPilotTargetDirectionReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotTargetDirectionReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]

getAutoPilotTargetDirection :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotTargetDirection thisArg = simpleRequest $ getAutoPilotTargetDirectionReq thisArg

getAutoPilotTargetDirectionStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotTargetDirectionStreamReq thisArg = makeStreamReq $ getAutoPilotTargetDirectionReq thisArg

getAutoPilotTargetDirectionStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotTargetDirectionStream thisArg = requestAddStream $ getAutoPilotTargetDirectionStreamReq thisArg 

{-|
The target heading, in degrees, between 0° and 360°.
 -}
getAutoPilotTargetHeadingReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotTargetHeadingReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_TargetHeading" [makeArgument 0 thisArg]

getAutoPilotTargetHeading :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetHeading thisArg = simpleRequest $ getAutoPilotTargetHeadingReq thisArg

getAutoPilotTargetHeadingStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetHeadingStreamReq thisArg = makeStreamReq $ getAutoPilotTargetHeadingReq thisArg

getAutoPilotTargetHeadingStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetHeadingStream thisArg = requestAddStream $ getAutoPilotTargetHeadingStreamReq thisArg 

{-|
The target pitch, in degrees, between -90° and +90°.
 -}
getAutoPilotTargetPitchReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotTargetPitchReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_TargetPitch" [makeArgument 0 thisArg]

getAutoPilotTargetPitch :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetPitch thisArg = simpleRequest $ getAutoPilotTargetPitchReq thisArg

getAutoPilotTargetPitchStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetPitchStreamReq thisArg = makeStreamReq $ getAutoPilotTargetPitchReq thisArg

getAutoPilotTargetPitchStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetPitchStream thisArg = requestAddStream $ getAutoPilotTargetPitchStreamReq thisArg 

{-|
The target roll, in degrees.NaNif no target roll is set.
 -}
getAutoPilotTargetRollReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq (Float)
getAutoPilotTargetRollReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]

getAutoPilotTargetRoll :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetRoll thisArg = simpleRequest $ getAutoPilotTargetRollReq thisArg

getAutoPilotTargetRollStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetRollStreamReq thisArg = makeStreamReq $ getAutoPilotTargetRollReq thisArg

getAutoPilotTargetRollStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetRollStream thisArg = requestAddStream $ getAutoPilotTargetRollStreamReq thisArg 

{-|
The target time to peak used to autotune the PID controllers.
A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
Defaults to 3 seconds for each axis.
 -}
getAutoPilotTimeToPeakReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotTimeToPeakReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_TimeToPeak" [makeArgument 0 thisArg]

getAutoPilotTimeToPeak :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotTimeToPeak thisArg = simpleRequest $ getAutoPilotTimeToPeakReq thisArg

getAutoPilotTimeToPeakStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotTimeToPeakStreamReq thisArg = makeStreamReq $ getAutoPilotTimeToPeakReq thisArg

getAutoPilotTimeToPeakStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotTimeToPeakStream thisArg = requestAddStream $ getAutoPilotTimeToPeakStreamReq thisArg 

{-|
Gains for the yaw PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotYawPIDGainsReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCCallReq ((Double, Double, Double))
getAutoPilotYawPIDGainsReq thisArg = makeCallReq "SpaceCenter" "AutoPilot_get_YawPIDGains" [makeArgument 0 thisArg]

getAutoPilotYawPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotYawPIDGains thisArg = simpleRequest $ getAutoPilotYawPIDGainsReq thisArg

getAutoPilotYawPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotYawPIDGainsStreamReq thisArg = makeStreamReq $ getAutoPilotYawPIDGainsReq thisArg

getAutoPilotYawPIDGainsStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotYawPIDGainsStream thisArg = requestAddStream $ getAutoPilotYawPIDGainsStreamReq thisArg 

{-|
The angle at which the autopilot considers the vessel to be pointing close to the target.
This determines the midpoint of the target velocity attenuation function.
A vector of three angles, in degrees, one for each of the pitch, roll and yaw axes.
Defaults to 1° for each axis.
 -}
setAutoPilotAttenuationAngleReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotAttenuationAngleReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_AttenuationAngle" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotAttenuationAngle :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotAttenuationAngle thisArg valueArg = simpleRequest $ setAutoPilotAttenuationAngleReq thisArg valueArg 

{-|
Whether the rotation rate controllers PID parameters should be automatically tuned using the
vessels moment of inertia and available torque. Defaults totrue.
See <see cref="M:SpaceCenter.AutoPilot.TimeToPeak" /> and  <see cref="M:SpaceCenter.AutoPilot.Overshoot" />.
 -}
setAutoPilotAutoTuneReq :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> KRPCCallReq ()
setAutoPilotAutoTuneReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_AutoTune" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotAutoTune :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> RPCContext ()
setAutoPilotAutoTune thisArg valueArg = simpleRequest $ setAutoPilotAutoTuneReq thisArg valueArg 

{-|
The time the vessel should take to come to a stop pointing in the target direction.
This determines the angular acceleration used to decelerate the vessel.
A vector of three times, in seconds, one for each of the pitch, roll and yaw axes.
Defaults to 5 seconds for each axis.
 -}
setAutoPilotDecelerationTimeReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotDecelerationTimeReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_DecelerationTime" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotDecelerationTime :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotDecelerationTime thisArg valueArg = simpleRequest $ setAutoPilotDecelerationTimeReq thisArg valueArg 

{-|
The target overshoot percentage used to autotune the PID controllers.
A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
Defaults to 0.01 for each axis.
 -}
setAutoPilotOvershootReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotOvershootReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_Overshoot" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotOvershoot :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotOvershoot thisArg valueArg = simpleRequest $ setAutoPilotOvershootReq thisArg valueArg 

{-|
Gains for the pitch PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotPitchPIDGainsReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotPitchPIDGainsReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_PitchPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotPitchPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotPitchPIDGains thisArg valueArg = simpleRequest $ setAutoPilotPitchPIDGainsReq thisArg valueArg 

{-|
The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).An error will be thrown if this property is set to a reference frame that rotates with the vessel being controlled,
as it is impossible to rotate the vessel in such a reference frame.
 -}
setAutoPilotReferenceFrameReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ()
setAutoPilotReferenceFrameReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotReferenceFrame :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setAutoPilotReferenceFrame thisArg valueArg = simpleRequest $ setAutoPilotReferenceFrameReq thisArg valueArg 

{-|
Gains for the roll PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotRollPIDGainsReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotRollPIDGainsReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_RollPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotRollPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotRollPIDGains thisArg valueArg = simpleRequest $ setAutoPilotRollPIDGainsReq thisArg valueArg 

{-|
The threshold at which the autopilot will try to match the target roll angle, if any.
Defaults to 5 degrees.
 -}
setAutoPilotRollThresholdReq :: KRPCHS.SpaceCenter.AutoPilot -> Double -> KRPCCallReq ()
setAutoPilotRollThresholdReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_RollThreshold" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotRollThreshold :: KRPCHS.SpaceCenter.AutoPilot -> Double -> RPCContext ()
setAutoPilotRollThreshold thisArg valueArg = simpleRequest $ setAutoPilotRollThresholdReq thisArg valueArg 

{-|
The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
setAutoPilotSASReq :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> KRPCCallReq ()
setAutoPilotSASReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotSAS :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> RPCContext ()
setAutoPilotSAS thisArg valueArg = simpleRequest $ setAutoPilotSASReq thisArg valueArg 

{-|
The current <see cref="T:SpaceCenter.SASMode" />.
These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
setAutoPilotSASModeReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.SASMode -> KRPCCallReq ()
setAutoPilotSASModeReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotSASMode :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.SASMode -> RPCContext ()
setAutoPilotSASMode thisArg valueArg = simpleRequest $ setAutoPilotSASModeReq thisArg valueArg 

{-|
The maximum amount of time that the vessel should need to come to a complete stop.
This determines the maximum angular velocity of the vessel.
A vector of three stopping times, in seconds, one for each of the pitch, roll and yaw axes.
Defaults to 0.5 seconds for each axis.
 -}
setAutoPilotStoppingTimeReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotStoppingTimeReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_StoppingTime" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotStoppingTime :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotStoppingTime thisArg valueArg = simpleRequest $ setAutoPilotStoppingTimeReq thisArg valueArg 

{-|
Direction vector corresponding to the target pitch and heading.
 -}
setAutoPilotTargetDirectionReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotTargetDirectionReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_TargetDirection" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotTargetDirection :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotTargetDirection thisArg valueArg = simpleRequest $ setAutoPilotTargetDirectionReq thisArg valueArg 

{-|
The target heading, in degrees, between 0° and 360°.
 -}
setAutoPilotTargetHeadingReq :: KRPCHS.SpaceCenter.AutoPilot -> Float -> KRPCCallReq ()
setAutoPilotTargetHeadingReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_TargetHeading" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotTargetHeading :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext ()
setAutoPilotTargetHeading thisArg valueArg = simpleRequest $ setAutoPilotTargetHeadingReq thisArg valueArg 

{-|
The target pitch, in degrees, between -90° and +90°.
 -}
setAutoPilotTargetPitchReq :: KRPCHS.SpaceCenter.AutoPilot -> Float -> KRPCCallReq ()
setAutoPilotTargetPitchReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_TargetPitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotTargetPitch :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext ()
setAutoPilotTargetPitch thisArg valueArg = simpleRequest $ setAutoPilotTargetPitchReq thisArg valueArg 

{-|
The target roll, in degrees.NaNif no target roll is set.
 -}
setAutoPilotTargetRollReq :: KRPCHS.SpaceCenter.AutoPilot -> Float -> KRPCCallReq ()
setAutoPilotTargetRollReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_TargetRoll" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotTargetRoll :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext ()
setAutoPilotTargetRoll thisArg valueArg = simpleRequest $ setAutoPilotTargetRollReq thisArg valueArg 

{-|
The target time to peak used to autotune the PID controllers.
A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
Defaults to 3 seconds for each axis.
 -}
setAutoPilotTimeToPeakReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotTimeToPeakReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_TimeToPeak" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotTimeToPeak :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotTimeToPeak thisArg valueArg = simpleRequest $ setAutoPilotTimeToPeakReq thisArg valueArg 

{-|
Gains for the yaw PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotYawPIDGainsReq :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> KRPCCallReq ()
setAutoPilotYawPIDGainsReq thisArg valueArg = makeCallReq "SpaceCenter" "AutoPilot_set_YawPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setAutoPilotYawPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotYawPIDGains thisArg valueArg = simpleRequest $ setAutoPilotYawPIDGainsReq thisArg valueArg 

{-|
Default distance from the camera to the subject, in meters.
 -}
getCameraDefaultDistanceReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraDefaultDistanceReq thisArg = makeCallReq "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]

getCameraDefaultDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraDefaultDistance thisArg = simpleRequest $ getCameraDefaultDistanceReq thisArg

getCameraDefaultDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraDefaultDistanceStreamReq thisArg = makeStreamReq $ getCameraDefaultDistanceReq thisArg

getCameraDefaultDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraDefaultDistanceStream thisArg = requestAddStream $ getCameraDefaultDistanceStreamReq thisArg 

{-|
The distance from the camera to the subject, in meters.
A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
getCameraDistanceReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraDistanceReq thisArg = makeCallReq "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]

getCameraDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraDistance thisArg = simpleRequest $ getCameraDistanceReq thisArg

getCameraDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraDistanceStreamReq thisArg = makeStreamReq $ getCameraDistanceReq thisArg

getCameraDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraDistanceStream thisArg = requestAddStream $ getCameraDistanceStreamReq thisArg 

{-|
In map mode, the celestial body that the camera is focussed on.
Returnsnullif the camera is not focussed on a celestial body.
Returns an error is the camera is not in map mode.
 -}
getCameraFocussedBodyReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBodyReq thisArg = makeCallReq "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]

getCameraFocussedBody :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBody thisArg = simpleRequest $ getCameraFocussedBodyReq thisArg

getCameraFocussedBodyStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBodyStreamReq thisArg = makeStreamReq $ getCameraFocussedBodyReq thisArg

getCameraFocussedBodyStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getCameraFocussedBodyStream thisArg = requestAddStream $ getCameraFocussedBodyStreamReq thisArg 

{-|
In map mode, the maneuver node that the camera is focussed on.
Returnsnullif the camera is not focussed on a maneuver node.
Returns an error is the camera is not in map mode.
 -}
getCameraFocussedNodeReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (KRPCHS.SpaceCenter.Node)
getCameraFocussedNodeReq thisArg = makeCallReq "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]

getCameraFocussedNode :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.Node)
getCameraFocussedNode thisArg = simpleRequest $ getCameraFocussedNodeReq thisArg

getCameraFocussedNodeStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.Node)
getCameraFocussedNodeStreamReq thisArg = makeStreamReq $ getCameraFocussedNodeReq thisArg

getCameraFocussedNodeStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Node))
getCameraFocussedNodeStream thisArg = requestAddStream $ getCameraFocussedNodeStreamReq thisArg 

{-|
In map mode, the vessel that the camera is focussed on.
Returnsnullif the camera is not focussed on a vessel.
Returns an error is the camera is not in map mode.
 -}
getCameraFocussedVesselReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVesselReq thisArg = makeCallReq "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]

getCameraFocussedVessel :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVessel thisArg = simpleRequest $ getCameraFocussedVesselReq thisArg

getCameraFocussedVesselStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVesselStreamReq thisArg = makeStreamReq $ getCameraFocussedVesselReq thisArg

getCameraFocussedVesselStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCameraFocussedVesselStream thisArg = requestAddStream $ getCameraFocussedVesselStreamReq thisArg 

{-|
The heading of the camera, in degrees.
 -}
getCameraHeadingReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraHeadingReq thisArg = makeCallReq "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]

getCameraHeading :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraHeading thisArg = simpleRequest $ getCameraHeadingReq thisArg

getCameraHeadingStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraHeadingStreamReq thisArg = makeStreamReq $ getCameraHeadingReq thisArg

getCameraHeadingStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraHeadingStream thisArg = requestAddStream $ getCameraHeadingStreamReq thisArg 

{-|
Maximum distance from the camera to the subject, in meters.
 -}
getCameraMaxDistanceReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraMaxDistanceReq thisArg = makeCallReq "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]

getCameraMaxDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMaxDistance thisArg = simpleRequest $ getCameraMaxDistanceReq thisArg

getCameraMaxDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMaxDistanceStreamReq thisArg = makeStreamReq $ getCameraMaxDistanceReq thisArg

getCameraMaxDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMaxDistanceStream thisArg = requestAddStream $ getCameraMaxDistanceStreamReq thisArg 

{-|
The maximum pitch of the camera.
 -}
getCameraMaxPitchReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraMaxPitchReq thisArg = makeCallReq "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]

getCameraMaxPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMaxPitch thisArg = simpleRequest $ getCameraMaxPitchReq thisArg

getCameraMaxPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMaxPitchStreamReq thisArg = makeStreamReq $ getCameraMaxPitchReq thisArg

getCameraMaxPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMaxPitchStream thisArg = requestAddStream $ getCameraMaxPitchStreamReq thisArg 

{-|
Minimum distance from the camera to the subject, in meters.
 -}
getCameraMinDistanceReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraMinDistanceReq thisArg = makeCallReq "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]

getCameraMinDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMinDistance thisArg = simpleRequest $ getCameraMinDistanceReq thisArg

getCameraMinDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMinDistanceStreamReq thisArg = makeStreamReq $ getCameraMinDistanceReq thisArg

getCameraMinDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMinDistanceStream thisArg = requestAddStream $ getCameraMinDistanceStreamReq thisArg 

{-|
The minimum pitch of the camera.
 -}
getCameraMinPitchReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraMinPitchReq thisArg = makeCallReq "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]

getCameraMinPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMinPitch thisArg = simpleRequest $ getCameraMinPitchReq thisArg

getCameraMinPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMinPitchStreamReq thisArg = makeStreamReq $ getCameraMinPitchReq thisArg

getCameraMinPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMinPitchStream thisArg = requestAddStream $ getCameraMinPitchStreamReq thisArg 

{-|
The current mode of the camera.
 -}
getCameraModeReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (KRPCHS.SpaceCenter.CameraMode)
getCameraModeReq thisArg = makeCallReq "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]

getCameraMode :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.CameraMode)
getCameraMode thisArg = simpleRequest $ getCameraModeReq thisArg

getCameraModeStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.CameraMode)
getCameraModeStreamReq thisArg = makeStreamReq $ getCameraModeReq thisArg

getCameraModeStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CameraMode))
getCameraModeStream thisArg = requestAddStream $ getCameraModeStreamReq thisArg 

{-|
The pitch of the camera, in degrees.
A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
getCameraPitchReq :: KRPCHS.SpaceCenter.Camera -> KRPCCallReq (Float)
getCameraPitchReq thisArg = makeCallReq "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]

getCameraPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraPitch thisArg = simpleRequest $ getCameraPitchReq thisArg

getCameraPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraPitchStreamReq thisArg = makeStreamReq $ getCameraPitchReq thisArg

getCameraPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraPitchStream thisArg = requestAddStream $ getCameraPitchStreamReq thisArg 

{-|
The distance from the camera to the subject, in meters.
A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
setCameraDistanceReq :: KRPCHS.SpaceCenter.Camera -> Float -> KRPCCallReq ()
setCameraDistanceReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_Distance" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraDistance :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext ()
setCameraDistance thisArg valueArg = simpleRequest $ setCameraDistanceReq thisArg valueArg 

{-|
In map mode, the celestial body that the camera is focussed on.
Returnsnullif the camera is not focussed on a celestial body.
Returns an error is the camera is not in map mode.
 -}
setCameraFocussedBodyReq :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ()
setCameraFocussedBodyReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_FocussedBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraFocussedBody :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setCameraFocussedBody thisArg valueArg = simpleRequest $ setCameraFocussedBodyReq thisArg valueArg 

{-|
In map mode, the maneuver node that the camera is focussed on.
Returnsnullif the camera is not focussed on a maneuver node.
Returns an error is the camera is not in map mode.
 -}
setCameraFocussedNodeReq :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Node -> KRPCCallReq ()
setCameraFocussedNodeReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_FocussedNode" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraFocussedNode :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Node -> RPCContext ()
setCameraFocussedNode thisArg valueArg = simpleRequest $ setCameraFocussedNodeReq thisArg valueArg 

{-|
In map mode, the vessel that the camera is focussed on.
Returnsnullif the camera is not focussed on a vessel.
Returns an error is the camera is not in map mode.
 -}
setCameraFocussedVesselReq :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ()
setCameraFocussedVesselReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_FocussedVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraFocussedVessel :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setCameraFocussedVessel thisArg valueArg = simpleRequest $ setCameraFocussedVesselReq thisArg valueArg 

{-|
The heading of the camera, in degrees.
 -}
setCameraHeadingReq :: KRPCHS.SpaceCenter.Camera -> Float -> KRPCCallReq ()
setCameraHeadingReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_Heading" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraHeading :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext ()
setCameraHeading thisArg valueArg = simpleRequest $ setCameraHeadingReq thisArg valueArg 

{-|
The current mode of the camera.
 -}
setCameraModeReq :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CameraMode -> KRPCCallReq ()
setCameraModeReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraMode :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CameraMode -> RPCContext ()
setCameraMode thisArg valueArg = simpleRequest $ setCameraModeReq thisArg valueArg 

{-|
The pitch of the camera, in degrees.
A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
setCameraPitchReq :: KRPCHS.SpaceCenter.Camera -> Float -> KRPCCallReq ()
setCameraPitchReq thisArg valueArg = makeCallReq "SpaceCenter" "Camera_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCameraPitch :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext ()
setCameraPitch thisArg valueArg = simpleRequest $ setCameraPitchReq thisArg valueArg 

{-|
Returnstrueif regular "on-rails" time warp can be used, at the specified warp
<paramref name="factor" />. The maximum time warp rate is limited by various things,
including how close the active vessel is to a planet. See
<a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">the KSP wikifor details.<param name="factor">The warp factor to check.
 -}
canRailsWarpAtReq :: Data.Int.Int32 -> KRPCCallReq (Bool)
canRailsWarpAtReq factorArg = makeCallReq "SpaceCenter" "CanRailsWarpAt" [makeArgument 0 factorArg]

canRailsWarpAt :: Data.Int.Int32 -> RPCContext (Bool)
canRailsWarpAt factorArg = simpleRequest $ canRailsWarpAtReq factorArg

canRailsWarpAtStreamReq :: Data.Int.Int32 -> KRPCStreamReq (Bool)
canRailsWarpAtStreamReq factorArg = makeStreamReq $ canRailsWarpAtReq factorArg

canRailsWarpAtStream :: Data.Int.Int32 -> RPCContext (KRPCStream (Bool))
canRailsWarpAtStream factorArg = requestAddStream $ canRailsWarpAtStreamReq factorArg 

{-|
Whether the cargo bay is open.
 -}
getCargoBayOpenReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCCallReq (Bool)
getCargoBayOpenReq thisArg = makeCallReq "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]

getCargoBayOpen :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (Bool)
getCargoBayOpen thisArg = simpleRequest $ getCargoBayOpenReq thisArg

getCargoBayOpenStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (Bool)
getCargoBayOpenStreamReq thisArg = makeStreamReq $ getCargoBayOpenReq thisArg

getCargoBayOpenStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (Bool))
getCargoBayOpenStream thisArg = requestAddStream $ getCargoBayOpenStreamReq thisArg 

{-|
The part object for this cargo bay.
 -}
getCargoBayPartReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getCargoBayPartReq thisArg = makeCallReq "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]

getCargoBayPart :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCHS.SpaceCenter.Part)
getCargoBayPart thisArg = simpleRequest $ getCargoBayPartReq thisArg

getCargoBayPartStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getCargoBayPartStreamReq thisArg = makeStreamReq $ getCargoBayPartReq thisArg

getCargoBayPartStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getCargoBayPartStream thisArg = requestAddStream $ getCargoBayPartStreamReq thisArg 

{-|
The state of the cargo bay.
 -}
getCargoBayStateReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCCallReq (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayStateReq thisArg = makeCallReq "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]

getCargoBayState :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayState thisArg = simpleRequest $ getCargoBayStateReq thisArg

getCargoBayStateStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayStateStreamReq thisArg = makeStreamReq $ getCargoBayStateReq thisArg

getCargoBayStateStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CargoBayState))
getCargoBayStateStream thisArg = requestAddStream $ getCargoBayStateStreamReq thisArg 

{-|
Whether the cargo bay is open.
 -}
setCargoBayOpenReq :: KRPCHS.SpaceCenter.CargoBay -> Bool -> KRPCCallReq ()
setCargoBayOpenReq thisArg valueArg = makeCallReq "SpaceCenter" "CargoBay_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCargoBayOpen :: KRPCHS.SpaceCenter.CargoBay -> Bool -> RPCContext ()
setCargoBayOpen thisArg valueArg = simpleRequest $ setCargoBayOpenReq thisArg valueArg 

{-|
Returns the angular velocity of the body in the specified reference
frame. The magnitude of the vector is the rotational speed of the body, in
radians per second, and the direction of the vector indicates the axis of
rotation, using the right-hand rule.<param name="referenceFrame">
 -}
celestialBodyAngularVelocityReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodyAngularVelocityReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

celestialBodyAngularVelocity :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyAngularVelocity thisArg referenceFrameArg = simpleRequest $ celestialBodyAngularVelocityReq thisArg referenceFrameArg

celestialBodyAngularVelocityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyAngularVelocityStreamReq thisArg referenceFrameArg = makeStreamReq $ celestialBodyAngularVelocityReq thisArg referenceFrameArg

celestialBodyAngularVelocityStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyAngularVelocityStream thisArg referenceFrameArg = requestAddStream $ celestialBodyAngularVelocityStreamReq thisArg referenceFrameArg 

{-|
The height of the surface relative to mean sea level at the given position,
in meters. When over water, this is the height of the sea-bed and is therefore a
negative value.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodyBedrockHeightReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCCallReq (Double)
celestialBodyBedrockHeightReq thisArg latitudeArg longitudeArg = makeCallReq "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]

celestialBodyBedrockHeight :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodyBedrockHeight thisArg latitudeArg longitudeArg = simpleRequest $ celestialBodyBedrockHeightReq thisArg latitudeArg longitudeArg

celestialBodyBedrockHeightStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Double)
celestialBodyBedrockHeightStreamReq thisArg latitudeArg longitudeArg = makeStreamReq $ celestialBodyBedrockHeightReq thisArg latitudeArg longitudeArg

celestialBodyBedrockHeightStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodyBedrockHeightStream thisArg latitudeArg longitudeArg = requestAddStream $ celestialBodyBedrockHeightStreamReq thisArg latitudeArg longitudeArg 

{-|
The position of the surface at the given latitude and longitude, in the given
reference frame. When over water, this is the position at the bottom of the sea-bed.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyBedrockPositionReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodyBedrockPositionReq thisArg latitudeArg longitudeArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]

celestialBodyBedrockPosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyBedrockPosition thisArg latitudeArg longitudeArg referenceFrameArg = simpleRequest $ celestialBodyBedrockPositionReq thisArg latitudeArg longitudeArg referenceFrameArg

celestialBodyBedrockPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyBedrockPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg = makeStreamReq $ celestialBodyBedrockPositionReq thisArg latitudeArg longitudeArg referenceFrameArg

celestialBodyBedrockPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyBedrockPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestAddStream $ celestialBodyBedrockPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-|
The biomes at the given latitude and longitude, in degrees.
 -}
celestialBodyBiomeAtReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCCallReq (Data.Text.Text)
celestialBodyBiomeAtReq thisArg latitudeArg longitudeArg = makeCallReq "SpaceCenter" "CelestialBody_BiomeAt" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]

celestialBodyBiomeAt :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Data.Text.Text)
celestialBodyBiomeAt thisArg latitudeArg longitudeArg = simpleRequest $ celestialBodyBiomeAtReq thisArg latitudeArg longitudeArg

celestialBodyBiomeAtStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Data.Text.Text)
celestialBodyBiomeAtStreamReq thisArg latitudeArg longitudeArg = makeStreamReq $ celestialBodyBiomeAtReq thisArg latitudeArg longitudeArg

celestialBodyBiomeAtStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Data.Text.Text))
celestialBodyBiomeAtStream thisArg latitudeArg longitudeArg = requestAddStream $ celestialBodyBiomeAtStreamReq thisArg latitudeArg longitudeArg 

{-|
Returns the direction in which the north pole of the celestial body is
pointing, as a unit vector, in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyDirectionReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodyDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

celestialBodyDirection :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyDirection thisArg referenceFrameArg = simpleRequest $ celestialBodyDirectionReq thisArg referenceFrameArg

celestialBodyDirectionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ celestialBodyDirectionReq thisArg referenceFrameArg

celestialBodyDirectionStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyDirectionStream thisArg referenceFrameArg = requestAddStream $ celestialBodyDirectionStreamReq thisArg referenceFrameArg 

{-|
The position at mean sea level at the given latitude and longitude, in the given reference frame.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyMSLPositionReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodyMSLPositionReq thisArg latitudeArg longitudeArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]

celestialBodyMSLPosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyMSLPosition thisArg latitudeArg longitudeArg referenceFrameArg = simpleRequest $ celestialBodyMSLPositionReq thisArg latitudeArg longitudeArg referenceFrameArg

celestialBodyMSLPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyMSLPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg = makeStreamReq $ celestialBodyMSLPositionReq thisArg latitudeArg longitudeArg referenceFrameArg

celestialBodyMSLPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyMSLPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestAddStream $ celestialBodyMSLPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-|
Returns the position vector of the center of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyPositionReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodyPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

celestialBodyPosition :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyPosition thisArg referenceFrameArg = simpleRequest $ celestialBodyPositionReq thisArg referenceFrameArg

celestialBodyPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ celestialBodyPositionReq thisArg referenceFrameArg

celestialBodyPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyPositionStream thisArg referenceFrameArg = requestAddStream $ celestialBodyPositionStreamReq thisArg referenceFrameArg 

{-|
Returns the rotation of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyRotationReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double, Double))
celestialBodyRotationReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

celestialBodyRotation :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
celestialBodyRotation thisArg referenceFrameArg = simpleRequest $ celestialBodyRotationReq thisArg referenceFrameArg

celestialBodyRotationStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
celestialBodyRotationStreamReq thisArg referenceFrameArg = makeStreamReq $ celestialBodyRotationReq thisArg referenceFrameArg

celestialBodyRotationStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
celestialBodyRotationStream thisArg referenceFrameArg = requestAddStream $ celestialBodyRotationStreamReq thisArg referenceFrameArg 

{-|
The height of the surface relative to mean sea level at the given position,
in meters. When over water this is equal to 0.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodySurfaceHeightReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCCallReq (Double)
celestialBodySurfaceHeightReq thisArg latitudeArg longitudeArg = makeCallReq "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]

celestialBodySurfaceHeight :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodySurfaceHeight thisArg latitudeArg longitudeArg = simpleRequest $ celestialBodySurfaceHeightReq thisArg latitudeArg longitudeArg

celestialBodySurfaceHeightStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Double)
celestialBodySurfaceHeightStreamReq thisArg latitudeArg longitudeArg = makeStreamReq $ celestialBodySurfaceHeightReq thisArg latitudeArg longitudeArg

celestialBodySurfaceHeightStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodySurfaceHeightStream thisArg latitudeArg longitudeArg = requestAddStream $ celestialBodySurfaceHeightStreamReq thisArg latitudeArg longitudeArg 

{-|
The position of the surface at the given latitude and longitude, in the given
reference frame. When over water, this is the position of the surface of the water.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodySurfacePositionReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodySurfacePositionReq thisArg latitudeArg longitudeArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]

celestialBodySurfacePosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodySurfacePosition thisArg latitudeArg longitudeArg referenceFrameArg = simpleRequest $ celestialBodySurfacePositionReq thisArg latitudeArg longitudeArg referenceFrameArg

celestialBodySurfacePositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodySurfacePositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg = makeStreamReq $ celestialBodySurfacePositionReq thisArg latitudeArg longitudeArg referenceFrameArg

celestialBodySurfacePositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodySurfacePositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestAddStream $ celestialBodySurfacePositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-|
Returns the velocity vector of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyVelocityReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
celestialBodyVelocityReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

celestialBodyVelocity :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyVelocity thisArg referenceFrameArg = simpleRequest $ celestialBodyVelocityReq thisArg referenceFrameArg

celestialBodyVelocityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyVelocityStreamReq thisArg referenceFrameArg = makeStreamReq $ celestialBodyVelocityReq thisArg referenceFrameArg

celestialBodyVelocityStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyVelocityStream thisArg referenceFrameArg = requestAddStream $ celestialBodyVelocityStreamReq thisArg referenceFrameArg 

{-|
The depth of the atmosphere, in meters.
 -}
getCelestialBodyAtmosphereDepthReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyAtmosphereDepthReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]

getCelestialBodyAtmosphereDepth :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyAtmosphereDepth thisArg = simpleRequest $ getCelestialBodyAtmosphereDepthReq thisArg

getCelestialBodyAtmosphereDepthStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyAtmosphereDepthStreamReq thisArg = makeStreamReq $ getCelestialBodyAtmosphereDepthReq thisArg

getCelestialBodyAtmosphereDepthStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyAtmosphereDepthStream thisArg = requestAddStream $ getCelestialBodyAtmosphereDepthStreamReq thisArg 

{-|
The biomes present on this body.
 -}
getCelestialBodyBiomesReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ([Data.Text.Text])
getCelestialBodyBiomesReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_Biomes" [makeArgument 0 thisArg]

getCelestialBodyBiomes :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext ([Data.Text.Text])
getCelestialBodyBiomes thisArg = simpleRequest $ getCelestialBodyBiomesReq thisArg

getCelestialBodyBiomesStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq ([Data.Text.Text])
getCelestialBodyBiomesStreamReq thisArg = makeStreamReq $ getCelestialBodyBiomesReq thisArg

getCelestialBodyBiomesStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream ([Data.Text.Text]))
getCelestialBodyBiomesStream thisArg = requestAddStream $ getCelestialBodyBiomesStreamReq thisArg 

{-|
The equatorial radius of the body, in meters.
 -}
getCelestialBodyEquatorialRadiusReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyEquatorialRadiusReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]

getCelestialBodyEquatorialRadius :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyEquatorialRadius thisArg = simpleRequest $ getCelestialBodyEquatorialRadiusReq thisArg

getCelestialBodyEquatorialRadiusStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyEquatorialRadiusStreamReq thisArg = makeStreamReq $ getCelestialBodyEquatorialRadiusReq thisArg

getCelestialBodyEquatorialRadiusStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyEquatorialRadiusStream thisArg = requestAddStream $ getCelestialBodyEquatorialRadiusStreamReq thisArg 

{-|
The altitude, in meters, above which a vessel is considered to be flying "high" when doing science.
 -}
getCelestialBodyFlyingHighAltitudeThresholdReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyFlyingHighAltitudeThresholdReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_FlyingHighAltitudeThreshold" [makeArgument 0 thisArg]

getCelestialBodyFlyingHighAltitudeThreshold :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyFlyingHighAltitudeThreshold thisArg = simpleRequest $ getCelestialBodyFlyingHighAltitudeThresholdReq thisArg

getCelestialBodyFlyingHighAltitudeThresholdStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyFlyingHighAltitudeThresholdStreamReq thisArg = makeStreamReq $ getCelestialBodyFlyingHighAltitudeThresholdReq thisArg

getCelestialBodyFlyingHighAltitudeThresholdStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyFlyingHighAltitudeThresholdStream thisArg = requestAddStream $ getCelestialBodyFlyingHighAltitudeThresholdStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Standard_gravitational_parameter">standard
gravitational parameterof the body inm^3s^{ -2}.
 -}
getCelestialBodyGravitationalParameterReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyGravitationalParameterReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]

getCelestialBodyGravitationalParameter :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyGravitationalParameter thisArg = simpleRequest $ getCelestialBodyGravitationalParameterReq thisArg

getCelestialBodyGravitationalParameterStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyGravitationalParameterStreamReq thisArg = makeStreamReq $ getCelestialBodyGravitationalParameterReq thisArg

getCelestialBodyGravitationalParameterStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyGravitationalParameterStream thisArg = requestAddStream $ getCelestialBodyGravitationalParameterStreamReq thisArg 

{-|
trueif the body has an atmosphere.
 -}
getCelestialBodyHasAtmosphereReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Bool)
getCelestialBodyHasAtmosphereReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]

getCelestialBodyHasAtmosphere :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphere thisArg = simpleRequest $ getCelestialBodyHasAtmosphereReq thisArg

getCelestialBodyHasAtmosphereStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Bool)
getCelestialBodyHasAtmosphereStreamReq thisArg = makeStreamReq $ getCelestialBodyHasAtmosphereReq thisArg

getCelestialBodyHasAtmosphereStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphereStream thisArg = requestAddStream $ getCelestialBodyHasAtmosphereStreamReq thisArg 

{-|
trueif there is oxygen in the atmosphere, required for air-breathing engines.
 -}
getCelestialBodyHasAtmosphericOxygenReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Bool)
getCelestialBodyHasAtmosphericOxygenReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]

getCelestialBodyHasAtmosphericOxygen :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphericOxygen thisArg = simpleRequest $ getCelestialBodyHasAtmosphericOxygenReq thisArg

getCelestialBodyHasAtmosphericOxygenStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Bool)
getCelestialBodyHasAtmosphericOxygenStreamReq thisArg = makeStreamReq $ getCelestialBodyHasAtmosphericOxygenReq thisArg

getCelestialBodyHasAtmosphericOxygenStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphericOxygenStream thisArg = requestAddStream $ getCelestialBodyHasAtmosphericOxygenStreamReq thisArg 

{-|
The mass of the body, in kilograms.
 -}
getCelestialBodyMassReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyMassReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]

getCelestialBodyMass :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyMass thisArg = simpleRequest $ getCelestialBodyMassReq thisArg

getCelestialBodyMassStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyMassStreamReq thisArg = makeStreamReq $ getCelestialBodyMassReq thisArg

getCelestialBodyMassStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyMassStream thisArg = requestAddStream $ getCelestialBodyMassStreamReq thisArg 

{-|
The name of the body.
 -}
getCelestialBodyNameReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Data.Text.Text)
getCelestialBodyNameReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]

getCelestialBodyName :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Data.Text.Text)
getCelestialBodyName thisArg = simpleRequest $ getCelestialBodyNameReq thisArg

getCelestialBodyNameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Data.Text.Text)
getCelestialBodyNameStreamReq thisArg = makeStreamReq $ getCelestialBodyNameReq thisArg

getCelestialBodyNameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Data.Text.Text))
getCelestialBodyNameStream thisArg = requestAddStream $ getCelestialBodyNameStreamReq thisArg 

{-|
The reference frame that is fixed relative to this celestial body, and
orientated in a fixed direction (it does not rotate with the body).
<list type="bullet">The origin is at the center of the body.The axes do not rotate.The x-axis points in an arbitrary direction through the
equator.The y-axis points from the center of the body towards
the north pole.The z-axis points in an arbitrary direction through the
equator.
 -}
getCelestialBodyNonRotatingReferenceFrameReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [makeArgument 0 thisArg]

getCelestialBodyNonRotatingReferenceFrame :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrame thisArg = simpleRequest $ getCelestialBodyNonRotatingReferenceFrameReq thisArg

getCelestialBodyNonRotatingReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrameStreamReq thisArg = makeStreamReq $ getCelestialBodyNonRotatingReferenceFrameReq thisArg

getCelestialBodyNonRotatingReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyNonRotatingReferenceFrameStream thisArg = requestAddStream $ getCelestialBodyNonRotatingReferenceFrameStreamReq thisArg 

{-|
The orbit of the body.
 -}
getCelestialBodyOrbitReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbitReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]

getCelestialBodyOrbit :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbit thisArg = simpleRequest $ getCelestialBodyOrbitReq thisArg

getCelestialBodyOrbitStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbitStreamReq thisArg = makeStreamReq $ getCelestialBodyOrbitReq thisArg

getCelestialBodyOrbitStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getCelestialBodyOrbitStream thisArg = requestAddStream $ getCelestialBodyOrbitStreamReq thisArg 

{-|
Gets the reference frame that is fixed relative to this celestial body, but
orientated with the body's orbital prograde/normal/radial directions.
<list type="bullet">The origin is at the center of the body.The axes rotate with the orbital prograde/normal/radial
directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.
 -}
getCelestialBodyOrbitalReferenceFrameReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]

getCelestialBodyOrbitalReferenceFrame :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrame thisArg = simpleRequest $ getCelestialBodyOrbitalReferenceFrameReq thisArg

getCelestialBodyOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrameStreamReq thisArg = makeStreamReq $ getCelestialBodyOrbitalReferenceFrameReq thisArg

getCelestialBodyOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyOrbitalReferenceFrameStream thisArg = requestAddStream $ getCelestialBodyOrbitalReferenceFrameStreamReq thisArg 

{-|
The reference frame that is fixed relative to the celestial body.
<list type="bullet">The origin is at the center of the body.The axes rotate with the body.The x-axis points from the center of the body
towards the intersection of the prime meridian and equator (the
position at 0° longitude, 0° latitude).The y-axis points from the center of the body
towards the north pole.The z-axis points from the center of the body
towards the equator at 90°E longitude.
 -}
getCelestialBodyReferenceFrameReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_ReferenceFrame" [makeArgument 0 thisArg]

getCelestialBodyReferenceFrame :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrame thisArg = simpleRequest $ getCelestialBodyReferenceFrameReq thisArg

getCelestialBodyReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrameStreamReq thisArg = makeStreamReq $ getCelestialBodyReferenceFrameReq thisArg

getCelestialBodyReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyReferenceFrameStream thisArg = requestAddStream $ getCelestialBodyReferenceFrameStreamReq thisArg 

{-|
The sidereal rotational period of the body, in seconds.
 -}
getCelestialBodyRotationalPeriodReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyRotationalPeriodReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]

getCelestialBodyRotationalPeriod :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalPeriod thisArg = simpleRequest $ getCelestialBodyRotationalPeriodReq thisArg

getCelestialBodyRotationalPeriodStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyRotationalPeriodStreamReq thisArg = makeStreamReq $ getCelestialBodyRotationalPeriodReq thisArg

getCelestialBodyRotationalPeriodStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalPeriodStream thisArg = requestAddStream $ getCelestialBodyRotationalPeriodStreamReq thisArg 

{-|
The rotational speed of the body, in radians per second.
 -}
getCelestialBodyRotationalSpeedReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodyRotationalSpeedReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]

getCelestialBodyRotationalSpeed :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalSpeed thisArg = simpleRequest $ getCelestialBodyRotationalSpeedReq thisArg

getCelestialBodyRotationalSpeedStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyRotationalSpeedStreamReq thisArg = makeStreamReq $ getCelestialBodyRotationalSpeedReq thisArg

getCelestialBodyRotationalSpeedStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalSpeedStream thisArg = requestAddStream $ getCelestialBodyRotationalSpeedStreamReq thisArg 

{-|
A list of celestial bodies that are in orbit around this celestial body.
 -}
getCelestialBodySatellitesReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellitesReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]

getCelestialBodySatellites :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellites thisArg = simpleRequest $ getCelestialBodySatellitesReq thisArg

getCelestialBodySatellitesStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellitesStreamReq thisArg = makeStreamReq $ getCelestialBodySatellitesReq thisArg

getCelestialBodySatellitesStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.CelestialBody]))
getCelestialBodySatellitesStream thisArg = requestAddStream $ getCelestialBodySatellitesStreamReq thisArg 

{-|
The altitude, in meters, above which a vessel is considered to be in "high" space when doing science.
 -}
getCelestialBodySpaceHighAltitudeThresholdReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodySpaceHighAltitudeThresholdReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_SpaceHighAltitudeThreshold" [makeArgument 0 thisArg]

getCelestialBodySpaceHighAltitudeThreshold :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySpaceHighAltitudeThreshold thisArg = simpleRequest $ getCelestialBodySpaceHighAltitudeThresholdReq thisArg

getCelestialBodySpaceHighAltitudeThresholdStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySpaceHighAltitudeThresholdStreamReq thisArg = makeStreamReq $ getCelestialBodySpaceHighAltitudeThresholdReq thisArg

getCelestialBodySpaceHighAltitudeThresholdStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySpaceHighAltitudeThresholdStream thisArg = requestAddStream $ getCelestialBodySpaceHighAltitudeThresholdStreamReq thisArg 

{-|
The radius of the sphere of influence of the body, in meters.
 -}
getCelestialBodySphereOfInfluenceReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodySphereOfInfluenceReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]

getCelestialBodySphereOfInfluence :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySphereOfInfluence thisArg = simpleRequest $ getCelestialBodySphereOfInfluenceReq thisArg

getCelestialBodySphereOfInfluenceStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySphereOfInfluenceStreamReq thisArg = makeStreamReq $ getCelestialBodySphereOfInfluenceReq thisArg

getCelestialBodySphereOfInfluenceStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySphereOfInfluenceStream thisArg = requestAddStream $ getCelestialBodySphereOfInfluenceStreamReq thisArg 

{-|
The acceleration due to gravity at sea level (mean altitude) on the body, inm/s^2.
 -}
getCelestialBodySurfaceGravityReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq (Float)
getCelestialBodySurfaceGravityReq thisArg = makeCallReq "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]

getCelestialBodySurfaceGravity :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySurfaceGravity thisArg = simpleRequest $ getCelestialBodySurfaceGravityReq thisArg

getCelestialBodySurfaceGravityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySurfaceGravityStreamReq thisArg = makeStreamReq $ getCelestialBodySurfaceGravityReq thisArg

getCelestialBodySurfaceGravityStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySurfaceGravityStream thisArg = requestAddStream $ getCelestialBodySurfaceGravityStreamReq thisArg 

{-|
Clears the current target.
 -}
clearTargetReq :: KRPCCallReq ()
clearTargetReq  = makeCallReq "SpaceCenter" "ClearTarget" []

clearTarget :: RPCContext ()
clearTarget  = simpleRequest $ clearTargetReq  

{-|
The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getControlSurfaceAvailableTorqueReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq ((Double, Double, Double))
getControlSurfaceAvailableTorqueReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]

getControlSurfaceAvailableTorque :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext ((Double, Double, Double))
getControlSurfaceAvailableTorque thisArg = simpleRequest $ getControlSurfaceAvailableTorqueReq thisArg

getControlSurfaceAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq ((Double, Double, Double))
getControlSurfaceAvailableTorqueStreamReq thisArg = makeStreamReq $ getControlSurfaceAvailableTorqueReq thisArg

getControlSurfaceAvailableTorqueStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream ((Double, Double, Double)))
getControlSurfaceAvailableTorqueStream thisArg = requestAddStream $ getControlSurfaceAvailableTorqueStreamReq thisArg 

{-|
Whether the control surface has been fully deployed.
 -}
getControlSurfaceDeployedReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (Bool)
getControlSurfaceDeployedReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]

getControlSurfaceDeployed :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceDeployed thisArg = simpleRequest $ getControlSurfaceDeployedReq thisArg

getControlSurfaceDeployedStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceDeployedStreamReq thisArg = makeStreamReq $ getControlSurfaceDeployedReq thisArg

getControlSurfaceDeployedStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceDeployedStream thisArg = requestAddStream $ getControlSurfaceDeployedStreamReq thisArg 

{-|
Whether the control surface movement is inverted.
 -}
getControlSurfaceInvertedReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (Bool)
getControlSurfaceInvertedReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]

getControlSurfaceInverted :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceInverted thisArg = simpleRequest $ getControlSurfaceInvertedReq thisArg

getControlSurfaceInvertedStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceInvertedStreamReq thisArg = makeStreamReq $ getControlSurfaceInvertedReq thisArg

getControlSurfaceInvertedStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceInvertedStream thisArg = requestAddStream $ getControlSurfaceInvertedStreamReq thisArg 

{-|
The part object for this control surface.
 -}
getControlSurfacePartReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getControlSurfacePartReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]

getControlSurfacePart :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCHS.SpaceCenter.Part)
getControlSurfacePart thisArg = simpleRequest $ getControlSurfacePartReq thisArg

getControlSurfacePartStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getControlSurfacePartStreamReq thisArg = makeStreamReq $ getControlSurfacePartReq thisArg

getControlSurfacePartStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getControlSurfacePartStream thisArg = requestAddStream $ getControlSurfacePartStreamReq thisArg 

{-|
Whether the control surface has pitch control enabled.
 -}
getControlSurfacePitchEnabledReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (Bool)
getControlSurfacePitchEnabledReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]

getControlSurfacePitchEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfacePitchEnabled thisArg = simpleRequest $ getControlSurfacePitchEnabledReq thisArg

getControlSurfacePitchEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfacePitchEnabledStreamReq thisArg = makeStreamReq $ getControlSurfacePitchEnabledReq thisArg

getControlSurfacePitchEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfacePitchEnabledStream thisArg = requestAddStream $ getControlSurfacePitchEnabledStreamReq thisArg 

{-|
Whether the control surface has roll control enabled.
 -}
getControlSurfaceRollEnabledReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (Bool)
getControlSurfaceRollEnabledReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]

getControlSurfaceRollEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceRollEnabled thisArg = simpleRequest $ getControlSurfaceRollEnabledReq thisArg

getControlSurfaceRollEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceRollEnabledStreamReq thisArg = makeStreamReq $ getControlSurfaceRollEnabledReq thisArg

getControlSurfaceRollEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceRollEnabledStream thisArg = requestAddStream $ getControlSurfaceRollEnabledStreamReq thisArg 

{-|
Surface area of the control surface inm^2.
 -}
getControlSurfaceSurfaceAreaReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (Float)
getControlSurfaceSurfaceAreaReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]

getControlSurfaceSurfaceArea :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Float)
getControlSurfaceSurfaceArea thisArg = simpleRequest $ getControlSurfaceSurfaceAreaReq thisArg

getControlSurfaceSurfaceAreaStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Float)
getControlSurfaceSurfaceAreaStreamReq thisArg = makeStreamReq $ getControlSurfaceSurfaceAreaReq thisArg

getControlSurfaceSurfaceAreaStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Float))
getControlSurfaceSurfaceAreaStream thisArg = requestAddStream $ getControlSurfaceSurfaceAreaStreamReq thisArg 

{-|
Whether the control surface has yaw control enabled.
 -}
getControlSurfaceYawEnabledReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCCallReq (Bool)
getControlSurfaceYawEnabledReq thisArg = makeCallReq "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]

getControlSurfaceYawEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceYawEnabled thisArg = simpleRequest $ getControlSurfaceYawEnabledReq thisArg

getControlSurfaceYawEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceYawEnabledStreamReq thisArg = makeStreamReq $ getControlSurfaceYawEnabledReq thisArg

getControlSurfaceYawEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceYawEnabledStream thisArg = requestAddStream $ getControlSurfaceYawEnabledStreamReq thisArg 

{-|
Whether the control surface has been fully deployed.
 -}
setControlSurfaceDeployedReq :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> KRPCCallReq ()
setControlSurfaceDeployedReq thisArg valueArg = makeCallReq "SpaceCenter" "ControlSurface_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSurfaceDeployed :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceDeployed thisArg valueArg = simpleRequest $ setControlSurfaceDeployedReq thisArg valueArg 

{-|
Whether the control surface movement is inverted.
 -}
setControlSurfaceInvertedReq :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> KRPCCallReq ()
setControlSurfaceInvertedReq thisArg valueArg = makeCallReq "SpaceCenter" "ControlSurface_set_Inverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSurfaceInverted :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceInverted thisArg valueArg = simpleRequest $ setControlSurfaceInvertedReq thisArg valueArg 

{-|
Whether the control surface has pitch control enabled.
 -}
setControlSurfacePitchEnabledReq :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> KRPCCallReq ()
setControlSurfacePitchEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "ControlSurface_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSurfacePitchEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfacePitchEnabled thisArg valueArg = simpleRequest $ setControlSurfacePitchEnabledReq thisArg valueArg 

{-|
Whether the control surface has roll control enabled.
 -}
setControlSurfaceRollEnabledReq :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> KRPCCallReq ()
setControlSurfaceRollEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "ControlSurface_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSurfaceRollEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceRollEnabled thisArg valueArg = simpleRequest $ setControlSurfaceRollEnabledReq thisArg valueArg 

{-|
Whether the control surface has yaw control enabled.
 -}
setControlSurfaceYawEnabledReq :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> KRPCCallReq ()
setControlSurfaceYawEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "ControlSurface_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSurfaceYawEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceYawEnabled thisArg valueArg = simpleRequest $ setControlSurfaceYawEnabledReq thisArg valueArg 

{-|
Activates the next stage. Equivalent to pressing the space bar in-game.A list of vessel objects that are jettisoned from the active vessel.
 -}
controlActivateNextStageReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStageReq thisArg = makeCallReq "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]

controlActivateNextStage :: KRPCHS.SpaceCenter.Control -> RPCContext ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStage thisArg = simpleRequest $ controlActivateNextStageReq thisArg

controlActivateNextStageStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStageStreamReq thisArg = makeStreamReq $ controlActivateNextStageReq thisArg

controlActivateNextStageStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
controlActivateNextStageStream thisArg = requestAddStream $ controlActivateNextStageStreamReq thisArg 

{-|
Creates a maneuver node at the given universal time, and returns a
<see cref="T:SpaceCenter.Node" /> object that can be used to modify it.
Optionally sets the magnitude of the delta-v for the maneuver node
in the prograde, normal and radial directions.<param name="ut">Universal time of the maneuver node.<param name="prograde">Delta-v in the prograde direction.<param name="normal">Delta-v in the normal direction.<param name="radial">Delta-v in the radial direction.
 -}
controlAddNodeReq :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> KRPCCallReq (KRPCHS.SpaceCenter.Node)
controlAddNodeReq thisArg utArg progradeArg normalArg radialArg = makeCallReq "SpaceCenter" "Control_AddNode" [makeArgument 0 thisArg, makeArgument 1 utArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg]

controlAddNode :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> RPCContext (KRPCHS.SpaceCenter.Node)
controlAddNode thisArg utArg progradeArg normalArg radialArg = simpleRequest $ controlAddNodeReq thisArg utArg progradeArg normalArg radialArg

controlAddNodeStreamReq :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> KRPCStreamReq (KRPCHS.SpaceCenter.Node)
controlAddNodeStreamReq thisArg utArg progradeArg normalArg radialArg = makeStreamReq $ controlAddNodeReq thisArg utArg progradeArg normalArg radialArg

controlAddNodeStream :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Node))
controlAddNodeStream thisArg utArg progradeArg normalArg radialArg = requestAddStream $ controlAddNodeStreamReq thisArg utArg progradeArg normalArg radialArg 

{-|
Returnstrueif the given action group is enabled.<param name="group">A number between 0 and 9 inclusive.
 -}
controlGetActionGroupReq :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> KRPCCallReq (Bool)
controlGetActionGroupReq thisArg groupArg = makeCallReq "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]

controlGetActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (Bool)
controlGetActionGroup thisArg groupArg = simpleRequest $ controlGetActionGroupReq thisArg groupArg

controlGetActionGroupStreamReq :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> KRPCStreamReq (Bool)
controlGetActionGroupStreamReq thisArg groupArg = makeStreamReq $ controlGetActionGroupReq thisArg groupArg

controlGetActionGroupStream :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (KRPCStream (Bool))
controlGetActionGroupStream thisArg groupArg = requestAddStream $ controlGetActionGroupStreamReq thisArg groupArg 

{-|
Remove all maneuver nodes.
 -}
controlRemoveNodesReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq ()
controlRemoveNodesReq thisArg = makeCallReq "SpaceCenter" "Control_RemoveNodes" [makeArgument 0 thisArg]

controlRemoveNodes :: KRPCHS.SpaceCenter.Control -> RPCContext ()
controlRemoveNodes thisArg = simpleRequest $ controlRemoveNodesReq thisArg 

{-|
Sets the state of the given action group (a value between 0 and 9
inclusive).<param name="group">A number between 0 and 9 inclusive.<param name="state">
 -}
controlSetActionGroupReq :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> Bool -> KRPCCallReq ()
controlSetActionGroupReq thisArg groupArg stateArg = makeCallReq "SpaceCenter" "Control_SetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg, makeArgument 2 stateArg]

controlSetActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> Bool -> RPCContext ()
controlSetActionGroup thisArg groupArg stateArg = simpleRequest $ controlSetActionGroupReq thisArg groupArg stateArg 

{-|
Toggles the state of the given action group.<param name="group">A number between 0 and 9 inclusive.
 -}
controlToggleActionGroupReq :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> KRPCCallReq ()
controlToggleActionGroupReq thisArg groupArg = makeCallReq "SpaceCenter" "Control_ToggleActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]

controlToggleActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext ()
controlToggleActionGroup thisArg groupArg = simpleRequest $ controlToggleActionGroupReq thisArg groupArg 

{-|
The state of the abort action group.
 -}
getControlAbortReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Bool)
getControlAbortReq thisArg = makeCallReq "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]

getControlAbort :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlAbort thisArg = simpleRequest $ getControlAbortReq thisArg

getControlAbortStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlAbortStreamReq thisArg = makeStreamReq $ getControlAbortReq thisArg

getControlAbortStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlAbortStream thisArg = requestAddStream $ getControlAbortStreamReq thisArg 

{-|
The state of the wheel brakes.
 -}
getControlBrakesReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Bool)
getControlBrakesReq thisArg = makeCallReq "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]

getControlBrakes :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlBrakes thisArg = simpleRequest $ getControlBrakesReq thisArg

getControlBrakesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlBrakesStreamReq thisArg = makeStreamReq $ getControlBrakesReq thisArg

getControlBrakesStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlBrakesStream thisArg = requestAddStream $ getControlBrakesStreamReq thisArg 

{-|
The current stage of the vessel. Corresponds to the stage number in
the in-game UI.
 -}
getControlCurrentStageReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Data.Int.Int32)
getControlCurrentStageReq thisArg = makeCallReq "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]

getControlCurrentStage :: KRPCHS.SpaceCenter.Control -> RPCContext (Data.Int.Int32)
getControlCurrentStage thisArg = simpleRequest $ getControlCurrentStageReq thisArg

getControlCurrentStageStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Data.Int.Int32)
getControlCurrentStageStreamReq thisArg = makeStreamReq $ getControlCurrentStageReq thisArg

getControlCurrentStageStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Data.Int.Int32))
getControlCurrentStageStream thisArg = requestAddStream $ getControlCurrentStageStreamReq thisArg 

{-|
The state of the forward translational control.
A value between -1 and 1.
Equivalent to the h and n keys.
 -}
getControlForwardReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlForwardReq thisArg = makeCallReq "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]

getControlForward :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlForward thisArg = simpleRequest $ getControlForwardReq thisArg

getControlForwardStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlForwardStreamReq thisArg = makeStreamReq $ getControlForwardReq thisArg

getControlForwardStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlForwardStream thisArg = requestAddStream $ getControlForwardStreamReq thisArg 

{-|
The state of the landing gear/legs.
 -}
getControlGearReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Bool)
getControlGearReq thisArg = makeCallReq "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]

getControlGear :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlGear thisArg = simpleRequest $ getControlGearReq thisArg

getControlGearStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlGearStreamReq thisArg = makeStreamReq $ getControlGearReq thisArg

getControlGearStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlGearStream thisArg = requestAddStream $ getControlGearStreamReq thisArg 

{-|
The state of the lights.
 -}
getControlLightsReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Bool)
getControlLightsReq thisArg = makeCallReq "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]

getControlLights :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlLights thisArg = simpleRequest $ getControlLightsReq thisArg

getControlLightsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlLightsStreamReq thisArg = makeStreamReq $ getControlLightsReq thisArg

getControlLightsStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlLightsStream thisArg = requestAddStream $ getControlLightsStreamReq thisArg 

{-|
Returns a list of all existing maneuver nodes, ordered by time from first to last.
 -}
getControlNodesReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq ([KRPCHS.SpaceCenter.Node])
getControlNodesReq thisArg = makeCallReq "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]

getControlNodes :: KRPCHS.SpaceCenter.Control -> RPCContext ([KRPCHS.SpaceCenter.Node])
getControlNodes thisArg = simpleRequest $ getControlNodesReq thisArg

getControlNodesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq ([KRPCHS.SpaceCenter.Node])
getControlNodesStreamReq thisArg = makeStreamReq $ getControlNodesReq thisArg

getControlNodesStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Node]))
getControlNodesStream thisArg = requestAddStream $ getControlNodesStreamReq thisArg 

{-|
The state of the pitch control.
A value between -1 and 1.
Equivalent to the w and s keys.
 -}
getControlPitchReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlPitchReq thisArg = makeCallReq "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]

getControlPitch :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlPitch thisArg = simpleRequest $ getControlPitchReq thisArg

getControlPitchStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlPitchStreamReq thisArg = makeStreamReq $ getControlPitchReq thisArg

getControlPitchStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlPitchStream thisArg = requestAddStream $ getControlPitchStreamReq thisArg 

{-|
The state of RCS.
 -}
getControlRCSReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Bool)
getControlRCSReq thisArg = makeCallReq "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]

getControlRCS :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlRCS thisArg = simpleRequest $ getControlRCSReq thisArg

getControlRCSStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlRCSStreamReq thisArg = makeStreamReq $ getControlRCSReq thisArg

getControlRCSStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlRCSStream thisArg = requestAddStream $ getControlRCSStreamReq thisArg 

{-|
The state of the right translational control.
A value between -1 and 1.
Equivalent to the j and l keys.
 -}
getControlRightReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlRightReq thisArg = makeCallReq "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]

getControlRight :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlRight thisArg = simpleRequest $ getControlRightReq thisArg

getControlRightStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlRightStreamReq thisArg = makeStreamReq $ getControlRightReq thisArg

getControlRightStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlRightStream thisArg = requestAddStream $ getControlRightStreamReq thisArg 

{-|
The state of the roll control.
A value between -1 and 1.
Equivalent to the q and e keys.
 -}
getControlRollReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlRollReq thisArg = makeCallReq "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]

getControlRoll :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlRoll thisArg = simpleRequest $ getControlRollReq thisArg

getControlRollStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlRollStreamReq thisArg = makeStreamReq $ getControlRollReq thisArg

getControlRollStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlRollStream thisArg = requestAddStream $ getControlRollStreamReq thisArg 

{-|
The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
getControlSASReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Bool)
getControlSASReq thisArg = makeCallReq "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]

getControlSAS :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlSAS thisArg = simpleRequest $ getControlSASReq thisArg

getControlSASStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlSASStreamReq thisArg = makeStreamReq $ getControlSASReq thisArg

getControlSASStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlSASStream thisArg = requestAddStream $ getControlSASStreamReq thisArg 

{-|
The current <see cref="T:SpaceCenter.SASMode" />.
These modes are equivalent to the mode buttons to
the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
getControlSASModeReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (KRPCHS.SpaceCenter.SASMode)
getControlSASModeReq thisArg = makeCallReq "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]

getControlSASMode :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCHS.SpaceCenter.SASMode)
getControlSASMode thisArg = simpleRequest $ getControlSASModeReq thisArg

getControlSASModeStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.SASMode)
getControlSASModeStreamReq thisArg = makeStreamReq $ getControlSASModeReq thisArg

getControlSASModeStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getControlSASModeStream thisArg = requestAddStream $ getControlSASModeStreamReq thisArg 

{-|
The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
This is the mode displayed next to the speed at the top of the navball.
 -}
getControlSpeedModeReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedModeReq thisArg = makeCallReq "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]

getControlSpeedMode :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedMode thisArg = simpleRequest $ getControlSpeedModeReq thisArg

getControlSpeedModeStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedModeStreamReq thisArg = makeStreamReq $ getControlSpeedModeReq thisArg

getControlSpeedModeStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SpeedMode))
getControlSpeedModeStream thisArg = requestAddStream $ getControlSpeedModeStreamReq thisArg 

{-|
The state of the throttle. A value between 0 and 1.
 -}
getControlThrottleReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlThrottleReq thisArg = makeCallReq "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]

getControlThrottle :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlThrottle thisArg = simpleRequest $ getControlThrottleReq thisArg

getControlThrottleStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlThrottleStreamReq thisArg = makeStreamReq $ getControlThrottleReq thisArg

getControlThrottleStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlThrottleStream thisArg = requestAddStream $ getControlThrottleStreamReq thisArg 

{-|
The state of the up translational control.
A value between -1 and 1.
Equivalent to the i and k keys.
 -}
getControlUpReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlUpReq thisArg = makeCallReq "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]

getControlUp :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlUp thisArg = simpleRequest $ getControlUpReq thisArg

getControlUpStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlUpStreamReq thisArg = makeStreamReq $ getControlUpReq thisArg

getControlUpStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlUpStream thisArg = requestAddStream $ getControlUpStreamReq thisArg 

{-|
The state of the wheel steering.
A value between -1 and 1.
A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
getControlWheelSteeringReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlWheelSteeringReq thisArg = makeCallReq "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]

getControlWheelSteering :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlWheelSteering thisArg = simpleRequest $ getControlWheelSteeringReq thisArg

getControlWheelSteeringStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlWheelSteeringStreamReq thisArg = makeStreamReq $ getControlWheelSteeringReq thisArg

getControlWheelSteeringStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlWheelSteeringStream thisArg = requestAddStream $ getControlWheelSteeringStreamReq thisArg 

{-|
The state of the wheel throttle.
A value between -1 and 1.
A value of 1 rotates the wheels forwards, a value of -1 rotates
the wheels backwards.
 -}
getControlWheelThrottleReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlWheelThrottleReq thisArg = makeCallReq "SpaceCenter" "Control_get_WheelThrottle" [makeArgument 0 thisArg]

getControlWheelThrottle :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlWheelThrottle thisArg = simpleRequest $ getControlWheelThrottleReq thisArg

getControlWheelThrottleStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlWheelThrottleStreamReq thisArg = makeStreamReq $ getControlWheelThrottleReq thisArg

getControlWheelThrottleStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlWheelThrottleStream thisArg = requestAddStream $ getControlWheelThrottleStreamReq thisArg 

{-|
The state of the yaw control.
A value between -1 and 1.
Equivalent to the a and d keys.
 -}
getControlYawReq :: KRPCHS.SpaceCenter.Control -> KRPCCallReq (Float)
getControlYawReq thisArg = makeCallReq "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]

getControlYaw :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlYaw thisArg = simpleRequest $ getControlYawReq thisArg

getControlYawStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlYawStreamReq thisArg = makeStreamReq $ getControlYawReq thisArg

getControlYawStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlYawStream thisArg = requestAddStream $ getControlYawStreamReq thisArg 

{-|
The state of the abort action group.
 -}
setControlAbortReq :: KRPCHS.SpaceCenter.Control -> Bool -> KRPCCallReq ()
setControlAbortReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Abort" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlAbort :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlAbort thisArg valueArg = simpleRequest $ setControlAbortReq thisArg valueArg 

{-|
The state of the wheel brakes.
 -}
setControlBrakesReq :: KRPCHS.SpaceCenter.Control -> Bool -> KRPCCallReq ()
setControlBrakesReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Brakes" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlBrakes :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlBrakes thisArg valueArg = simpleRequest $ setControlBrakesReq thisArg valueArg 

{-|
The state of the forward translational control.
A value between -1 and 1.
Equivalent to the h and n keys.
 -}
setControlForwardReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlForwardReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Forward" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlForward :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlForward thisArg valueArg = simpleRequest $ setControlForwardReq thisArg valueArg 

{-|
The state of the landing gear/legs.
 -}
setControlGearReq :: KRPCHS.SpaceCenter.Control -> Bool -> KRPCCallReq ()
setControlGearReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Gear" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlGear :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlGear thisArg valueArg = simpleRequest $ setControlGearReq thisArg valueArg 

{-|
The state of the lights.
 -}
setControlLightsReq :: KRPCHS.SpaceCenter.Control -> Bool -> KRPCCallReq ()
setControlLightsReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Lights" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlLights :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlLights thisArg valueArg = simpleRequest $ setControlLightsReq thisArg valueArg 

{-|
The state of the pitch control.
A value between -1 and 1.
Equivalent to the w and s keys.
 -}
setControlPitchReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlPitchReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlPitch :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlPitch thisArg valueArg = simpleRequest $ setControlPitchReq thisArg valueArg 

{-|
The state of RCS.
 -}
setControlRCSReq :: KRPCHS.SpaceCenter.Control -> Bool -> KRPCCallReq ()
setControlRCSReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_RCS" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlRCS :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlRCS thisArg valueArg = simpleRequest $ setControlRCSReq thisArg valueArg 

{-|
The state of the right translational control.
A value between -1 and 1.
Equivalent to the j and l keys.
 -}
setControlRightReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlRightReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Right" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlRight :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlRight thisArg valueArg = simpleRequest $ setControlRightReq thisArg valueArg 

{-|
The state of the roll control.
A value between -1 and 1.
Equivalent to the q and e keys.
 -}
setControlRollReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlRollReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Roll" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlRoll :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlRoll thisArg valueArg = simpleRequest $ setControlRollReq thisArg valueArg 

{-|
The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
setControlSASReq :: KRPCHS.SpaceCenter.Control -> Bool -> KRPCCallReq ()
setControlSASReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSAS :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlSAS thisArg valueArg = simpleRequest $ setControlSASReq thisArg valueArg 

{-|
The current <see cref="T:SpaceCenter.SASMode" />.
These modes are equivalent to the mode buttons to
the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
setControlSASModeReq :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SASMode -> KRPCCallReq ()
setControlSASModeReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSASMode :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SASMode -> RPCContext ()
setControlSASMode thisArg valueArg = simpleRequest $ setControlSASModeReq thisArg valueArg 

{-|
The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
This is the mode displayed next to the speed at the top of the navball.
 -}
setControlSpeedModeReq :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SpeedMode -> KRPCCallReq ()
setControlSpeedModeReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_SpeedMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlSpeedMode :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SpeedMode -> RPCContext ()
setControlSpeedMode thisArg valueArg = simpleRequest $ setControlSpeedModeReq thisArg valueArg 

{-|
The state of the throttle. A value between 0 and 1.
 -}
setControlThrottleReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlThrottleReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Throttle" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlThrottle :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlThrottle thisArg valueArg = simpleRequest $ setControlThrottleReq thisArg valueArg 

{-|
The state of the up translational control.
A value between -1 and 1.
Equivalent to the i and k keys.
 -}
setControlUpReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlUpReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Up" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlUp :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlUp thisArg valueArg = simpleRequest $ setControlUpReq thisArg valueArg 

{-|
The state of the wheel steering.
A value between -1 and 1.
A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
setControlWheelSteeringReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlWheelSteeringReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_WheelSteering" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlWheelSteering :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlWheelSteering thisArg valueArg = simpleRequest $ setControlWheelSteeringReq thisArg valueArg 

{-|
The state of the wheel throttle.
A value between -1 and 1.
A value of 1 rotates the wheels forwards, a value of -1 rotates
the wheels backwards.
 -}
setControlWheelThrottleReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlWheelThrottleReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_WheelThrottle" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlWheelThrottle :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlWheelThrottle thisArg valueArg = simpleRequest $ setControlWheelThrottleReq thisArg valueArg 

{-|
The state of the yaw control.
A value between -1 and 1.
Equivalent to the a and d keys.
 -}
setControlYawReq :: KRPCHS.SpaceCenter.Control -> Float -> KRPCCallReq ()
setControlYawReq thisArg valueArg = makeCallReq "SpaceCenter" "Control_set_Yaw" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setControlYaw :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlYaw thisArg valueArg = simpleRequest $ setControlYawReq thisArg valueArg 

{-|
Fires the decoupler. Returns the new vessel created when the decoupler fires.
Throws an exception if the decoupler has already fired.
 -}
decouplerDecoupleReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
decouplerDecoupleReq thisArg = makeCallReq "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]

decouplerDecouple :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCHS.SpaceCenter.Vessel)
decouplerDecouple thisArg = simpleRequest $ decouplerDecoupleReq thisArg

decouplerDecoupleStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
decouplerDecoupleStreamReq thisArg = makeStreamReq $ decouplerDecoupleReq thisArg

decouplerDecoupleStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
decouplerDecoupleStream thisArg = requestAddStream $ decouplerDecoupleStreamReq thisArg 

{-|
Whether the decoupler has fired.
 -}
getDecouplerDecoupledReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCCallReq (Bool)
getDecouplerDecoupledReq thisArg = makeCallReq "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]

getDecouplerDecoupled :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Bool)
getDecouplerDecoupled thisArg = simpleRequest $ getDecouplerDecoupledReq thisArg

getDecouplerDecoupledStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Bool)
getDecouplerDecoupledStreamReq thisArg = makeStreamReq $ getDecouplerDecoupledReq thisArg

getDecouplerDecoupledStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Bool))
getDecouplerDecoupledStream thisArg = requestAddStream $ getDecouplerDecoupledStreamReq thisArg 

{-|
The impulse that the decoupler imparts when it is fired, in Newton seconds.
 -}
getDecouplerImpulseReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCCallReq (Float)
getDecouplerImpulseReq thisArg = makeCallReq "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]

getDecouplerImpulse :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Float)
getDecouplerImpulse thisArg = simpleRequest $ getDecouplerImpulseReq thisArg

getDecouplerImpulseStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Float)
getDecouplerImpulseStreamReq thisArg = makeStreamReq $ getDecouplerImpulseReq thisArg

getDecouplerImpulseStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Float))
getDecouplerImpulseStream thisArg = requestAddStream $ getDecouplerImpulseStreamReq thisArg 

{-|
The part object for this decoupler.
 -}
getDecouplerPartReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getDecouplerPartReq thisArg = makeCallReq "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]

getDecouplerPart :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCHS.SpaceCenter.Part)
getDecouplerPart thisArg = simpleRequest $ getDecouplerPartReq thisArg

getDecouplerPartStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDecouplerPartStreamReq thisArg = makeStreamReq $ getDecouplerPartReq thisArg

getDecouplerPartStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDecouplerPartStream thisArg = requestAddStream $ getDecouplerPartStreamReq thisArg 

{-|
Whether the decoupler is enabled in the staging sequence.
 -}
getDecouplerStagedReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCCallReq (Bool)
getDecouplerStagedReq thisArg = makeCallReq "SpaceCenter" "Decoupler_get_Staged" [makeArgument 0 thisArg]

getDecouplerStaged :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Bool)
getDecouplerStaged thisArg = simpleRequest $ getDecouplerStagedReq thisArg

getDecouplerStagedStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Bool)
getDecouplerStagedStreamReq thisArg = makeStreamReq $ getDecouplerStagedReq thisArg

getDecouplerStagedStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Bool))
getDecouplerStagedStream thisArg = requestAddStream $ getDecouplerStagedStreamReq thisArg 

{-|
The direction that docking port points in, in the given reference frame.
 -}
dockingPortDirectionReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
dockingPortDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

dockingPortDirection :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortDirection thisArg referenceFrameArg = simpleRequest $ dockingPortDirectionReq thisArg referenceFrameArg

dockingPortDirectionStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
dockingPortDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ dockingPortDirectionReq thisArg referenceFrameArg

dockingPortDirectionStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortDirectionStream thisArg referenceFrameArg = requestAddStream $ dockingPortDirectionStreamReq thisArg referenceFrameArg 

{-|
The position of the docking port in the given reference frame.
 -}
dockingPortPositionReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
dockingPortPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

dockingPortPosition :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortPosition thisArg referenceFrameArg = simpleRequest $ dockingPortPositionReq thisArg referenceFrameArg

dockingPortPositionStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
dockingPortPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ dockingPortPositionReq thisArg referenceFrameArg

dockingPortPositionStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortPositionStream thisArg referenceFrameArg = requestAddStream $ dockingPortPositionStreamReq thisArg referenceFrameArg 

{-|
The rotation of the docking port, in the given reference frame.
 -}
dockingPortRotationReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double, Double))
dockingPortRotationReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

dockingPortRotation :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
dockingPortRotation thisArg referenceFrameArg = simpleRequest $ dockingPortRotationReq thisArg referenceFrameArg

dockingPortRotationStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
dockingPortRotationStreamReq thisArg referenceFrameArg = makeStreamReq $ dockingPortRotationReq thisArg referenceFrameArg

dockingPortRotationStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
dockingPortRotationStream thisArg referenceFrameArg = requestAddStream $ dockingPortRotationStreamReq thisArg referenceFrameArg 

{-|
Undocks the docking port and returns the new <see cref="T:SpaceCenter.Vessel" /> that is created.
This method can be called for either docking port in a docked pair.
Throws an exception if the docking port is not docked to anything.After undocking, the active vessel may change. See <see cref="M:SpaceCenter.ActiveVessel" />.
 -}
dockingPortUndockReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
dockingPortUndockReq thisArg = makeCallReq "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]

dockingPortUndock :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Vessel)
dockingPortUndock thisArg = simpleRequest $ dockingPortUndockReq thisArg

dockingPortUndockStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
dockingPortUndockStreamReq thisArg = makeStreamReq $ dockingPortUndockReq thisArg

dockingPortUndockStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
dockingPortUndockStream thisArg = requestAddStream $ dockingPortUndockStreamReq thisArg 

{-|
The part that this docking port is docked to. Returnsnullif this
docking port is not docked to anything.
 -}
getDockingPortDockedPartReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPartReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]

getDockingPortDockedPart :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPart thisArg = simpleRequest $ getDockingPortDockedPartReq thisArg

getDockingPortDockedPartStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPartStreamReq thisArg = makeStreamReq $ getDockingPortDockedPartReq thisArg

getDockingPortDockedPartStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortDockedPartStream thisArg = requestAddStream $ getDockingPortDockedPartStreamReq thisArg 

{-|
Whether the docking port has a shield.
 -}
getDockingPortHasShieldReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (Bool)
getDockingPortHasShieldReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]

getDockingPortHasShield :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Bool)
getDockingPortHasShield thisArg = simpleRequest $ getDockingPortHasShieldReq thisArg

getDockingPortHasShieldStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Bool)
getDockingPortHasShieldStreamReq thisArg = makeStreamReq $ getDockingPortHasShieldReq thisArg

getDockingPortHasShieldStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortHasShieldStream thisArg = requestAddStream $ getDockingPortHasShieldStreamReq thisArg 

{-|
The part object for this docking port.
 -}
getDockingPortPartReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getDockingPortPartReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]

getDockingPortPart :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Part)
getDockingPortPart thisArg = simpleRequest $ getDockingPortPartReq thisArg

getDockingPortPartStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDockingPortPartStreamReq thisArg = makeStreamReq $ getDockingPortPartReq thisArg

getDockingPortPartStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortPartStream thisArg = requestAddStream $ getDockingPortPartStreamReq thisArg 

{-|
The distance a docking port must move away when it undocks before it
becomes ready to dock with another port, in meters.
 -}
getDockingPortReengageDistanceReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (Float)
getDockingPortReengageDistanceReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]

getDockingPortReengageDistance :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Float)
getDockingPortReengageDistance thisArg = simpleRequest $ getDockingPortReengageDistanceReq thisArg

getDockingPortReengageDistanceStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Float)
getDockingPortReengageDistanceStreamReq thisArg = makeStreamReq $ getDockingPortReengageDistanceReq thisArg

getDockingPortReengageDistanceStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Float))
getDockingPortReengageDistanceStream thisArg = requestAddStream $ getDockingPortReengageDistanceStreamReq thisArg 

{-|
The reference frame that is fixed relative to this docking port, and
oriented with the port.
<list type="bullet">The origin is at the position of the docking port.The axes rotate with the docking port.The x-axis points out to the right side of the docking port.The y-axis points in the direction the docking port is facing.The z-axis points out of the bottom off the docking port.This reference frame is not necessarily equivalent to the reference frame
for the part, returned by <see cref="M:SpaceCenter.Part.ReferenceFrame" />.
 -}
getDockingPortReferenceFrameReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_ReferenceFrame" [makeArgument 0 thisArg]

getDockingPortReferenceFrame :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrame thisArg = simpleRequest $ getDockingPortReferenceFrameReq thisArg

getDockingPortReferenceFrameStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrameStreamReq thisArg = makeStreamReq $ getDockingPortReferenceFrameReq thisArg

getDockingPortReferenceFrameStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getDockingPortReferenceFrameStream thisArg = requestAddStream $ getDockingPortReferenceFrameStreamReq thisArg 

{-|
The state of the docking ports shield, if it has one.

Returnstrueif the docking port has a shield, and the shield is
closed. Otherwise returnsfalse. When set totrue, the shield is
closed, and when set tofalsethe shield is opened. If the docking
port does not have a shield, setting this attribute has no effect.
 -}
getDockingPortShieldedReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (Bool)
getDockingPortShieldedReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_Shielded" [makeArgument 0 thisArg]

getDockingPortShielded :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Bool)
getDockingPortShielded thisArg = simpleRequest $ getDockingPortShieldedReq thisArg

getDockingPortShieldedStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Bool)
getDockingPortShieldedStreamReq thisArg = makeStreamReq $ getDockingPortShieldedReq thisArg

getDockingPortShieldedStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortShieldedStream thisArg = requestAddStream $ getDockingPortShieldedStreamReq thisArg 

{-|
The current state of the docking port.
 -}
getDockingPortStateReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortStateReq thisArg = makeCallReq "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]

getDockingPortState :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortState thisArg = simpleRequest $ getDockingPortStateReq thisArg

getDockingPortStateStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortStateStreamReq thisArg = makeStreamReq $ getDockingPortStateReq thisArg

getDockingPortStateStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPortState))
getDockingPortStateStream thisArg = requestAddStream $ getDockingPortStateStreamReq thisArg 

{-|
The state of the docking ports shield, if it has one.

Returnstrueif the docking port has a shield, and the shield is
closed. Otherwise returnsfalse. When set totrue, the shield is
closed, and when set tofalsethe shield is opened. If the docking
port does not have a shield, setting this attribute has no effect.
 -}
setDockingPortShieldedReq :: KRPCHS.SpaceCenter.DockingPort -> Bool -> KRPCCallReq ()
setDockingPortShieldedReq thisArg valueArg = makeCallReq "SpaceCenter" "DockingPort_set_Shielded" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setDockingPortShielded :: KRPCHS.SpaceCenter.DockingPort -> Bool -> RPCContext ()
setDockingPortShielded thisArg valueArg = simpleRequest $ setDockingPortShieldedReq thisArg valueArg 

{-|
Toggle the current engine mode.
 -}
engineToggleModeReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq ()
engineToggleModeReq thisArg = makeCallReq "SpaceCenter" "Engine_ToggleMode" [makeArgument 0 thisArg]

engineToggleMode :: KRPCHS.SpaceCenter.Engine -> RPCContext ()
engineToggleMode thisArg = simpleRequest $ engineToggleModeReq thisArg 

{-|
Whether the engine is active. Setting this attribute may have no effect,
depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
getEngineActiveReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineActiveReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]

getEngineActive :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineActive thisArg = simpleRequest $ getEngineActiveReq thisArg

getEngineActiveStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineActiveStreamReq thisArg = makeStreamReq $ getEngineActiveReq thisArg

getEngineActiveStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineActiveStream thisArg = requestAddStream $ getEngineActiveStreamReq thisArg 

{-|
Whether the engine will automatically switch modes.
 -}
getEngineAutoModeSwitchReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineAutoModeSwitchReq thisArg = makeCallReq "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]

getEngineAutoModeSwitch :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineAutoModeSwitch thisArg = simpleRequest $ getEngineAutoModeSwitchReq thisArg

getEngineAutoModeSwitchStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineAutoModeSwitchStreamReq thisArg = makeStreamReq $ getEngineAutoModeSwitchReq thisArg

getEngineAutoModeSwitchStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineAutoModeSwitchStream thisArg = requestAddStream $ getEngineAutoModeSwitchStreamReq thisArg 

{-|
The amount of thrust, in Newtons, that would be produced by the engine
when activated and with its throttle set to 100%.
Returns zero if the engine does not have any fuel.
Takes the engine's current <see cref="M:SpaceCenter.Engine.ThrustLimit" /> and atmospheric conditions into account.
 -}
getEngineAvailableThrustReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineAvailableThrustReq thisArg = makeCallReq "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]

getEngineAvailableThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineAvailableThrust thisArg = simpleRequest $ getEngineAvailableThrustReq thisArg

getEngineAvailableThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineAvailableThrustStreamReq thisArg = makeStreamReq $ getEngineAvailableThrustReq thisArg

getEngineAvailableThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineAvailableThrustStream thisArg = requestAddStream $ getEngineAvailableThrustStreamReq thisArg 

{-|
The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
Returns zero if the engine is inactive, or not gimballed.
 -}
getEngineAvailableTorqueReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq ((Double, Double, Double))
getEngineAvailableTorqueReq thisArg = makeCallReq "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]

getEngineAvailableTorque :: KRPCHS.SpaceCenter.Engine -> RPCContext ((Double, Double, Double))
getEngineAvailableTorque thisArg = simpleRequest $ getEngineAvailableTorqueReq thisArg

getEngineAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ((Double, Double, Double))
getEngineAvailableTorqueStreamReq thisArg = makeStreamReq $ getEngineAvailableTorqueReq thisArg

getEngineAvailableTorqueStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ((Double, Double, Double)))
getEngineAvailableTorqueStream thisArg = requestAddStream $ getEngineAvailableTorqueStreamReq thisArg 

{-|
Whether the engine can be restarted once shutdown. If the engine cannot be shutdown,
returnsfalse. For example, this istruefor liquid fueled rockets
andfalsefor solid rocket boosters.
 -}
getEngineCanRestartReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineCanRestartReq thisArg = makeCallReq "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]

getEngineCanRestart :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineCanRestart thisArg = simpleRequest $ getEngineCanRestartReq thisArg

getEngineCanRestartStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineCanRestartStreamReq thisArg = makeStreamReq $ getEngineCanRestartReq thisArg

getEngineCanRestartStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineCanRestartStream thisArg = requestAddStream $ getEngineCanRestartStreamReq thisArg 

{-|
Whether the engine can be shutdown once activated. For example, this istruefor liquid fueled rockets andfalsefor solid rocket boosters.
 -}
getEngineCanShutdownReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineCanShutdownReq thisArg = makeCallReq "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]

getEngineCanShutdown :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineCanShutdown thisArg = simpleRequest $ getEngineCanShutdownReq thisArg

getEngineCanShutdownStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineCanShutdownStreamReq thisArg = makeStreamReq $ getEngineCanShutdownReq thisArg

getEngineCanShutdownStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineCanShutdownStream thisArg = requestAddStream $ getEngineCanShutdownStreamReq thisArg 

{-|
The gimbal limiter of the engine. A value between 0 and 1.
Returns 0 if the gimbal is locked.
 -}
getEngineGimbalLimitReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineGimbalLimitReq thisArg = makeCallReq "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]

getEngineGimbalLimit :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineGimbalLimit thisArg = simpleRequest $ getEngineGimbalLimitReq thisArg

getEngineGimbalLimitStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineGimbalLimitStreamReq thisArg = makeStreamReq $ getEngineGimbalLimitReq thisArg

getEngineGimbalLimitStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalLimitStream thisArg = requestAddStream $ getEngineGimbalLimitStreamReq thisArg 

{-|
Whether the engines gimbal is locked in place. Setting this attribute has
no effect if the engine is not gimballed.
 -}
getEngineGimbalLockedReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineGimbalLockedReq thisArg = makeCallReq "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]

getEngineGimbalLocked :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineGimbalLocked thisArg = simpleRequest $ getEngineGimbalLockedReq thisArg

getEngineGimbalLockedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineGimbalLockedStreamReq thisArg = makeStreamReq $ getEngineGimbalLockedReq thisArg

getEngineGimbalLockedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineGimbalLockedStream thisArg = requestAddStream $ getEngineGimbalLockedStreamReq thisArg 

{-|
The range over which the gimbal can move, in degrees.
Returns 0 if the engine is not gimballed.
 -}
getEngineGimbalRangeReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineGimbalRangeReq thisArg = makeCallReq "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]

getEngineGimbalRange :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineGimbalRange thisArg = simpleRequest $ getEngineGimbalRangeReq thisArg

getEngineGimbalRangeStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineGimbalRangeStreamReq thisArg = makeStreamReq $ getEngineGimbalRangeReq thisArg

getEngineGimbalRangeStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalRangeStream thisArg = requestAddStream $ getEngineGimbalRangeStreamReq thisArg 

{-|
Whether the engine is gimballed.
 -}
getEngineGimballedReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineGimballedReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]

getEngineGimballed :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineGimballed thisArg = simpleRequest $ getEngineGimballedReq thisArg

getEngineGimballedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineGimballedStreamReq thisArg = makeStreamReq $ getEngineGimballedReq thisArg

getEngineGimballedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineGimballedStream thisArg = requestAddStream $ getEngineGimballedStreamReq thisArg 

{-|
Whether the engine has any fuel available.The engine must be activated for this property to update correctly.
 -}
getEngineHasFuelReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineHasFuelReq thisArg = makeCallReq "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]

getEngineHasFuel :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineHasFuel thisArg = simpleRequest $ getEngineHasFuelReq thisArg

getEngineHasFuelStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineHasFuelStreamReq thisArg = makeStreamReq $ getEngineHasFuelReq thisArg

getEngineHasFuelStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineHasFuelStream thisArg = requestAddStream $ getEngineHasFuelStreamReq thisArg 

{-|
Whether the engine has multiple modes of operation.
 -}
getEngineHasModesReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineHasModesReq thisArg = makeCallReq "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]

getEngineHasModes :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineHasModes thisArg = simpleRequest $ getEngineHasModesReq thisArg

getEngineHasModesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineHasModesStreamReq thisArg = makeStreamReq $ getEngineHasModesReq thisArg

getEngineHasModesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineHasModesStream thisArg = requestAddStream $ getEngineHasModesStreamReq thisArg 

{-|
The specific impulse of the engine at sea level on Kerbin, in seconds.
 -}
getEngineKerbinSeaLevelSpecificImpulseReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineKerbinSeaLevelSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]

getEngineKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineKerbinSeaLevelSpecificImpulse thisArg = simpleRequest $ getEngineKerbinSeaLevelSpecificImpulseReq thisArg

getEngineKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineKerbinSeaLevelSpecificImpulseStreamReq thisArg = makeStreamReq $ getEngineKerbinSeaLevelSpecificImpulseReq thisArg

getEngineKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineKerbinSeaLevelSpecificImpulseStream thisArg = requestAddStream $ getEngineKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-|
The amount of thrust, in Newtons, that would be produced by the engine
when activated and fueled, with its throttle and throttle limiter set to 100%.
 -}
getEngineMaxThrustReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineMaxThrustReq thisArg = makeCallReq "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]

getEngineMaxThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineMaxThrust thisArg = simpleRequest $ getEngineMaxThrustReq thisArg

getEngineMaxThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineMaxThrustStreamReq thisArg = makeStreamReq $ getEngineMaxThrustReq thisArg

getEngineMaxThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineMaxThrustStream thisArg = requestAddStream $ getEngineMaxThrustStreamReq thisArg 

{-|
The maximum amount of thrust that can be produced by the engine in a
vacuum, in Newtons. This is the amount of thrust produced by the engine
when activated, <see cref="M:SpaceCenter.Engine.ThrustLimit" /> is set to 100%, the main
vessel's throttle is set to 100% and the engine is in a vacuum.
 -}
getEngineMaxVacuumThrustReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineMaxVacuumThrustReq thisArg = makeCallReq "SpaceCenter" "Engine_get_MaxVacuumThrust" [makeArgument 0 thisArg]

getEngineMaxVacuumThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineMaxVacuumThrust thisArg = simpleRequest $ getEngineMaxVacuumThrustReq thisArg

getEngineMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineMaxVacuumThrustStreamReq thisArg = makeStreamReq $ getEngineMaxVacuumThrustReq thisArg

getEngineMaxVacuumThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineMaxVacuumThrustStream thisArg = requestAddStream $ getEngineMaxVacuumThrustStreamReq thisArg 

{-|
The name of the current engine mode.
 -}
getEngineModeReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Data.Text.Text)
getEngineModeReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]

getEngineMode :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Text.Text)
getEngineMode thisArg = simpleRequest $ getEngineModeReq thisArg

getEngineModeStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Text.Text)
getEngineModeStreamReq thisArg = makeStreamReq $ getEngineModeReq thisArg

getEngineModeStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Text.Text))
getEngineModeStream thisArg = requestAddStream $ getEngineModeStreamReq thisArg 

{-|
The available modes for the engine.
A dictionary mapping mode names to <see cref="T:SpaceCenter.Engine" /> objects.
 -}
getEngineModesReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModesReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]

getEngineModes :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModes thisArg = simpleRequest $ getEngineModesReq thisArg

getEngineModesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModesStreamReq thisArg = makeStreamReq $ getEngineModesReq thisArg

getEngineModesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine)))
getEngineModesStream thisArg = requestAddStream $ getEngineModesStreamReq thisArg 

{-|
The part object for this engine.
 -}
getEnginePartReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getEnginePartReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]

getEnginePart :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCHS.SpaceCenter.Part)
getEnginePart thisArg = simpleRequest $ getEnginePartReq thisArg

getEnginePartStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getEnginePartStreamReq thisArg = makeStreamReq $ getEnginePartReq thisArg

getEnginePartStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getEnginePartStream thisArg = requestAddStream $ getEnginePartStreamReq thisArg 

{-|
The names of the propellants that the engine consumes.
 -}
getEnginePropellantNamesReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq ([Data.Text.Text])
getEnginePropellantNamesReq thisArg = makeCallReq "SpaceCenter" "Engine_get_PropellantNames" [makeArgument 0 thisArg]

getEnginePropellantNames :: KRPCHS.SpaceCenter.Engine -> RPCContext ([Data.Text.Text])
getEnginePropellantNames thisArg = simpleRequest $ getEnginePropellantNamesReq thisArg

getEnginePropellantNamesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([Data.Text.Text])
getEnginePropellantNamesStreamReq thisArg = makeStreamReq $ getEnginePropellantNamesReq thisArg

getEnginePropellantNamesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([Data.Text.Text]))
getEnginePropellantNamesStream thisArg = requestAddStream $ getEnginePropellantNamesStreamReq thisArg 

{-|
The ratio of resources that the engine consumes. A dictionary mapping resource names
to the ratio at which they are consumed by the engine.For example, if the ratios are 0.6 for LiquidFuel and 0.4 for Oxidizer, then for every 0.6 units of
LiquidFuel that the engine burns, it will burn 0.4 units of Oxidizer.
 -}
getEnginePropellantRatiosReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatiosReq thisArg = makeCallReq "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]

getEnginePropellantRatios :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatios thisArg = simpleRequest $ getEnginePropellantRatiosReq thisArg

getEnginePropellantRatiosStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatiosStreamReq thisArg = makeStreamReq $ getEnginePropellantRatiosReq thisArg

getEnginePropellantRatiosStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getEnginePropellantRatiosStream thisArg = requestAddStream $ getEnginePropellantRatiosStreamReq thisArg 

{-|
The propellants that the engine consumes.
 -}
getEnginePropellantsReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellantsReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]

getEnginePropellants :: KRPCHS.SpaceCenter.Engine -> RPCContext ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellants thisArg = simpleRequest $ getEnginePropellantsReq thisArg

getEnginePropellantsStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellantsStreamReq thisArg = makeStreamReq $ getEnginePropellantsReq thisArg

getEnginePropellantsStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Propellant]))
getEnginePropellantsStream thisArg = requestAddStream $ getEnginePropellantsStreamReq thisArg 

{-|
The current specific impulse of the engine, in seconds. Returns zero
if the engine is not active.
 -}
getEngineSpecificImpulseReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]

getEngineSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineSpecificImpulse thisArg = simpleRequest $ getEngineSpecificImpulseReq thisArg

getEngineSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineSpecificImpulseStreamReq thisArg = makeStreamReq $ getEngineSpecificImpulseReq thisArg

getEngineSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineSpecificImpulseStream thisArg = requestAddStream $ getEngineSpecificImpulseStreamReq thisArg 

{-|
The current throttle setting for the engine. A value between 0 and 1.
This is not necessarily the same as the vessel's main throttle
setting, as some engines take time to adjust their throttle
(such as jet engines).
 -}
getEngineThrottleReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineThrottleReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Throttle" [makeArgument 0 thisArg]

getEngineThrottle :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrottle thisArg = simpleRequest $ getEngineThrottleReq thisArg

getEngineThrottleStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrottleStreamReq thisArg = makeStreamReq $ getEngineThrottleReq thisArg

getEngineThrottleStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrottleStream thisArg = requestAddStream $ getEngineThrottleStreamReq thisArg 

{-|
Whether the <see cref="M:SpaceCenter.Control.Throttle" /> affects the engine. For example,
this istruefor liquid fueled rockets, andfalsefor solid rocket
boosters.
 -}
getEngineThrottleLockedReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Bool)
getEngineThrottleLockedReq thisArg = makeCallReq "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]

getEngineThrottleLocked :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineThrottleLocked thisArg = simpleRequest $ getEngineThrottleLockedReq thisArg

getEngineThrottleLockedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineThrottleLockedStreamReq thisArg = makeStreamReq $ getEngineThrottleLockedReq thisArg

getEngineThrottleLockedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineThrottleLockedStream thisArg = requestAddStream $ getEngineThrottleLockedStreamReq thisArg 

{-|
The current amount of thrust being produced by the engine, in Newtons.
 -}
getEngineThrustReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineThrustReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]

getEngineThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrust thisArg = simpleRequest $ getEngineThrustReq thisArg

getEngineThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrustStreamReq thisArg = makeStreamReq $ getEngineThrustReq thisArg

getEngineThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrustStream thisArg = requestAddStream $ getEngineThrustStreamReq thisArg 

{-|
The thrust limiter of the engine. A value between 0 and 1. Setting this
attribute may have no effect, for example the thrust limit for a solid
rocket booster cannot be changed in flight.
 -}
getEngineThrustLimitReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineThrustLimitReq thisArg = makeCallReq "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]

getEngineThrustLimit :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrustLimit thisArg = simpleRequest $ getEngineThrustLimitReq thisArg

getEngineThrustLimitStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrustLimitStreamReq thisArg = makeStreamReq $ getEngineThrustLimitReq thisArg

getEngineThrustLimitStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrustLimitStream thisArg = requestAddStream $ getEngineThrustLimitStreamReq thisArg 

{-|
The components of the engine that generate thrust.For example, this corresponds to the rocket nozzel on a solid rocket booster,
or the individual nozzels on a RAPIER engine.
The overall thrust produced by the engine, as reported by <see cref="M:SpaceCenter.Engine.AvailableThrust" />,
<see cref="M:SpaceCenter.Engine.MaxThrust" /> and others, is the sum of the thrust generated by each thruster.
 -}
getEngineThrustersReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq ([KRPCHS.SpaceCenter.Thruster])
getEngineThrustersReq thisArg = makeCallReq "SpaceCenter" "Engine_get_Thrusters" [makeArgument 0 thisArg]

getEngineThrusters :: KRPCHS.SpaceCenter.Engine -> RPCContext ([KRPCHS.SpaceCenter.Thruster])
getEngineThrusters thisArg = simpleRequest $ getEngineThrustersReq thisArg

getEngineThrustersStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([KRPCHS.SpaceCenter.Thruster])
getEngineThrustersStreamReq thisArg = makeStreamReq $ getEngineThrustersReq thisArg

getEngineThrustersStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getEngineThrustersStream thisArg = requestAddStream $ getEngineThrustersStreamReq thisArg 

{-|
The vacuum specific impulse of the engine, in seconds.
 -}
getEngineVacuumSpecificImpulseReq :: KRPCHS.SpaceCenter.Engine -> KRPCCallReq (Float)
getEngineVacuumSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]

getEngineVacuumSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineVacuumSpecificImpulse thisArg = simpleRequest $ getEngineVacuumSpecificImpulseReq thisArg

getEngineVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineVacuumSpecificImpulseStreamReq thisArg = makeStreamReq $ getEngineVacuumSpecificImpulseReq thisArg

getEngineVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineVacuumSpecificImpulseStream thisArg = requestAddStream $ getEngineVacuumSpecificImpulseStreamReq thisArg 

{-|
Whether the engine is active. Setting this attribute may have no effect,
depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
setEngineActiveReq :: KRPCHS.SpaceCenter.Engine -> Bool -> KRPCCallReq ()
setEngineActiveReq thisArg valueArg = makeCallReq "SpaceCenter" "Engine_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setEngineActive :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext ()
setEngineActive thisArg valueArg = simpleRequest $ setEngineActiveReq thisArg valueArg 

{-|
Whether the engine will automatically switch modes.
 -}
setEngineAutoModeSwitchReq :: KRPCHS.SpaceCenter.Engine -> Bool -> KRPCCallReq ()
setEngineAutoModeSwitchReq thisArg valueArg = makeCallReq "SpaceCenter" "Engine_set_AutoModeSwitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setEngineAutoModeSwitch :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext ()
setEngineAutoModeSwitch thisArg valueArg = simpleRequest $ setEngineAutoModeSwitchReq thisArg valueArg 

{-|
The gimbal limiter of the engine. A value between 0 and 1.
Returns 0 if the gimbal is locked.
 -}
setEngineGimbalLimitReq :: KRPCHS.SpaceCenter.Engine -> Float -> KRPCCallReq ()
setEngineGimbalLimitReq thisArg valueArg = makeCallReq "SpaceCenter" "Engine_set_GimbalLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setEngineGimbalLimit :: KRPCHS.SpaceCenter.Engine -> Float -> RPCContext ()
setEngineGimbalLimit thisArg valueArg = simpleRequest $ setEngineGimbalLimitReq thisArg valueArg 

{-|
Whether the engines gimbal is locked in place. Setting this attribute has
no effect if the engine is not gimballed.
 -}
setEngineGimbalLockedReq :: KRPCHS.SpaceCenter.Engine -> Bool -> KRPCCallReq ()
setEngineGimbalLockedReq thisArg valueArg = makeCallReq "SpaceCenter" "Engine_set_GimbalLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setEngineGimbalLocked :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext ()
setEngineGimbalLocked thisArg valueArg = simpleRequest $ setEngineGimbalLockedReq thisArg valueArg 

{-|
The name of the current engine mode.
 -}
setEngineModeReq :: KRPCHS.SpaceCenter.Engine -> Data.Text.Text -> KRPCCallReq ()
setEngineModeReq thisArg valueArg = makeCallReq "SpaceCenter" "Engine_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setEngineMode :: KRPCHS.SpaceCenter.Engine -> Data.Text.Text -> RPCContext ()
setEngineMode thisArg valueArg = simpleRequest $ setEngineModeReq thisArg valueArg 

{-|
The thrust limiter of the engine. A value between 0 and 1. Setting this
attribute may have no effect, for example the thrust limit for a solid
rocket booster cannot be changed in flight.
 -}
setEngineThrustLimitReq :: KRPCHS.SpaceCenter.Engine -> Float -> KRPCCallReq ()
setEngineThrustLimitReq thisArg valueArg = makeCallReq "SpaceCenter" "Engine_set_ThrustLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setEngineThrustLimit :: KRPCHS.SpaceCenter.Engine -> Float -> RPCContext ()
setEngineThrustLimit thisArg valueArg = simpleRequest $ setEngineThrustLimitReq thisArg valueArg 

{-|
Dump the experimental data contained by the experiment.
 -}
experimentDumpReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq ()
experimentDumpReq thisArg = makeCallReq "SpaceCenter" "Experiment_Dump" [makeArgument 0 thisArg]

experimentDump :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentDump thisArg = simpleRequest $ experimentDumpReq thisArg 

{-|
Reset the experiment.
 -}
experimentResetReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq ()
experimentResetReq thisArg = makeCallReq "SpaceCenter" "Experiment_Reset" [makeArgument 0 thisArg]

experimentReset :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentReset thisArg = simpleRequest $ experimentResetReq thisArg 

{-|
Run the experiment.
 -}
experimentRunReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq ()
experimentRunReq thisArg = makeCallReq "SpaceCenter" "Experiment_Run" [makeArgument 0 thisArg]

experimentRun :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentRun thisArg = simpleRequest $ experimentRunReq thisArg 

{-|
Transmit all experimental data contained by this part.
 -}
experimentTransmitReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq ()
experimentTransmitReq thisArg = makeCallReq "SpaceCenter" "Experiment_Transmit" [makeArgument 0 thisArg]

experimentTransmit :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentTransmit thisArg = simpleRequest $ experimentTransmitReq thisArg 

{-|
Determines if the experiment is available given the current conditions.
 -}
getExperimentAvailableReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (Bool)
getExperimentAvailableReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Available" [makeArgument 0 thisArg]

getExperimentAvailable :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentAvailable thisArg = simpleRequest $ getExperimentAvailableReq thisArg

getExperimentAvailableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentAvailableStreamReq thisArg = makeStreamReq $ getExperimentAvailableReq thisArg

getExperimentAvailableStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentAvailableStream thisArg = requestAddStream $ getExperimentAvailableStreamReq thisArg 

{-|
The name of the biome the experiment is currently in.
 -}
getExperimentBiomeReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (Data.Text.Text)
getExperimentBiomeReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Biome" [makeArgument 0 thisArg]

getExperimentBiome :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Data.Text.Text)
getExperimentBiome thisArg = simpleRequest $ getExperimentBiomeReq thisArg

getExperimentBiomeStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Data.Text.Text)
getExperimentBiomeStreamReq thisArg = makeStreamReq $ getExperimentBiomeReq thisArg

getExperimentBiomeStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Data.Text.Text))
getExperimentBiomeStream thisArg = requestAddStream $ getExperimentBiomeStreamReq thisArg 

{-|
The data contained in this experiment.
 -}
getExperimentDataReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq ([KRPCHS.SpaceCenter.ScienceData])
getExperimentDataReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Data" [makeArgument 0 thisArg]

getExperimentData :: KRPCHS.SpaceCenter.Experiment -> RPCContext ([KRPCHS.SpaceCenter.ScienceData])
getExperimentData thisArg = simpleRequest $ getExperimentDataReq thisArg

getExperimentDataStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq ([KRPCHS.SpaceCenter.ScienceData])
getExperimentDataStreamReq thisArg = makeStreamReq $ getExperimentDataReq thisArg

getExperimentDataStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ScienceData]))
getExperimentDataStream thisArg = requestAddStream $ getExperimentDataStreamReq thisArg 

{-|
Whether the experiment has been deployed.
 -}
getExperimentDeployedReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (Bool)
getExperimentDeployedReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Deployed" [makeArgument 0 thisArg]

getExperimentDeployed :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentDeployed thisArg = simpleRequest $ getExperimentDeployedReq thisArg

getExperimentDeployedStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentDeployedStreamReq thisArg = makeStreamReq $ getExperimentDeployedReq thisArg

getExperimentDeployedStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentDeployedStream thisArg = requestAddStream $ getExperimentDeployedStreamReq thisArg 

{-|
Whether the experiment contains data.
 -}
getExperimentHasDataReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (Bool)
getExperimentHasDataReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_HasData" [makeArgument 0 thisArg]

getExperimentHasData :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentHasData thisArg = simpleRequest $ getExperimentHasDataReq thisArg

getExperimentHasDataStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentHasDataStreamReq thisArg = makeStreamReq $ getExperimentHasDataReq thisArg

getExperimentHasDataStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentHasDataStream thisArg = requestAddStream $ getExperimentHasDataStreamReq thisArg 

{-|
Whether the experiment is inoperable.
 -}
getExperimentInoperableReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (Bool)
getExperimentInoperableReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Inoperable" [makeArgument 0 thisArg]

getExperimentInoperable :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentInoperable thisArg = simpleRequest $ getExperimentInoperableReq thisArg

getExperimentInoperableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentInoperableStreamReq thisArg = makeStreamReq $ getExperimentInoperableReq thisArg

getExperimentInoperableStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentInoperableStream thisArg = requestAddStream $ getExperimentInoperableStreamReq thisArg 

{-|
The part object for this experiment.
 -}
getExperimentPartReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getExperimentPartReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Part" [makeArgument 0 thisArg]

getExperimentPart :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCHS.SpaceCenter.Part)
getExperimentPart thisArg = simpleRequest $ getExperimentPartReq thisArg

getExperimentPartStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getExperimentPartStreamReq thisArg = makeStreamReq $ getExperimentPartReq thisArg

getExperimentPartStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getExperimentPartStream thisArg = requestAddStream $ getExperimentPartStreamReq thisArg 

{-|
Whether the experiment can be re-run.
 -}
getExperimentRerunnableReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (Bool)
getExperimentRerunnableReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_Rerunnable" [makeArgument 0 thisArg]

getExperimentRerunnable :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentRerunnable thisArg = simpleRequest $ getExperimentRerunnableReq thisArg

getExperimentRerunnableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentRerunnableStreamReq thisArg = makeStreamReq $ getExperimentRerunnableReq thisArg

getExperimentRerunnableStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentRerunnableStream thisArg = requestAddStream $ getExperimentRerunnableStreamReq thisArg 

{-|
Containing information on the corresponding specific science result for the current conditions.
Returns null if experiment is unavailable.
 -}
getExperimentScienceSubjectReq :: KRPCHS.SpaceCenter.Experiment -> KRPCCallReq (KRPCHS.SpaceCenter.ScienceSubject)
getExperimentScienceSubjectReq thisArg = makeCallReq "SpaceCenter" "Experiment_get_ScienceSubject" [makeArgument 0 thisArg]

getExperimentScienceSubject :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCHS.SpaceCenter.ScienceSubject)
getExperimentScienceSubject thisArg = simpleRequest $ getExperimentScienceSubjectReq thisArg

getExperimentScienceSubjectStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (KRPCHS.SpaceCenter.ScienceSubject)
getExperimentScienceSubjectStreamReq thisArg = makeStreamReq $ getExperimentScienceSubjectReq thisArg

getExperimentScienceSubjectStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ScienceSubject))
getExperimentScienceSubjectStream thisArg = requestAddStream $ getExperimentScienceSubjectStreamReq thisArg 

{-|
Jettison the fairing. Has no effect if it has already been jettisoned.
 -}
fairingJettisonReq :: KRPCHS.SpaceCenter.Fairing -> KRPCCallReq ()
fairingJettisonReq thisArg = makeCallReq "SpaceCenter" "Fairing_Jettison" [makeArgument 0 thisArg]

fairingJettison :: KRPCHS.SpaceCenter.Fairing -> RPCContext ()
fairingJettison thisArg = simpleRequest $ fairingJettisonReq thisArg 

{-|
Whether the fairing has been jettisoned.
 -}
getFairingJettisonedReq :: KRPCHS.SpaceCenter.Fairing -> KRPCCallReq (Bool)
getFairingJettisonedReq thisArg = makeCallReq "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]

getFairingJettisoned :: KRPCHS.SpaceCenter.Fairing -> RPCContext (Bool)
getFairingJettisoned thisArg = simpleRequest $ getFairingJettisonedReq thisArg

getFairingJettisonedStreamReq :: KRPCHS.SpaceCenter.Fairing -> KRPCStreamReq (Bool)
getFairingJettisonedStreamReq thisArg = makeStreamReq $ getFairingJettisonedReq thisArg

getFairingJettisonedStream :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCStream (Bool))
getFairingJettisonedStream thisArg = requestAddStream $ getFairingJettisonedStreamReq thisArg 

{-|
The part object for this fairing.
 -}
getFairingPartReq :: KRPCHS.SpaceCenter.Fairing -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getFairingPartReq thisArg = makeCallReq "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]

getFairingPart :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCHS.SpaceCenter.Part)
getFairingPart thisArg = simpleRequest $ getFairingPartReq thisArg

getFairingPartStreamReq :: KRPCHS.SpaceCenter.Fairing -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getFairingPartStreamReq thisArg = makeStreamReq $ getFairingPartReq thisArg

getFairingPartStream :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getFairingPartStream thisArg = requestAddStream $ getFairingPartStreamReq thisArg 

{-|
The total aerodynamic forces acting on the vessel, as a vector pointing in the direction of the force, with its
magnitude equal to the strength of the force in Newtons.
 -}
getFlightAerodynamicForceReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightAerodynamicForceReq thisArg = makeCallReq "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]

getFlightAerodynamicForce :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAerodynamicForce thisArg = simpleRequest $ getFlightAerodynamicForceReq thisArg

getFlightAerodynamicForceStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAerodynamicForceStreamReq thisArg = makeStreamReq $ getFlightAerodynamicForceReq thisArg

getFlightAerodynamicForceStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAerodynamicForceStream thisArg = requestAddStream $ getFlightAerodynamicForceStreamReq thisArg 

{-|
Gets the pitch angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightAngleOfAttackReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightAngleOfAttackReq thisArg = makeCallReq "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]

getFlightAngleOfAttack :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightAngleOfAttack thisArg = simpleRequest $ getFlightAngleOfAttackReq thisArg

getFlightAngleOfAttackStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightAngleOfAttackStreamReq thisArg = makeStreamReq $ getFlightAngleOfAttackReq thisArg

getFlightAngleOfAttackStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightAngleOfAttackStream thisArg = requestAddStream $ getFlightAngleOfAttackStreamReq thisArg 

{-|
The unit direction vector pointing in the anti-normal direction.
 -}
getFlightAntiNormalReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightAntiNormalReq thisArg = makeCallReq "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]

getFlightAntiNormal :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAntiNormal thisArg = simpleRequest $ getFlightAntiNormalReq thisArg

getFlightAntiNormalStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAntiNormalStreamReq thisArg = makeStreamReq $ getFlightAntiNormalReq thisArg

getFlightAntiNormalStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiNormalStream thisArg = requestAddStream $ getFlightAntiNormalStreamReq thisArg 

{-|
The unit direction vector pointing in the anti-radial direction.
 -}
getFlightAntiRadialReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightAntiRadialReq thisArg = makeCallReq "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]

getFlightAntiRadial :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAntiRadial thisArg = simpleRequest $ getFlightAntiRadialReq thisArg

getFlightAntiRadialStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAntiRadialStreamReq thisArg = makeStreamReq $ getFlightAntiRadialReq thisArg

getFlightAntiRadialStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiRadialStream thisArg = requestAddStream $ getFlightAntiRadialStreamReq thisArg 

{-|
The current density of the atmosphere around the vessel, inkg/m^3.
 -}
getFlightAtmosphereDensityReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightAtmosphereDensityReq thisArg = makeCallReq "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]

getFlightAtmosphereDensity :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightAtmosphereDensity thisArg = simpleRequest $ getFlightAtmosphereDensityReq thisArg

getFlightAtmosphereDensityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightAtmosphereDensityStreamReq thisArg = makeStreamReq $ getFlightAtmosphereDensityReq thisArg

getFlightAtmosphereDensityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightAtmosphereDensityStream thisArg = requestAddStream $ getFlightAtmosphereDensityStreamReq thisArg 

{-|
Gets the <a href="https://en.wikipedia.org/wiki/Ballistic_coefficient">ballistic coefficient.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightBallisticCoefficientReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightBallisticCoefficientReq thisArg = makeCallReq "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]

getFlightBallisticCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightBallisticCoefficient thisArg = simpleRequest $ getFlightBallisticCoefficientReq thisArg

getFlightBallisticCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightBallisticCoefficientStreamReq thisArg = makeStreamReq $ getFlightBallisticCoefficientReq thisArg

getFlightBallisticCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightBallisticCoefficientStream thisArg = requestAddStream $ getFlightBallisticCoefficientStreamReq thisArg 

{-|
The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
Measured from the center of mass of the vessel.
 -}
getFlightBedrockAltitudeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightBedrockAltitudeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]

getFlightBedrockAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightBedrockAltitude thisArg = simpleRequest $ getFlightBedrockAltitudeReq thisArg

getFlightBedrockAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightBedrockAltitudeStreamReq thisArg = makeStreamReq $ getFlightBedrockAltitudeReq thisArg

getFlightBedrockAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightBedrockAltitudeStream thisArg = requestAddStream $ getFlightBedrockAltitudeStreamReq thisArg 

{-|
The position of the center of mass of the vessel.
 -}
getFlightCenterOfMassReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightCenterOfMassReq thisArg = makeCallReq "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]

getFlightCenterOfMass :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightCenterOfMass thisArg = simpleRequest $ getFlightCenterOfMassReq thisArg

getFlightCenterOfMassStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightCenterOfMassStreamReq thisArg = makeStreamReq $ getFlightCenterOfMassReq thisArg

getFlightCenterOfMassStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightCenterOfMassStream thisArg = requestAddStream $ getFlightCenterOfMassStreamReq thisArg 

{-|
The direction vector that the vessel is pointing in.
 -}
getFlightDirectionReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightDirectionReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]

getFlightDirection :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightDirection thisArg = simpleRequest $ getFlightDirectionReq thisArg

getFlightDirectionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightDirectionStreamReq thisArg = makeStreamReq $ getFlightDirectionReq thisArg

getFlightDirectionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDirectionStream thisArg = requestAddStream $ getFlightDirectionStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic dragcurrently acting on the vessel,
as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.
 -}
getFlightDragReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightDragReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]

getFlightDrag :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightDrag thisArg = simpleRequest $ getFlightDragReq thisArg

getFlightDragStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightDragStreamReq thisArg = makeStreamReq $ getFlightDragReq thisArg

getFlightDragStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDragStream thisArg = requestAddStream $ getFlightDragStreamReq thisArg 

{-|
Gets the coefficient of drag. This is the amount of drag produced by the vessel. It depends on air speed,
air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightDragCoefficientReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightDragCoefficientReq thisArg = makeCallReq "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]

getFlightDragCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightDragCoefficient thisArg = simpleRequest $ getFlightDragCoefficientReq thisArg

getFlightDragCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightDragCoefficientStreamReq thisArg = makeStreamReq $ getFlightDragCoefficientReq thisArg

getFlightDragCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightDragCoefficientStream thisArg = requestAddStream $ getFlightDragCoefficientStreamReq thisArg 

{-|
The dynamic pressure acting on the vessel, in Pascals. This is a measure of the strength of the
aerodynamic forces. It is equal to\frac{1}{2} . \mbox{air density} .  \mbox{velocity}^2.
It is commonly denotedQ.
 -}
getFlightDynamicPressureReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightDynamicPressureReq thisArg = makeCallReq "SpaceCenter" "Flight_get_DynamicPressure" [makeArgument 0 thisArg]

getFlightDynamicPressure :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightDynamicPressure thisArg = simpleRequest $ getFlightDynamicPressureReq thisArg

getFlightDynamicPressureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightDynamicPressureStreamReq thisArg = makeStreamReq $ getFlightDynamicPressureReq thisArg

getFlightDynamicPressureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightDynamicPressureStream thisArg = requestAddStream $ getFlightDynamicPressureStreamReq thisArg 

{-|
The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level,
and is negative when the vessel is over the sea.
 -}
getFlightElevationReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightElevationReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]

getFlightElevation :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightElevation thisArg = simpleRequest $ getFlightElevationReq thisArg

getFlightElevationStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightElevationStreamReq thisArg = makeStreamReq $ getFlightElevationReq thisArg

getFlightElevationStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightElevationStream thisArg = requestAddStream $ getFlightElevationStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Equivalent_airspeed">equivalent air speedof the vessel, inm/s.
 -}
getFlightEquivalentAirSpeedReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightEquivalentAirSpeedReq thisArg = makeCallReq "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]

getFlightEquivalentAirSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightEquivalentAirSpeed thisArg = simpleRequest $ getFlightEquivalentAirSpeedReq thisArg

getFlightEquivalentAirSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightEquivalentAirSpeedStreamReq thisArg = makeStreamReq $ getFlightEquivalentAirSpeedReq thisArg

getFlightEquivalentAirSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightEquivalentAirSpeedStream thisArg = requestAddStream $ getFlightEquivalentAirSpeedStreamReq thisArg 

{-|
The current G force acting on the vessel inm/s^2.
 -}
getFlightGForceReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightGForceReq thisArg = makeCallReq "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]

getFlightGForce :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightGForce thisArg = simpleRequest $ getFlightGForceReq thisArg

getFlightGForceStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightGForceStreamReq thisArg = makeStreamReq $ getFlightGForceReq thisArg

getFlightGForceStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightGForceStream thisArg = requestAddStream $ getFlightGForceStreamReq thisArg 

{-|
The heading angle of the vessel relative to north, in degrees. A value between 0° and 360°.
 -}
getFlightHeadingReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightHeadingReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]

getFlightHeading :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightHeading thisArg = simpleRequest $ getFlightHeadingReq thisArg

getFlightHeadingStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightHeadingStreamReq thisArg = makeStreamReq $ getFlightHeadingReq thisArg

getFlightHeadingStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightHeadingStream thisArg = requestAddStream $ getFlightHeadingStreamReq thisArg 

{-|
The horizontal speed of the vessel in meters per second.
 -}
getFlightHorizontalSpeedReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightHorizontalSpeedReq thisArg = makeCallReq "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]

getFlightHorizontalSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightHorizontalSpeed thisArg = simpleRequest $ getFlightHorizontalSpeedReq thisArg

getFlightHorizontalSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightHorizontalSpeedStreamReq thisArg = makeStreamReq $ getFlightHorizontalSpeedReq thisArg

getFlightHorizontalSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightHorizontalSpeedStream thisArg = requestAddStream $ getFlightHorizontalSpeedStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Latitude">latitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLatitudeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightLatitudeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]

getFlightLatitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightLatitude thisArg = simpleRequest $ getFlightLatitudeReq thisArg

getFlightLatitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightLatitudeStreamReq thisArg = makeStreamReq $ getFlightLatitudeReq thisArg

getFlightLatitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightLatitudeStream thisArg = requestAddStream $ getFlightLatitudeStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic liftcurrently acting on the vessel,
as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.
 -}
getFlightLiftReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightLiftReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]

getFlightLift :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightLift thisArg = simpleRequest $ getFlightLiftReq thisArg

getFlightLiftStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightLiftStreamReq thisArg = makeStreamReq $ getFlightLiftReq thisArg

getFlightLiftStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightLiftStream thisArg = requestAddStream $ getFlightLiftStreamReq thisArg 

{-|
Gets the coefficient of lift. This is the amount of lift produced by the vessel, and depends on air speed, air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightLiftCoefficientReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightLiftCoefficientReq thisArg = makeCallReq "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]

getFlightLiftCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightLiftCoefficient thisArg = simpleRequest $ getFlightLiftCoefficientReq thisArg

getFlightLiftCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightLiftCoefficientStreamReq thisArg = makeStreamReq $ getFlightLiftCoefficientReq thisArg

getFlightLiftCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightLiftCoefficientStream thisArg = requestAddStream $ getFlightLiftCoefficientStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Longitude">longitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLongitudeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightLongitudeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]

getFlightLongitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightLongitude thisArg = simpleRequest $ getFlightLongitudeReq thisArg

getFlightLongitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightLongitudeStreamReq thisArg = makeStreamReq $ getFlightLongitudeReq thisArg

getFlightLongitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightLongitudeStream thisArg = requestAddStream $ getFlightLongitudeStreamReq thisArg 

{-|
The speed of the vessel, in multiples of the speed of sound.
 -}
getFlightMachReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightMachReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]

getFlightMach :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightMach thisArg = simpleRequest $ getFlightMachReq thisArg

getFlightMachStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightMachStreamReq thisArg = makeStreamReq $ getFlightMachReq thisArg

getFlightMachStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightMachStream thisArg = requestAddStream $ getFlightMachStreamReq thisArg 

{-|
The altitude above sea level, in meters.
Measured from the center of mass of the vessel.
 -}
getFlightMeanAltitudeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightMeanAltitudeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]

getFlightMeanAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightMeanAltitude thisArg = simpleRequest $ getFlightMeanAltitudeReq thisArg

getFlightMeanAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightMeanAltitudeStreamReq thisArg = makeStreamReq $ getFlightMeanAltitudeReq thisArg

getFlightMeanAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightMeanAltitudeStream thisArg = requestAddStream $ getFlightMeanAltitudeStreamReq thisArg 

{-|
The unit direction vector pointing in the normal direction.
 -}
getFlightNormalReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightNormalReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]

getFlightNormal :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightNormal thisArg = simpleRequest $ getFlightNormalReq thisArg

getFlightNormalStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightNormalStreamReq thisArg = makeStreamReq $ getFlightNormalReq thisArg

getFlightNormalStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightNormalStream thisArg = requestAddStream $ getFlightNormalStreamReq thisArg 

{-|
The pitch angle of the vessel relative to the horizon, in degrees. A value between -90° and +90°.
 -}
getFlightPitchReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightPitchReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]

getFlightPitch :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightPitch thisArg = simpleRequest $ getFlightPitchReq thisArg

getFlightPitchStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightPitchStreamReq thisArg = makeStreamReq $ getFlightPitchReq thisArg

getFlightPitchStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightPitchStream thisArg = requestAddStream $ getFlightPitchStreamReq thisArg 

{-|
The unit direction vector pointing in the prograde direction.
 -}
getFlightProgradeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightProgradeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]

getFlightPrograde :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightPrograde thisArg = simpleRequest $ getFlightProgradeReq thisArg

getFlightProgradeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightProgradeStreamReq thisArg = makeStreamReq $ getFlightProgradeReq thisArg

getFlightProgradeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightProgradeStream thisArg = requestAddStream $ getFlightProgradeStreamReq thisArg 

{-|
The unit direction vector pointing in the radial direction.
 -}
getFlightRadialReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightRadialReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]

getFlightRadial :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightRadial thisArg = simpleRequest $ getFlightRadialReq thisArg

getFlightRadialStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightRadialStreamReq thisArg = makeStreamReq $ getFlightRadialReq thisArg

getFlightRadialStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRadialStream thisArg = requestAddStream $ getFlightRadialStreamReq thisArg 

{-|
The unit direction vector pointing in the retrograde direction.
 -}
getFlightRetrogradeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightRetrogradeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]

getFlightRetrograde :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightRetrograde thisArg = simpleRequest $ getFlightRetrogradeReq thisArg

getFlightRetrogradeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightRetrogradeStreamReq thisArg = makeStreamReq $ getFlightRetrogradeReq thisArg

getFlightRetrogradeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRetrogradeStream thisArg = requestAddStream $ getFlightRetrogradeStreamReq thisArg 

{-|
The vessels Reynolds number.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightReynoldsNumberReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightReynoldsNumberReq thisArg = makeCallReq "SpaceCenter" "Flight_get_ReynoldsNumber" [makeArgument 0 thisArg]

getFlightReynoldsNumber :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightReynoldsNumber thisArg = simpleRequest $ getFlightReynoldsNumberReq thisArg

getFlightReynoldsNumberStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightReynoldsNumberStreamReq thisArg = makeStreamReq $ getFlightReynoldsNumberReq thisArg

getFlightReynoldsNumberStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightReynoldsNumberStream thisArg = requestAddStream $ getFlightReynoldsNumberStreamReq thisArg 

{-|
The roll angle of the vessel relative to the horizon, in degrees. A value between -180° and +180°.
 -}
getFlightRollReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightRollReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]

getFlightRoll :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightRoll thisArg = simpleRequest $ getFlightRollReq thisArg

getFlightRollStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightRollStreamReq thisArg = makeStreamReq $ getFlightRollReq thisArg

getFlightRollStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightRollStream thisArg = requestAddStream $ getFlightRollStreamReq thisArg 

{-|
The rotation of the vessel.
 -}
getFlightRotationReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double, Double))
getFlightRotationReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]

getFlightRotation :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double, Double))
getFlightRotation thisArg = simpleRequest $ getFlightRotationReq thisArg

getFlightRotationStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double, Double))
getFlightRotationStreamReq thisArg = makeStreamReq $ getFlightRotationReq thisArg

getFlightRotationStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getFlightRotationStream thisArg = requestAddStream $ getFlightRotationStreamReq thisArg 

{-|
Gets the yaw angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightSideslipAngleReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightSideslipAngleReq thisArg = makeCallReq "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]

getFlightSideslipAngle :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightSideslipAngle thisArg = simpleRequest $ getFlightSideslipAngleReq thisArg

getFlightSideslipAngleStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightSideslipAngleStreamReq thisArg = makeStreamReq $ getFlightSideslipAngleReq thisArg

getFlightSideslipAngleStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightSideslipAngleStream thisArg = requestAddStream $ getFlightSideslipAngleStreamReq thisArg 

{-|
The speed of the vessel in meters per second.
 -}
getFlightSpeedReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightSpeedReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]

getFlightSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightSpeed thisArg = simpleRequest $ getFlightSpeedReq thisArg

getFlightSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightSpeedStreamReq thisArg = makeStreamReq $ getFlightSpeedReq thisArg

getFlightSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightSpeedStream thisArg = requestAddStream $ getFlightSpeedStreamReq thisArg 

{-|
The speed of sound, in the atmosphere around the vessel, inm/s.
 -}
getFlightSpeedOfSoundReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightSpeedOfSoundReq thisArg = makeCallReq "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]

getFlightSpeedOfSound :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightSpeedOfSound thisArg = simpleRequest $ getFlightSpeedOfSoundReq thisArg

getFlightSpeedOfSoundStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightSpeedOfSoundStreamReq thisArg = makeStreamReq $ getFlightSpeedOfSoundReq thisArg

getFlightSpeedOfSoundStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightSpeedOfSoundStream thisArg = requestAddStream $ getFlightSpeedOfSoundStreamReq thisArg 

{-|
Gets the current amount of stall, between 0 and 1. A value greater than 0.005 indicates a minor stall
and a value greater than 0.5 indicates a large-scale stall.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightStallFractionReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightStallFractionReq thisArg = makeCallReq "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]

getFlightStallFraction :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStallFraction thisArg = simpleRequest $ getFlightStallFractionReq thisArg

getFlightStallFractionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStallFractionStreamReq thisArg = makeStreamReq $ getFlightStallFractionReq thisArg

getFlightStallFractionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStallFractionStream thisArg = requestAddStream $ getFlightStallFractionStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">static (ambient) temperatureof the
atmosphere around the vessel, in Kelvin.
 -}
getFlightStaticAirTemperatureReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightStaticAirTemperatureReq thisArg = makeCallReq "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]

getFlightStaticAirTemperature :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticAirTemperature thisArg = simpleRequest $ getFlightStaticAirTemperatureReq thisArg

getFlightStaticAirTemperatureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticAirTemperatureStreamReq thisArg = makeStreamReq $ getFlightStaticAirTemperatureReq thisArg

getFlightStaticAirTemperatureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticAirTemperatureStream thisArg = requestAddStream $ getFlightStaticAirTemperatureStreamReq thisArg 

{-|
The static atmospheric pressure acting on the vessel, in Pascals.
 -}
getFlightStaticPressureReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightStaticPressureReq thisArg = makeCallReq "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]

getFlightStaticPressure :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticPressure thisArg = simpleRequest $ getFlightStaticPressureReq thisArg

getFlightStaticPressureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticPressureStreamReq thisArg = makeStreamReq $ getFlightStaticPressureReq thisArg

getFlightStaticPressureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticPressureStream thisArg = requestAddStream $ getFlightStaticPressureStreamReq thisArg 

{-|
The static atmospheric pressure at mean sea level, in Pascals.
 -}
getFlightStaticPressureAtMSLReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightStaticPressureAtMSLReq thisArg = makeCallReq "SpaceCenter" "Flight_get_StaticPressureAtMSL" [makeArgument 0 thisArg]

getFlightStaticPressureAtMSL :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticPressureAtMSL thisArg = simpleRequest $ getFlightStaticPressureAtMSLReq thisArg

getFlightStaticPressureAtMSLStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticPressureAtMSLStreamReq thisArg = makeStreamReq $ getFlightStaticPressureAtMSLReq thisArg

getFlightStaticPressureAtMSLStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticPressureAtMSLStream thisArg = requestAddStream $ getFlightStaticPressureAtMSLStreamReq thisArg 

{-|
The altitude above the surface of the body or sea level, whichever is closer, in meters.
Measured from the center of mass of the vessel.
 -}
getFlightSurfaceAltitudeReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightSurfaceAltitudeReq thisArg = makeCallReq "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]

getFlightSurfaceAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightSurfaceAltitude thisArg = simpleRequest $ getFlightSurfaceAltitudeReq thisArg

getFlightSurfaceAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightSurfaceAltitudeStreamReq thisArg = makeStreamReq $ getFlightSurfaceAltitudeReq thisArg

getFlightSurfaceAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightSurfaceAltitudeStream thisArg = requestAddStream $ getFlightSurfaceAltitudeStreamReq thisArg 

{-|
An estimate of the current terminal velocity of the vessel, inm/s.
This is the speed at which the drag forces cancel out the force of gravity.
 -}
getFlightTerminalVelocityReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightTerminalVelocityReq thisArg = makeCallReq "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]

getFlightTerminalVelocity :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTerminalVelocity thisArg = simpleRequest $ getFlightTerminalVelocityReq thisArg

getFlightTerminalVelocityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTerminalVelocityStreamReq thisArg = makeStreamReq $ getFlightTerminalVelocityReq thisArg

getFlightTerminalVelocityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTerminalVelocityStream thisArg = requestAddStream $ getFlightTerminalVelocityStreamReq thisArg 

{-|
Gets the thrust specific fuel consumption for the jet engines on the vessel. This is a measure of the
efficiency of the engines, with a lower value indicating a more efficient vessel. This value is the
number of Newtons of fuel that are burned, per hour, to produce one newton of thrust.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightThrustSpecificFuelConsumptionReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightThrustSpecificFuelConsumptionReq thisArg = makeCallReq "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]

getFlightThrustSpecificFuelConsumption :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightThrustSpecificFuelConsumption thisArg = simpleRequest $ getFlightThrustSpecificFuelConsumptionReq thisArg

getFlightThrustSpecificFuelConsumptionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightThrustSpecificFuelConsumptionStreamReq thisArg = makeStreamReq $ getFlightThrustSpecificFuelConsumptionReq thisArg

getFlightThrustSpecificFuelConsumptionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightThrustSpecificFuelConsumptionStream thisArg = requestAddStream $ getFlightThrustSpecificFuelConsumptionStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">total air temperatureof the atmosphere
around the vessel, in Kelvin. This temperature includes the <see cref="M:SpaceCenter.Flight.StaticAirTemperature" /> and the vessel's kinetic energy.
 -}
getFlightTotalAirTemperatureReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightTotalAirTemperatureReq thisArg = makeCallReq "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]

getFlightTotalAirTemperature :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTotalAirTemperature thisArg = simpleRequest $ getFlightTotalAirTemperatureReq thisArg

getFlightTotalAirTemperatureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTotalAirTemperatureStreamReq thisArg = makeStreamReq $ getFlightTotalAirTemperatureReq thisArg

getFlightTotalAirTemperatureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTotalAirTemperatureStream thisArg = requestAddStream $ getFlightTotalAirTemperatureStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/True_airspeed">true air speedof the vessel, inm/s.
 -}
getFlightTrueAirSpeedReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Float)
getFlightTrueAirSpeedReq thisArg = makeCallReq "SpaceCenter" "Flight_get_TrueAirSpeed" [makeArgument 0 thisArg]

getFlightTrueAirSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTrueAirSpeed thisArg = simpleRequest $ getFlightTrueAirSpeedReq thisArg

getFlightTrueAirSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTrueAirSpeedStreamReq thisArg = makeStreamReq $ getFlightTrueAirSpeedReq thisArg

getFlightTrueAirSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTrueAirSpeedStream thisArg = requestAddStream $ getFlightTrueAirSpeedStreamReq thisArg 

{-|
The velocity vector of the vessel. The magnitude of the vector is the speed of the vessel in meters per second.
The direction of the vector is the direction of the vessels motion.
 -}
getFlightVelocityReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq ((Double, Double, Double))
getFlightVelocityReq thisArg = makeCallReq "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]

getFlightVelocity :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightVelocity thisArg = simpleRequest $ getFlightVelocityReq thisArg

getFlightVelocityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightVelocityStreamReq thisArg = makeStreamReq $ getFlightVelocityReq thisArg

getFlightVelocityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightVelocityStream thisArg = requestAddStream $ getFlightVelocityStreamReq thisArg 

{-|
The vertical speed of the vessel in meters per second.
 -}
getFlightVerticalSpeedReq :: KRPCHS.SpaceCenter.Flight -> KRPCCallReq (Double)
getFlightVerticalSpeedReq thisArg = makeCallReq "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]

getFlightVerticalSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightVerticalSpeed thisArg = simpleRequest $ getFlightVerticalSpeedReq thisArg

getFlightVerticalSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightVerticalSpeedStreamReq thisArg = makeStreamReq $ getFlightVerticalSpeedReq thisArg

getFlightVerticalSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightVerticalSpeedStream thisArg = requestAddStream $ getFlightVerticalSpeedStreamReq thisArg 

{-|
Remove the force.
 -}
forceRemoveReq :: KRPCHS.SpaceCenter.Force -> KRPCCallReq ()
forceRemoveReq thisArg = makeCallReq "SpaceCenter" "Force_Remove" [makeArgument 0 thisArg]

forceRemove :: KRPCHS.SpaceCenter.Force -> RPCContext ()
forceRemove thisArg = simpleRequest $ forceRemoveReq thisArg 

{-|
The force vector. The magnitude of the vector is the strength of the force in Newtons.
 -}
getForceForceVectorReq :: KRPCHS.SpaceCenter.Force -> KRPCCallReq ((Double, Double, Double))
getForceForceVectorReq thisArg = makeCallReq "SpaceCenter" "Force_get_ForceVector" [makeArgument 0 thisArg]

getForceForceVector :: KRPCHS.SpaceCenter.Force -> RPCContext ((Double, Double, Double))
getForceForceVector thisArg = simpleRequest $ getForceForceVectorReq thisArg

getForceForceVectorStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq ((Double, Double, Double))
getForceForceVectorStreamReq thisArg = makeStreamReq $ getForceForceVectorReq thisArg

getForceForceVectorStream :: KRPCHS.SpaceCenter.Force -> RPCContext (KRPCStream ((Double, Double, Double)))
getForceForceVectorStream thisArg = requestAddStream $ getForceForceVectorStreamReq thisArg 

{-|
The part that this force is applied to.
 -}
getForcePartReq :: KRPCHS.SpaceCenter.Force -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getForcePartReq thisArg = makeCallReq "SpaceCenter" "Force_get_Part" [makeArgument 0 thisArg]

getForcePart :: KRPCHS.SpaceCenter.Force -> RPCContext (KRPCHS.SpaceCenter.Part)
getForcePart thisArg = simpleRequest $ getForcePartReq thisArg

getForcePartStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getForcePartStreamReq thisArg = makeStreamReq $ getForcePartReq thisArg

getForcePartStream :: KRPCHS.SpaceCenter.Force -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getForcePartStream thisArg = requestAddStream $ getForcePartStreamReq thisArg 

{-|
The position at which the force acts.
 -}
getForcePositionReq :: KRPCHS.SpaceCenter.Force -> KRPCCallReq ((Double, Double, Double))
getForcePositionReq thisArg = makeCallReq "SpaceCenter" "Force_get_Position" [makeArgument 0 thisArg]

getForcePosition :: KRPCHS.SpaceCenter.Force -> RPCContext ((Double, Double, Double))
getForcePosition thisArg = simpleRequest $ getForcePositionReq thisArg

getForcePositionStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq ((Double, Double, Double))
getForcePositionStreamReq thisArg = makeStreamReq $ getForcePositionReq thisArg

getForcePositionStream :: KRPCHS.SpaceCenter.Force -> RPCContext (KRPCStream ((Double, Double, Double)))
getForcePositionStream thisArg = requestAddStream $ getForcePositionStreamReq thisArg 

{-|
The reference frame of the force vector and position.
 -}
getForceReferenceFrameReq :: KRPCHS.SpaceCenter.Force -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getForceReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Force_get_ReferenceFrame" [makeArgument 0 thisArg]

getForceReferenceFrame :: KRPCHS.SpaceCenter.Force -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getForceReferenceFrame thisArg = simpleRequest $ getForceReferenceFrameReq thisArg

getForceReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Force -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getForceReferenceFrameStreamReq thisArg = makeStreamReq $ getForceReferenceFrameReq thisArg

getForceReferenceFrameStream :: KRPCHS.SpaceCenter.Force -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getForceReferenceFrameStream thisArg = requestAddStream $ getForceReferenceFrameStreamReq thisArg 

{-|
The force vector. The magnitude of the vector is the strength of the force in Newtons.
 -}
setForceForceVectorReq :: KRPCHS.SpaceCenter.Force -> (Double, Double, Double) -> KRPCCallReq ()
setForceForceVectorReq thisArg valueArg = makeCallReq "SpaceCenter" "Force_set_ForceVector" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setForceForceVector :: KRPCHS.SpaceCenter.Force -> (Double, Double, Double) -> RPCContext ()
setForceForceVector thisArg valueArg = simpleRequest $ setForceForceVectorReq thisArg valueArg 

{-|
The position at which the force acts.
 -}
setForcePositionReq :: KRPCHS.SpaceCenter.Force -> (Double, Double, Double) -> KRPCCallReq ()
setForcePositionReq thisArg valueArg = makeCallReq "SpaceCenter" "Force_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setForcePosition :: KRPCHS.SpaceCenter.Force -> (Double, Double, Double) -> RPCContext ()
setForcePosition thisArg valueArg = simpleRequest $ setForcePositionReq thisArg valueArg 

{-|
The reference frame of the force vector and position.
 -}
setForceReferenceFrameReq :: KRPCHS.SpaceCenter.Force -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ()
setForceReferenceFrameReq thisArg valueArg = makeCallReq "SpaceCenter" "Force_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setForceReferenceFrame :: KRPCHS.SpaceCenter.Force -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setForceReferenceFrame thisArg valueArg = simpleRequest $ setForceReferenceFrameReq thisArg valueArg 

{-|
The area of the intake's opening, in square meters.
 -}
getIntakeAreaReq :: KRPCHS.SpaceCenter.Intake -> KRPCCallReq (Float)
getIntakeAreaReq thisArg = makeCallReq "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]

getIntakeArea :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeArea thisArg = simpleRequest $ getIntakeAreaReq thisArg

getIntakeAreaStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeAreaStreamReq thisArg = makeStreamReq $ getIntakeAreaReq thisArg

getIntakeAreaStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeAreaStream thisArg = requestAddStream $ getIntakeAreaStreamReq thisArg 

{-|
The rate of flow into the intake, in units of resource per second.
 -}
getIntakeFlowReq :: KRPCHS.SpaceCenter.Intake -> KRPCCallReq (Float)
getIntakeFlowReq thisArg = makeCallReq "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]

getIntakeFlow :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeFlow thisArg = simpleRequest $ getIntakeFlowReq thisArg

getIntakeFlowStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeFlowStreamReq thisArg = makeStreamReq $ getIntakeFlowReq thisArg

getIntakeFlowStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeFlowStream thisArg = requestAddStream $ getIntakeFlowStreamReq thisArg 

{-|
Whether the intake is open.
 -}
getIntakeOpenReq :: KRPCHS.SpaceCenter.Intake -> KRPCCallReq (Bool)
getIntakeOpenReq thisArg = makeCallReq "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]

getIntakeOpen :: KRPCHS.SpaceCenter.Intake -> RPCContext (Bool)
getIntakeOpen thisArg = simpleRequest $ getIntakeOpenReq thisArg

getIntakeOpenStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Bool)
getIntakeOpenStreamReq thisArg = makeStreamReq $ getIntakeOpenReq thisArg

getIntakeOpenStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Bool))
getIntakeOpenStream thisArg = requestAddStream $ getIntakeOpenStreamReq thisArg 

{-|
The part object for this intake.
 -}
getIntakePartReq :: KRPCHS.SpaceCenter.Intake -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getIntakePartReq thisArg = makeCallReq "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]

getIntakePart :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCHS.SpaceCenter.Part)
getIntakePart thisArg = simpleRequest $ getIntakePartReq thisArg

getIntakePartStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getIntakePartStreamReq thisArg = makeStreamReq $ getIntakePartReq thisArg

getIntakePartStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getIntakePartStream thisArg = requestAddStream $ getIntakePartStreamReq thisArg 

{-|
Speed of the flow into the intake, inm/s.
 -}
getIntakeSpeedReq :: KRPCHS.SpaceCenter.Intake -> KRPCCallReq (Float)
getIntakeSpeedReq thisArg = makeCallReq "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]

getIntakeSpeed :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeSpeed thisArg = simpleRequest $ getIntakeSpeedReq thisArg

getIntakeSpeedStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeSpeedStreamReq thisArg = makeStreamReq $ getIntakeSpeedReq thisArg

getIntakeSpeedStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeSpeedStream thisArg = requestAddStream $ getIntakeSpeedStreamReq thisArg 

{-|
Whether the intake is open.
 -}
setIntakeOpenReq :: KRPCHS.SpaceCenter.Intake -> Bool -> KRPCCallReq ()
setIntakeOpenReq thisArg valueArg = makeCallReq "SpaceCenter" "Intake_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setIntakeOpen :: KRPCHS.SpaceCenter.Intake -> Bool -> RPCContext ()
setIntakeOpen thisArg valueArg = simpleRequest $ setIntakeOpenReq thisArg valueArg 

{-|
Whether the landing gear is deployable.
 -}
getLandingGearDeployableReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCCallReq (Bool)
getLandingGearDeployableReq thisArg = makeCallReq "SpaceCenter" "LandingGear_get_Deployable" [makeArgument 0 thisArg]

getLandingGearDeployable :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (Bool)
getLandingGearDeployable thisArg = simpleRequest $ getLandingGearDeployableReq thisArg

getLandingGearDeployableStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (Bool)
getLandingGearDeployableStreamReq thisArg = makeStreamReq $ getLandingGearDeployableReq thisArg

getLandingGearDeployableStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployableStream thisArg = requestAddStream $ getLandingGearDeployableStreamReq thisArg 

{-|
Whether the landing gear is deployed.Fixed landing gear are always deployed.
Returns an error if you try to deploy fixed landing gear.
 -}
getLandingGearDeployedReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCCallReq (Bool)
getLandingGearDeployedReq thisArg = makeCallReq "SpaceCenter" "LandingGear_get_Deployed" [makeArgument 0 thisArg]

getLandingGearDeployed :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (Bool)
getLandingGearDeployed thisArg = simpleRequest $ getLandingGearDeployedReq thisArg

getLandingGearDeployedStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (Bool)
getLandingGearDeployedStreamReq thisArg = makeStreamReq $ getLandingGearDeployedReq thisArg

getLandingGearDeployedStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployedStream thisArg = requestAddStream $ getLandingGearDeployedStreamReq thisArg 

{-|
The part object for this landing gear.
 -}
getLandingGearPartReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getLandingGearPartReq thisArg = makeCallReq "SpaceCenter" "LandingGear_get_Part" [makeArgument 0 thisArg]

getLandingGearPart :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCHS.SpaceCenter.Part)
getLandingGearPart thisArg = simpleRequest $ getLandingGearPartReq thisArg

getLandingGearPartStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLandingGearPartStreamReq thisArg = makeStreamReq $ getLandingGearPartReq thisArg

getLandingGearPartStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLandingGearPartStream thisArg = requestAddStream $ getLandingGearPartStreamReq thisArg 

{-|
Gets the current state of the landing gear.Fixed landing gear are always deployed.
 -}
getLandingGearStateReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCCallReq (KRPCHS.SpaceCenter.LandingGearState)
getLandingGearStateReq thisArg = makeCallReq "SpaceCenter" "LandingGear_get_State" [makeArgument 0 thisArg]

getLandingGearState :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCHS.SpaceCenter.LandingGearState)
getLandingGearState thisArg = simpleRequest $ getLandingGearStateReq thisArg

getLandingGearStateStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingGearState)
getLandingGearStateStreamReq thisArg = makeStreamReq $ getLandingGearStateReq thisArg

getLandingGearStateStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingGearState))
getLandingGearStateStream thisArg = requestAddStream $ getLandingGearStateStreamReq thisArg 

{-|
Whether the landing gear is deployed.Fixed landing gear are always deployed.
Returns an error if you try to deploy fixed landing gear.
 -}
setLandingGearDeployedReq :: KRPCHS.SpaceCenter.LandingGear -> Bool -> KRPCCallReq ()
setLandingGearDeployedReq thisArg valueArg = makeCallReq "SpaceCenter" "LandingGear_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLandingGearDeployed :: KRPCHS.SpaceCenter.LandingGear -> Bool -> RPCContext ()
setLandingGearDeployed thisArg valueArg = simpleRequest $ setLandingGearDeployedReq thisArg valueArg 

{-|
Whether the landing leg is deployed.Fixed landing legs are always deployed.
Returns an error if you try to deploy fixed landing gear.
 -}
getLandingLegDeployedReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCCallReq (Bool)
getLandingLegDeployedReq thisArg = makeCallReq "SpaceCenter" "LandingLeg_get_Deployed" [makeArgument 0 thisArg]

getLandingLegDeployed :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (Bool)
getLandingLegDeployed thisArg = simpleRequest $ getLandingLegDeployedReq thisArg

getLandingLegDeployedStreamReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCStreamReq (Bool)
getLandingLegDeployedStreamReq thisArg = makeStreamReq $ getLandingLegDeployedReq thisArg

getLandingLegDeployedStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (Bool))
getLandingLegDeployedStream thisArg = requestAddStream $ getLandingLegDeployedStreamReq thisArg 

{-|
The part object for this landing leg.
 -}
getLandingLegPartReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getLandingLegPartReq thisArg = makeCallReq "SpaceCenter" "LandingLeg_get_Part" [makeArgument 0 thisArg]

getLandingLegPart :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCHS.SpaceCenter.Part)
getLandingLegPart thisArg = simpleRequest $ getLandingLegPartReq thisArg

getLandingLegPartStreamReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLandingLegPartStreamReq thisArg = makeStreamReq $ getLandingLegPartReq thisArg

getLandingLegPartStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLandingLegPartStream thisArg = requestAddStream $ getLandingLegPartStreamReq thisArg 

{-|
The current state of the landing leg.
 -}
getLandingLegStateReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCCallReq (KRPCHS.SpaceCenter.LandingLegState)
getLandingLegStateReq thisArg = makeCallReq "SpaceCenter" "LandingLeg_get_State" [makeArgument 0 thisArg]

getLandingLegState :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCHS.SpaceCenter.LandingLegState)
getLandingLegState thisArg = simpleRequest $ getLandingLegStateReq thisArg

getLandingLegStateStreamReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingLegState)
getLandingLegStateStreamReq thisArg = makeStreamReq $ getLandingLegStateReq thisArg

getLandingLegStateStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingLegState))
getLandingLegStateStream thisArg = requestAddStream $ getLandingLegStateStreamReq thisArg 

{-|
Whether the landing leg is deployed.Fixed landing legs are always deployed.
Returns an error if you try to deploy fixed landing gear.
 -}
setLandingLegDeployedReq :: KRPCHS.SpaceCenter.LandingLeg -> Bool -> KRPCCallReq ()
setLandingLegDeployedReq thisArg valueArg = makeCallReq "SpaceCenter" "LandingLeg_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLandingLegDeployed :: KRPCHS.SpaceCenter.LandingLeg -> Bool -> RPCContext ()
setLandingLegDeployed thisArg valueArg = simpleRequest $ setLandingLegDeployedReq thisArg valueArg 

{-|
Releases the docking clamp. Has no effect if the clamp has already been released.
 -}
launchClampReleaseReq :: KRPCHS.SpaceCenter.LaunchClamp -> KRPCCallReq ()
launchClampReleaseReq thisArg = makeCallReq "SpaceCenter" "LaunchClamp_Release" [makeArgument 0 thisArg]

launchClampRelease :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext ()
launchClampRelease thisArg = simpleRequest $ launchClampReleaseReq thisArg 

{-|
The part object for this launch clamp.
 -}
getLaunchClampPartReq :: KRPCHS.SpaceCenter.LaunchClamp -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getLaunchClampPartReq thisArg = makeCallReq "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]

getLaunchClampPart :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (KRPCHS.SpaceCenter.Part)
getLaunchClampPart thisArg = simpleRequest $ getLaunchClampPartReq thisArg

getLaunchClampPartStreamReq :: KRPCHS.SpaceCenter.LaunchClamp -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLaunchClampPartStreamReq thisArg = makeStreamReq $ getLaunchClampPartReq thisArg

getLaunchClampPartStream :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLaunchClampPartStream thisArg = requestAddStream $ getLaunchClampPartStreamReq thisArg 

{-|
Launch a vessel.<param name="craftDirectory">Name of the directory in the current saves "Ships" directory, that contains the craft file. For example"VAB"or"SPH".<param name="name">Name of the vessel to launch. This is the name of the ".craft" file in the save directory, without the ".craft" file extension.<param name="launchSite">Name of the launch site. For example"LaunchPad"or"Runway".
 -}
launchVesselReq :: Data.Text.Text -> Data.Text.Text -> Data.Text.Text -> KRPCCallReq ()
launchVesselReq craftDirectoryArg nameArg launchSiteArg = makeCallReq "SpaceCenter" "LaunchVessel" [makeArgument 0 craftDirectoryArg, makeArgument 1 nameArg, makeArgument 2 launchSiteArg]

launchVessel :: Data.Text.Text -> Data.Text.Text -> Data.Text.Text -> RPCContext ()
launchVessel craftDirectoryArg nameArg launchSiteArg = simpleRequest $ launchVesselReq craftDirectoryArg nameArg launchSiteArg 

{-|
Launch a new vessel from the SPH onto the runway.<param name="name">Name of the vessel to launch.This is equivalent to calling <see cref="M:SpaceCenter.LaunchVessel" /> with the craft directory set to "SPH" and the launch site set to "Runway".
 -}
launchVesselFromSPHReq :: Data.Text.Text -> KRPCCallReq ()
launchVesselFromSPHReq nameArg = makeCallReq "SpaceCenter" "LaunchVesselFromSPH" [makeArgument 0 nameArg]

launchVesselFromSPH :: Data.Text.Text -> RPCContext ()
launchVesselFromSPH nameArg = simpleRequest $ launchVesselFromSPHReq nameArg 

{-|
Launch a new vessel from the VAB onto the launchpad.<param name="name">Name of the vessel to launch.This is equivalent to calling <see cref="M:SpaceCenter.LaunchVessel" /> with the craft directory set to "VAB" and the launch site set to "LaunchPad".
 -}
launchVesselFromVABReq :: Data.Text.Text -> KRPCCallReq ()
launchVesselFromVABReq nameArg = makeCallReq "SpaceCenter" "LaunchVesselFromVAB" [makeArgument 0 nameArg]

launchVesselFromVAB :: Data.Text.Text -> RPCContext ()
launchVesselFromVAB nameArg = simpleRequest $ launchVesselFromVABReq nameArg 

{-|
Returns a list of vessels from the given <paramref name="craftDirectory" /> that can be launched.<param name="craftDirectory">Name of the directory in the current saves "Ships" directory. For example"VAB"or"SPH".
 -}
launchableVesselsReq :: Data.Text.Text -> KRPCCallReq ([Data.Text.Text])
launchableVesselsReq craftDirectoryArg = makeCallReq "SpaceCenter" "LaunchableVessels" [makeArgument 0 craftDirectoryArg]

launchableVessels :: Data.Text.Text -> RPCContext ([Data.Text.Text])
launchableVessels craftDirectoryArg = simpleRequest $ launchableVesselsReq craftDirectoryArg

launchableVesselsStreamReq :: Data.Text.Text -> KRPCStreamReq ([Data.Text.Text])
launchableVesselsStreamReq craftDirectoryArg = makeStreamReq $ launchableVesselsReq craftDirectoryArg

launchableVesselsStream :: Data.Text.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
launchableVesselsStream craftDirectoryArg = requestAddStream $ launchableVesselsStreamReq craftDirectoryArg 

{-|
Whether the light is switched on.
 -}
getLightActiveReq :: KRPCHS.SpaceCenter.Light -> KRPCCallReq (Bool)
getLightActiveReq thisArg = makeCallReq "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]

getLightActive :: KRPCHS.SpaceCenter.Light -> RPCContext (Bool)
getLightActive thisArg = simpleRequest $ getLightActiveReq thisArg

getLightActiveStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (Bool)
getLightActiveStreamReq thisArg = makeStreamReq $ getLightActiveReq thisArg

getLightActiveStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (Bool))
getLightActiveStream thisArg = requestAddStream $ getLightActiveStreamReq thisArg 

{-|
The color of the light, as an RGB triple.
 -}
getLightColorReq :: KRPCHS.SpaceCenter.Light -> KRPCCallReq ((Float, Float, Float))
getLightColorReq thisArg = makeCallReq "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]

getLightColor :: KRPCHS.SpaceCenter.Light -> RPCContext ((Float, Float, Float))
getLightColor thisArg = simpleRequest $ getLightColorReq thisArg

getLightColorStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq ((Float, Float, Float))
getLightColorStreamReq thisArg = makeStreamReq $ getLightColorReq thisArg

getLightColorStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream ((Float, Float, Float)))
getLightColorStream thisArg = requestAddStream $ getLightColorStreamReq thisArg 

{-|
The part object for this light.
 -}
getLightPartReq :: KRPCHS.SpaceCenter.Light -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getLightPartReq thisArg = makeCallReq "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]

getLightPart :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCHS.SpaceCenter.Part)
getLightPart thisArg = simpleRequest $ getLightPartReq thisArg

getLightPartStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLightPartStreamReq thisArg = makeStreamReq $ getLightPartReq thisArg

getLightPartStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLightPartStream thisArg = requestAddStream $ getLightPartStreamReq thisArg 

{-|
The current power usage, in units of charge per second.
 -}
getLightPowerUsageReq :: KRPCHS.SpaceCenter.Light -> KRPCCallReq (Float)
getLightPowerUsageReq thisArg = makeCallReq "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]

getLightPowerUsage :: KRPCHS.SpaceCenter.Light -> RPCContext (Float)
getLightPowerUsage thisArg = simpleRequest $ getLightPowerUsageReq thisArg

getLightPowerUsageStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (Float)
getLightPowerUsageStreamReq thisArg = makeStreamReq $ getLightPowerUsageReq thisArg

getLightPowerUsageStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (Float))
getLightPowerUsageStream thisArg = requestAddStream $ getLightPowerUsageStreamReq thisArg 

{-|
Whether the light is switched on.
 -}
setLightActiveReq :: KRPCHS.SpaceCenter.Light -> Bool -> KRPCCallReq ()
setLightActiveReq thisArg valueArg = makeCallReq "SpaceCenter" "Light_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLightActive :: KRPCHS.SpaceCenter.Light -> Bool -> RPCContext ()
setLightActive thisArg valueArg = simpleRequest $ setLightActiveReq thisArg valueArg 

{-|
The color of the light, as an RGB triple.
 -}
setLightColorReq :: KRPCHS.SpaceCenter.Light -> (Float, Float, Float) -> KRPCCallReq ()
setLightColorReq thisArg valueArg = makeCallReq "SpaceCenter" "Light_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLightColor :: KRPCHS.SpaceCenter.Light -> (Float, Float, Float) -> RPCContext ()
setLightColor thisArg valueArg = simpleRequest $ setLightColorReq thisArg valueArg 

{-|
Load the game with the given name.
This will create a load a save file calledname.sfsfrom the folder of the current save game.
 -}
loadReq :: Data.Text.Text -> KRPCCallReq ()
loadReq nameArg = makeCallReq "SpaceCenter" "Load" [makeArgument 0 nameArg]

load :: Data.Text.Text -> RPCContext ()
load nameArg = simpleRequest $ loadReq nameArg 

{-|
Returns the value of a field.<param name="name">Name of the field.
 -}
moduleGetFieldReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCCallReq (Data.Text.Text)
moduleGetFieldReq thisArg nameArg = makeCallReq "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]

moduleGetField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Data.Text.Text)
moduleGetField thisArg nameArg = simpleRequest $ moduleGetFieldReq thisArg nameArg

moduleGetFieldStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Data.Text.Text)
moduleGetFieldStreamReq thisArg nameArg = makeStreamReq $ moduleGetFieldReq thisArg nameArg

moduleGetFieldStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Data.Text.Text))
moduleGetFieldStream thisArg nameArg = requestAddStream $ moduleGetFieldStreamReq thisArg nameArg 

{-|
trueif the part has an action with the given name.<param name="name">
 -}
moduleHasActionReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCCallReq (Bool)
moduleHasActionReq thisArg nameArg = makeCallReq "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]

moduleHasAction :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasAction thisArg nameArg = simpleRequest $ moduleHasActionReq thisArg nameArg

moduleHasActionStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasActionStreamReq thisArg nameArg = makeStreamReq $ moduleHasActionReq thisArg nameArg

moduleHasActionStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasActionStream thisArg nameArg = requestAddStream $ moduleHasActionStreamReq thisArg nameArg 

{-|
trueif the module has an event with the given name.<param name="name">
 -}
moduleHasEventReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCCallReq (Bool)
moduleHasEventReq thisArg nameArg = makeCallReq "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]

moduleHasEvent :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasEvent thisArg nameArg = simpleRequest $ moduleHasEventReq thisArg nameArg

moduleHasEventStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasEventStreamReq thisArg nameArg = makeStreamReq $ moduleHasEventReq thisArg nameArg

moduleHasEventStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasEventStream thisArg nameArg = requestAddStream $ moduleHasEventStreamReq thisArg nameArg 

{-|
Returnstrueif the module has a field with the given name.<param name="name">Name of the field.
 -}
moduleHasFieldReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCCallReq (Bool)
moduleHasFieldReq thisArg nameArg = makeCallReq "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]

moduleHasField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasField thisArg nameArg = simpleRequest $ moduleHasFieldReq thisArg nameArg

moduleHasFieldStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasFieldStreamReq thisArg nameArg = makeStreamReq $ moduleHasFieldReq thisArg nameArg

moduleHasFieldStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasFieldStream thisArg nameArg = requestAddStream $ moduleHasFieldStreamReq thisArg nameArg 

{-|
Set the value of a field to its original value.<param name="name">Name of the field.
 -}
moduleResetFieldReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCCallReq ()
moduleResetFieldReq thisArg nameArg = makeCallReq "SpaceCenter" "Module_ResetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]

moduleResetField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext ()
moduleResetField thisArg nameArg = simpleRequest $ moduleResetFieldReq thisArg nameArg 

{-|
Set the value of an action with the given name.<param name="name"><param name="value">
 -}
moduleSetActionReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Bool -> KRPCCallReq ()
moduleSetActionReq thisArg nameArg valueArg = makeCallReq "SpaceCenter" "Module_SetAction" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]

moduleSetAction :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Bool -> RPCContext ()
moduleSetAction thisArg nameArg valueArg = simpleRequest $ moduleSetActionReq thisArg nameArg valueArg 

{-|
Set the value of a field to the given floating point number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldFloatReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Float -> KRPCCallReq ()
moduleSetFieldFloatReq thisArg nameArg valueArg = makeCallReq "SpaceCenter" "Module_SetFieldFloat" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]

moduleSetFieldFloat :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Float -> RPCContext ()
moduleSetFieldFloat thisArg nameArg valueArg = simpleRequest $ moduleSetFieldFloatReq thisArg nameArg valueArg 

{-|
Set the value of a field to the given integer number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldIntReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Int.Int32 -> KRPCCallReq ()
moduleSetFieldIntReq thisArg nameArg valueArg = makeCallReq "SpaceCenter" "Module_SetFieldInt" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]

moduleSetFieldInt :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Int.Int32 -> RPCContext ()
moduleSetFieldInt thisArg nameArg valueArg = simpleRequest $ moduleSetFieldIntReq thisArg nameArg valueArg 

{-|
Set the value of a field to the given string.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldStringReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Text.Text -> KRPCCallReq ()
moduleSetFieldStringReq thisArg nameArg valueArg = makeCallReq "SpaceCenter" "Module_SetFieldString" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]

moduleSetFieldString :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Text.Text -> RPCContext ()
moduleSetFieldString thisArg nameArg valueArg = simpleRequest $ moduleSetFieldStringReq thisArg nameArg valueArg 

{-|
Trigger the named event. Equivalent to clicking the button in the right-click menu of the part.<param name="name">
 -}
moduleTriggerEventReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCCallReq ()
moduleTriggerEventReq thisArg nameArg = makeCallReq "SpaceCenter" "Module_TriggerEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]

moduleTriggerEvent :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext ()
moduleTriggerEvent thisArg nameArg = simpleRequest $ moduleTriggerEventReq thisArg nameArg 

{-|
A list of all the names of the modules actions. These are the parts actions that can be assigned
to action groups in the in-game editor.
 -}
getModuleActionsReq :: KRPCHS.SpaceCenter.Module -> KRPCCallReq ([Data.Text.Text])
getModuleActionsReq thisArg = makeCallReq "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]

getModuleActions :: KRPCHS.SpaceCenter.Module -> RPCContext ([Data.Text.Text])
getModuleActions thisArg = simpleRequest $ getModuleActionsReq thisArg

getModuleActionsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq ([Data.Text.Text])
getModuleActionsStreamReq thisArg = makeStreamReq $ getModuleActionsReq thisArg

getModuleActionsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream ([Data.Text.Text]))
getModuleActionsStream thisArg = requestAddStream $ getModuleActionsStreamReq thisArg 

{-|
A list of the names of all of the modules events. Events are the clickable buttons
visible in the right-click menu of the part.
 -}
getModuleEventsReq :: KRPCHS.SpaceCenter.Module -> KRPCCallReq ([Data.Text.Text])
getModuleEventsReq thisArg = makeCallReq "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]

getModuleEvents :: KRPCHS.SpaceCenter.Module -> RPCContext ([Data.Text.Text])
getModuleEvents thisArg = simpleRequest $ getModuleEventsReq thisArg

getModuleEventsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq ([Data.Text.Text])
getModuleEventsStreamReq thisArg = makeStreamReq $ getModuleEventsReq thisArg

getModuleEventsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream ([Data.Text.Text]))
getModuleEventsStream thisArg = requestAddStream $ getModuleEventsStreamReq thisArg 

{-|
The modules field names and their associated values, as a dictionary.
These are the values visible in the right-click menu of the part.
 -}
getModuleFieldsReq :: KRPCHS.SpaceCenter.Module -> KRPCCallReq (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFieldsReq thisArg = makeCallReq "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]

getModuleFields :: KRPCHS.SpaceCenter.Module -> RPCContext (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFields thisArg = simpleRequest $ getModuleFieldsReq thisArg

getModuleFieldsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFieldsStreamReq thisArg = makeStreamReq $ getModuleFieldsReq thisArg

getModuleFieldsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Data.Text.Text)))
getModuleFieldsStream thisArg = requestAddStream $ getModuleFieldsStreamReq thisArg 

{-|
Name of the PartModule. For example, "ModuleEngines".
 -}
getModuleNameReq :: KRPCHS.SpaceCenter.Module -> KRPCCallReq (Data.Text.Text)
getModuleNameReq thisArg = makeCallReq "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]

getModuleName :: KRPCHS.SpaceCenter.Module -> RPCContext (Data.Text.Text)
getModuleName thisArg = simpleRequest $ getModuleNameReq thisArg

getModuleNameStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (Data.Text.Text)
getModuleNameStreamReq thisArg = makeStreamReq $ getModuleNameReq thisArg

getModuleNameStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (Data.Text.Text))
getModuleNameStream thisArg = requestAddStream $ getModuleNameStreamReq thisArg 

{-|
The part that contains this module.
 -}
getModulePartReq :: KRPCHS.SpaceCenter.Module -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getModulePartReq thisArg = makeCallReq "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]

getModulePart :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCHS.SpaceCenter.Part)
getModulePart thisArg = simpleRequest $ getModulePartReq thisArg

getModulePartStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getModulePartStreamReq thisArg = makeStreamReq $ getModulePartReq thisArg

getModulePartStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getModulePartStream thisArg = requestAddStream $ getModulePartStreamReq thisArg 

{-|
Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
is the delta-v of the burn in m/s.<param name="referenceFrame">Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingBurnVector" />.
 -}
nodeBurnVectorReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
nodeBurnVectorReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

nodeBurnVector :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeBurnVector thisArg referenceFrameArg = simpleRequest $ nodeBurnVectorReq thisArg referenceFrameArg

nodeBurnVectorStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeBurnVectorStreamReq thisArg referenceFrameArg = makeStreamReq $ nodeBurnVectorReq thisArg referenceFrameArg

nodeBurnVectorStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeBurnVectorStream thisArg referenceFrameArg = requestAddStream $ nodeBurnVectorStreamReq thisArg referenceFrameArg 

{-|
Returns the unit direction vector of the maneuver nodes burn in the given reference frame.<param name="referenceFrame">
 -}
nodeDirectionReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
nodeDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

nodeDirection :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeDirection thisArg referenceFrameArg = simpleRequest $ nodeDirectionReq thisArg referenceFrameArg

nodeDirectionStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ nodeDirectionReq thisArg referenceFrameArg

nodeDirectionStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeDirectionStream thisArg referenceFrameArg = requestAddStream $ nodeDirectionStreamReq thisArg referenceFrameArg 

{-|
Returns the position vector of the maneuver node in the given reference frame.<param name="referenceFrame">
 -}
nodePositionReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
nodePositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

nodePosition :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodePosition thisArg referenceFrameArg = simpleRequest $ nodePositionReq thisArg referenceFrameArg

nodePositionStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodePositionStreamReq thisArg referenceFrameArg = makeStreamReq $ nodePositionReq thisArg referenceFrameArg

nodePositionStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodePositionStream thisArg referenceFrameArg = requestAddStream $ nodePositionStreamReq thisArg referenceFrameArg 

{-|
Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
is the delta-v of the burn in m/s. The direction and magnitude change as the burn is executed.<param name="referenceFrame">
 -}
nodeRemainingBurnVectorReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
nodeRemainingBurnVectorReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

nodeRemainingBurnVector :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeRemainingBurnVector thisArg referenceFrameArg = simpleRequest $ nodeRemainingBurnVectorReq thisArg referenceFrameArg

nodeRemainingBurnVectorStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeRemainingBurnVectorStreamReq thisArg referenceFrameArg = makeStreamReq $ nodeRemainingBurnVectorReq thisArg referenceFrameArg

nodeRemainingBurnVectorStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeRemainingBurnVectorStream thisArg referenceFrameArg = requestAddStream $ nodeRemainingBurnVectorStreamReq thisArg referenceFrameArg 

{-|
Removes the maneuver node.
 -}
nodeRemoveReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq ()
nodeRemoveReq thisArg = makeCallReq "SpaceCenter" "Node_Remove" [makeArgument 0 thisArg]

nodeRemove :: KRPCHS.SpaceCenter.Node -> RPCContext ()
nodeRemove thisArg = simpleRequest $ nodeRemoveReq thisArg 

{-|
The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
getNodeDeltaVReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Float)
getNodeDeltaVReq thisArg = makeCallReq "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]

getNodeDeltaV :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeDeltaV thisArg = simpleRequest $ getNodeDeltaVReq thisArg

getNodeDeltaVStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeDeltaVStreamReq thisArg = makeStreamReq $ getNodeDeltaVReq thisArg

getNodeDeltaVStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeDeltaVStream thisArg = requestAddStream $ getNodeDeltaVStreamReq thisArg 

{-|
The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
getNodeNormalReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Float)
getNodeNormalReq thisArg = makeCallReq "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]

getNodeNormal :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeNormal thisArg = simpleRequest $ getNodeNormalReq thisArg

getNodeNormalStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeNormalStreamReq thisArg = makeStreamReq $ getNodeNormalReq thisArg

getNodeNormalStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeNormalStream thisArg = requestAddStream $ getNodeNormalStreamReq thisArg 

{-|
The orbit that results from executing the maneuver node.
 -}
getNodeOrbitReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (KRPCHS.SpaceCenter.Orbit)
getNodeOrbitReq thisArg = makeCallReq "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]

getNodeOrbit :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getNodeOrbit thisArg = simpleRequest $ getNodeOrbitReq thisArg

getNodeOrbitStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getNodeOrbitStreamReq thisArg = makeStreamReq $ getNodeOrbitReq thisArg

getNodeOrbitStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getNodeOrbitStream thisArg = requestAddStream $ getNodeOrbitStreamReq thisArg 

{-|
Gets the reference frame that is fixed relative to the maneuver node, and
orientated with the orbital prograde/normal/radial directions of the
original orbit at the maneuver node's position.
<list type="bullet">The origin is at the position of the maneuver node.The x-axis points in the orbital anti-radial direction of the original
orbit, at the position of the maneuver node.The y-axis points in the orbital prograde direction of the original
orbit, at the position of the maneuver node.The z-axis points in the orbital normal direction of the original orbit,
at the position of the maneuver node.
 -}
getNodeOrbitalReferenceFrameReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Node_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]

getNodeOrbitalReferenceFrame :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrame thisArg = simpleRequest $ getNodeOrbitalReferenceFrameReq thisArg

getNodeOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrameStreamReq thisArg = makeStreamReq $ getNodeOrbitalReferenceFrameReq thisArg

getNodeOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeOrbitalReferenceFrameStream thisArg = requestAddStream $ getNodeOrbitalReferenceFrameStreamReq thisArg 

{-|
The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
getNodeProgradeReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Float)
getNodeProgradeReq thisArg = makeCallReq "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]

getNodePrograde :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodePrograde thisArg = simpleRequest $ getNodeProgradeReq thisArg

getNodeProgradeStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeProgradeStreamReq thisArg = makeStreamReq $ getNodeProgradeReq thisArg

getNodeProgradeStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeProgradeStream thisArg = requestAddStream $ getNodeProgradeStreamReq thisArg 

{-|
The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
getNodeRadialReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Float)
getNodeRadialReq thisArg = makeCallReq "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]

getNodeRadial :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeRadial thisArg = simpleRequest $ getNodeRadialReq thisArg

getNodeRadialStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeRadialStreamReq thisArg = makeStreamReq $ getNodeRadialReq thisArg

getNodeRadialStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeRadialStream thisArg = requestAddStream $ getNodeRadialStreamReq thisArg 

{-|
Gets the reference frame that is fixed relative to the maneuver node's burn.
<list type="bullet">The origin is at the position of the maneuver node.The y-axis points in the direction of the burn.The x-axis and z-axis point in arbitrary but fixed directions.
 -}
getNodeReferenceFrameReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]

getNodeReferenceFrame :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrame thisArg = simpleRequest $ getNodeReferenceFrameReq thisArg

getNodeReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrameStreamReq thisArg = makeStreamReq $ getNodeReferenceFrameReq thisArg

getNodeReferenceFrameStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeReferenceFrameStream thisArg = requestAddStream $ getNodeReferenceFrameStreamReq thisArg 

{-|
Gets the remaining delta-v of the maneuver node, in meters per second. Changes as the node
is executed. This is equivalent to the delta-v reported in-game.
 -}
getNodeRemainingDeltaVReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Float)
getNodeRemainingDeltaVReq thisArg = makeCallReq "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]

getNodeRemainingDeltaV :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeRemainingDeltaV thisArg = simpleRequest $ getNodeRemainingDeltaVReq thisArg

getNodeRemainingDeltaVStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeRemainingDeltaVStreamReq thisArg = makeStreamReq $ getNodeRemainingDeltaVReq thisArg

getNodeRemainingDeltaVStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeRemainingDeltaVStream thisArg = requestAddStream $ getNodeRemainingDeltaVStreamReq thisArg 

{-|
The time until the maneuver node will be encountered, in seconds.
 -}
getNodeTimeToReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Double)
getNodeTimeToReq thisArg = makeCallReq "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]

getNodeTimeTo :: KRPCHS.SpaceCenter.Node -> RPCContext (Double)
getNodeTimeTo thisArg = simpleRequest $ getNodeTimeToReq thisArg

getNodeTimeToStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeTimeToStreamReq thisArg = makeStreamReq $ getNodeTimeToReq thisArg

getNodeTimeToStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Double))
getNodeTimeToStream thisArg = requestAddStream $ getNodeTimeToStreamReq thisArg 

{-|
The universal time at which the maneuver will occur, in seconds.
 -}
getNodeUTReq :: KRPCHS.SpaceCenter.Node -> KRPCCallReq (Double)
getNodeUTReq thisArg = makeCallReq "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]

getNodeUT :: KRPCHS.SpaceCenter.Node -> RPCContext (Double)
getNodeUT thisArg = simpleRequest $ getNodeUTReq thisArg

getNodeUTStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeUTStreamReq thisArg = makeStreamReq $ getNodeUTReq thisArg

getNodeUTStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Double))
getNodeUTStream thisArg = requestAddStream $ getNodeUTStreamReq thisArg 

{-|
The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
setNodeDeltaVReq :: KRPCHS.SpaceCenter.Node -> Float -> KRPCCallReq ()
setNodeDeltaVReq thisArg valueArg = makeCallReq "SpaceCenter" "Node_set_DeltaV" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setNodeDeltaV :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodeDeltaV thisArg valueArg = simpleRequest $ setNodeDeltaVReq thisArg valueArg 

{-|
The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
setNodeNormalReq :: KRPCHS.SpaceCenter.Node -> Float -> KRPCCallReq ()
setNodeNormalReq thisArg valueArg = makeCallReq "SpaceCenter" "Node_set_Normal" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setNodeNormal :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodeNormal thisArg valueArg = simpleRequest $ setNodeNormalReq thisArg valueArg 

{-|
The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
setNodeProgradeReq :: KRPCHS.SpaceCenter.Node -> Float -> KRPCCallReq ()
setNodeProgradeReq thisArg valueArg = makeCallReq "SpaceCenter" "Node_set_Prograde" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setNodePrograde :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodePrograde thisArg valueArg = simpleRequest $ setNodeProgradeReq thisArg valueArg 

{-|
The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
setNodeRadialReq :: KRPCHS.SpaceCenter.Node -> Float -> KRPCCallReq ()
setNodeRadialReq thisArg valueArg = makeCallReq "SpaceCenter" "Node_set_Radial" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setNodeRadial :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodeRadial thisArg valueArg = simpleRequest $ setNodeRadialReq thisArg valueArg 

{-|
The universal time at which the maneuver will occur, in seconds.
 -}
setNodeUTReq :: KRPCHS.SpaceCenter.Node -> Double -> KRPCCallReq ()
setNodeUTReq thisArg valueArg = makeCallReq "SpaceCenter" "Node_set_UT" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setNodeUT :: KRPCHS.SpaceCenter.Node -> Double -> RPCContext ()
setNodeUT thisArg valueArg = simpleRequest $ setNodeUTReq thisArg valueArg 

{-|
The eccentric anomaly at the given universal time.<param name="ut">The universal time, in seconds.
 -}
orbitEccentricAnomalyAtUTReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCCallReq (Double)
orbitEccentricAnomalyAtUTReq thisArg utArg = makeCallReq "SpaceCenter" "Orbit_EccentricAnomalyAtUT" [makeArgument 0 thisArg, makeArgument 1 utArg]

orbitEccentricAnomalyAtUT :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (Double)
orbitEccentricAnomalyAtUT thisArg utArg = simpleRequest $ orbitEccentricAnomalyAtUTReq thisArg utArg

orbitEccentricAnomalyAtUTStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitEccentricAnomalyAtUTStreamReq thisArg utArg = makeStreamReq $ orbitEccentricAnomalyAtUTReq thisArg utArg

orbitEccentricAnomalyAtUTStream :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (KRPCStream (Double))
orbitEccentricAnomalyAtUTStream thisArg utArg = requestAddStream $ orbitEccentricAnomalyAtUTStreamReq thisArg utArg 

{-|
The orbital speed at the given time, in meters per second.<param name="time">Time from now, in seconds.
 -}
orbitOrbitalSpeedAtReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCCallReq (Double)
orbitOrbitalSpeedAtReq thisArg timeArg = makeCallReq "SpaceCenter" "Orbit_OrbitalSpeedAt" [makeArgument 0 thisArg, makeArgument 1 timeArg]

orbitOrbitalSpeedAt :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (Double)
orbitOrbitalSpeedAt thisArg timeArg = simpleRequest $ orbitOrbitalSpeedAtReq thisArg timeArg

orbitOrbitalSpeedAtStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitOrbitalSpeedAtStreamReq thisArg timeArg = makeStreamReq $ orbitOrbitalSpeedAtReq thisArg timeArg

orbitOrbitalSpeedAtStream :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (KRPCStream (Double))
orbitOrbitalSpeedAtStream thisArg timeArg = requestAddStream $ orbitOrbitalSpeedAtStreamReq thisArg timeArg 

{-|
The orbital radius at the point in the orbit given by the true anomaly.<param name="trueAnomaly">The true anomaly.
 -}
orbitRadiusAtTrueAnomalyReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCCallReq (Double)
orbitRadiusAtTrueAnomalyReq thisArg trueAnomalyArg = makeCallReq "SpaceCenter" "Orbit_RadiusAtTrueAnomaly" [makeArgument 0 thisArg, makeArgument 1 trueAnomalyArg]

orbitRadiusAtTrueAnomaly :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (Double)
orbitRadiusAtTrueAnomaly thisArg trueAnomalyArg = simpleRequest $ orbitRadiusAtTrueAnomalyReq thisArg trueAnomalyArg

orbitRadiusAtTrueAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitRadiusAtTrueAnomalyStreamReq thisArg trueAnomalyArg = makeStreamReq $ orbitRadiusAtTrueAnomalyReq thisArg trueAnomalyArg

orbitRadiusAtTrueAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (KRPCStream (Double))
orbitRadiusAtTrueAnomalyStream thisArg trueAnomalyArg = requestAddStream $ orbitRadiusAtTrueAnomalyStreamReq thisArg trueAnomalyArg 

{-|
The true anomaly at the given orbital radius.<param name="radius">The orbital radius in meters.
 -}
orbitTrueAnomalyAtRadiusReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCCallReq (Double)
orbitTrueAnomalyAtRadiusReq thisArg radiusArg = makeCallReq "SpaceCenter" "Orbit_TrueAnomalyAtRadius" [makeArgument 0 thisArg, makeArgument 1 radiusArg]

orbitTrueAnomalyAtRadius :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (Double)
orbitTrueAnomalyAtRadius thisArg radiusArg = simpleRequest $ orbitTrueAnomalyAtRadiusReq thisArg radiusArg

orbitTrueAnomalyAtRadiusStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitTrueAnomalyAtRadiusStreamReq thisArg radiusArg = makeStreamReq $ orbitTrueAnomalyAtRadiusReq thisArg radiusArg

orbitTrueAnomalyAtRadiusStream :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (KRPCStream (Double))
orbitTrueAnomalyAtRadiusStream thisArg radiusArg = requestAddStream $ orbitTrueAnomalyAtRadiusStreamReq thisArg radiusArg 

{-|
The true anomaly at the given time.<param name="ut">The universal time in seconds.
 -}
orbitTrueAnomalyAtUTReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCCallReq (Double)
orbitTrueAnomalyAtUTReq thisArg utArg = makeCallReq "SpaceCenter" "Orbit_TrueAnomalyAtUT" [makeArgument 0 thisArg, makeArgument 1 utArg]

orbitTrueAnomalyAtUT :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (Double)
orbitTrueAnomalyAtUT thisArg utArg = simpleRequest $ orbitTrueAnomalyAtUTReq thisArg utArg

orbitTrueAnomalyAtUTStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitTrueAnomalyAtUTStreamReq thisArg utArg = makeStreamReq $ orbitTrueAnomalyAtUTReq thisArg utArg

orbitTrueAnomalyAtUTStream :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (KRPCStream (Double))
orbitTrueAnomalyAtUTStream thisArg utArg = requestAddStream $ orbitTrueAnomalyAtUTStreamReq thisArg utArg 

{-|
The universal time, in seconds, corresponding to the given true anomaly.<param name="trueAnomaly">True anomaly.
 -}
orbitUTAtTrueAnomalyReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCCallReq (Double)
orbitUTAtTrueAnomalyReq thisArg trueAnomalyArg = makeCallReq "SpaceCenter" "Orbit_UTAtTrueAnomaly" [makeArgument 0 thisArg, makeArgument 1 trueAnomalyArg]

orbitUTAtTrueAnomaly :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (Double)
orbitUTAtTrueAnomaly thisArg trueAnomalyArg = simpleRequest $ orbitUTAtTrueAnomalyReq thisArg trueAnomalyArg

orbitUTAtTrueAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> Double -> KRPCStreamReq (Double)
orbitUTAtTrueAnomalyStreamReq thisArg trueAnomalyArg = makeStreamReq $ orbitUTAtTrueAnomalyReq thisArg trueAnomalyArg

orbitUTAtTrueAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> Double -> RPCContext (KRPCStream (Double))
orbitUTAtTrueAnomalyStream thisArg trueAnomalyArg = requestAddStream $ orbitUTAtTrueAnomalyStreamReq thisArg trueAnomalyArg 

{-|
Gets the apoapsis of the orbit, in meters, from the center of mass of the body being orbited.For the apoapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.ApoapsisAltitude" />.
 -}
getOrbitApoapsisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitApoapsisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]

getOrbitApoapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitApoapsis thisArg = simpleRequest $ getOrbitApoapsisReq thisArg

getOrbitApoapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitApoapsisStreamReq thisArg = makeStreamReq $ getOrbitApoapsisReq thisArg

getOrbitApoapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisStream thisArg = requestAddStream $ getOrbitApoapsisStreamReq thisArg 

{-|
The apoapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Apoapsis" /> minus the equatorial radius of the body.
 -}
getOrbitApoapsisAltitudeReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitApoapsisAltitudeReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]

getOrbitApoapsisAltitude :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitApoapsisAltitude thisArg = simpleRequest $ getOrbitApoapsisAltitudeReq thisArg

getOrbitApoapsisAltitudeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitApoapsisAltitudeStreamReq thisArg = makeStreamReq $ getOrbitApoapsisAltitudeReq thisArg

getOrbitApoapsisAltitudeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisAltitudeStream thisArg = requestAddStream $ getOrbitApoapsisAltitudeStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Argument_of_periapsis">argument of periapsis, in radians.
 -}
getOrbitArgumentOfPeriapsisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitArgumentOfPeriapsisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]

getOrbitArgumentOfPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitArgumentOfPeriapsis thisArg = simpleRequest $ getOrbitArgumentOfPeriapsisReq thisArg

getOrbitArgumentOfPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitArgumentOfPeriapsisStreamReq thisArg = makeStreamReq $ getOrbitArgumentOfPeriapsisReq thisArg

getOrbitArgumentOfPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitArgumentOfPeriapsisStream thisArg = requestAddStream $ getOrbitArgumentOfPeriapsisStreamReq thisArg 

{-|
The celestial body (e.g. planet or moon) around which the object is orbiting.
 -}
getOrbitBodyReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBodyReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]

getOrbitBody :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBody thisArg = simpleRequest $ getOrbitBodyReq thisArg

getOrbitBodyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBodyStreamReq thisArg = makeStreamReq $ getOrbitBodyReq thisArg

getOrbitBodyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getOrbitBodyStream thisArg = requestAddStream $ getOrbitBodyStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Eccentric_anomaly">eccentric anomaly.
 -}
getOrbitEccentricAnomalyReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitEccentricAnomalyReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]

getOrbitEccentricAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEccentricAnomaly thisArg = simpleRequest $ getOrbitEccentricAnomalyReq thisArg

getOrbitEccentricAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEccentricAnomalyStreamReq thisArg = makeStreamReq $ getOrbitEccentricAnomalyReq thisArg

getOrbitEccentricAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricAnomalyStream thisArg = requestAddStream $ getOrbitEccentricAnomalyStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Orbital_eccentricity">eccentricityof the orbit.
 -}
getOrbitEccentricityReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitEccentricityReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]

getOrbitEccentricity :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEccentricity thisArg = simpleRequest $ getOrbitEccentricityReq thisArg

getOrbitEccentricityStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEccentricityStreamReq thisArg = makeStreamReq $ getOrbitEccentricityReq thisArg

getOrbitEccentricityStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricityStream thisArg = requestAddStream $ getOrbitEccentricityStreamReq thisArg 

{-|
The time since the epoch (the point at which the
<a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epochwas measured, in seconds.
 -}
getOrbitEpochReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitEpochReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]

getOrbitEpoch :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEpoch thisArg = simpleRequest $ getOrbitEpochReq thisArg

getOrbitEpochStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEpochStreamReq thisArg = makeStreamReq $ getOrbitEpochReq thisArg

getOrbitEpochStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEpochStream thisArg = requestAddStream $ getOrbitEpochStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Orbital_inclination">inclinationof the orbit,
in radians.
 -}
getOrbitInclinationReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitInclinationReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]

getOrbitInclination :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitInclination thisArg = simpleRequest $ getOrbitInclinationReq thisArg

getOrbitInclinationStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitInclinationStreamReq thisArg = makeStreamReq $ getOrbitInclinationReq thisArg

getOrbitInclinationStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitInclinationStream thisArg = requestAddStream $ getOrbitInclinationStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Longitude_of_the_ascending_node">longitude of the
ascending node, in radians.
 -}
getOrbitLongitudeOfAscendingNodeReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitLongitudeOfAscendingNodeReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]

getOrbitLongitudeOfAscendingNode :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitLongitudeOfAscendingNode thisArg = simpleRequest $ getOrbitLongitudeOfAscendingNodeReq thisArg

getOrbitLongitudeOfAscendingNodeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitLongitudeOfAscendingNodeStreamReq thisArg = makeStreamReq $ getOrbitLongitudeOfAscendingNodeReq thisArg

getOrbitLongitudeOfAscendingNodeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitLongitudeOfAscendingNodeStream thisArg = requestAddStream $ getOrbitLongitudeOfAscendingNodeStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly.
 -}
getOrbitMeanAnomalyReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitMeanAnomalyReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]

getOrbitMeanAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitMeanAnomaly thisArg = simpleRequest $ getOrbitMeanAnomalyReq thisArg

getOrbitMeanAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitMeanAnomalyStreamReq thisArg = makeStreamReq $ getOrbitMeanAnomalyReq thisArg

getOrbitMeanAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyStream thisArg = requestAddStream $ getOrbitMeanAnomalyStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epoch.
 -}
getOrbitMeanAnomalyAtEpochReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitMeanAnomalyAtEpochReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]

getOrbitMeanAnomalyAtEpoch :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitMeanAnomalyAtEpoch thisArg = simpleRequest $ getOrbitMeanAnomalyAtEpochReq thisArg

getOrbitMeanAnomalyAtEpochStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitMeanAnomalyAtEpochStreamReq thisArg = makeStreamReq $ getOrbitMeanAnomalyAtEpochReq thisArg

getOrbitMeanAnomalyAtEpochStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyAtEpochStream thisArg = requestAddStream $ getOrbitMeanAnomalyAtEpochStreamReq thisArg 

{-|
If the object is going to change sphere of influence in the future, returns the new orbit
after the change. Otherwise returnsnull.
 -}
getOrbitNextOrbitReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbitReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]

getOrbitNextOrbit :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbit thisArg = simpleRequest $ getOrbitNextOrbitReq thisArg

getOrbitNextOrbitStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbitStreamReq thisArg = makeStreamReq $ getOrbitNextOrbitReq thisArg

getOrbitNextOrbitStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getOrbitNextOrbitStream thisArg = requestAddStream $ getOrbitNextOrbitStreamReq thisArg 

{-|
The current orbital speed in meters per second.
 -}
getOrbitOrbitalSpeedReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitOrbitalSpeedReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_OrbitalSpeed" [makeArgument 0 thisArg]

getOrbitOrbitalSpeed :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitOrbitalSpeed thisArg = simpleRequest $ getOrbitOrbitalSpeedReq thisArg

getOrbitOrbitalSpeedStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitOrbitalSpeedStreamReq thisArg = makeStreamReq $ getOrbitOrbitalSpeedReq thisArg

getOrbitOrbitalSpeedStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitOrbitalSpeedStream thisArg = requestAddStream $ getOrbitOrbitalSpeedStreamReq thisArg 

{-|
The periapsis of the orbit, in meters, from the center of mass of the body being orbited.For the periapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.PeriapsisAltitude" />.
 -}
getOrbitPeriapsisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitPeriapsisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]

getOrbitPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriapsis thisArg = simpleRequest $ getOrbitPeriapsisReq thisArg

getOrbitPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriapsisStreamReq thisArg = makeStreamReq $ getOrbitPeriapsisReq thisArg

getOrbitPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisStream thisArg = requestAddStream $ getOrbitPeriapsisStreamReq thisArg 

{-|
The periapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Periapsis" /> minus the equatorial radius of the body.
 -}
getOrbitPeriapsisAltitudeReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitPeriapsisAltitudeReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]

getOrbitPeriapsisAltitude :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriapsisAltitude thisArg = simpleRequest $ getOrbitPeriapsisAltitudeReq thisArg

getOrbitPeriapsisAltitudeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriapsisAltitudeStreamReq thisArg = makeStreamReq $ getOrbitPeriapsisAltitudeReq thisArg

getOrbitPeriapsisAltitudeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisAltitudeStream thisArg = requestAddStream $ getOrbitPeriapsisAltitudeStreamReq thisArg 

{-|
The orbital period, in seconds.
 -}
getOrbitPeriodReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitPeriodReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]

getOrbitPeriod :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriod thisArg = simpleRequest $ getOrbitPeriodReq thisArg

getOrbitPeriodStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriodStreamReq thisArg = makeStreamReq $ getOrbitPeriodReq thisArg

getOrbitPeriodStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriodStream thisArg = requestAddStream $ getOrbitPeriodStreamReq thisArg 

{-|
The current radius of the orbit, in meters. This is the distance between the center
of mass of the object in orbit, and the center of mass of the body around which it is orbiting.This value will change over time if the orbit is elliptical.
 -}
getOrbitRadiusReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitRadiusReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]

getOrbitRadius :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitRadius thisArg = simpleRequest $ getOrbitRadiusReq thisArg

getOrbitRadiusStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitRadiusStreamReq thisArg = makeStreamReq $ getOrbitRadiusReq thisArg

getOrbitRadiusStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitRadiusStream thisArg = requestAddStream $ getOrbitRadiusStreamReq thisArg 

{-|
The semi-major axis of the orbit, in meters.
 -}
getOrbitSemiMajorAxisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitSemiMajorAxisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]

getOrbitSemiMajorAxis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSemiMajorAxis thisArg = simpleRequest $ getOrbitSemiMajorAxisReq thisArg

getOrbitSemiMajorAxisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSemiMajorAxisStreamReq thisArg = makeStreamReq $ getOrbitSemiMajorAxisReq thisArg

getOrbitSemiMajorAxisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMajorAxisStream thisArg = requestAddStream $ getOrbitSemiMajorAxisStreamReq thisArg 

{-|
The semi-minor axis of the orbit, in meters.
 -}
getOrbitSemiMinorAxisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitSemiMinorAxisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]

getOrbitSemiMinorAxis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSemiMinorAxis thisArg = simpleRequest $ getOrbitSemiMinorAxisReq thisArg

getOrbitSemiMinorAxisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSemiMinorAxisStreamReq thisArg = makeStreamReq $ getOrbitSemiMinorAxisReq thisArg

getOrbitSemiMinorAxisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMinorAxisStream thisArg = requestAddStream $ getOrbitSemiMinorAxisStreamReq thisArg 

{-|
The current orbital speed of the object in meters per second.This value will change over time if the orbit is elliptical.
 -}
getOrbitSpeedReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitSpeedReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]

getOrbitSpeed :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSpeed thisArg = simpleRequest $ getOrbitSpeedReq thisArg

getOrbitSpeedStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSpeedStreamReq thisArg = makeStreamReq $ getOrbitSpeedReq thisArg

getOrbitSpeedStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSpeedStream thisArg = requestAddStream $ getOrbitSpeedStreamReq thisArg 

{-|
The time until the object reaches apoapsis, in seconds.
 -}
getOrbitTimeToApoapsisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitTimeToApoapsisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]

getOrbitTimeToApoapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToApoapsis thisArg = simpleRequest $ getOrbitTimeToApoapsisReq thisArg

getOrbitTimeToApoapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToApoapsisStreamReq thisArg = makeStreamReq $ getOrbitTimeToApoapsisReq thisArg

getOrbitTimeToApoapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToApoapsisStream thisArg = requestAddStream $ getOrbitTimeToApoapsisStreamReq thisArg 

{-|
The time until the object reaches periapsis, in seconds.
 -}
getOrbitTimeToPeriapsisReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitTimeToPeriapsisReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]

getOrbitTimeToPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToPeriapsis thisArg = simpleRequest $ getOrbitTimeToPeriapsisReq thisArg

getOrbitTimeToPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToPeriapsisStreamReq thisArg = makeStreamReq $ getOrbitTimeToPeriapsisReq thisArg

getOrbitTimeToPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToPeriapsisStream thisArg = requestAddStream $ getOrbitTimeToPeriapsisStreamReq thisArg 

{-|
The time until the object changes sphere of influence, in seconds. ReturnsNaNif the
object is not going to change sphere of influence.
 -}
getOrbitTimeToSOIChangeReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitTimeToSOIChangeReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]

getOrbitTimeToSOIChange :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToSOIChange thisArg = simpleRequest $ getOrbitTimeToSOIChangeReq thisArg

getOrbitTimeToSOIChangeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToSOIChangeStreamReq thisArg = makeStreamReq $ getOrbitTimeToSOIChangeReq thisArg

getOrbitTimeToSOIChangeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToSOIChangeStream thisArg = requestAddStream $ getOrbitTimeToSOIChangeStreamReq thisArg 

{-|
The <a href="https://en.wikipedia.org/wiki/True_anomaly">true anomaly.
 -}
getOrbitTrueAnomalyReq :: KRPCHS.SpaceCenter.Orbit -> KRPCCallReq (Double)
getOrbitTrueAnomalyReq thisArg = makeCallReq "SpaceCenter" "Orbit_get_TrueAnomaly" [makeArgument 0 thisArg]

getOrbitTrueAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTrueAnomaly thisArg = simpleRequest $ getOrbitTrueAnomalyReq thisArg

getOrbitTrueAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTrueAnomalyStreamReq thisArg = makeStreamReq $ getOrbitTrueAnomalyReq thisArg

getOrbitTrueAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTrueAnomalyStream thisArg = requestAddStream $ getOrbitTrueAnomalyStreamReq thisArg 

{-|
The unit direction vector from which the orbits longitude of ascending node is measured,
in the given reference frame.<param name="referenceFrame">
 -}
orbitStaticReferencePlaneDirectionReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
orbitStaticReferencePlaneDirectionReq referenceFrameArg = makeCallReq "SpaceCenter" "Orbit_static_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]

orbitStaticReferencePlaneDirection :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitStaticReferencePlaneDirection referenceFrameArg = simpleRequest $ orbitStaticReferencePlaneDirectionReq referenceFrameArg

orbitStaticReferencePlaneDirectionStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
orbitStaticReferencePlaneDirectionStreamReq referenceFrameArg = makeStreamReq $ orbitStaticReferencePlaneDirectionReq referenceFrameArg

orbitStaticReferencePlaneDirectionStream :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitStaticReferencePlaneDirectionStream referenceFrameArg = requestAddStream $ orbitStaticReferencePlaneDirectionStreamReq referenceFrameArg 

{-|
The unit direction vector that is normal to the orbits reference plane, in the given
reference frame. The reference plane is the plane from which the orbits inclination is measured.<param name="referenceFrame">
 -}
orbitStaticReferencePlaneNormalReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
orbitStaticReferencePlaneNormalReq referenceFrameArg = makeCallReq "SpaceCenter" "Orbit_static_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]

orbitStaticReferencePlaneNormal :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitStaticReferencePlaneNormal referenceFrameArg = simpleRequest $ orbitStaticReferencePlaneNormalReq referenceFrameArg

orbitStaticReferencePlaneNormalStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
orbitStaticReferencePlaneNormalStreamReq referenceFrameArg = makeStreamReq $ orbitStaticReferencePlaneNormalReq referenceFrameArg

orbitStaticReferencePlaneNormalStream :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitStaticReferencePlaneNormalStream referenceFrameArg = requestAddStream $ orbitStaticReferencePlaneNormalStreamReq referenceFrameArg 

{-|
Deploys the parachute. This has no effect if the parachute has already
been deployed.
 -}
parachuteDeployReq :: KRPCHS.SpaceCenter.Parachute -> KRPCCallReq ()
parachuteDeployReq thisArg = makeCallReq "SpaceCenter" "Parachute_Deploy" [makeArgument 0 thisArg]

parachuteDeploy :: KRPCHS.SpaceCenter.Parachute -> RPCContext ()
parachuteDeploy thisArg = simpleRequest $ parachuteDeployReq thisArg 

{-|
The altitude at which the parachute will full deploy, in meters.
 -}
getParachuteDeployAltitudeReq :: KRPCHS.SpaceCenter.Parachute -> KRPCCallReq (Float)
getParachuteDeployAltitudeReq thisArg = makeCallReq "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]

getParachuteDeployAltitude :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Float)
getParachuteDeployAltitude thisArg = simpleRequest $ getParachuteDeployAltitudeReq thisArg

getParachuteDeployAltitudeStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Float)
getParachuteDeployAltitudeStreamReq thisArg = makeStreamReq $ getParachuteDeployAltitudeReq thisArg

getParachuteDeployAltitudeStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployAltitudeStream thisArg = requestAddStream $ getParachuteDeployAltitudeStreamReq thisArg 

{-|
The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 -}
getParachuteDeployMinPressureReq :: KRPCHS.SpaceCenter.Parachute -> KRPCCallReq (Float)
getParachuteDeployMinPressureReq thisArg = makeCallReq "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]

getParachuteDeployMinPressure :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Float)
getParachuteDeployMinPressure thisArg = simpleRequest $ getParachuteDeployMinPressureReq thisArg

getParachuteDeployMinPressureStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Float)
getParachuteDeployMinPressureStreamReq thisArg = makeStreamReq $ getParachuteDeployMinPressureReq thisArg

getParachuteDeployMinPressureStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployMinPressureStream thisArg = requestAddStream $ getParachuteDeployMinPressureStreamReq thisArg 

{-|
Whether the parachute has been deployed.
 -}
getParachuteDeployedReq :: KRPCHS.SpaceCenter.Parachute -> KRPCCallReq (Bool)
getParachuteDeployedReq thisArg = makeCallReq "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]

getParachuteDeployed :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Bool)
getParachuteDeployed thisArg = simpleRequest $ getParachuteDeployedReq thisArg

getParachuteDeployedStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Bool)
getParachuteDeployedStreamReq thisArg = makeStreamReq $ getParachuteDeployedReq thisArg

getParachuteDeployedStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Bool))
getParachuteDeployedStream thisArg = requestAddStream $ getParachuteDeployedStreamReq thisArg 

{-|
The part object for this parachute.
 -}
getParachutePartReq :: KRPCHS.SpaceCenter.Parachute -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getParachutePartReq thisArg = makeCallReq "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]

getParachutePart :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCHS.SpaceCenter.Part)
getParachutePart thisArg = simpleRequest $ getParachutePartReq thisArg

getParachutePartStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getParachutePartStreamReq thisArg = makeStreamReq $ getParachutePartReq thisArg

getParachutePartStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getParachutePartStream thisArg = requestAddStream $ getParachutePartStreamReq thisArg 

{-|
The current state of the parachute.
 -}
getParachuteStateReq :: KRPCHS.SpaceCenter.Parachute -> KRPCCallReq (KRPCHS.SpaceCenter.ParachuteState)
getParachuteStateReq thisArg = makeCallReq "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]

getParachuteState :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCHS.SpaceCenter.ParachuteState)
getParachuteState thisArg = simpleRequest $ getParachuteStateReq thisArg

getParachuteStateStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (KRPCHS.SpaceCenter.ParachuteState)
getParachuteStateStreamReq thisArg = makeStreamReq $ getParachuteStateReq thisArg

getParachuteStateStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ParachuteState))
getParachuteStateStream thisArg = requestAddStream $ getParachuteStateStreamReq thisArg 

{-|
The altitude at which the parachute will full deploy, in meters.
 -}
setParachuteDeployAltitudeReq :: KRPCHS.SpaceCenter.Parachute -> Float -> KRPCCallReq ()
setParachuteDeployAltitudeReq thisArg valueArg = makeCallReq "SpaceCenter" "Parachute_set_DeployAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setParachuteDeployAltitude :: KRPCHS.SpaceCenter.Parachute -> Float -> RPCContext ()
setParachuteDeployAltitude thisArg valueArg = simpleRequest $ setParachuteDeployAltitudeReq thisArg valueArg 

{-|
The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 -}
setParachuteDeployMinPressureReq :: KRPCHS.SpaceCenter.Parachute -> Float -> KRPCCallReq ()
setParachuteDeployMinPressureReq thisArg valueArg = makeCallReq "SpaceCenter" "Parachute_set_DeployMinPressure" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setParachuteDeployMinPressure :: KRPCHS.SpaceCenter.Parachute -> Float -> RPCContext ()
setParachuteDeployMinPressure thisArg valueArg = simpleRequest $ setParachuteDeployMinPressureReq thisArg valueArg 

{-|
Exert a constant force on the part, acting at the given position.
Returns an object that can be used to remove or modify the force.
 -}
partAddForceReq :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq (KRPCHS.SpaceCenter.Force)
partAddForceReq thisArg forceArg positionArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_AddForce" [makeArgument 0 thisArg, makeArgument 1 forceArg, makeArgument 2 positionArg, makeArgument 3 referenceFrameArg]

partAddForce :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCHS.SpaceCenter.Force)
partAddForce thisArg forceArg positionArg referenceFrameArg = simpleRequest $ partAddForceReq thisArg forceArg positionArg referenceFrameArg

partAddForceStreamReq :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (KRPCHS.SpaceCenter.Force)
partAddForceStreamReq thisArg forceArg positionArg referenceFrameArg = makeStreamReq $ partAddForceReq thisArg forceArg positionArg referenceFrameArg

partAddForceStream :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Force))
partAddForceStream thisArg forceArg positionArg referenceFrameArg = requestAddStream $ partAddForceStreamReq thisArg forceArg positionArg referenceFrameArg 

{-|
The position of the parts center of mass in the given reference frame.
If the part is physicsless, this is equivalent to <see cref="M:SpaceCenter.Part.Position" />.<param name="referenceFrame">
 -}
partCenterOfMassReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
partCenterOfMassReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

partCenterOfMass :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partCenterOfMass thisArg referenceFrameArg = simpleRequest $ partCenterOfMassReq thisArg referenceFrameArg

partCenterOfMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partCenterOfMassStreamReq thisArg referenceFrameArg = makeStreamReq $ partCenterOfMassReq thisArg referenceFrameArg

partCenterOfMassStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partCenterOfMassStream thisArg referenceFrameArg = requestAddStream $ partCenterOfMassStreamReq thisArg referenceFrameArg 

{-|
The direction of the part in the given reference frame.<param name="referenceFrame">
 -}
partDirectionReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
partDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

partDirection :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partDirection thisArg referenceFrameArg = simpleRequest $ partDirectionReq thisArg referenceFrameArg

partDirectionStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ partDirectionReq thisArg referenceFrameArg

partDirectionStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partDirectionStream thisArg referenceFrameArg = requestAddStream $ partDirectionStreamReq thisArg referenceFrameArg 

{-|
Exert an instantaneous force on the part, acting at the given position.The force is applied instantaneously in a single physics update.
 -}
partInstantaneousForceReq :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ()
partInstantaneousForceReq thisArg forceArg positionArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_InstantaneousForce" [makeArgument 0 thisArg, makeArgument 1 forceArg, makeArgument 2 positionArg, makeArgument 3 referenceFrameArg]

partInstantaneousForce :: KRPCHS.SpaceCenter.Part -> (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
partInstantaneousForce thisArg forceArg positionArg referenceFrameArg = simpleRequest $ partInstantaneousForceReq thisArg forceArg positionArg referenceFrameArg 

{-|
The position of the part in the given reference frame.This is a fixed position in the part, defined by the parts model.
It s not necessarily the same as the parts center of mass.
Use <see cref="M:SpaceCenter.Part.CenterOfMass" /> to get the parts center of mass.<param name="referenceFrame">
 -}
partPositionReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
partPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

partPosition :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partPosition thisArg referenceFrameArg = simpleRequest $ partPositionReq thisArg referenceFrameArg

partPositionStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ partPositionReq thisArg referenceFrameArg

partPositionStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partPositionStream thisArg referenceFrameArg = requestAddStream $ partPositionStreamReq thisArg referenceFrameArg 

{-|
The rotation of the part in the given reference frame.<param name="referenceFrame">
 -}
partRotationReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double, Double))
partRotationReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

partRotation :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
partRotation thisArg referenceFrameArg = simpleRequest $ partRotationReq thisArg referenceFrameArg

partRotationStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
partRotationStreamReq thisArg referenceFrameArg = makeStreamReq $ partRotationReq thisArg referenceFrameArg

partRotationStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
partRotationStream thisArg referenceFrameArg = requestAddStream $ partRotationStreamReq thisArg referenceFrameArg 

{-|
The velocity of the part in the given reference frame.<param name="referenceFrame">
 -}
partVelocityReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
partVelocityReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

partVelocity :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partVelocity thisArg referenceFrameArg = simpleRequest $ partVelocityReq thisArg referenceFrameArg

partVelocityStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partVelocityStreamReq thisArg referenceFrameArg = makeStreamReq $ partVelocityReq thisArg referenceFrameArg

partVelocityStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partVelocityStream thisArg referenceFrameArg = requestAddStream $ partVelocityStreamReq thisArg referenceFrameArg 

{-|
Whether the part is axially attached to its parent, i.e. on the top
or bottom of its parent. If the part has no parent, returnsfalse.
 -}
getPartAxiallyAttachedReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Bool)
getPartAxiallyAttachedReq thisArg = makeCallReq "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]

getPartAxiallyAttached :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartAxiallyAttached thisArg = simpleRequest $ getPartAxiallyAttachedReq thisArg

getPartAxiallyAttachedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartAxiallyAttachedStreamReq thisArg = makeStreamReq $ getPartAxiallyAttachedReq thisArg

getPartAxiallyAttachedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartAxiallyAttachedStream thisArg = requestAddStream $ getPartAxiallyAttachedStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.CargoBay" /> if the part is a cargo bay, otherwisenull.
 -}
getPartCargoBayReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBayReq thisArg = makeCallReq "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]

getPartCargoBay :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBay thisArg = simpleRequest $ getPartCargoBayReq thisArg

getPartCargoBayStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBayStreamReq thisArg = makeStreamReq $ getPartCargoBayReq thisArg

getPartCargoBayStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CargoBay))
getPartCargoBayStream thisArg = requestAddStream $ getPartCargoBayStreamReq thisArg 

{-|
The reference frame that is fixed relative to this part, and centered on its center of mass.
<list type="bullet">The origin is at the center of mass of the part, as returned by <see cref="M:SpaceCenter.Part.CenterOfMass" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartCenterOfMassReferenceFrameReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]

getPartCenterOfMassReferenceFrame :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrame thisArg = simpleRequest $ getPartCenterOfMassReferenceFrameReq thisArg

getPartCenterOfMassReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrameStreamReq thisArg = makeStreamReq $ getPartCenterOfMassReferenceFrameReq thisArg

getPartCenterOfMassReferenceFrameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartCenterOfMassReferenceFrameStream thisArg = requestAddStream $ getPartCenterOfMassReferenceFrameStreamReq thisArg 

{-|
The parts children. Returns an empty list if the part has no children.
This, in combination with <see cref="M:SpaceCenter.Part.Parent" />, can be used to traverse the vessels parts tree.
 -}
getPartChildrenReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
getPartChildrenReq thisArg = makeCallReq "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]

getPartChildren :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartChildren thisArg = simpleRequest $ getPartChildrenReq thisArg

getPartChildrenStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartChildrenStreamReq thisArg = makeStreamReq $ getPartChildrenReq thisArg

getPartChildrenStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartChildrenStream thisArg = requestAddStream $ getPartChildrenStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.ControlSurface" /> if the part is an aerodynamic control surface, otherwisenull.
 -}
getPartControlSurfaceReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurfaceReq thisArg = makeCallReq "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]

getPartControlSurface :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurface thisArg = simpleRequest $ getPartControlSurfaceReq thisArg

getPartControlSurfaceStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurfaceStreamReq thisArg = makeStreamReq $ getPartControlSurfaceReq thisArg

getPartControlSurfaceStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ControlSurface))
getPartControlSurfaceStream thisArg = requestAddStream $ getPartControlSurfaceStreamReq thisArg 

{-|
The cost of the part, in units of funds.
 -}
getPartCostReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartCostReq thisArg = makeCallReq "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]

getPartCost :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartCost thisArg = simpleRequest $ getPartCostReq thisArg

getPartCostStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartCostStreamReq thisArg = makeStreamReq $ getPartCostReq thisArg

getPartCostStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartCostStream thisArg = requestAddStream $ getPartCostStreamReq thisArg 

{-|
Whether this part is crossfeed capable.
 -}
getPartCrossfeedReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Bool)
getPartCrossfeedReq thisArg = makeCallReq "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]

getPartCrossfeed :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartCrossfeed thisArg = simpleRequest $ getPartCrossfeedReq thisArg

getPartCrossfeedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartCrossfeedStreamReq thisArg = makeStreamReq $ getPartCrossfeedReq thisArg

getPartCrossfeedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartCrossfeedStream thisArg = requestAddStream $ getPartCrossfeedStreamReq thisArg 

{-|
The stage in which this part will be decoupled. Returns -1 if the part is never decoupled from the vessel.
 -}
getPartDecoupleStageReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Data.Int.Int32)
getPartDecoupleStageReq thisArg = makeCallReq "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]

getPartDecoupleStage :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Int.Int32)
getPartDecoupleStage thisArg = simpleRequest $ getPartDecoupleStageReq thisArg

getPartDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Int.Int32)
getPartDecoupleStageStreamReq thisArg = makeStreamReq $ getPartDecoupleStageReq thisArg

getPartDecoupleStageStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Int.Int32))
getPartDecoupleStageStream thisArg = requestAddStream $ getPartDecoupleStageStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Decoupler" /> if the part is a decoupler, otherwisenull.
 -}
getPartDecouplerReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Decoupler)
getPartDecouplerReq thisArg = makeCallReq "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]

getPartDecoupler :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Decoupler)
getPartDecoupler thisArg = simpleRequest $ getPartDecouplerReq thisArg

getPartDecouplerStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Decoupler)
getPartDecouplerStreamReq thisArg = makeStreamReq $ getPartDecouplerReq thisArg

getPartDecouplerStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Decoupler))
getPartDecouplerStream thisArg = requestAddStream $ getPartDecouplerStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.DockingPort" /> if the part is a docking port, otherwisenull.
 -}
getPartDockingPortReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPortReq thisArg = makeCallReq "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]

getPartDockingPort :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPort thisArg = simpleRequest $ getPartDockingPortReq thisArg

getPartDockingPortStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPortStreamReq thisArg = makeStreamReq $ getPartDockingPortReq thisArg

getPartDockingPortStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getPartDockingPortStream thisArg = requestAddStream $ getPartDockingPortStreamReq thisArg 

{-|
The mass of the part, not including any resources it contains, in kilograms. Returns zero if the part is massless.
 -}
getPartDryMassReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartDryMassReq thisArg = makeCallReq "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]

getPartDryMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartDryMass thisArg = simpleRequest $ getPartDryMassReq thisArg

getPartDryMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartDryMassStreamReq thisArg = makeStreamReq $ getPartDryMassReq thisArg

getPartDryMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartDryMassStream thisArg = requestAddStream $ getPartDryMassStreamReq thisArg 

{-|
The dynamic pressure acting on the part, in Pascals.
 -}
getPartDynamicPressureReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartDynamicPressureReq thisArg = makeCallReq "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]

getPartDynamicPressure :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartDynamicPressure thisArg = simpleRequest $ getPartDynamicPressureReq thisArg

getPartDynamicPressureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartDynamicPressureStreamReq thisArg = makeStreamReq $ getPartDynamicPressureReq thisArg

getPartDynamicPressureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartDynamicPressureStream thisArg = requestAddStream $ getPartDynamicPressureStreamReq thisArg 

{-|
An <see cref="T:SpaceCenter.Engine" /> if the part is an engine, otherwisenull.
 -}
getPartEngineReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Engine)
getPartEngineReq thisArg = makeCallReq "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]

getPartEngine :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Engine)
getPartEngine thisArg = simpleRequest $ getPartEngineReq thisArg

getPartEngineStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Engine)
getPartEngineStreamReq thisArg = makeStreamReq $ getPartEngineReq thisArg

getPartEngineStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Engine))
getPartEngineStream thisArg = requestAddStream $ getPartEngineStreamReq thisArg 

{-|
An <see cref="T:SpaceCenter.Experiment" /> if the part is a science experiment, otherwisenull.
 -}
getPartExperimentReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Experiment)
getPartExperimentReq thisArg = makeCallReq "SpaceCenter" "Part_get_Experiment" [makeArgument 0 thisArg]

getPartExperiment :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Experiment)
getPartExperiment thisArg = simpleRequest $ getPartExperimentReq thisArg

getPartExperimentStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Experiment)
getPartExperimentStreamReq thisArg = makeStreamReq $ getPartExperimentReq thisArg

getPartExperimentStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Experiment))
getPartExperimentStream thisArg = requestAddStream $ getPartExperimentStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Fairing" /> if the part is a fairing, otherwisenull.
 -}
getPartFairingReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Fairing)
getPartFairingReq thisArg = makeCallReq "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]

getPartFairing :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Fairing)
getPartFairing thisArg = simpleRequest $ getPartFairingReq thisArg

getPartFairingStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Fairing)
getPartFairingStreamReq thisArg = makeStreamReq $ getPartFairingReq thisArg

getPartFairingStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Fairing))
getPartFairingStream thisArg = requestAddStream $ getPartFairingStreamReq thisArg 

{-|
The parts that are connected to this part via fuel lines, where the direction of the fuel line is into this part.
 -}
getPartFuelLinesFromReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFromReq thisArg = makeCallReq "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]

getPartFuelLinesFrom :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFrom thisArg = simpleRequest $ getPartFuelLinesFromReq thisArg

getPartFuelLinesFromStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFromStreamReq thisArg = makeStreamReq $ getPartFuelLinesFromReq thisArg

getPartFuelLinesFromStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesFromStream thisArg = requestAddStream $ getPartFuelLinesFromStreamReq thisArg 

{-|
The parts that are connected to this part via fuel lines, where the direction of the fuel line is out of this part.
 -}
getPartFuelLinesToReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesToReq thisArg = makeCallReq "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]

getPartFuelLinesTo :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesTo thisArg = simpleRequest $ getPartFuelLinesToReq thisArg

getPartFuelLinesToStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesToStreamReq thisArg = makeStreamReq $ getPartFuelLinesToReq thisArg

getPartFuelLinesToStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesToStream thisArg = requestAddStream $ getPartFuelLinesToStreamReq thisArg 

{-|
The impact tolerance of the part, in meters per second.
 -}
getPartImpactToleranceReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartImpactToleranceReq thisArg = makeCallReq "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]

getPartImpactTolerance :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartImpactTolerance thisArg = simpleRequest $ getPartImpactToleranceReq thisArg

getPartImpactToleranceStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartImpactToleranceStreamReq thisArg = makeStreamReq $ getPartImpactToleranceReq thisArg

getPartImpactToleranceStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartImpactToleranceStream thisArg = requestAddStream $ getPartImpactToleranceStreamReq thisArg 

{-|
The inertia tensor of the part in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getPartInertiaTensorReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq ([Double])
getPartInertiaTensorReq thisArg = makeCallReq "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]

getPartInertiaTensor :: KRPCHS.SpaceCenter.Part -> RPCContext ([Double])
getPartInertiaTensor thisArg = simpleRequest $ getPartInertiaTensorReq thisArg

getPartInertiaTensorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([Double])
getPartInertiaTensorStreamReq thisArg = makeStreamReq $ getPartInertiaTensorReq thisArg

getPartInertiaTensorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([Double]))
getPartInertiaTensorStream thisArg = requestAddStream $ getPartInertiaTensorStreamReq thisArg 

{-|
An <see cref="T:SpaceCenter.Intake" /> if the part is an intake, otherwisenull.This includes any part that generates thrust. This covers many different types of engine,
including liquid fuel rockets, solid rocket boosters and jet engines.
For RCS thrusters see <see cref="T:SpaceCenter.RCS" />.
 -}
getPartIntakeReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Intake)
getPartIntakeReq thisArg = makeCallReq "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]

getPartIntake :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Intake)
getPartIntake thisArg = simpleRequest $ getPartIntakeReq thisArg

getPartIntakeStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Intake)
getPartIntakeStreamReq thisArg = makeStreamReq $ getPartIntakeReq thisArg

getPartIntakeStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Intake))
getPartIntakeStream thisArg = requestAddStream $ getPartIntakeStreamReq thisArg 

{-|
Whether this part is a fuel line.
 -}
getPartIsFuelLineReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Bool)
getPartIsFuelLineReq thisArg = makeCallReq "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]

getPartIsFuelLine :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartIsFuelLine thisArg = simpleRequest $ getPartIsFuelLineReq thisArg

getPartIsFuelLineStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartIsFuelLineStreamReq thisArg = makeStreamReq $ getPartIsFuelLineReq thisArg

getPartIsFuelLineStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartIsFuelLineStream thisArg = requestAddStream $ getPartIsFuelLineStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.LandingGear" /> if the part is a landing gear, otherwisenull.
 -}
getPartLandingGearReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.LandingGear)
getPartLandingGearReq thisArg = makeCallReq "SpaceCenter" "Part_get_LandingGear" [makeArgument 0 thisArg]

getPartLandingGear :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LandingGear)
getPartLandingGear thisArg = simpleRequest $ getPartLandingGearReq thisArg

getPartLandingGearStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingGear)
getPartLandingGearStreamReq thisArg = makeStreamReq $ getPartLandingGearReq thisArg

getPartLandingGearStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingGear))
getPartLandingGearStream thisArg = requestAddStream $ getPartLandingGearStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.LandingLeg" /> if the part is a landing leg, otherwisenull.
 -}
getPartLandingLegReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.LandingLeg)
getPartLandingLegReq thisArg = makeCallReq "SpaceCenter" "Part_get_LandingLeg" [makeArgument 0 thisArg]

getPartLandingLeg :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LandingLeg)
getPartLandingLeg thisArg = simpleRequest $ getPartLandingLegReq thisArg

getPartLandingLegStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingLeg)
getPartLandingLegStreamReq thisArg = makeStreamReq $ getPartLandingLegReq thisArg

getPartLandingLegStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingLeg))
getPartLandingLegStream thisArg = requestAddStream $ getPartLandingLegStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.LaunchClamp" /> if the part is a launch clamp, otherwisenull.
 -}
getPartLaunchClampReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClampReq thisArg = makeCallReq "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]

getPartLaunchClamp :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClamp thisArg = simpleRequest $ getPartLaunchClampReq thisArg

getPartLaunchClampStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClampStreamReq thisArg = makeStreamReq $ getPartLaunchClampReq thisArg

getPartLaunchClampStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LaunchClamp))
getPartLaunchClampStream thisArg = requestAddStream $ getPartLaunchClampStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Light" /> if the part is a light, otherwisenull.
 -}
getPartLightReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Light)
getPartLightReq thisArg = makeCallReq "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]

getPartLight :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Light)
getPartLight thisArg = simpleRequest $ getPartLightReq thisArg

getPartLightStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Light)
getPartLightStreamReq thisArg = makeStreamReq $ getPartLightReq thisArg

getPartLightStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Light))
getPartLightStream thisArg = requestAddStream $ getPartLightStreamReq thisArg 

{-|
The current mass of the part, including resources it contains, in kilograms.
Returns zero if the part is massless.
 -}
getPartMassReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartMassReq thisArg = makeCallReq "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]

getPartMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMass thisArg = simpleRequest $ getPartMassReq thisArg

getPartMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMassStreamReq thisArg = makeStreamReq $ getPartMassReq thisArg

getPartMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMassStream thisArg = requestAddStream $ getPartMassStreamReq thisArg 

{-|
Whether the part is <a href="http://wiki.kerbalspaceprogram.com/wiki/Massless_part">massless.
 -}
getPartMasslessReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Bool)
getPartMasslessReq thisArg = makeCallReq "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]

getPartMassless :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartMassless thisArg = simpleRequest $ getPartMasslessReq thisArg

getPartMasslessStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartMasslessStreamReq thisArg = makeStreamReq $ getPartMasslessReq thisArg

getPartMasslessStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartMasslessStream thisArg = requestAddStream $ getPartMasslessStreamReq thisArg 

{-|
Maximum temperature that the skin of the part can survive, in Kelvin.
 -}
getPartMaxSkinTemperatureReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartMaxSkinTemperatureReq thisArg = makeCallReq "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]

getPartMaxSkinTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMaxSkinTemperature thisArg = simpleRequest $ getPartMaxSkinTemperatureReq thisArg

getPartMaxSkinTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMaxSkinTemperatureStreamReq thisArg = makeStreamReq $ getPartMaxSkinTemperatureReq thisArg

getPartMaxSkinTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMaxSkinTemperatureStream thisArg = requestAddStream $ getPartMaxSkinTemperatureStreamReq thisArg 

{-|
Maximum temperature that the part can survive, in Kelvin.
 -}
getPartMaxTemperatureReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartMaxTemperatureReq thisArg = makeCallReq "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]

getPartMaxTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMaxTemperature thisArg = simpleRequest $ getPartMaxTemperatureReq thisArg

getPartMaxTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMaxTemperatureStreamReq thisArg = makeStreamReq $ getPartMaxTemperatureReq thisArg

getPartMaxTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMaxTemperatureStream thisArg = requestAddStream $ getPartMaxTemperatureStreamReq thisArg 

{-|
The modules for this part.
 -}
getPartModulesReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq ([KRPCHS.SpaceCenter.Module])
getPartModulesReq thisArg = makeCallReq "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]

getPartModules :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Module])
getPartModules thisArg = simpleRequest $ getPartModulesReq thisArg

getPartModulesStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Module])
getPartModulesStreamReq thisArg = makeStreamReq $ getPartModulesReq thisArg

getPartModulesStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Module]))
getPartModulesStream thisArg = requestAddStream $ getPartModulesStreamReq thisArg 

{-|
The moment of inertia of the part inkg.m^2around its center of mass
in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 -}
getPartMomentOfInertiaReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq ((Double, Double, Double))
getPartMomentOfInertiaReq thisArg = makeCallReq "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]

getPartMomentOfInertia :: KRPCHS.SpaceCenter.Part -> RPCContext ((Double, Double, Double))
getPartMomentOfInertia thisArg = simpleRequest $ getPartMomentOfInertiaReq thisArg

getPartMomentOfInertiaStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ((Double, Double, Double))
getPartMomentOfInertiaStreamReq thisArg = makeStreamReq $ getPartMomentOfInertiaReq thisArg

getPartMomentOfInertiaStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ((Double, Double, Double)))
getPartMomentOfInertiaStream thisArg = requestAddStream $ getPartMomentOfInertiaStreamReq thisArg 

{-|
Internal name of the part, as used in
<a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation">part cfg files.
For example "Mark1-2Pod".
 -}
getPartNameReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Data.Text.Text)
getPartNameReq thisArg = makeCallReq "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]

getPartName :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartName thisArg = simpleRequest $ getPartNameReq thisArg

getPartNameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartNameStreamReq thisArg = makeStreamReq $ getPartNameReq thisArg

getPartNameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartNameStream thisArg = requestAddStream $ getPartNameStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Parachute" /> if the part is a parachute, otherwisenull.
 -}
getPartParachuteReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Parachute)
getPartParachuteReq thisArg = makeCallReq "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]

getPartParachute :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Parachute)
getPartParachute thisArg = simpleRequest $ getPartParachuteReq thisArg

getPartParachuteStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Parachute)
getPartParachuteStreamReq thisArg = makeStreamReq $ getPartParachuteReq thisArg

getPartParachuteStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Parachute))
getPartParachuteStream thisArg = requestAddStream $ getPartParachuteStreamReq thisArg 

{-|
The parts parent. Returnsnullif the part does not have a parent.
This, in combination with <see cref="M:SpaceCenter.Part.Children" />, can be used to traverse the vessels parts tree.
 -}
getPartParentReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getPartParentReq thisArg = makeCallReq "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]

getPartParent :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartParent thisArg = simpleRequest $ getPartParentReq thisArg

getPartParentStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartParentStreamReq thisArg = makeStreamReq $ getPartParentReq thisArg

getPartParentStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartParentStream thisArg = requestAddStream $ getPartParentStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.RCS" /> if the part is an RCS block/thruster, otherwisenull.
 -}
getPartRCSReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.RCS)
getPartRCSReq thisArg = makeCallReq "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]

getPartRCS :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.RCS)
getPartRCS thisArg = simpleRequest $ getPartRCSReq thisArg

getPartRCSStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.RCS)
getPartRCSStreamReq thisArg = makeStreamReq $ getPartRCSReq thisArg

getPartRCSStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.RCS))
getPartRCSStream thisArg = requestAddStream $ getPartRCSStreamReq thisArg 

{-|
Whether the part is radially attached to its parent, i.e. on the side of its parent.
If the part has no parent, returnsfalse.
 -}
getPartRadiallyAttachedReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Bool)
getPartRadiallyAttachedReq thisArg = makeCallReq "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]

getPartRadiallyAttached :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartRadiallyAttached thisArg = simpleRequest $ getPartRadiallyAttachedReq thisArg

getPartRadiallyAttachedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartRadiallyAttachedStreamReq thisArg = makeStreamReq $ getPartRadiallyAttachedReq thisArg

getPartRadiallyAttachedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartRadiallyAttachedStream thisArg = requestAddStream $ getPartRadiallyAttachedStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Radiator" /> if the part is a radiator, otherwisenull.
 -}
getPartRadiatorReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Radiator)
getPartRadiatorReq thisArg = makeCallReq "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]

getPartRadiator :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Radiator)
getPartRadiator thisArg = simpleRequest $ getPartRadiatorReq thisArg

getPartRadiatorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Radiator)
getPartRadiatorStreamReq thisArg = makeStreamReq $ getPartRadiatorReq thisArg

getPartRadiatorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Radiator))
getPartRadiatorStream thisArg = requestAddStream $ getPartRadiatorStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.ReactionWheel" /> if the part is a reaction wheel, otherwisenull.
 -}
getPartReactionWheelReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheelReq thisArg = makeCallReq "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]

getPartReactionWheel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheel thisArg = simpleRequest $ getPartReactionWheelReq thisArg

getPartReactionWheelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheelStreamReq thisArg = makeStreamReq $ getPartReactionWheelReq thisArg

getPartReactionWheelStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReactionWheel))
getPartReactionWheelStream thisArg = requestAddStream $ getPartReactionWheelStreamReq thisArg 

{-|
The reference frame that is fixed relative to this part, and centered on a fixed position within the part, defined by the parts model.
<list type="bullet">The origin is at the position of the part, as returned by <see cref="M:SpaceCenter.Part.Position" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartReferenceFrameReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]

getPartReferenceFrame :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrame thisArg = simpleRequest $ getPartReferenceFrameReq thisArg

getPartReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrameStreamReq thisArg = makeStreamReq $ getPartReferenceFrameReq thisArg

getPartReferenceFrameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartReferenceFrameStream thisArg = requestAddStream $ getPartReferenceFrameStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.ResourceConverter" /> if the part is a resource converter, otherwisenull.
 -}
getPartResourceConverterReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverterReq thisArg = makeCallReq "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]

getPartResourceConverter :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverter thisArg = simpleRequest $ getPartResourceConverterReq thisArg

getPartResourceConverterStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverterStreamReq thisArg = makeStreamReq $ getPartResourceConverterReq thisArg

getPartResourceConverterStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceConverter))
getPartResourceConverterStream thisArg = requestAddStream $ getPartResourceConverterStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.ResourceHarvester" /> if the part is a resource harvester, otherwisenull.
 -}
getPartResourceHarvesterReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvesterReq thisArg = makeCallReq "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]

getPartResourceHarvester :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvester thisArg = simpleRequest $ getPartResourceHarvesterReq thisArg

getPartResourceHarvesterStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvesterStreamReq thisArg = makeStreamReq $ getPartResourceHarvesterReq thisArg

getPartResourceHarvesterStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvester))
getPartResourceHarvesterStream thisArg = requestAddStream $ getPartResourceHarvesterStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Resources" /> object for the part.
 -}
getPartResourcesReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Resources)
getPartResourcesReq thisArg = makeCallReq "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]

getPartResources :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Resources)
getPartResources thisArg = simpleRequest $ getPartResourcesReq thisArg

getPartResourcesStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
getPartResourcesStreamReq thisArg = makeStreamReq $ getPartResourcesReq thisArg

getPartResourcesStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
getPartResourcesStream thisArg = requestAddStream $ getPartResourcesStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Sensor" /> if the part is a sensor, otherwisenull.
 -}
getPartSensorReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Sensor)
getPartSensorReq thisArg = makeCallReq "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]

getPartSensor :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Sensor)
getPartSensor thisArg = simpleRequest $ getPartSensorReq thisArg

getPartSensorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Sensor)
getPartSensorStreamReq thisArg = makeStreamReq $ getPartSensorReq thisArg

getPartSensorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Sensor))
getPartSensorStream thisArg = requestAddStream $ getPartSensorStreamReq thisArg 

{-|
Whether the part is shielded from the exterior of the vessel, for example by a fairing.
 -}
getPartShieldedReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Bool)
getPartShieldedReq thisArg = makeCallReq "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]

getPartShielded :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartShielded thisArg = simpleRequest $ getPartShieldedReq thisArg

getPartShieldedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartShieldedStreamReq thisArg = makeStreamReq $ getPartShieldedReq thisArg

getPartShieldedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartShieldedStream thisArg = requestAddStream $ getPartShieldedStreamReq thisArg 

{-|
Temperature of the skin of the part, in Kelvin.
 -}
getPartSkinTemperatureReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartSkinTemperatureReq thisArg = makeCallReq "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]

getPartSkinTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartSkinTemperature thisArg = simpleRequest $ getPartSkinTemperatureReq thisArg

getPartSkinTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartSkinTemperatureStreamReq thisArg = makeStreamReq $ getPartSkinTemperatureReq thisArg

getPartSkinTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartSkinTemperatureStream thisArg = requestAddStream $ getPartSkinTemperatureStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.SolarPanel" /> if the part is a solar panel, otherwisenull.
 -}
getPartSolarPanelReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanelReq thisArg = makeCallReq "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]

getPartSolarPanel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanel thisArg = simpleRequest $ getPartSolarPanelReq thisArg

getPartSolarPanelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanelStreamReq thisArg = makeStreamReq $ getPartSolarPanelReq thisArg

getPartSolarPanelStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SolarPanel))
getPartSolarPanelStream thisArg = requestAddStream $ getPartSolarPanelStreamReq thisArg 

{-|
The stage in which this part will be activated. Returns -1 if the part is not activated by staging.
 -}
getPartStageReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Data.Int.Int32)
getPartStageReq thisArg = makeCallReq "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]

getPartStage :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Int.Int32)
getPartStage thisArg = simpleRequest $ getPartStageReq thisArg

getPartStageStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Int.Int32)
getPartStageStreamReq thisArg = makeStreamReq $ getPartStageReq thisArg

getPartStageStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Int.Int32))
getPartStageStream thisArg = requestAddStream $ getPartStageStreamReq thisArg 

{-|
The name tag for the part. Can be set to a custom string using the in-game user interface.This requires either the <a href="http://github.com/krpc/NameTag/releases/latest">NameTagor
<a href="http://forum.kerbalspaceprogram.com/index.php?/topic/61827-/">kOSmods to be installed.
 -}
getPartTagReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Data.Text.Text)
getPartTagReq thisArg = makeCallReq "SpaceCenter" "Part_get_Tag" [makeArgument 0 thisArg]

getPartTag :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartTag thisArg = simpleRequest $ getPartTagReq thisArg

getPartTagStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartTagStreamReq thisArg = makeStreamReq $ getPartTagReq thisArg

getPartTagStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartTagStream thisArg = requestAddStream $ getPartTagStreamReq thisArg 

{-|
Temperature of the part, in Kelvin.
 -}
getPartTemperatureReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Double)
getPartTemperatureReq thisArg = makeCallReq "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]

getPartTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartTemperature thisArg = simpleRequest $ getPartTemperatureReq thisArg

getPartTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartTemperatureStreamReq thisArg = makeStreamReq $ getPartTemperatureReq thisArg

getPartTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartTemperatureStream thisArg = requestAddStream $ getPartTemperatureStreamReq thisArg 

{-|
The rate at which heat energy is conducting into or out of the part via contact with other parts.
Measured in energy per unit time, or power, in Watts.
A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConductionFluxReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalConductionFluxReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]

getPartThermalConductionFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalConductionFlux thisArg = simpleRequest $ getPartThermalConductionFluxReq thisArg

getPartThermalConductionFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalConductionFluxStreamReq thisArg = makeStreamReq $ getPartThermalConductionFluxReq thisArg

getPartThermalConductionFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalConductionFluxStream thisArg = requestAddStream $ getPartThermalConductionFluxStreamReq thisArg 

{-|
The rate at which heat energy is convecting into or out of the part from the surrounding atmosphere.
Measured in energy per unit time, or power, in Watts.
A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConvectionFluxReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalConvectionFluxReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]

getPartThermalConvectionFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalConvectionFlux thisArg = simpleRequest $ getPartThermalConvectionFluxReq thisArg

getPartThermalConvectionFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalConvectionFluxStreamReq thisArg = makeStreamReq $ getPartThermalConvectionFluxReq thisArg

getPartThermalConvectionFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalConvectionFluxStream thisArg = requestAddStream $ getPartThermalConvectionFluxStreamReq thisArg 

{-|
The rate at which heat energy is begin generated by the part.
For example, some engines generate heat by combusting fuel.
Measured in energy per unit time, or power, in Watts.
A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalInternalFluxReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalInternalFluxReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalInternalFlux" [makeArgument 0 thisArg]

getPartThermalInternalFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalInternalFlux thisArg = simpleRequest $ getPartThermalInternalFluxReq thisArg

getPartThermalInternalFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalInternalFluxStreamReq thisArg = makeStreamReq $ getPartThermalInternalFluxReq thisArg

getPartThermalInternalFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalInternalFluxStream thisArg = requestAddStream $ getPartThermalInternalFluxStreamReq thisArg 

{-|
A measure of how much energy it takes to increase the internal temperature of the part, in Joules per Kelvin.
 -}
getPartThermalMassReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalMassReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]

getPartThermalMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalMass thisArg = simpleRequest $ getPartThermalMassReq thisArg

getPartThermalMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalMassStreamReq thisArg = makeStreamReq $ getPartThermalMassReq thisArg

getPartThermalMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalMassStream thisArg = requestAddStream $ getPartThermalMassStreamReq thisArg 

{-|
The rate at which heat energy is radiating into or out of the part from the surrounding environment.
Measured in energy per unit time, or power, in Watts.
A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalRadiationFluxReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalRadiationFluxReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]

getPartThermalRadiationFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalRadiationFlux thisArg = simpleRequest $ getPartThermalRadiationFluxReq thisArg

getPartThermalRadiationFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalRadiationFluxStreamReq thisArg = makeStreamReq $ getPartThermalRadiationFluxReq thisArg

getPartThermalRadiationFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalRadiationFluxStream thisArg = requestAddStream $ getPartThermalRadiationFluxStreamReq thisArg 

{-|
A measure of how much energy it takes to increase the temperature of the resources contained in the part, in Joules per Kelvin.
 -}
getPartThermalResourceMassReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalResourceMassReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]

getPartThermalResourceMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalResourceMass thisArg = simpleRequest $ getPartThermalResourceMassReq thisArg

getPartThermalResourceMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalResourceMassStreamReq thisArg = makeStreamReq $ getPartThermalResourceMassReq thisArg

getPartThermalResourceMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalResourceMassStream thisArg = requestAddStream $ getPartThermalResourceMassStreamReq thisArg 

{-|
A measure of how much energy it takes to increase the skin temperature of the part, in Joules per Kelvin.
 -}
getPartThermalSkinMassReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalSkinMassReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]

getPartThermalSkinMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalSkinMass thisArg = simpleRequest $ getPartThermalSkinMassReq thisArg

getPartThermalSkinMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalSkinMassStreamReq thisArg = makeStreamReq $ getPartThermalSkinMassReq thisArg

getPartThermalSkinMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinMassStream thisArg = requestAddStream $ getPartThermalSkinMassStreamReq thisArg 

{-|
The rate at which heat energy is transferring between the part's skin and its internals.
Measured in energy per unit time, or power, in Watts.
A positive value means the part's internals are gaining heat energy,
and negative means its skin is gaining heat energy.
 -}
getPartThermalSkinToInternalFluxReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Float)
getPartThermalSkinToInternalFluxReq thisArg = makeCallReq "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [makeArgument 0 thisArg]

getPartThermalSkinToInternalFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalSkinToInternalFlux thisArg = simpleRequest $ getPartThermalSkinToInternalFluxReq thisArg

getPartThermalSkinToInternalFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalSkinToInternalFluxStreamReq thisArg = makeStreamReq $ getPartThermalSkinToInternalFluxReq thisArg

getPartThermalSkinToInternalFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinToInternalFluxStream thisArg = requestAddStream $ getPartThermalSkinToInternalFluxStreamReq thisArg 

{-|
Title of the part, as shown when the part is right clicked in-game. For example "Mk1-2 Command Pod".
 -}
getPartTitleReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (Data.Text.Text)
getPartTitleReq thisArg = makeCallReq "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]

getPartTitle :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartTitle thisArg = simpleRequest $ getPartTitleReq thisArg

getPartTitleStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartTitleStreamReq thisArg = makeStreamReq $ getPartTitleReq thisArg

getPartTitleStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartTitleStream thisArg = requestAddStream $ getPartTitleStreamReq thisArg 

{-|
The vessel that contains this part.
 -}
getPartVesselReq :: KRPCHS.SpaceCenter.Part -> KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getPartVesselReq thisArg = makeCallReq "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]

getPartVessel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getPartVessel thisArg = simpleRequest $ getPartVesselReq thisArg

getPartVesselStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getPartVesselStreamReq thisArg = makeStreamReq $ getPartVesselReq thisArg

getPartVesselStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getPartVesselStream thisArg = requestAddStream $ getPartVesselStreamReq thisArg 

{-|
The name tag for the part. Can be set to a custom string using the in-game user interface.This requires either the <a href="http://github.com/krpc/NameTag/releases/latest">NameTagor
<a href="http://forum.kerbalspaceprogram.com/index.php?/topic/61827-/">kOSmods to be installed.
 -}
setPartTagReq :: KRPCHS.SpaceCenter.Part -> Data.Text.Text -> KRPCCallReq ()
setPartTagReq thisArg valueArg = makeCallReq "SpaceCenter" "Part_set_Tag" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPartTag :: KRPCHS.SpaceCenter.Part -> Data.Text.Text -> RPCContext ()
setPartTag thisArg valueArg = simpleRequest $ setPartTagReq thisArg valueArg 

{-|
A list of all parts that are decoupled in the given <paramref name="stage" />.<param name="stage">
 -}
partsInDecoupleStageReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStageReq thisArg stageArg = makeCallReq "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]

partsInDecoupleStage :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStage thisArg stageArg = simpleRequest $ partsInDecoupleStageReq thisArg stageArg

partsInDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStageStreamReq thisArg stageArg = makeStreamReq $ partsInDecoupleStageReq thisArg stageArg

partsInDecoupleStageStream :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInDecoupleStageStream thisArg stageArg = requestAddStream $ partsInDecoupleStageStreamReq thisArg stageArg 

{-|
A list of all parts that are activated in the given <paramref name="stage" />.<param name="stage">
 -}
partsInStageReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
partsInStageReq thisArg stageArg = makeCallReq "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]

partsInStage :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsInStage thisArg stageArg = simpleRequest $ partsInStageReq thisArg stageArg

partsInStageStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsInStageStreamReq thisArg stageArg = makeStreamReq $ partsInStageReq thisArg stageArg

partsInStageStream :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInStageStream thisArg stageArg = requestAddStream $ partsInStageStreamReq thisArg stageArg 

{-|
A list of modules (combined across all parts in the vessel) whose
<see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsModulesWithNameReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCCallReq ([KRPCHS.SpaceCenter.Module])
partsModulesWithNameReq thisArg moduleNameArg = makeCallReq "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]

partsModulesWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Module])
partsModulesWithName thisArg moduleNameArg = simpleRequest $ partsModulesWithNameReq thisArg moduleNameArg

partsModulesWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Module])
partsModulesWithNameStreamReq thisArg moduleNameArg = makeStreamReq $ partsModulesWithNameReq thisArg moduleNameArg

partsModulesWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Module]))
partsModulesWithNameStream thisArg moduleNameArg = requestAddStream $ partsModulesWithNameStreamReq thisArg moduleNameArg 

{-|
A list of all parts that contain a <see cref="T:SpaceCenter.Module" /> whose
<see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsWithModuleReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
partsWithModuleReq thisArg moduleNameArg = makeCallReq "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]

partsWithModule :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithModule thisArg moduleNameArg = simpleRequest $ partsWithModuleReq thisArg moduleNameArg

partsWithModuleStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithModuleStreamReq thisArg moduleNameArg = makeStreamReq $ partsWithModuleReq thisArg moduleNameArg

partsWithModuleStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithModuleStream thisArg moduleNameArg = requestAddStream $ partsWithModuleStreamReq thisArg moduleNameArg 

{-|
A list of parts whose <see cref="M:SpaceCenter.Part.Name" /> is <paramref name="name" />.<param name="name">
 -}
partsWithNameReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
partsWithNameReq thisArg nameArg = makeCallReq "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]

partsWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithName thisArg nameArg = simpleRequest $ partsWithNameReq thisArg nameArg

partsWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithNameStreamReq thisArg nameArg = makeStreamReq $ partsWithNameReq thisArg nameArg

partsWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithNameStream thisArg nameArg = requestAddStream $ partsWithNameStreamReq thisArg nameArg 

{-|
A list of all parts whose <see cref="M:SpaceCenter.Part.Tag" /> is <paramref name="tag" />.<param name="tag">
 -}
partsWithTagReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
partsWithTagReq thisArg tagArg = makeCallReq "SpaceCenter" "Parts_WithTag" [makeArgument 0 thisArg, makeArgument 1 tagArg]

partsWithTag :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithTag thisArg tagArg = simpleRequest $ partsWithTagReq thisArg tagArg

partsWithTagStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithTagStreamReq thisArg tagArg = makeStreamReq $ partsWithTagReq thisArg tagArg

partsWithTagStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithTagStream thisArg tagArg = requestAddStream $ partsWithTagStreamReq thisArg tagArg 

{-|
A list of all parts whose <see cref="M:SpaceCenter.Part.Title" /> is <paramref name="title" />.<param name="title">
 -}
partsWithTitleReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
partsWithTitleReq thisArg titleArg = makeCallReq "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]

partsWithTitle :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithTitle thisArg titleArg = simpleRequest $ partsWithTitleReq thisArg titleArg

partsWithTitleStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithTitleStreamReq thisArg titleArg = makeStreamReq $ partsWithTitleReq thisArg titleArg

partsWithTitleStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithTitleStream thisArg titleArg = requestAddStream $ partsWithTitleStreamReq thisArg titleArg 

{-|
A list of all of the vessels parts.
 -}
getPartsAllReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Part])
getPartsAllReq thisArg = makeCallReq "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]

getPartsAll :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartsAll thisArg = simpleRequest $ getPartsAllReq thisArg

getPartsAllStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartsAllStreamReq thisArg = makeStreamReq $ getPartsAllReq thisArg

getPartsAllStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartsAllStream thisArg = requestAddStream $ getPartsAllStreamReq thisArg 

{-|
A list of all cargo bays in the vessel.
 -}
getPartsCargoBaysReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBaysReq thisArg = makeCallReq "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]

getPartsCargoBays :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBays thisArg = simpleRequest $ getPartsCargoBaysReq thisArg

getPartsCargoBaysStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBaysStreamReq thisArg = makeStreamReq $ getPartsCargoBaysReq thisArg

getPartsCargoBaysStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.CargoBay]))
getPartsCargoBaysStream thisArg = requestAddStream $ getPartsCargoBaysStreamReq thisArg 

{-|
A list of all control surfaces in the vessel.
 -}
getPartsControlSurfacesReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfacesReq thisArg = makeCallReq "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]

getPartsControlSurfaces :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfaces thisArg = simpleRequest $ getPartsControlSurfacesReq thisArg

getPartsControlSurfacesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfacesStreamReq thisArg = makeStreamReq $ getPartsControlSurfacesReq thisArg

getPartsControlSurfacesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ControlSurface]))
getPartsControlSurfacesStream thisArg = requestAddStream $ getPartsControlSurfacesStreamReq thisArg 

{-|
The part from which the vessel is controlled.
 -}
getPartsControllingReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getPartsControllingReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]

getPartsControlling :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartsControlling thisArg = simpleRequest $ getPartsControllingReq thisArg

getPartsControllingStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartsControllingStreamReq thisArg = makeStreamReq $ getPartsControllingReq thisArg

getPartsControllingStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsControllingStream thisArg = requestAddStream $ getPartsControllingStreamReq thisArg 

{-|
A list of all decouplers in the vessel.
 -}
getPartsDecouplersReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplersReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]

getPartsDecouplers :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplers thisArg = simpleRequest $ getPartsDecouplersReq thisArg

getPartsDecouplersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplersStreamReq thisArg = makeStreamReq $ getPartsDecouplersReq thisArg

getPartsDecouplersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Decoupler]))
getPartsDecouplersStream thisArg = requestAddStream $ getPartsDecouplersStreamReq thisArg 

{-|
A list of all docking ports in the vessel.
 -}
getPartsDockingPortsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPortsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]

getPartsDockingPorts :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPorts thisArg = simpleRequest $ getPartsDockingPortsReq thisArg

getPartsDockingPortsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPortsStreamReq thisArg = makeStreamReq $ getPartsDockingPortsReq thisArg

getPartsDockingPortsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.DockingPort]))
getPartsDockingPortsStream thisArg = requestAddStream $ getPartsDockingPortsStreamReq thisArg 

{-|
A list of all engines in the vessel.This includes any part that generates thrust. This covers many different types of engine,
including liquid fuel rockets, solid rocket boosters, jet engines and RCS thrusters.
 -}
getPartsEnginesReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Engine])
getPartsEnginesReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]

getPartsEngines :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Engine])
getPartsEngines thisArg = simpleRequest $ getPartsEnginesReq thisArg

getPartsEnginesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Engine])
getPartsEnginesStreamReq thisArg = makeStreamReq $ getPartsEnginesReq thisArg

getPartsEnginesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Engine]))
getPartsEnginesStream thisArg = requestAddStream $ getPartsEnginesStreamReq thisArg 

{-|
A list of all science experiments in the vessel.
 -}
getPartsExperimentsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Experiment])
getPartsExperimentsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Experiments" [makeArgument 0 thisArg]

getPartsExperiments :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Experiment])
getPartsExperiments thisArg = simpleRequest $ getPartsExperimentsReq thisArg

getPartsExperimentsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Experiment])
getPartsExperimentsStreamReq thisArg = makeStreamReq $ getPartsExperimentsReq thisArg

getPartsExperimentsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Experiment]))
getPartsExperimentsStream thisArg = requestAddStream $ getPartsExperimentsStreamReq thisArg 

{-|
A list of all fairings in the vessel.
 -}
getPartsFairingsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Fairing])
getPartsFairingsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]

getPartsFairings :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Fairing])
getPartsFairings thisArg = simpleRequest $ getPartsFairingsReq thisArg

getPartsFairingsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Fairing])
getPartsFairingsStreamReq thisArg = makeStreamReq $ getPartsFairingsReq thisArg

getPartsFairingsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Fairing]))
getPartsFairingsStream thisArg = requestAddStream $ getPartsFairingsStreamReq thisArg 

{-|
A list of all intakes in the vessel.
 -}
getPartsIntakesReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Intake])
getPartsIntakesReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]

getPartsIntakes :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Intake])
getPartsIntakes thisArg = simpleRequest $ getPartsIntakesReq thisArg

getPartsIntakesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Intake])
getPartsIntakesStreamReq thisArg = makeStreamReq $ getPartsIntakesReq thisArg

getPartsIntakesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Intake]))
getPartsIntakesStream thisArg = requestAddStream $ getPartsIntakesStreamReq thisArg 

{-|
A list of all landing gear attached to the vessel.
 -}
getPartsLandingGearReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.LandingGear])
getPartsLandingGearReq thisArg = makeCallReq "SpaceCenter" "Parts_get_LandingGear" [makeArgument 0 thisArg]

getPartsLandingGear :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LandingGear])
getPartsLandingGear thisArg = simpleRequest $ getPartsLandingGearReq thisArg

getPartsLandingGearStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LandingGear])
getPartsLandingGearStreamReq thisArg = makeStreamReq $ getPartsLandingGearReq thisArg

getPartsLandingGearStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LandingGear]))
getPartsLandingGearStream thisArg = requestAddStream $ getPartsLandingGearStreamReq thisArg 

{-|
A list of all landing legs attached to the vessel.
 -}
getPartsLandingLegsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.LandingLeg])
getPartsLandingLegsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_LandingLegs" [makeArgument 0 thisArg]

getPartsLandingLegs :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LandingLeg])
getPartsLandingLegs thisArg = simpleRequest $ getPartsLandingLegsReq thisArg

getPartsLandingLegsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LandingLeg])
getPartsLandingLegsStreamReq thisArg = makeStreamReq $ getPartsLandingLegsReq thisArg

getPartsLandingLegsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LandingLeg]))
getPartsLandingLegsStream thisArg = requestAddStream $ getPartsLandingLegsStreamReq thisArg 

{-|
A list of all launch clamps attached to the vessel.
 -}
getPartsLaunchClampsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClampsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]

getPartsLaunchClamps :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClamps thisArg = simpleRequest $ getPartsLaunchClampsReq thisArg

getPartsLaunchClampsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClampsStreamReq thisArg = makeStreamReq $ getPartsLaunchClampsReq thisArg

getPartsLaunchClampsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LaunchClamp]))
getPartsLaunchClampsStream thisArg = requestAddStream $ getPartsLaunchClampsStreamReq thisArg 

{-|
A list of all lights in the vessel.
 -}
getPartsLightsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Light])
getPartsLightsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]

getPartsLights :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Light])
getPartsLights thisArg = simpleRequest $ getPartsLightsReq thisArg

getPartsLightsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Light])
getPartsLightsStreamReq thisArg = makeStreamReq $ getPartsLightsReq thisArg

getPartsLightsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Light]))
getPartsLightsStream thisArg = requestAddStream $ getPartsLightsStreamReq thisArg 

{-|
A list of all parachutes in the vessel.
 -}
getPartsParachutesReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutesReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]

getPartsParachutes :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutes thisArg = simpleRequest $ getPartsParachutesReq thisArg

getPartsParachutesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutesStreamReq thisArg = makeStreamReq $ getPartsParachutesReq thisArg

getPartsParachutesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Parachute]))
getPartsParachutesStream thisArg = requestAddStream $ getPartsParachutesStreamReq thisArg 

{-|
A list of all RCS blocks/thrusters in the vessel.
 -}
getPartsRCSReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.RCS])
getPartsRCSReq thisArg = makeCallReq "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]

getPartsRCS :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.RCS])
getPartsRCS thisArg = simpleRequest $ getPartsRCSReq thisArg

getPartsRCSStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.RCS])
getPartsRCSStreamReq thisArg = makeStreamReq $ getPartsRCSReq thisArg

getPartsRCSStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.RCS]))
getPartsRCSStream thisArg = requestAddStream $ getPartsRCSStreamReq thisArg 

{-|
A list of all radiators in the vessel.
 -}
getPartsRadiatorsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiatorsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]

getPartsRadiators :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiators thisArg = simpleRequest $ getPartsRadiatorsReq thisArg

getPartsRadiatorsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiatorsStreamReq thisArg = makeStreamReq $ getPartsRadiatorsReq thisArg

getPartsRadiatorsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Radiator]))
getPartsRadiatorsStream thisArg = requestAddStream $ getPartsRadiatorsStreamReq thisArg 

{-|
A list of all reaction wheels in the vessel.
 -}
getPartsReactionWheelsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheelsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]

getPartsReactionWheels :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheels thisArg = simpleRequest $ getPartsReactionWheelsReq thisArg

getPartsReactionWheelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheelsStreamReq thisArg = makeStreamReq $ getPartsReactionWheelsReq thisArg

getPartsReactionWheelsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ReactionWheel]))
getPartsReactionWheelsStream thisArg = requestAddStream $ getPartsReactionWheelsStreamReq thisArg 

{-|
A list of all resource converters in the vessel.
 -}
getPartsResourceConvertersReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConvertersReq thisArg = makeCallReq "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]

getPartsResourceConverters :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConverters thisArg = simpleRequest $ getPartsResourceConvertersReq thisArg

getPartsResourceConvertersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConvertersStreamReq thisArg = makeStreamReq $ getPartsResourceConvertersReq thisArg

getPartsResourceConvertersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ResourceConverter]))
getPartsResourceConvertersStream thisArg = requestAddStream $ getPartsResourceConvertersStreamReq thisArg 

{-|
A list of all resource harvesters in the vessel.
 -}
getPartsResourceHarvestersReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvestersReq thisArg = makeCallReq "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]

getPartsResourceHarvesters :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvesters thisArg = simpleRequest $ getPartsResourceHarvestersReq thisArg

getPartsResourceHarvestersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvestersStreamReq thisArg = makeStreamReq $ getPartsResourceHarvestersReq thisArg

getPartsResourceHarvestersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ResourceHarvester]))
getPartsResourceHarvestersStream thisArg = requestAddStream $ getPartsResourceHarvestersStreamReq thisArg 

{-|
The vessels root part.
 -}
getPartsRootReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getPartsRootReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]

getPartsRoot :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartsRoot thisArg = simpleRequest $ getPartsRootReq thisArg

getPartsRootStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartsRootStreamReq thisArg = makeStreamReq $ getPartsRootReq thisArg

getPartsRootStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsRootStream thisArg = requestAddStream $ getPartsRootStreamReq thisArg 

{-|
A list of all sensors in the vessel.
 -}
getPartsSensorsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.Sensor])
getPartsSensorsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]

getPartsSensors :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Sensor])
getPartsSensors thisArg = simpleRequest $ getPartsSensorsReq thisArg

getPartsSensorsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Sensor])
getPartsSensorsStreamReq thisArg = makeStreamReq $ getPartsSensorsReq thisArg

getPartsSensorsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Sensor]))
getPartsSensorsStream thisArg = requestAddStream $ getPartsSensorsStreamReq thisArg 

{-|
A list of all solar panels in the vessel.
 -}
getPartsSolarPanelsReq :: KRPCHS.SpaceCenter.Parts -> KRPCCallReq ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanelsReq thisArg = makeCallReq "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]

getPartsSolarPanels :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanels thisArg = simpleRequest $ getPartsSolarPanelsReq thisArg

getPartsSolarPanelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanelsStreamReq thisArg = makeStreamReq $ getPartsSolarPanelsReq thisArg

getPartsSolarPanelsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.SolarPanel]))
getPartsSolarPanelsStream thisArg = requestAddStream $ getPartsSolarPanelsStreamReq thisArg 

{-|
The part from which the vessel is controlled.
 -}
setPartsControllingReq :: KRPCHS.SpaceCenter.Parts -> KRPCHS.SpaceCenter.Part -> KRPCCallReq ()
setPartsControllingReq thisArg valueArg = makeCallReq "SpaceCenter" "Parts_set_Controlling" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPartsControlling :: KRPCHS.SpaceCenter.Parts -> KRPCHS.SpaceCenter.Part -> RPCContext ()
setPartsControlling thisArg valueArg = simpleRequest $ setPartsControllingReq thisArg valueArg 

{-|
The reachable resources connected to this propellant.
 -}
getPropellantConnectedResourcesReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq ([KRPCHS.SpaceCenter.Resource])
getPropellantConnectedResourcesReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_ConnectedResources" [makeArgument 0 thisArg]

getPropellantConnectedResources :: KRPCHS.SpaceCenter.Propellant -> RPCContext ([KRPCHS.SpaceCenter.Resource])
getPropellantConnectedResources thisArg = simpleRequest $ getPropellantConnectedResourcesReq thisArg

getPropellantConnectedResourcesStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
getPropellantConnectedResourcesStreamReq thisArg = makeStreamReq $ getPropellantConnectedResourcesReq thisArg

getPropellantConnectedResourcesStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
getPropellantConnectedResourcesStream thisArg = requestAddStream $ getPropellantConnectedResourcesStreamReq thisArg 

{-|
The current amount of propellant.
 -}
getPropellantCurrentAmountReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Double)
getPropellantCurrentAmountReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_CurrentAmount" [makeArgument 0 thisArg]

getPropellantCurrentAmount :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantCurrentAmount thisArg = simpleRequest $ getPropellantCurrentAmountReq thisArg

getPropellantCurrentAmountStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantCurrentAmountStreamReq thisArg = makeStreamReq $ getPropellantCurrentAmountReq thisArg

getPropellantCurrentAmountStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantCurrentAmountStream thisArg = requestAddStream $ getPropellantCurrentAmountStreamReq thisArg 

{-|
The required amount of propellant.
 -}
getPropellantCurrentRequirementReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Double)
getPropellantCurrentRequirementReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_CurrentRequirement" [makeArgument 0 thisArg]

getPropellantCurrentRequirement :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantCurrentRequirement thisArg = simpleRequest $ getPropellantCurrentRequirementReq thisArg

getPropellantCurrentRequirementStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantCurrentRequirementStreamReq thisArg = makeStreamReq $ getPropellantCurrentRequirementReq thisArg

getPropellantCurrentRequirementStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantCurrentRequirementStream thisArg = requestAddStream $ getPropellantCurrentRequirementStreamReq thisArg 

{-|
If this propellant has a stack gauge or not.
 -}
getPropellantDrawStackGaugeReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Bool)
getPropellantDrawStackGaugeReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_DrawStackGauge" [makeArgument 0 thisArg]

getPropellantDrawStackGauge :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantDrawStackGauge thisArg = simpleRequest $ getPropellantDrawStackGaugeReq thisArg

getPropellantDrawStackGaugeStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantDrawStackGaugeStreamReq thisArg = makeStreamReq $ getPropellantDrawStackGaugeReq thisArg

getPropellantDrawStackGaugeStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantDrawStackGaugeStream thisArg = requestAddStream $ getPropellantDrawStackGaugeStreamReq thisArg 

{-|
If this propellant should be ignored when calculating required mass flow given specific impulse.
 -}
getPropellantIgnoreForIspReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Bool)
getPropellantIgnoreForIspReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_IgnoreForIsp" [makeArgument 0 thisArg]

getPropellantIgnoreForIsp :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantIgnoreForIsp thisArg = simpleRequest $ getPropellantIgnoreForIspReq thisArg

getPropellantIgnoreForIspStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIgnoreForIspStreamReq thisArg = makeStreamReq $ getPropellantIgnoreForIspReq thisArg

getPropellantIgnoreForIspStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantIgnoreForIspStream thisArg = requestAddStream $ getPropellantIgnoreForIspStreamReq thisArg 

{-|
If this propellant should be ignored for thrust curve calculations.
 -}
getPropellantIgnoreForThrustCurveReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Bool)
getPropellantIgnoreForThrustCurveReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_IgnoreForThrustCurve" [makeArgument 0 thisArg]

getPropellantIgnoreForThrustCurve :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantIgnoreForThrustCurve thisArg = simpleRequest $ getPropellantIgnoreForThrustCurveReq thisArg

getPropellantIgnoreForThrustCurveStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIgnoreForThrustCurveStreamReq thisArg = makeStreamReq $ getPropellantIgnoreForThrustCurveReq thisArg

getPropellantIgnoreForThrustCurveStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantIgnoreForThrustCurveStream thisArg = requestAddStream $ getPropellantIgnoreForThrustCurveStreamReq thisArg 

{-|
If this propellant is deprived.
 -}
getPropellantIsDeprivedReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Bool)
getPropellantIsDeprivedReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_IsDeprived" [makeArgument 0 thisArg]

getPropellantIsDeprived :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantIsDeprived thisArg = simpleRequest $ getPropellantIsDeprivedReq thisArg

getPropellantIsDeprivedStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIsDeprivedStreamReq thisArg = makeStreamReq $ getPropellantIsDeprivedReq thisArg

getPropellantIsDeprivedStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantIsDeprivedStream thisArg = requestAddStream $ getPropellantIsDeprivedStreamReq thisArg 

{-|
The name of the propellant.
 -}
getPropellantNameReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Data.Text.Text)
getPropellantNameReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_Name" [makeArgument 0 thisArg]

getPropellantName :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Data.Text.Text)
getPropellantName thisArg = simpleRequest $ getPropellantNameReq thisArg

getPropellantNameStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Data.Text.Text)
getPropellantNameStreamReq thisArg = makeStreamReq $ getPropellantNameReq thisArg

getPropellantNameStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Data.Text.Text))
getPropellantNameStream thisArg = requestAddStream $ getPropellantNameStreamReq thisArg 

{-|
The propellant ratio.
 -}
getPropellantRatioReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Float)
getPropellantRatioReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_Ratio" [makeArgument 0 thisArg]

getPropellantRatio :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Float)
getPropellantRatio thisArg = simpleRequest $ getPropellantRatioReq thisArg

getPropellantRatioStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Float)
getPropellantRatioStreamReq thisArg = makeStreamReq $ getPropellantRatioReq thisArg

getPropellantRatioStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Float))
getPropellantRatioStream thisArg = requestAddStream $ getPropellantRatioStreamReq thisArg 

{-|
The total amount of the underlying resource currently reachable given resource flow rules.
 -}
getPropellantTotalResourceAvailableReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Double)
getPropellantTotalResourceAvailableReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_TotalResourceAvailable" [makeArgument 0 thisArg]

getPropellantTotalResourceAvailable :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantTotalResourceAvailable thisArg = simpleRequest $ getPropellantTotalResourceAvailableReq thisArg

getPropellantTotalResourceAvailableStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantTotalResourceAvailableStreamReq thisArg = makeStreamReq $ getPropellantTotalResourceAvailableReq thisArg

getPropellantTotalResourceAvailableStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantTotalResourceAvailableStream thisArg = requestAddStream $ getPropellantTotalResourceAvailableStreamReq thisArg 

{-|
The total vehicle capacity for the underlying propellant resource, restricted by resource flow rules.
 -}
getPropellantTotalResourceCapacityReq :: KRPCHS.SpaceCenter.Propellant -> KRPCCallReq (Double)
getPropellantTotalResourceCapacityReq thisArg = makeCallReq "SpaceCenter" "Propellant_get_TotalResourceCapacity" [makeArgument 0 thisArg]

getPropellantTotalResourceCapacity :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantTotalResourceCapacity thisArg = simpleRequest $ getPropellantTotalResourceCapacityReq thisArg

getPropellantTotalResourceCapacityStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantTotalResourceCapacityStreamReq thisArg = makeStreamReq $ getPropellantTotalResourceCapacityReq thisArg

getPropellantTotalResourceCapacityStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantTotalResourceCapacityStream thisArg = requestAddStream $ getPropellantTotalResourceCapacityStreamReq thisArg 

{-|
Load a quicksave.This is the same as calling <see cref="M:SpaceCenter.Load" /> with the name "quicksave".
 -}
quickloadReq :: KRPCCallReq ()
quickloadReq  = makeCallReq "SpaceCenter" "Quickload" []

quickload :: RPCContext ()
quickload  = simpleRequest $ quickloadReq  

{-|
Save a quicksave.This is the same as calling <see cref="M:SpaceCenter.Save" /> with the name "quicksave".
 -}
quicksaveReq :: KRPCCallReq ()
quicksaveReq  = makeCallReq "SpaceCenter" "Quicksave" []

quicksave :: RPCContext ()
quicksave  = simpleRequest $ quicksaveReq  

{-|
Whether the RCS thrusters are active.
An RCS thruster is inactive if the RCS action group is disabled (<see cref="M:SpaceCenter.Control.RCS" />),
the RCS thruster itself is not enabled (<see cref="M:SpaceCenter.RCS.Enabled" />) or
it is covered by a fairing (<see cref="M:SpaceCenter.Part.Shielded" />).
 -}
getRCSActiveReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSActiveReq thisArg = makeCallReq "SpaceCenter" "RCS_get_Active" [makeArgument 0 thisArg]

getRCSActive :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSActive thisArg = simpleRequest $ getRCSActiveReq thisArg

getRCSActiveStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSActiveStreamReq thisArg = makeStreamReq $ getRCSActiveReq thisArg

getRCSActiveStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSActiveStream thisArg = requestAddStream $ getRCSActiveStreamReq thisArg 

{-|
The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
Returns zero if the RCS is inactive.
 -}
getRCSAvailableTorqueReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq ((Double, Double, Double))
getRCSAvailableTorqueReq thisArg = makeCallReq "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]

getRCSAvailableTorque :: KRPCHS.SpaceCenter.RCS -> RPCContext ((Double, Double, Double))
getRCSAvailableTorque thisArg = simpleRequest $ getRCSAvailableTorqueReq thisArg

getRCSAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ((Double, Double, Double))
getRCSAvailableTorqueStreamReq thisArg = makeStreamReq $ getRCSAvailableTorqueReq thisArg

getRCSAvailableTorqueStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ((Double, Double, Double)))
getRCSAvailableTorqueStream thisArg = requestAddStream $ getRCSAvailableTorqueStreamReq thisArg 

{-|
Whether the RCS thrusters are enabled.
 -}
getRCSEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]

getRCSEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSEnabled thisArg = simpleRequest $ getRCSEnabledReq thisArg

getRCSEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSEnabledStreamReq thisArg = makeStreamReq $ getRCSEnabledReq thisArg

getRCSEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSEnabledStream thisArg = requestAddStream $ getRCSEnabledStreamReq thisArg 

{-|
Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSForwardEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSForwardEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]

getRCSForwardEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSForwardEnabled thisArg = simpleRequest $ getRCSForwardEnabledReq thisArg

getRCSForwardEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSForwardEnabledStreamReq thisArg = makeStreamReq $ getRCSForwardEnabledReq thisArg

getRCSForwardEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSForwardEnabledStream thisArg = requestAddStream $ getRCSForwardEnabledStreamReq thisArg 

{-|
Whether the RCS has fuel available.The RCS thruster must be activated for this property to update correctly.
 -}
getRCSHasFuelReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSHasFuelReq thisArg = makeCallReq "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]

getRCSHasFuel :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSHasFuel thisArg = simpleRequest $ getRCSHasFuelReq thisArg

getRCSHasFuelStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSHasFuelStreamReq thisArg = makeStreamReq $ getRCSHasFuelReq thisArg

getRCSHasFuelStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSHasFuelStream thisArg = requestAddStream $ getRCSHasFuelStreamReq thisArg 

{-|
The specific impulse of the RCS at sea level on Kerbin, in seconds.
 -}
getRCSKerbinSeaLevelSpecificImpulseReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Float)
getRCSKerbinSeaLevelSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]

getRCSKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSKerbinSeaLevelSpecificImpulse thisArg = simpleRequest $ getRCSKerbinSeaLevelSpecificImpulseReq thisArg

getRCSKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSKerbinSeaLevelSpecificImpulseStreamReq thisArg = makeStreamReq $ getRCSKerbinSeaLevelSpecificImpulseReq thisArg

getRCSKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSKerbinSeaLevelSpecificImpulseStream thisArg = requestAddStream $ getRCSKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-|
The maximum amount of thrust that can be produced by the RCS thrusters when active, in Newtons.
 -}
getRCSMaxThrustReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Float)
getRCSMaxThrustReq thisArg = makeCallReq "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]

getRCSMaxThrust :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSMaxThrust thisArg = simpleRequest $ getRCSMaxThrustReq thisArg

getRCSMaxThrustStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSMaxThrustStreamReq thisArg = makeStreamReq $ getRCSMaxThrustReq thisArg

getRCSMaxThrustStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSMaxThrustStream thisArg = requestAddStream $ getRCSMaxThrustStreamReq thisArg 

{-|
The maximum amount of thrust that can be produced by the RCS thrusters when active in a vacuum, in Newtons.
 -}
getRCSMaxVacuumThrustReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Float)
getRCSMaxVacuumThrustReq thisArg = makeCallReq "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]

getRCSMaxVacuumThrust :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSMaxVacuumThrust thisArg = simpleRequest $ getRCSMaxVacuumThrustReq thisArg

getRCSMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSMaxVacuumThrustStreamReq thisArg = makeStreamReq $ getRCSMaxVacuumThrustReq thisArg

getRCSMaxVacuumThrustStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSMaxVacuumThrustStream thisArg = requestAddStream $ getRCSMaxVacuumThrustStreamReq thisArg 

{-|
The part object for this RCS.
 -}
getRCSPartReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getRCSPartReq thisArg = makeCallReq "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]

getRCSPart :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCHS.SpaceCenter.Part)
getRCSPart thisArg = simpleRequest $ getRCSPartReq thisArg

getRCSPartStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getRCSPartStreamReq thisArg = makeStreamReq $ getRCSPartReq thisArg

getRCSPartStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getRCSPartStream thisArg = requestAddStream $ getRCSPartStreamReq thisArg 

{-|
Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSPitchEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSPitchEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]

getRCSPitchEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSPitchEnabled thisArg = simpleRequest $ getRCSPitchEnabledReq thisArg

getRCSPitchEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSPitchEnabledStreamReq thisArg = makeStreamReq $ getRCSPitchEnabledReq thisArg

getRCSPitchEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSPitchEnabledStream thisArg = requestAddStream $ getRCSPitchEnabledStreamReq thisArg 

{-|
The ratios of resources that the RCS consumes. A dictionary mapping resource names
to the ratios at which they are consumed by the RCS.
 -}
getRCSPropellantRatiosReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatiosReq thisArg = makeCallReq "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]

getRCSPropellantRatios :: KRPCHS.SpaceCenter.RCS -> RPCContext (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatios thisArg = simpleRequest $ getRCSPropellantRatiosReq thisArg

getRCSPropellantRatiosStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatiosStreamReq thisArg = makeStreamReq $ getRCSPropellantRatiosReq thisArg

getRCSPropellantRatiosStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getRCSPropellantRatiosStream thisArg = requestAddStream $ getRCSPropellantRatiosStreamReq thisArg 

{-|
The names of resources that the RCS consumes.
 -}
getRCSPropellantsReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq ([Data.Text.Text])
getRCSPropellantsReq thisArg = makeCallReq "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]

getRCSPropellants :: KRPCHS.SpaceCenter.RCS -> RPCContext ([Data.Text.Text])
getRCSPropellants thisArg = simpleRequest $ getRCSPropellantsReq thisArg

getRCSPropellantsStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ([Data.Text.Text])
getRCSPropellantsStreamReq thisArg = makeStreamReq $ getRCSPropellantsReq thisArg

getRCSPropellantsStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ([Data.Text.Text]))
getRCSPropellantsStream thisArg = requestAddStream $ getRCSPropellantsStreamReq thisArg 

{-|
Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRightEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSRightEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]

getRCSRightEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSRightEnabled thisArg = simpleRequest $ getRCSRightEnabledReq thisArg

getRCSRightEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSRightEnabledStreamReq thisArg = makeStreamReq $ getRCSRightEnabledReq thisArg

getRCSRightEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSRightEnabledStream thisArg = requestAddStream $ getRCSRightEnabledStreamReq thisArg 

{-|
Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRollEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSRollEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]

getRCSRollEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSRollEnabled thisArg = simpleRequest $ getRCSRollEnabledReq thisArg

getRCSRollEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSRollEnabledStreamReq thisArg = makeStreamReq $ getRCSRollEnabledReq thisArg

getRCSRollEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSRollEnabledStream thisArg = requestAddStream $ getRCSRollEnabledStreamReq thisArg 

{-|
The current specific impulse of the RCS, in seconds. Returns zero
if the RCS is not active.
 -}
getRCSSpecificImpulseReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Float)
getRCSSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]

getRCSSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSSpecificImpulse thisArg = simpleRequest $ getRCSSpecificImpulseReq thisArg

getRCSSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSSpecificImpulseStreamReq thisArg = makeStreamReq $ getRCSSpecificImpulseReq thisArg

getRCSSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSSpecificImpulseStream thisArg = requestAddStream $ getRCSSpecificImpulseStreamReq thisArg 

{-|
A list of thrusters, one of each nozzel in the RCS part.
 -}
getRCSThrustersReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq ([KRPCHS.SpaceCenter.Thruster])
getRCSThrustersReq thisArg = makeCallReq "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]

getRCSThrusters :: KRPCHS.SpaceCenter.RCS -> RPCContext ([KRPCHS.SpaceCenter.Thruster])
getRCSThrusters thisArg = simpleRequest $ getRCSThrustersReq thisArg

getRCSThrustersStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ([KRPCHS.SpaceCenter.Thruster])
getRCSThrustersStreamReq thisArg = makeStreamReq $ getRCSThrustersReq thisArg

getRCSThrustersStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getRCSThrustersStream thisArg = requestAddStream $ getRCSThrustersStreamReq thisArg 

{-|
Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSUpEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSUpEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]

getRCSUpEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSUpEnabled thisArg = simpleRequest $ getRCSUpEnabledReq thisArg

getRCSUpEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSUpEnabledStreamReq thisArg = makeStreamReq $ getRCSUpEnabledReq thisArg

getRCSUpEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSUpEnabledStream thisArg = requestAddStream $ getRCSUpEnabledStreamReq thisArg 

{-|
The vacuum specific impulse of the RCS, in seconds.
 -}
getRCSVacuumSpecificImpulseReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Float)
getRCSVacuumSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]

getRCSVacuumSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSVacuumSpecificImpulse thisArg = simpleRequest $ getRCSVacuumSpecificImpulseReq thisArg

getRCSVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSVacuumSpecificImpulseStreamReq thisArg = makeStreamReq $ getRCSVacuumSpecificImpulseReq thisArg

getRCSVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSVacuumSpecificImpulseStream thisArg = requestAddStream $ getRCSVacuumSpecificImpulseStreamReq thisArg 

{-|
Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSYawEnabledReq :: KRPCHS.SpaceCenter.RCS -> KRPCCallReq (Bool)
getRCSYawEnabledReq thisArg = makeCallReq "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]

getRCSYawEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSYawEnabled thisArg = simpleRequest $ getRCSYawEnabledReq thisArg

getRCSYawEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSYawEnabledStreamReq thisArg = makeStreamReq $ getRCSYawEnabledReq thisArg

getRCSYawEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSYawEnabledStream thisArg = requestAddStream $ getRCSYawEnabledStreamReq thisArg 

{-|
Whether the RCS thrusters are enabled.
 -}
setRCSEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSEnabled thisArg valueArg = simpleRequest $ setRCSEnabledReq thisArg valueArg 

{-|
Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSForwardEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSForwardEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_ForwardEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSForwardEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSForwardEnabled thisArg valueArg = simpleRequest $ setRCSForwardEnabledReq thisArg valueArg 

{-|
Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSPitchEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSPitchEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSPitchEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSPitchEnabled thisArg valueArg = simpleRequest $ setRCSPitchEnabledReq thisArg valueArg 

{-|
Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRightEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSRightEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_RightEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSRightEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSRightEnabled thisArg valueArg = simpleRequest $ setRCSRightEnabledReq thisArg valueArg 

{-|
Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRollEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSRollEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSRollEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSRollEnabled thisArg valueArg = simpleRequest $ setRCSRollEnabledReq thisArg valueArg 

{-|
Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSUpEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSUpEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_UpEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSUpEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSUpEnabled thisArg valueArg = simpleRequest $ setRCSUpEnabledReq thisArg valueArg 

{-|
Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSYawEnabledReq :: KRPCHS.SpaceCenter.RCS -> Bool -> KRPCCallReq ()
setRCSYawEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "RCS_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRCSYawEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSYawEnabled thisArg valueArg = simpleRequest $ setRCSYawEnabledReq thisArg valueArg 

{-|
Whether the radiator is deployable.
 -}
getRadiatorDeployableReq :: KRPCHS.SpaceCenter.Radiator -> KRPCCallReq (Bool)
getRadiatorDeployableReq thisArg = makeCallReq "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]

getRadiatorDeployable :: KRPCHS.SpaceCenter.Radiator -> RPCContext (Bool)
getRadiatorDeployable thisArg = simpleRequest $ getRadiatorDeployableReq thisArg

getRadiatorDeployableStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (Bool)
getRadiatorDeployableStreamReq thisArg = makeStreamReq $ getRadiatorDeployableReq thisArg

getRadiatorDeployableStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployableStream thisArg = requestAddStream $ getRadiatorDeployableStreamReq thisArg 

{-|
For a deployable radiator,trueif the radiator is extended.
If the radiator is not deployable, this is alwaystrue.
 -}
getRadiatorDeployedReq :: KRPCHS.SpaceCenter.Radiator -> KRPCCallReq (Bool)
getRadiatorDeployedReq thisArg = makeCallReq "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]

getRadiatorDeployed :: KRPCHS.SpaceCenter.Radiator -> RPCContext (Bool)
getRadiatorDeployed thisArg = simpleRequest $ getRadiatorDeployedReq thisArg

getRadiatorDeployedStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (Bool)
getRadiatorDeployedStreamReq thisArg = makeStreamReq $ getRadiatorDeployedReq thisArg

getRadiatorDeployedStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployedStream thisArg = requestAddStream $ getRadiatorDeployedStreamReq thisArg 

{-|
The part object for this radiator.
 -}
getRadiatorPartReq :: KRPCHS.SpaceCenter.Radiator -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getRadiatorPartReq thisArg = makeCallReq "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]

getRadiatorPart :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCHS.SpaceCenter.Part)
getRadiatorPart thisArg = simpleRequest $ getRadiatorPartReq thisArg

getRadiatorPartStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getRadiatorPartStreamReq thisArg = makeStreamReq $ getRadiatorPartReq thisArg

getRadiatorPartStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getRadiatorPartStream thisArg = requestAddStream $ getRadiatorPartStreamReq thisArg 

{-|
The current state of the radiator.A fixed radiator is always <see cref="M:SpaceCenter.RadiatorState.Extended" />.
 -}
getRadiatorStateReq :: KRPCHS.SpaceCenter.Radiator -> KRPCCallReq (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorStateReq thisArg = makeCallReq "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]

getRadiatorState :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorState thisArg = simpleRequest $ getRadiatorStateReq thisArg

getRadiatorStateStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorStateStreamReq thisArg = makeStreamReq $ getRadiatorStateReq thisArg

getRadiatorStateStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.RadiatorState))
getRadiatorStateStream thisArg = requestAddStream $ getRadiatorStateStreamReq thisArg 

{-|
For a deployable radiator,trueif the radiator is extended.
If the radiator is not deployable, this is alwaystrue.
 -}
setRadiatorDeployedReq :: KRPCHS.SpaceCenter.Radiator -> Bool -> KRPCCallReq ()
setRadiatorDeployedReq thisArg valueArg = makeCallReq "SpaceCenter" "Radiator_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRadiatorDeployed :: KRPCHS.SpaceCenter.Radiator -> Bool -> RPCContext ()
setRadiatorDeployed thisArg valueArg = simpleRequest $ setRadiatorDeployedReq thisArg valueArg 

{-|
Whether the reaction wheel is active.
 -}
getReactionWheelActiveReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCCallReq (Bool)
getReactionWheelActiveReq thisArg = makeCallReq "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]

getReactionWheelActive :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (Bool)
getReactionWheelActive thisArg = simpleRequest $ getReactionWheelActiveReq thisArg

getReactionWheelActiveStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (Bool)
getReactionWheelActiveStreamReq thisArg = makeStreamReq $ getReactionWheelActiveReq thisArg

getReactionWheelActiveStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelActiveStream thisArg = requestAddStream $ getReactionWheelActiveStreamReq thisArg 

{-|
The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
Returns zero if the reaction wheel is inactive or broken.
 -}
getReactionWheelAvailableTorqueReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCCallReq ((Double, Double, Double))
getReactionWheelAvailableTorqueReq thisArg = makeCallReq "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]

getReactionWheelAvailableTorque :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelAvailableTorque thisArg = simpleRequest $ getReactionWheelAvailableTorqueReq thisArg

getReactionWheelAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq ((Double, Double, Double))
getReactionWheelAvailableTorqueStreamReq thisArg = makeStreamReq $ getReactionWheelAvailableTorqueReq thisArg

getReactionWheelAvailableTorqueStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelAvailableTorqueStream thisArg = requestAddStream $ getReactionWheelAvailableTorqueStreamReq thisArg 

{-|
Whether the reaction wheel is broken.
 -}
getReactionWheelBrokenReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCCallReq (Bool)
getReactionWheelBrokenReq thisArg = makeCallReq "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]

getReactionWheelBroken :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (Bool)
getReactionWheelBroken thisArg = simpleRequest $ getReactionWheelBrokenReq thisArg

getReactionWheelBrokenStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (Bool)
getReactionWheelBrokenStreamReq thisArg = makeStreamReq $ getReactionWheelBrokenReq thisArg

getReactionWheelBrokenStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelBrokenStream thisArg = requestAddStream $ getReactionWheelBrokenStreamReq thisArg 

{-|
The maximum torque the reaction wheel can provide, is it active,
in the pitch, roll and yaw axes of the vessel, in Newton meters.
These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getReactionWheelMaxTorqueReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCCallReq ((Double, Double, Double))
getReactionWheelMaxTorqueReq thisArg = makeCallReq "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]

getReactionWheelMaxTorque :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelMaxTorque thisArg = simpleRequest $ getReactionWheelMaxTorqueReq thisArg

getReactionWheelMaxTorqueStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq ((Double, Double, Double))
getReactionWheelMaxTorqueStreamReq thisArg = makeStreamReq $ getReactionWheelMaxTorqueReq thisArg

getReactionWheelMaxTorqueStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelMaxTorqueStream thisArg = requestAddStream $ getReactionWheelMaxTorqueStreamReq thisArg 

{-|
The part object for this reaction wheel.
 -}
getReactionWheelPartReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getReactionWheelPartReq thisArg = makeCallReq "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]

getReactionWheelPart :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCHS.SpaceCenter.Part)
getReactionWheelPart thisArg = simpleRequest $ getReactionWheelPartReq thisArg

getReactionWheelPartStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getReactionWheelPartStreamReq thisArg = makeStreamReq $ getReactionWheelPartReq thisArg

getReactionWheelPartStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getReactionWheelPartStream thisArg = requestAddStream $ getReactionWheelPartStreamReq thisArg 

{-|
Whether the reaction wheel is active.
 -}
setReactionWheelActiveReq :: KRPCHS.SpaceCenter.ReactionWheel -> Bool -> KRPCCallReq ()
setReactionWheelActiveReq thisArg valueArg = makeCallReq "SpaceCenter" "ReactionWheel_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setReactionWheelActive :: KRPCHS.SpaceCenter.ReactionWheel -> Bool -> RPCContext ()
setReactionWheelActive thisArg valueArg = simpleRequest $ setReactionWheelActiveReq thisArg valueArg 

{-|
True if the specified converter is active.<param name="index">Index of the converter.
 -}
resourceConverterActiveReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq (Bool)
resourceConverterActiveReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterActive :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Bool)
resourceConverterActive thisArg indexArg = simpleRequest $ resourceConverterActiveReq thisArg indexArg

resourceConverterActiveStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Bool)
resourceConverterActiveStreamReq thisArg indexArg = makeStreamReq $ resourceConverterActiveReq thisArg indexArg

resourceConverterActiveStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Bool))
resourceConverterActiveStream thisArg indexArg = requestAddStream $ resourceConverterActiveStreamReq thisArg indexArg 

{-|
List of the names of resources consumed by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterInputsReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq ([Data.Text.Text])
resourceConverterInputsReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterInputs :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ([Data.Text.Text])
resourceConverterInputs thisArg indexArg = simpleRequest $ resourceConverterInputsReq thisArg indexArg

resourceConverterInputsStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq ([Data.Text.Text])
resourceConverterInputsStreamReq thisArg indexArg = makeStreamReq $ resourceConverterInputsReq thisArg indexArg

resourceConverterInputsStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream ([Data.Text.Text]))
resourceConverterInputsStream thisArg indexArg = requestAddStream $ resourceConverterInputsStreamReq thisArg indexArg 

{-|
The name of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterNameReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq (Data.Text.Text)
resourceConverterNameReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterName :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Data.Text.Text)
resourceConverterName thisArg indexArg = simpleRequest $ resourceConverterNameReq thisArg indexArg

resourceConverterNameStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Data.Text.Text)
resourceConverterNameStreamReq thisArg indexArg = makeStreamReq $ resourceConverterNameReq thisArg indexArg

resourceConverterNameStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Data.Text.Text))
resourceConverterNameStream thisArg indexArg = requestAddStream $ resourceConverterNameStreamReq thisArg indexArg 

{-|
List of the names of resources produced by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterOutputsReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq ([Data.Text.Text])
resourceConverterOutputsReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterOutputs :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ([Data.Text.Text])
resourceConverterOutputs thisArg indexArg = simpleRequest $ resourceConverterOutputsReq thisArg indexArg

resourceConverterOutputsStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq ([Data.Text.Text])
resourceConverterOutputsStreamReq thisArg indexArg = makeStreamReq $ resourceConverterOutputsReq thisArg indexArg

resourceConverterOutputsStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream ([Data.Text.Text]))
resourceConverterOutputsStream thisArg indexArg = requestAddStream $ resourceConverterOutputsStreamReq thisArg indexArg 

{-|
Start the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStartReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq ()
resourceConverterStartReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_Start" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterStart :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ()
resourceConverterStart thisArg indexArg = simpleRequest $ resourceConverterStartReq thisArg indexArg 

{-|
The state of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStateReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterStateReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterState :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterState thisArg indexArg = simpleRequest $ resourceConverterStateReq thisArg indexArg

resourceConverterStateStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterStateStreamReq thisArg indexArg = makeStreamReq $ resourceConverterStateReq thisArg indexArg

resourceConverterStateStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceConverterState))
resourceConverterStateStream thisArg indexArg = requestAddStream $ resourceConverterStateStreamReq thisArg indexArg 

{-|
Status information for the specified converter.
This is the full status message shown in the in-game UI.<param name="index">Index of the converter.
 -}
resourceConverterStatusInfoReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq (Data.Text.Text)
resourceConverterStatusInfoReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterStatusInfo :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Data.Text.Text)
resourceConverterStatusInfo thisArg indexArg = simpleRequest $ resourceConverterStatusInfoReq thisArg indexArg

resourceConverterStatusInfoStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Data.Text.Text)
resourceConverterStatusInfoStreamReq thisArg indexArg = makeStreamReq $ resourceConverterStatusInfoReq thisArg indexArg

resourceConverterStatusInfoStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Data.Text.Text))
resourceConverterStatusInfoStream thisArg indexArg = requestAddStream $ resourceConverterStatusInfoStreamReq thisArg indexArg 

{-|
Stop the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStopReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCCallReq ()
resourceConverterStopReq thisArg indexArg = makeCallReq "SpaceCenter" "ResourceConverter_Stop" [makeArgument 0 thisArg, makeArgument 1 indexArg]

resourceConverterStop :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ()
resourceConverterStop thisArg indexArg = simpleRequest $ resourceConverterStopReq thisArg indexArg 

{-|
The number of converters in the part.
 -}
getResourceConverterCountReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCCallReq (Data.Int.Int32)
getResourceConverterCountReq thisArg = makeCallReq "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]

getResourceConverterCount :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (Data.Int.Int32)
getResourceConverterCount thisArg = simpleRequest $ getResourceConverterCountReq thisArg

getResourceConverterCountStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCStreamReq (Data.Int.Int32)
getResourceConverterCountStreamReq thisArg = makeStreamReq $ getResourceConverterCountReq thisArg

getResourceConverterCountStream :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCStream (Data.Int.Int32))
getResourceConverterCountStream thisArg = requestAddStream $ getResourceConverterCountStreamReq thisArg 

{-|
The part object for this converter.
 -}
getResourceConverterPartReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getResourceConverterPartReq thisArg = makeCallReq "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]

getResourceConverterPart :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourceConverterPart thisArg = simpleRequest $ getResourceConverterPartReq thisArg

getResourceConverterPartStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourceConverterPartStreamReq thisArg = makeStreamReq $ getResourceConverterPartReq thisArg

getResourceConverterPartStream :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceConverterPartStream thisArg = requestAddStream $ getResourceConverterPartStreamReq thisArg 

{-|
Whether the harvester is actively drilling.
 -}
getResourceHarvesterActiveReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (Bool)
getResourceHarvesterActiveReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]

getResourceHarvesterActive :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterActive thisArg = simpleRequest $ getResourceHarvesterActiveReq thisArg

getResourceHarvesterActiveStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Bool)
getResourceHarvesterActiveStreamReq thisArg = makeStreamReq $ getResourceHarvesterActiveReq thisArg

getResourceHarvesterActiveStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterActiveStream thisArg = requestAddStream $ getResourceHarvesterActiveStreamReq thisArg 

{-|
The core temperature of the drill, in Kelvin.
 -}
getResourceHarvesterCoreTemperatureReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (Float)
getResourceHarvesterCoreTemperatureReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]

getResourceHarvesterCoreTemperature :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterCoreTemperature thisArg = simpleRequest $ getResourceHarvesterCoreTemperatureReq thisArg

getResourceHarvesterCoreTemperatureStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterCoreTemperatureStreamReq thisArg = makeStreamReq $ getResourceHarvesterCoreTemperatureReq thisArg

getResourceHarvesterCoreTemperatureStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterCoreTemperatureStream thisArg = requestAddStream $ getResourceHarvesterCoreTemperatureStreamReq thisArg 

{-|
Whether the harvester is deployed.
 -}
getResourceHarvesterDeployedReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (Bool)
getResourceHarvesterDeployedReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]

getResourceHarvesterDeployed :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterDeployed thisArg = simpleRequest $ getResourceHarvesterDeployedReq thisArg

getResourceHarvesterDeployedStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Bool)
getResourceHarvesterDeployedStreamReq thisArg = makeStreamReq $ getResourceHarvesterDeployedReq thisArg

getResourceHarvesterDeployedStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterDeployedStream thisArg = requestAddStream $ getResourceHarvesterDeployedStreamReq thisArg 

{-|
The rate at which the drill is extracting ore, in units per second.
 -}
getResourceHarvesterExtractionRateReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (Float)
getResourceHarvesterExtractionRateReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]

getResourceHarvesterExtractionRate :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterExtractionRate thisArg = simpleRequest $ getResourceHarvesterExtractionRateReq thisArg

getResourceHarvesterExtractionRateStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterExtractionRateStreamReq thisArg = makeStreamReq $ getResourceHarvesterExtractionRateReq thisArg

getResourceHarvesterExtractionRateStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterExtractionRateStream thisArg = requestAddStream $ getResourceHarvesterExtractionRateStreamReq thisArg 

{-|
The core temperature at which the drill will operate with peak efficiency, in Kelvin.
 -}
getResourceHarvesterOptimumCoreTemperatureReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (Float)
getResourceHarvesterOptimumCoreTemperatureReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]

getResourceHarvesterOptimumCoreTemperature :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterOptimumCoreTemperature thisArg = simpleRequest $ getResourceHarvesterOptimumCoreTemperatureReq thisArg

getResourceHarvesterOptimumCoreTemperatureStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterOptimumCoreTemperatureStreamReq thisArg = makeStreamReq $ getResourceHarvesterOptimumCoreTemperatureReq thisArg

getResourceHarvesterOptimumCoreTemperatureStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterOptimumCoreTemperatureStream thisArg = requestAddStream $ getResourceHarvesterOptimumCoreTemperatureStreamReq thisArg 

{-|
The part object for this harvester.
 -}
getResourceHarvesterPartReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPartReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]

getResourceHarvesterPart :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPart thisArg = simpleRequest $ getResourceHarvesterPartReq thisArg

getResourceHarvesterPartStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPartStreamReq thisArg = makeStreamReq $ getResourceHarvesterPartReq thisArg

getResourceHarvesterPartStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceHarvesterPartStream thisArg = requestAddStream $ getResourceHarvesterPartStreamReq thisArg 

{-|
The state of the harvester.
 -}
getResourceHarvesterStateReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterStateReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]

getResourceHarvesterState :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterState thisArg = simpleRequest $ getResourceHarvesterStateReq thisArg

getResourceHarvesterStateStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterStateStreamReq thisArg = makeStreamReq $ getResourceHarvesterStateReq thisArg

getResourceHarvesterStateStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvesterState))
getResourceHarvesterStateStream thisArg = requestAddStream $ getResourceHarvesterStateStreamReq thisArg 

{-|
The thermal efficiency of the drill, as a percentage of its maximum.
 -}
getResourceHarvesterThermalEfficiencyReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCCallReq (Float)
getResourceHarvesterThermalEfficiencyReq thisArg = makeCallReq "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]

getResourceHarvesterThermalEfficiency :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterThermalEfficiency thisArg = simpleRequest $ getResourceHarvesterThermalEfficiencyReq thisArg

getResourceHarvesterThermalEfficiencyStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterThermalEfficiencyStreamReq thisArg = makeStreamReq $ getResourceHarvesterThermalEfficiencyReq thisArg

getResourceHarvesterThermalEfficiencyStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterThermalEfficiencyStream thisArg = requestAddStream $ getResourceHarvesterThermalEfficiencyStreamReq thisArg 

{-|
Whether the harvester is actively drilling.
 -}
setResourceHarvesterActiveReq :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> KRPCCallReq ()
setResourceHarvesterActiveReq thisArg valueArg = makeCallReq "SpaceCenter" "ResourceHarvester_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setResourceHarvesterActive :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> RPCContext ()
setResourceHarvesterActive thisArg valueArg = simpleRequest $ setResourceHarvesterActiveReq thisArg valueArg 

{-|
Whether the harvester is deployed.
 -}
setResourceHarvesterDeployedReq :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> KRPCCallReq ()
setResourceHarvesterDeployedReq thisArg valueArg = makeCallReq "SpaceCenter" "ResourceHarvester_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setResourceHarvesterDeployed :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> RPCContext ()
setResourceHarvesterDeployed thisArg valueArg = simpleRequest $ setResourceHarvesterDeployedReq thisArg valueArg 

{-|
The amount of the resource that has been transferred.
 -}
getResourceTransferAmountReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCCallReq (Float)
getResourceTransferAmountReq thisArg = makeCallReq "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]

getResourceTransferAmount :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (Float)
getResourceTransferAmount thisArg = simpleRequest $ getResourceTransferAmountReq thisArg

getResourceTransferAmountStreamReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCStreamReq (Float)
getResourceTransferAmountStreamReq thisArg = makeStreamReq $ getResourceTransferAmountReq thisArg

getResourceTransferAmountStream :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (KRPCStream (Float))
getResourceTransferAmountStream thisArg = requestAddStream $ getResourceTransferAmountStreamReq thisArg 

{-|
Whether the transfer has completed.
 -}
getResourceTransferCompleteReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCCallReq (Bool)
getResourceTransferCompleteReq thisArg = makeCallReq "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]

getResourceTransferComplete :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (Bool)
getResourceTransferComplete thisArg = simpleRequest $ getResourceTransferCompleteReq thisArg

getResourceTransferCompleteStreamReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCStreamReq (Bool)
getResourceTransferCompleteStreamReq thisArg = makeStreamReq $ getResourceTransferCompleteReq thisArg

getResourceTransferCompleteStream :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (KRPCStream (Bool))
getResourceTransferCompleteStream thisArg = requestAddStream $ getResourceTransferCompleteStreamReq thisArg 

{-|
Start transferring a resource transfer between a pair of parts. The transfer will move at most
<paramref name="maxAmount" /> units of the resource, depending on how much of the resource is
available in the source part and how much storage is available in the destination part.
Use <see cref="M:SpaceCenter.ResourceTransfer.Complete" /> to check if the transfer is complete.
Use <see cref="M:SpaceCenter.ResourceTransfer.Amount" /> to see how much of the resource has been transferred.<param name="fromPart">The part to transfer to.<param name="toPart">The part to transfer from.<param name="resource">The name of the resource to transfer.<param name="maxAmount">The maximum amount of resource to transfer.
 -}
resourceTransferStaticStartReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStaticStartReq fromPartArg toPartArg resourceArg maxAmountArg = makeCallReq "SpaceCenter" "ResourceTransfer_static_Start" [makeArgument 0 fromPartArg, makeArgument 1 toPartArg, makeArgument 2 resourceArg, makeArgument 3 maxAmountArg]

resourceTransferStaticStart :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> RPCContext (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStaticStart fromPartArg toPartArg resourceArg maxAmountArg = simpleRequest $ resourceTransferStaticStartReq fromPartArg toPartArg resourceArg maxAmountArg

resourceTransferStaticStartStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStaticStartStreamReq fromPartArg toPartArg resourceArg maxAmountArg = makeStreamReq $ resourceTransferStaticStartReq fromPartArg toPartArg resourceArg maxAmountArg

resourceTransferStaticStartStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceTransfer))
resourceTransferStaticStartStream fromPartArg toPartArg resourceArg maxAmountArg = requestAddStream $ resourceTransferStaticStartStreamReq fromPartArg toPartArg resourceArg maxAmountArg 

{-|
The amount of the resource that is currently stored in the part.
 -}
getResourceAmountReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (Float)
getResourceAmountReq thisArg = makeCallReq "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]

getResourceAmount :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceAmount thisArg = simpleRequest $ getResourceAmountReq thisArg

getResourceAmountStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceAmountStreamReq thisArg = makeStreamReq $ getResourceAmountReq thisArg

getResourceAmountStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceAmountStream thisArg = requestAddStream $ getResourceAmountStreamReq thisArg 

{-|
The density of the resource, inkg/l.
 -}
getResourceDensityReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (Float)
getResourceDensityReq thisArg = makeCallReq "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]

getResourceDensity :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceDensity thisArg = simpleRequest $ getResourceDensityReq thisArg

getResourceDensityStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceDensityStreamReq thisArg = makeStreamReq $ getResourceDensityReq thisArg

getResourceDensityStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceDensityStream thisArg = requestAddStream $ getResourceDensityStreamReq thisArg 

{-|
Whether use of this resource is enabled.
 -}
getResourceEnabledReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (Bool)
getResourceEnabledReq thisArg = makeCallReq "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]

getResourceEnabled :: KRPCHS.SpaceCenter.Resource -> RPCContext (Bool)
getResourceEnabled thisArg = simpleRequest $ getResourceEnabledReq thisArg

getResourceEnabledStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Bool)
getResourceEnabledStreamReq thisArg = makeStreamReq $ getResourceEnabledReq thisArg

getResourceEnabledStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Bool))
getResourceEnabledStream thisArg = requestAddStream $ getResourceEnabledStreamReq thisArg 

{-|
The flow mode of the resource.
 -}
getResourceFlowModeReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowModeReq thisArg = makeCallReq "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]

getResourceFlowMode :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowMode thisArg = simpleRequest $ getResourceFlowModeReq thisArg

getResourceFlowModeStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowModeStreamReq thisArg = makeStreamReq $ getResourceFlowModeReq thisArg

getResourceFlowModeStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
getResourceFlowModeStream thisArg = requestAddStream $ getResourceFlowModeStreamReq thisArg 

{-|
The total amount of the resource that can be stored in the part.
 -}
getResourceMaxReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (Float)
getResourceMaxReq thisArg = makeCallReq "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]

getResourceMax :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceMax thisArg = simpleRequest $ getResourceMaxReq thisArg

getResourceMaxStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceMaxStreamReq thisArg = makeStreamReq $ getResourceMaxReq thisArg

getResourceMaxStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceMaxStream thisArg = requestAddStream $ getResourceMaxStreamReq thisArg 

{-|
The name of the resource.
 -}
getResourceNameReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (Data.Text.Text)
getResourceNameReq thisArg = makeCallReq "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]

getResourceName :: KRPCHS.SpaceCenter.Resource -> RPCContext (Data.Text.Text)
getResourceName thisArg = simpleRequest $ getResourceNameReq thisArg

getResourceNameStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Data.Text.Text)
getResourceNameStreamReq thisArg = makeStreamReq $ getResourceNameReq thisArg

getResourceNameStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Data.Text.Text))
getResourceNameStream thisArg = requestAddStream $ getResourceNameStreamReq thisArg 

{-|
The part containing the resource.
 -}
getResourcePartReq :: KRPCHS.SpaceCenter.Resource -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getResourcePartReq thisArg = makeCallReq "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]

getResourcePart :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourcePart thisArg = simpleRequest $ getResourcePartReq thisArg

getResourcePartStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourcePartStreamReq thisArg = makeStreamReq $ getResourcePartReq thisArg

getResourcePartStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourcePartStream thisArg = requestAddStream $ getResourcePartStreamReq thisArg 

{-|
Whether use of this resource is enabled.
 -}
setResourceEnabledReq :: KRPCHS.SpaceCenter.Resource -> Bool -> KRPCCallReq ()
setResourceEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "Resource_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setResourceEnabled :: KRPCHS.SpaceCenter.Resource -> Bool -> RPCContext ()
setResourceEnabled thisArg valueArg = simpleRequest $ setResourceEnabledReq thisArg valueArg 

{-|
Returns the amount of a resource that is currently stored.<param name="name">The name of the resource.
 -}
resourcesAmountReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCCallReq (Float)
resourcesAmountReq thisArg nameArg = makeCallReq "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]

resourcesAmount :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Float)
resourcesAmount thisArg nameArg = simpleRequest $ resourcesAmountReq thisArg nameArg

resourcesAmountStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Float)
resourcesAmountStreamReq thisArg nameArg = makeStreamReq $ resourcesAmountReq thisArg nameArg

resourcesAmountStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesAmountStream thisArg nameArg = requestAddStream $ resourcesAmountStreamReq thisArg nameArg 

{-|
Check whether the named resource can be stored.<param name="name">The name of the resource.
 -}
resourcesHasResourceReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCCallReq (Bool)
resourcesHasResourceReq thisArg nameArg = makeCallReq "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]

resourcesHasResource :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Bool)
resourcesHasResource thisArg nameArg = simpleRequest $ resourcesHasResourceReq thisArg nameArg

resourcesHasResourceStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Bool)
resourcesHasResourceStreamReq thisArg nameArg = makeStreamReq $ resourcesHasResourceReq thisArg nameArg

resourcesHasResourceStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
resourcesHasResourceStream thisArg nameArg = requestAddStream $ resourcesHasResourceStreamReq thisArg nameArg 

{-|
Returns the amount of a resource that can be stored.<param name="name">The name of the resource.
 -}
resourcesMaxReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCCallReq (Float)
resourcesMaxReq thisArg nameArg = makeCallReq "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]

resourcesMax :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Float)
resourcesMax thisArg nameArg = simpleRequest $ resourcesMaxReq thisArg nameArg

resourcesMaxStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Float)
resourcesMaxStreamReq thisArg nameArg = makeStreamReq $ resourcesMaxReq thisArg nameArg

resourcesMaxStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesMaxStream thisArg nameArg = requestAddStream $ resourcesMaxStreamReq thisArg nameArg 

{-|
All the individual resources with the given name that can be stored.
 -}
resourcesWithResourceReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCCallReq ([KRPCHS.SpaceCenter.Resource])
resourcesWithResourceReq thisArg nameArg = makeCallReq "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]

resourcesWithResource :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Resource])
resourcesWithResource thisArg nameArg = simpleRequest $ resourcesWithResourceReq thisArg nameArg

resourcesWithResourceStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
resourcesWithResourceStreamReq thisArg nameArg = makeStreamReq $ resourcesWithResourceReq thisArg nameArg

resourcesWithResourceStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
resourcesWithResourceStream thisArg nameArg = requestAddStream $ resourcesWithResourceStreamReq thisArg nameArg 

{-|
All the individual resources that can be stored.
 -}
getResourcesAllReq :: KRPCHS.SpaceCenter.Resources -> KRPCCallReq ([KRPCHS.SpaceCenter.Resource])
getResourcesAllReq thisArg = makeCallReq "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]

getResourcesAll :: KRPCHS.SpaceCenter.Resources -> RPCContext ([KRPCHS.SpaceCenter.Resource])
getResourcesAll thisArg = simpleRequest $ getResourcesAllReq thisArg

getResourcesAllStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
getResourcesAllStreamReq thisArg = makeStreamReq $ getResourcesAllReq thisArg

getResourcesAllStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
getResourcesAllStream thisArg = requestAddStream $ getResourcesAllStreamReq thisArg 

{-|
Whether use of all the resources are enabled.This is true if all of the resources are enabled. If any of the resources are not enabled, this is false.
 -}
getResourcesEnabledReq :: KRPCHS.SpaceCenter.Resources -> KRPCCallReq (Bool)
getResourcesEnabledReq thisArg = makeCallReq "SpaceCenter" "Resources_get_Enabled" [makeArgument 0 thisArg]

getResourcesEnabled :: KRPCHS.SpaceCenter.Resources -> RPCContext (Bool)
getResourcesEnabled thisArg = simpleRequest $ getResourcesEnabledReq thisArg

getResourcesEnabledStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq (Bool)
getResourcesEnabledStreamReq thisArg = makeStreamReq $ getResourcesEnabledReq thisArg

getResourcesEnabledStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream (Bool))
getResourcesEnabledStream thisArg = requestAddStream $ getResourcesEnabledStreamReq thisArg 

{-|
A list of resource names that can be stored.
 -}
getResourcesNamesReq :: KRPCHS.SpaceCenter.Resources -> KRPCCallReq ([Data.Text.Text])
getResourcesNamesReq thisArg = makeCallReq "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]

getResourcesNames :: KRPCHS.SpaceCenter.Resources -> RPCContext ([Data.Text.Text])
getResourcesNames thisArg = simpleRequest $ getResourcesNamesReq thisArg

getResourcesNamesStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq ([Data.Text.Text])
getResourcesNamesStreamReq thisArg = makeStreamReq $ getResourcesNamesReq thisArg

getResourcesNamesStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream ([Data.Text.Text]))
getResourcesNamesStream thisArg = requestAddStream $ getResourcesNamesStreamReq thisArg 

{-|
Whether use of all the resources are enabled.This is true if all of the resources are enabled. If any of the resources are not enabled, this is false.
 -}
setResourcesEnabledReq :: KRPCHS.SpaceCenter.Resources -> Bool -> KRPCCallReq ()
setResourcesEnabledReq thisArg valueArg = makeCallReq "SpaceCenter" "Resources_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setResourcesEnabled :: KRPCHS.SpaceCenter.Resources -> Bool -> RPCContext ()
setResourcesEnabled thisArg valueArg = simpleRequest $ setResourcesEnabledReq thisArg valueArg 

{-|
Returns the density of a resource, in kg/l.<param name="name">The name of the resource.
 -}
resourcesStaticDensityReq :: Data.Text.Text -> KRPCCallReq (Float)
resourcesStaticDensityReq nameArg = makeCallReq "SpaceCenter" "Resources_static_Density" [makeArgument 0 nameArg]

resourcesStaticDensity :: Data.Text.Text -> RPCContext (Float)
resourcesStaticDensity nameArg = simpleRequest $ resourcesStaticDensityReq nameArg

resourcesStaticDensityStreamReq :: Data.Text.Text -> KRPCStreamReq (Float)
resourcesStaticDensityStreamReq nameArg = makeStreamReq $ resourcesStaticDensityReq nameArg

resourcesStaticDensityStream :: Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesStaticDensityStream nameArg = requestAddStream $ resourcesStaticDensityStreamReq nameArg 

{-|
Returns the flow mode of a resource.<param name="name">The name of the resource.
 -}
resourcesStaticFlowModeReq :: Data.Text.Text -> KRPCCallReq (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesStaticFlowModeReq nameArg = makeCallReq "SpaceCenter" "Resources_static_FlowMode" [makeArgument 0 nameArg]

resourcesStaticFlowMode :: Data.Text.Text -> RPCContext (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesStaticFlowMode nameArg = simpleRequest $ resourcesStaticFlowModeReq nameArg

resourcesStaticFlowModeStreamReq :: Data.Text.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesStaticFlowModeStreamReq nameArg = makeStreamReq $ resourcesStaticFlowModeReq nameArg

resourcesStaticFlowModeStream :: Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
resourcesStaticFlowModeStream nameArg = requestAddStream $ resourcesStaticFlowModeStreamReq nameArg 

{-|
Save the game with a given name.
This will create a save file calledname.sfsin the folder of the current save game.
 -}
saveReq :: Data.Text.Text -> KRPCCallReq ()
saveReq nameArg = makeCallReq "SpaceCenter" "Save" [makeArgument 0 nameArg]

save :: Data.Text.Text -> RPCContext ()
save nameArg = simpleRequest $ saveReq nameArg 

{-|
Data amount.
 -}
getScienceDataDataAmountReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCCallReq (Float)
getScienceDataDataAmountReq thisArg = makeCallReq "SpaceCenter" "ScienceData_get_DataAmount" [makeArgument 0 thisArg]

getScienceDataDataAmount :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (Float)
getScienceDataDataAmount thisArg = simpleRequest $ getScienceDataDataAmountReq thisArg

getScienceDataDataAmountStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataDataAmountStreamReq thisArg = makeStreamReq $ getScienceDataDataAmountReq thisArg

getScienceDataDataAmountStream :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (KRPCStream (Float))
getScienceDataDataAmountStream thisArg = requestAddStream $ getScienceDataDataAmountStreamReq thisArg 

{-|
Science value.
 -}
getScienceDataScienceValueReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCCallReq (Float)
getScienceDataScienceValueReq thisArg = makeCallReq "SpaceCenter" "ScienceData_get_ScienceValue" [makeArgument 0 thisArg]

getScienceDataScienceValue :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (Float)
getScienceDataScienceValue thisArg = simpleRequest $ getScienceDataScienceValueReq thisArg

getScienceDataScienceValueStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataScienceValueStreamReq thisArg = makeStreamReq $ getScienceDataScienceValueReq thisArg

getScienceDataScienceValueStream :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (KRPCStream (Float))
getScienceDataScienceValueStream thisArg = requestAddStream $ getScienceDataScienceValueStreamReq thisArg 

{-|
Transmit value.
 -}
getScienceDataTransmitValueReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCCallReq (Float)
getScienceDataTransmitValueReq thisArg = makeCallReq "SpaceCenter" "ScienceData_get_TransmitValue" [makeArgument 0 thisArg]

getScienceDataTransmitValue :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (Float)
getScienceDataTransmitValue thisArg = simpleRequest $ getScienceDataTransmitValueReq thisArg

getScienceDataTransmitValueStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataTransmitValueStreamReq thisArg = makeStreamReq $ getScienceDataTransmitValueReq thisArg

getScienceDataTransmitValueStream :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (KRPCStream (Float))
getScienceDataTransmitValueStream thisArg = requestAddStream $ getScienceDataTransmitValueStreamReq thisArg 

{-|
Multiply science value by this to determine data amount in mits.
 -}
getScienceSubjectDataScaleReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Float)
getScienceSubjectDataScaleReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_DataScale" [makeArgument 0 thisArg]

getScienceSubjectDataScale :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Float)
getScienceSubjectDataScale thisArg = simpleRequest $ getScienceSubjectDataScaleReq thisArg

getScienceSubjectDataScaleStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectDataScaleStreamReq thisArg = makeStreamReq $ getScienceSubjectDataScaleReq thisArg

getScienceSubjectDataScaleStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Float))
getScienceSubjectDataScaleStream thisArg = requestAddStream $ getScienceSubjectDataScaleStreamReq thisArg 

{-|
Whether the experiment has been completed.
 -}
getScienceSubjectIsCompleteReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Bool)
getScienceSubjectIsCompleteReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_IsComplete" [makeArgument 0 thisArg]

getScienceSubjectIsComplete :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Bool)
getScienceSubjectIsComplete thisArg = simpleRequest $ getScienceSubjectIsCompleteReq thisArg

getScienceSubjectIsCompleteStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Bool)
getScienceSubjectIsCompleteStreamReq thisArg = makeStreamReq $ getScienceSubjectIsCompleteReq thisArg

getScienceSubjectIsCompleteStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Bool))
getScienceSubjectIsCompleteStream thisArg = requestAddStream $ getScienceSubjectIsCompleteStreamReq thisArg 

{-|
Amount of science already earned from this subject, not updated until after transmission/recovery.
 -}
getScienceSubjectScienceReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Float)
getScienceSubjectScienceReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_Science" [makeArgument 0 thisArg]

getScienceSubjectScience :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Float)
getScienceSubjectScience thisArg = simpleRequest $ getScienceSubjectScienceReq thisArg

getScienceSubjectScienceStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectScienceStreamReq thisArg = makeStreamReq $ getScienceSubjectScienceReq thisArg

getScienceSubjectScienceStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Float))
getScienceSubjectScienceStream thisArg = requestAddStream $ getScienceSubjectScienceStreamReq thisArg 

{-|
Total science allowable for this subject.
 -}
getScienceSubjectScienceCapReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Float)
getScienceSubjectScienceCapReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_ScienceCap" [makeArgument 0 thisArg]

getScienceSubjectScienceCap :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Float)
getScienceSubjectScienceCap thisArg = simpleRequest $ getScienceSubjectScienceCapReq thisArg

getScienceSubjectScienceCapStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectScienceCapStreamReq thisArg = makeStreamReq $ getScienceSubjectScienceCapReq thisArg

getScienceSubjectScienceCapStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Float))
getScienceSubjectScienceCapStream thisArg = requestAddStream $ getScienceSubjectScienceCapStreamReq thisArg 

{-|
Diminishing value multiplier for decreasing the science value returned from repeated experiments.
 -}
getScienceSubjectScientificValueReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Float)
getScienceSubjectScientificValueReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_ScientificValue" [makeArgument 0 thisArg]

getScienceSubjectScientificValue :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Float)
getScienceSubjectScientificValue thisArg = simpleRequest $ getScienceSubjectScientificValueReq thisArg

getScienceSubjectScientificValueStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectScientificValueStreamReq thisArg = makeStreamReq $ getScienceSubjectScientificValueReq thisArg

getScienceSubjectScientificValueStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Float))
getScienceSubjectScientificValueStream thisArg = requestAddStream $ getScienceSubjectScientificValueStreamReq thisArg 

{-|
Multiplier for specific Celestial Body/Experiment Situation combination.
 -}
getScienceSubjectSubjectValueReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Float)
getScienceSubjectSubjectValueReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_SubjectValue" [makeArgument 0 thisArg]

getScienceSubjectSubjectValue :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Float)
getScienceSubjectSubjectValue thisArg = simpleRequest $ getScienceSubjectSubjectValueReq thisArg

getScienceSubjectSubjectValueStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Float)
getScienceSubjectSubjectValueStreamReq thisArg = makeStreamReq $ getScienceSubjectSubjectValueReq thisArg

getScienceSubjectSubjectValueStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Float))
getScienceSubjectSubjectValueStream thisArg = requestAddStream $ getScienceSubjectSubjectValueStreamReq thisArg 

{-|
Title of science subject, displayed in science archives
 -}
getScienceSubjectTitleReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCCallReq (Data.Text.Text)
getScienceSubjectTitleReq thisArg = makeCallReq "SpaceCenter" "ScienceSubject_get_Title" [makeArgument 0 thisArg]

getScienceSubjectTitle :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (Data.Text.Text)
getScienceSubjectTitle thisArg = simpleRequest $ getScienceSubjectTitleReq thisArg

getScienceSubjectTitleStreamReq :: KRPCHS.SpaceCenter.ScienceSubject -> KRPCStreamReq (Data.Text.Text)
getScienceSubjectTitleStreamReq thisArg = makeStreamReq $ getScienceSubjectTitleReq thisArg

getScienceSubjectTitleStream :: KRPCHS.SpaceCenter.ScienceSubject -> RPCContext (KRPCStream (Data.Text.Text))
getScienceSubjectTitleStream thisArg = requestAddStream $ getScienceSubjectTitleStreamReq thisArg 

{-|
Whether the sensor is active.
 -}
getSensorActiveReq :: KRPCHS.SpaceCenter.Sensor -> KRPCCallReq (Bool)
getSensorActiveReq thisArg = makeCallReq "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]

getSensorActive :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Bool)
getSensorActive thisArg = simpleRequest $ getSensorActiveReq thisArg

getSensorActiveStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Bool)
getSensorActiveStreamReq thisArg = makeStreamReq $ getSensorActiveReq thisArg

getSensorActiveStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Bool))
getSensorActiveStream thisArg = requestAddStream $ getSensorActiveStreamReq thisArg 

{-|
The part object for this sensor.
 -}
getSensorPartReq :: KRPCHS.SpaceCenter.Sensor -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getSensorPartReq thisArg = makeCallReq "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]

getSensorPart :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCHS.SpaceCenter.Part)
getSensorPart thisArg = simpleRequest $ getSensorPartReq thisArg

getSensorPartStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getSensorPartStreamReq thisArg = makeStreamReq $ getSensorPartReq thisArg

getSensorPartStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getSensorPartStream thisArg = requestAddStream $ getSensorPartStreamReq thisArg 

{-|
The current power usage of the sensor, in units of charge per second.
 -}
getSensorPowerUsageReq :: KRPCHS.SpaceCenter.Sensor -> KRPCCallReq (Float)
getSensorPowerUsageReq thisArg = makeCallReq "SpaceCenter" "Sensor_get_PowerUsage" [makeArgument 0 thisArg]

getSensorPowerUsage :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Float)
getSensorPowerUsage thisArg = simpleRequest $ getSensorPowerUsageReq thisArg

getSensorPowerUsageStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Float)
getSensorPowerUsageStreamReq thisArg = makeStreamReq $ getSensorPowerUsageReq thisArg

getSensorPowerUsageStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Float))
getSensorPowerUsageStream thisArg = requestAddStream $ getSensorPowerUsageStreamReq thisArg 

{-|
The current value of the sensor.
 -}
getSensorValueReq :: KRPCHS.SpaceCenter.Sensor -> KRPCCallReq (Data.Text.Text)
getSensorValueReq thisArg = makeCallReq "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]

getSensorValue :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Data.Text.Text)
getSensorValue thisArg = simpleRequest $ getSensorValueReq thisArg

getSensorValueStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Data.Text.Text)
getSensorValueStreamReq thisArg = makeStreamReq $ getSensorValueReq thisArg

getSensorValueStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Data.Text.Text))
getSensorValueStream thisArg = requestAddStream $ getSensorValueStreamReq thisArg 

{-|
Whether the sensor is active.
 -}
setSensorActiveReq :: KRPCHS.SpaceCenter.Sensor -> Bool -> KRPCCallReq ()
setSensorActiveReq thisArg valueArg = makeCallReq "SpaceCenter" "Sensor_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setSensorActive :: KRPCHS.SpaceCenter.Sensor -> Bool -> RPCContext ()
setSensorActive thisArg valueArg = simpleRequest $ setSensorActiveReq thisArg valueArg 

{-|
Whether the solar panel is extended.
 -}
getSolarPanelDeployedReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCCallReq (Bool)
getSolarPanelDeployedReq thisArg = makeCallReq "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]

getSolarPanelDeployed :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Bool)
getSolarPanelDeployed thisArg = simpleRequest $ getSolarPanelDeployedReq thisArg

getSolarPanelDeployedStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Bool)
getSolarPanelDeployedStreamReq thisArg = makeStreamReq $ getSolarPanelDeployedReq thisArg

getSolarPanelDeployedStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Bool))
getSolarPanelDeployedStream thisArg = requestAddStream $ getSolarPanelDeployedStreamReq thisArg 

{-|
The current amount of energy being generated by the solar panel, in
units of charge per second.
 -}
getSolarPanelEnergyFlowReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCCallReq (Float)
getSolarPanelEnergyFlowReq thisArg = makeCallReq "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]

getSolarPanelEnergyFlow :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Float)
getSolarPanelEnergyFlow thisArg = simpleRequest $ getSolarPanelEnergyFlowReq thisArg

getSolarPanelEnergyFlowStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Float)
getSolarPanelEnergyFlowStreamReq thisArg = makeStreamReq $ getSolarPanelEnergyFlowReq thisArg

getSolarPanelEnergyFlowStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelEnergyFlowStream thisArg = requestAddStream $ getSolarPanelEnergyFlowStreamReq thisArg 

{-|
The part object for this solar panel.
 -}
getSolarPanelPartReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getSolarPanelPartReq thisArg = makeCallReq "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]

getSolarPanelPart :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCHS.SpaceCenter.Part)
getSolarPanelPart thisArg = simpleRequest $ getSolarPanelPartReq thisArg

getSolarPanelPartStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getSolarPanelPartStreamReq thisArg = makeStreamReq $ getSolarPanelPartReq thisArg

getSolarPanelPartStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getSolarPanelPartStream thisArg = requestAddStream $ getSolarPanelPartStreamReq thisArg 

{-|
The current state of the solar panel.
 -}
getSolarPanelStateReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCCallReq (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelStateReq thisArg = makeCallReq "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]

getSolarPanelState :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelState thisArg = simpleRequest $ getSolarPanelStateReq thisArg

getSolarPanelStateStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelStateStreamReq thisArg = makeStreamReq $ getSolarPanelStateReq thisArg

getSolarPanelStateStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SolarPanelState))
getSolarPanelStateStream thisArg = requestAddStream $ getSolarPanelStateStreamReq thisArg 

{-|
The current amount of sunlight that is incident on the solar panel,
as a percentage. A value between 0 and 1.
 -}
getSolarPanelSunExposureReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCCallReq (Float)
getSolarPanelSunExposureReq thisArg = makeCallReq "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]

getSolarPanelSunExposure :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Float)
getSolarPanelSunExposure thisArg = simpleRequest $ getSolarPanelSunExposureReq thisArg

getSolarPanelSunExposureStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Float)
getSolarPanelSunExposureStreamReq thisArg = makeStreamReq $ getSolarPanelSunExposureReq thisArg

getSolarPanelSunExposureStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelSunExposureStream thisArg = requestAddStream $ getSolarPanelSunExposureStreamReq thisArg 

{-|
Whether the solar panel is extended.
 -}
setSolarPanelDeployedReq :: KRPCHS.SpaceCenter.SolarPanel -> Bool -> KRPCCallReq ()
setSolarPanelDeployedReq thisArg valueArg = makeCallReq "SpaceCenter" "SolarPanel_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setSolarPanelDeployed :: KRPCHS.SpaceCenter.SolarPanel -> Bool -> RPCContext ()
setSolarPanelDeployed thisArg valueArg = simpleRequest $ setSolarPanelDeployedReq thisArg valueArg 

{-|
Position around which the gimbal pivots.
 -}
thrusterGimbalPositionReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
thrusterGimbalPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

thrusterGimbalPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterGimbalPosition thisArg referenceFrameArg = simpleRequest $ thrusterGimbalPositionReq thisArg referenceFrameArg

thrusterGimbalPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterGimbalPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ thrusterGimbalPositionReq thisArg referenceFrameArg

thrusterGimbalPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterGimbalPositionStream thisArg referenceFrameArg = requestAddStream $ thrusterGimbalPositionStreamReq thisArg referenceFrameArg 

{-|
The direction of the force generated by the thruster, when the engine is in its
initial position (no gimballing), in the given reference frame.
This is opposite to the direction in which the thruster expels propellant.<param name="referenceFrame">
 -}
thrusterInitialThrustDirectionReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
thrusterInitialThrustDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

thrusterInitialThrustDirection :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustDirection thisArg referenceFrameArg = simpleRequest $ thrusterInitialThrustDirectionReq thisArg referenceFrameArg

thrusterInitialThrustDirectionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterInitialThrustDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ thrusterInitialThrustDirectionReq thisArg referenceFrameArg

thrusterInitialThrustDirectionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustDirectionStream thisArg referenceFrameArg = requestAddStream $ thrusterInitialThrustDirectionStreamReq thisArg referenceFrameArg 

{-|
The position at which the thruster generates thrust, when the engine is in its
initial position (no gimballing), in the given reference frame.<param name="referenceFrame">This position can move when the gimbal rotates. This is because the thrust position and
gimbal position are not necessarily the same.
 -}
thrusterInitialThrustPositionReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
thrusterInitialThrustPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

thrusterInitialThrustPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustPosition thisArg referenceFrameArg = simpleRequest $ thrusterInitialThrustPositionReq thisArg referenceFrameArg

thrusterInitialThrustPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterInitialThrustPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ thrusterInitialThrustPositionReq thisArg referenceFrameArg

thrusterInitialThrustPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustPositionStream thisArg referenceFrameArg = requestAddStream $ thrusterInitialThrustPositionStreamReq thisArg referenceFrameArg 

{-|
The direction of the force generated by the thruster, in the given reference frame.
This is opposite to the direction in which the thruster expels propellant.
For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustDirectionReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
thrusterThrustDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

thrusterThrustDirection :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustDirection thisArg referenceFrameArg = simpleRequest $ thrusterThrustDirectionReq thisArg referenceFrameArg

thrusterThrustDirectionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterThrustDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ thrusterThrustDirectionReq thisArg referenceFrameArg

thrusterThrustDirectionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustDirectionStream thisArg referenceFrameArg = requestAddStream $ thrusterThrustDirectionStreamReq thisArg referenceFrameArg 

{-|
The position at which the thruster generates thrust, in the given reference frame.
For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustPositionReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
thrusterThrustPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

thrusterThrustPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustPosition thisArg referenceFrameArg = simpleRequest $ thrusterThrustPositionReq thisArg referenceFrameArg

thrusterThrustPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterThrustPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ thrusterThrustPositionReq thisArg referenceFrameArg

thrusterThrustPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustPositionStream thisArg referenceFrameArg = requestAddStream $ thrusterThrustPositionStreamReq thisArg referenceFrameArg 

{-|
The current gimbal angle in the pitch, roll and yaw axes.
 -}
getThrusterGimbalAngleReq :: KRPCHS.SpaceCenter.Thruster -> KRPCCallReq ((Double, Double, Double))
getThrusterGimbalAngleReq thisArg = makeCallReq "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]

getThrusterGimbalAngle :: KRPCHS.SpaceCenter.Thruster -> RPCContext ((Double, Double, Double))
getThrusterGimbalAngle thisArg = simpleRequest $ getThrusterGimbalAngleReq thisArg

getThrusterGimbalAngleStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq ((Double, Double, Double))
getThrusterGimbalAngleStreamReq thisArg = makeStreamReq $ getThrusterGimbalAngleReq thisArg

getThrusterGimbalAngleStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream ((Double, Double, Double)))
getThrusterGimbalAngleStream thisArg = requestAddStream $ getThrusterGimbalAngleStreamReq thisArg 

{-|
Whether the thruster is gimballed.
 -}
getThrusterGimballedReq :: KRPCHS.SpaceCenter.Thruster -> KRPCCallReq (Bool)
getThrusterGimballedReq thisArg = makeCallReq "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]

getThrusterGimballed :: KRPCHS.SpaceCenter.Thruster -> RPCContext (Bool)
getThrusterGimballed thisArg = simpleRequest $ getThrusterGimballedReq thisArg

getThrusterGimballedStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (Bool)
getThrusterGimballedStreamReq thisArg = makeStreamReq $ getThrusterGimballedReq thisArg

getThrusterGimballedStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (Bool))
getThrusterGimballedStream thisArg = requestAddStream $ getThrusterGimballedStreamReq thisArg 

{-|
The <see cref="T:SpaceCenter.Part" /> that contains this thruster.
 -}
getThrusterPartReq :: KRPCHS.SpaceCenter.Thruster -> KRPCCallReq (KRPCHS.SpaceCenter.Part)
getThrusterPartReq thisArg = makeCallReq "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]

getThrusterPart :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCHS.SpaceCenter.Part)
getThrusterPart thisArg = simpleRequest $ getThrusterPartReq thisArg

getThrusterPartStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getThrusterPartStreamReq thisArg = makeStreamReq $ getThrusterPartReq thisArg

getThrusterPartStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getThrusterPartStream thisArg = requestAddStream $ getThrusterPartStreamReq thisArg 

{-|
A reference frame that is fixed relative to the thruster and orientated with
its thrust direction (<see cref="M:SpaceCenter.Thruster.ThrustDirection" />).
For gimballed engines, this takes into account the current rotation of the gimbal.
<list type="bullet">The origin is at the position of thrust for this thruster (<see cref="M:SpaceCenter.Thruster.ThrustPosition" />).The axes rotate with the thrust direction.
This is the direction in which the thruster expels propellant, including any gimballing.The y-axis points along the thrust direction.The x-axis and z-axis are perpendicular to the thrust direction.
 -}
getThrusterThrustReferenceFrameReq :: KRPCHS.SpaceCenter.Thruster -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [makeArgument 0 thisArg]

getThrusterThrustReferenceFrame :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrame thisArg = simpleRequest $ getThrusterThrustReferenceFrameReq thisArg

getThrusterThrustReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrameStreamReq thisArg = makeStreamReq $ getThrusterThrustReferenceFrameReq thisArg

getThrusterThrustReferenceFrameStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getThrusterThrustReferenceFrameStream thisArg = requestAddStream $ getThrusterThrustReferenceFrameStreamReq thisArg 

{-|
Converts a direction vector from one reference frame to another.<param name="direction">Direction vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the direction vector is in.<param name="to">The reference frame to covert the direction vector to.The corresponding direction vector in reference frame <paramref name="to" />.
 -}
transformDirectionReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
transformDirectionReq directionArg fromArg toArg = makeCallReq "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]

transformDirection :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformDirection directionArg fromArg toArg = simpleRequest $ transformDirectionReq directionArg fromArg toArg

transformDirectionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformDirectionStreamReq directionArg fromArg toArg = makeStreamReq $ transformDirectionReq directionArg fromArg toArg

transformDirectionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformDirectionStream directionArg fromArg toArg = requestAddStream $ transformDirectionStreamReq directionArg fromArg toArg 

{-|
Converts a position vector from one reference frame to another.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position vector is in.<param name="to">The reference frame to covert the position vector to.The corresponding position vector in reference frame <paramref name="to" />.
 -}
transformPositionReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
transformPositionReq positionArg fromArg toArg = makeCallReq "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]

transformPosition :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformPosition positionArg fromArg toArg = simpleRequest $ transformPositionReq positionArg fromArg toArg

transformPositionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformPositionStreamReq positionArg fromArg toArg = makeStreamReq $ transformPositionReq positionArg fromArg toArg

transformPositionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformPositionStream positionArg fromArg toArg = requestAddStream $ transformPositionStreamReq positionArg fromArg toArg 

{-|
Converts a rotation from one reference frame to another.<param name="rotation">Rotation in reference frame <paramref name="from" />.<param name="from">The reference frame that the rotation is in.<param name="to">The corresponding rotation in reference frame <paramref name="to" />.The corresponding rotation in reference frame <paramref name="to" />.
 -}
transformRotationReq :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double, Double))
transformRotationReq rotationArg fromArg toArg = makeCallReq "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]

transformRotation :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
transformRotation rotationArg fromArg toArg = simpleRequest $ transformRotationReq rotationArg fromArg toArg

transformRotationStreamReq :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
transformRotationStreamReq rotationArg fromArg toArg = makeStreamReq $ transformRotationReq rotationArg fromArg toArg

transformRotationStream :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
transformRotationStream rotationArg fromArg toArg = requestAddStream $ transformRotationStreamReq rotationArg fromArg toArg 

{-|
Converts a velocity vector (acting at the specified position vector) from one
reference frame to another. The position vector is required to take the
relative angular velocity of the reference frames into account.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="velocity">Velocity vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position and velocity vectors are in.<param name="to">The reference frame to covert the velocity vector to.The corresponding velocity in reference frame <paramref name="to" />.
 -}
transformVelocityReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
transformVelocityReq positionArg velocityArg fromArg toArg = makeCallReq "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]

transformVelocity :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformVelocity positionArg velocityArg fromArg toArg = simpleRequest $ transformVelocityReq positionArg velocityArg fromArg toArg

transformVelocityStreamReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformVelocityStreamReq positionArg velocityArg fromArg toArg = makeStreamReq $ transformVelocityReq positionArg velocityArg fromArg toArg

transformVelocityStream :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformVelocityStream positionArg velocityArg fromArg toArg = requestAddStream $ transformVelocityStreamReq positionArg velocityArg fromArg toArg 

{-|
Returns the angular velocity of the vessel in the given reference frame. The magnitude of the returned
vector is the rotational speed in radians per second, and the direction of the vector indicates the
axis of rotation (using the right hand rule).<param name="referenceFrame">
 -}
vesselAngularVelocityReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
vesselAngularVelocityReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

vesselAngularVelocity :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselAngularVelocity thisArg referenceFrameArg = simpleRequest $ vesselAngularVelocityReq thisArg referenceFrameArg

vesselAngularVelocityStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselAngularVelocityStreamReq thisArg referenceFrameArg = makeStreamReq $ vesselAngularVelocityReq thisArg referenceFrameArg

vesselAngularVelocityStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselAngularVelocityStream thisArg referenceFrameArg = requestAddStream $ vesselAngularVelocityStreamReq thisArg referenceFrameArg 

{-|
Returns the direction in which the vessel is pointing, as a unit vector, in the given reference frame.<param name="referenceFrame">
 -}
vesselDirectionReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
vesselDirectionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

vesselDirection :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselDirection thisArg referenceFrameArg = simpleRequest $ vesselDirectionReq thisArg referenceFrameArg

vesselDirectionStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselDirectionStreamReq thisArg referenceFrameArg = makeStreamReq $ vesselDirectionReq thisArg referenceFrameArg

vesselDirectionStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselDirectionStream thisArg referenceFrameArg = requestAddStream $ vesselDirectionStreamReq thisArg referenceFrameArg 

{-|
Returns a <see cref="T:SpaceCenter.Flight" /> object that can be used to get flight
telemetry for the vessel, in the specified reference frame.<param name="referenceFrame">
Reference frame. Defaults to the vessel's surface reference frame (<see cref="M:SpaceCenter.Vessel.SurfaceReferenceFrame" />).
 -}
vesselFlightReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq (KRPCHS.SpaceCenter.Flight)
vesselFlightReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

vesselFlight :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCHS.SpaceCenter.Flight)
vesselFlight thisArg referenceFrameArg = simpleRequest $ vesselFlightReq thisArg referenceFrameArg

vesselFlightStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (KRPCHS.SpaceCenter.Flight)
vesselFlightStreamReq thisArg referenceFrameArg = makeStreamReq $ vesselFlightReq thisArg referenceFrameArg

vesselFlightStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Flight))
vesselFlightStream thisArg referenceFrameArg = requestAddStream $ vesselFlightStreamReq thisArg referenceFrameArg 

{-|
Returns the position vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselPositionReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
vesselPositionReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

vesselPosition :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselPosition thisArg referenceFrameArg = simpleRequest $ vesselPositionReq thisArg referenceFrameArg

vesselPositionStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselPositionStreamReq thisArg referenceFrameArg = makeStreamReq $ vesselPositionReq thisArg referenceFrameArg

vesselPositionStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselPositionStream thisArg referenceFrameArg = requestAddStream $ vesselPositionStreamReq thisArg referenceFrameArg 

{-|
Recover the vessel.
 -}
vesselRecoverReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ()
vesselRecoverReq thisArg = makeCallReq "SpaceCenter" "Vessel_Recover" [makeArgument 0 thisArg]

vesselRecover :: KRPCHS.SpaceCenter.Vessel -> RPCContext ()
vesselRecover thisArg = simpleRequest $ vesselRecoverReq thisArg 

{-|
Returns a <see cref="T:SpaceCenter.Resources" /> object, that can used to get
information about resources stored in a given <paramref name="stage" />.<param name="stage">Get resources for parts that are decoupled in this stage.<param name="cumulative">Whenfalse, returns the resources for parts
decoupled in just the given stage. Whentruereturns the resources decoupled in
the given stage and all subsequent stages combined.
 -}
vesselResourcesInDecoupleStageReq :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> KRPCCallReq (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStageReq thisArg stageArg cumulativeArg = makeCallReq "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg, makeArgument 2 cumulativeArg]

vesselResourcesInDecoupleStage :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> RPCContext (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStage thisArg stageArg cumulativeArg = simpleRequest $ vesselResourcesInDecoupleStageReq thisArg stageArg cumulativeArg

vesselResourcesInDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStageStreamReq thisArg stageArg cumulativeArg = makeStreamReq $ vesselResourcesInDecoupleStageReq thisArg stageArg cumulativeArg

vesselResourcesInDecoupleStageStream :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
vesselResourcesInDecoupleStageStream thisArg stageArg cumulativeArg = requestAddStream $ vesselResourcesInDecoupleStageStreamReq thisArg stageArg cumulativeArg 

{-|
Returns the rotation of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselRotationReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double, Double))
vesselRotationReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

vesselRotation :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
vesselRotation thisArg referenceFrameArg = simpleRequest $ vesselRotationReq thisArg referenceFrameArg

vesselRotationStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
vesselRotationStreamReq thisArg referenceFrameArg = makeStreamReq $ vesselRotationReq thisArg referenceFrameArg

vesselRotationStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
vesselRotationStream thisArg referenceFrameArg = requestAddStream $ vesselRotationStreamReq thisArg referenceFrameArg 

{-|
Returns the velocity vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselVelocityReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ((Double, Double, Double))
vesselVelocityReq thisArg referenceFrameArg = makeCallReq "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]

vesselVelocity :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselVelocity thisArg referenceFrameArg = simpleRequest $ vesselVelocityReq thisArg referenceFrameArg

vesselVelocityStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselVelocityStreamReq thisArg referenceFrameArg = makeStreamReq $ vesselVelocityReq thisArg referenceFrameArg

vesselVelocityStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselVelocityStream thisArg referenceFrameArg = requestAddStream $ vesselVelocityStreamReq thisArg referenceFrameArg 

{-|
An <see cref="T:SpaceCenter.AutoPilot" /> object, that can be used to perform
simple auto-piloting of the vessel.
 -}
getVesselAutoPilotReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilotReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]

getVesselAutoPilot :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilot thisArg = simpleRequest $ getVesselAutoPilotReq thisArg

getVesselAutoPilotStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilotStreamReq thisArg = makeStreamReq $ getVesselAutoPilotReq thisArg

getVesselAutoPilotStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.AutoPilot))
getVesselAutoPilotStream thisArg = requestAddStream $ getVesselAutoPilotStreamReq thisArg 

{-|
The maximum torque that the aerodynamic control surfaces can generate.
Returns the torques inN.maround each of the coordinate axes of the
vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableControlSurfaceTorqueReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ((Double, Double, Double))
getVesselAvailableControlSurfaceTorqueReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AvailableControlSurfaceTorque" [makeArgument 0 thisArg]

getVesselAvailableControlSurfaceTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableControlSurfaceTorque thisArg = simpleRequest $ getVesselAvailableControlSurfaceTorqueReq thisArg

getVesselAvailableControlSurfaceTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableControlSurfaceTorqueStreamReq thisArg = makeStreamReq $ getVesselAvailableControlSurfaceTorqueReq thisArg

getVesselAvailableControlSurfaceTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableControlSurfaceTorqueStream thisArg = requestAddStream $ getVesselAvailableControlSurfaceTorqueStreamReq thisArg 

{-|
The maximum torque that the currently active and gimballed engines can generate.
Returns the torques inN.maround each of the coordinate axes of the
vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableEngineTorqueReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ((Double, Double, Double))
getVesselAvailableEngineTorqueReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AvailableEngineTorque" [makeArgument 0 thisArg]

getVesselAvailableEngineTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableEngineTorque thisArg = simpleRequest $ getVesselAvailableEngineTorqueReq thisArg

getVesselAvailableEngineTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableEngineTorqueStreamReq thisArg = makeStreamReq $ getVesselAvailableEngineTorqueReq thisArg

getVesselAvailableEngineTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableEngineTorqueStream thisArg = requestAddStream $ getVesselAvailableEngineTorqueStreamReq thisArg 

{-|
The maximum torque that the currently active RCS thrusters can generate.
Returns the torques inN.maround each of the coordinate axes of the
vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableRCSTorqueReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ((Double, Double, Double))
getVesselAvailableRCSTorqueReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AvailableRCSTorque" [makeArgument 0 thisArg]

getVesselAvailableRCSTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableRCSTorque thisArg = simpleRequest $ getVesselAvailableRCSTorqueReq thisArg

getVesselAvailableRCSTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableRCSTorqueStreamReq thisArg = makeStreamReq $ getVesselAvailableRCSTorqueReq thisArg

getVesselAvailableRCSTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableRCSTorqueStream thisArg = requestAddStream $ getVesselAvailableRCSTorqueStreamReq thisArg 

{-|
The maximum torque that the currently active and powered reaction wheels can generate.
Returns the torques inN.maround each of the coordinate axes of the
vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableReactionWheelTorqueReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ((Double, Double, Double))
getVesselAvailableReactionWheelTorqueReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AvailableReactionWheelTorque" [makeArgument 0 thisArg]

getVesselAvailableReactionWheelTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableReactionWheelTorque thisArg = simpleRequest $ getVesselAvailableReactionWheelTorqueReq thisArg

getVesselAvailableReactionWheelTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableReactionWheelTorqueStreamReq thisArg = makeStreamReq $ getVesselAvailableReactionWheelTorqueReq thisArg

getVesselAvailableReactionWheelTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableReactionWheelTorqueStream thisArg = requestAddStream $ getVesselAvailableReactionWheelTorqueStreamReq thisArg 

{-|
Gets the total available thrust that can be produced by the vessel's
active engines, in Newtons. This is computed by summing
<see cref="M:SpaceCenter.Engine.AvailableThrust" /> for every active engine in the vessel.
 -}
getVesselAvailableThrustReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselAvailableThrustReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]

getVesselAvailableThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselAvailableThrust thisArg = simpleRequest $ getVesselAvailableThrustReq thisArg

getVesselAvailableThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselAvailableThrustStreamReq thisArg = makeStreamReq $ getVesselAvailableThrustReq thisArg

getVesselAvailableThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselAvailableThrustStream thisArg = requestAddStream $ getVesselAvailableThrustStreamReq thisArg 

{-|
The maximum torque that the vessel generate. Includes contributions from reaction wheels,
RCS, gimballed engines and aerodynamic control surfaces.
Returns the torques inN.maround each of the coordinate axes of the
vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableTorqueReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ((Double, Double, Double))
getVesselAvailableTorqueReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_AvailableTorque" [makeArgument 0 thisArg]

getVesselAvailableTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableTorque thisArg = simpleRequest $ getVesselAvailableTorqueReq thisArg

getVesselAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableTorqueStreamReq thisArg = makeStreamReq $ getVesselAvailableTorqueReq thisArg

getVesselAvailableTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableTorqueStream thisArg = requestAddStream $ getVesselAvailableTorqueStreamReq thisArg 

{-|
The name of the biome the vessel is currently in.
 -}
getVesselBiomeReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Data.Text.Text)
getVesselBiomeReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Biome" [makeArgument 0 thisArg]

getVesselBiome :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Data.Text.Text)
getVesselBiome thisArg = simpleRequest $ getVesselBiomeReq thisArg

getVesselBiomeStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Data.Text.Text)
getVesselBiomeStreamReq thisArg = makeStreamReq $ getVesselBiomeReq thisArg

getVesselBiomeStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Data.Text.Text))
getVesselBiomeStream thisArg = requestAddStream $ getVesselBiomeStreamReq thisArg 

{-|
Returns a <see cref="T:SpaceCenter.Control" /> object that can be used to manipulate
the vessel's control inputs. For example, its pitch/yaw/roll controls,
RCS and thrust.
 -}
getVesselControlReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.Control)
getVesselControlReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]

getVesselControl :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Control)
getVesselControl thisArg = simpleRequest $ getVesselControlReq thisArg

getVesselControlStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Control)
getVesselControlStreamReq thisArg = makeStreamReq $ getVesselControlReq thisArg

getVesselControlStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Control))
getVesselControlStream thisArg = requestAddStream $ getVesselControlStreamReq thisArg 

{-|
The total mass of the vessel, excluding resources, in kg.
 -}
getVesselDryMassReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselDryMassReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]

getVesselDryMass :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselDryMass thisArg = simpleRequest $ getVesselDryMassReq thisArg

getVesselDryMassStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselDryMassStreamReq thisArg = makeStreamReq $ getVesselDryMassReq thisArg

getVesselDryMassStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselDryMassStream thisArg = requestAddStream $ getVesselDryMassStreamReq thisArg 

{-|
The inertia tensor of the vessel around its center of mass, in the vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getVesselInertiaTensorReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ([Double])
getVesselInertiaTensorReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]

getVesselInertiaTensor :: KRPCHS.SpaceCenter.Vessel -> RPCContext ([Double])
getVesselInertiaTensor thisArg = simpleRequest $ getVesselInertiaTensorReq thisArg

getVesselInertiaTensorStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ([Double])
getVesselInertiaTensorStreamReq thisArg = makeStreamReq $ getVesselInertiaTensorReq thisArg

getVesselInertiaTensorStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ([Double]))
getVesselInertiaTensorStream thisArg = requestAddStream $ getVesselInertiaTensorStreamReq thisArg 

{-|
The combined specific impulse of all active engines at sea level on Kerbin, in seconds.
This is computed using the formula
<a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselKerbinSeaLevelSpecificImpulseReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselKerbinSeaLevelSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]

getVesselKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselKerbinSeaLevelSpecificImpulse thisArg = simpleRequest $ getVesselKerbinSeaLevelSpecificImpulseReq thisArg

getVesselKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselKerbinSeaLevelSpecificImpulseStreamReq thisArg = makeStreamReq $ getVesselKerbinSeaLevelSpecificImpulseReq thisArg

getVesselKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselKerbinSeaLevelSpecificImpulseStream thisArg = requestAddStream $ getVesselKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-|
The mission elapsed time in seconds.
 -}
getVesselMETReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Double)
getVesselMETReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]

getVesselMET :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Double)
getVesselMET thisArg = simpleRequest $ getVesselMETReq thisArg

getVesselMETStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Double)
getVesselMETStreamReq thisArg = makeStreamReq $ getVesselMETReq thisArg

getVesselMETStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Double))
getVesselMETStream thisArg = requestAddStream $ getVesselMETStreamReq thisArg 

{-|
The total mass of the vessel, including resources, in kg.
 -}
getVesselMassReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselMassReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]

getVesselMass :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMass thisArg = simpleRequest $ getVesselMassReq thisArg

getVesselMassStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMassStreamReq thisArg = makeStreamReq $ getVesselMassReq thisArg

getVesselMassStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMassStream thisArg = requestAddStream $ getVesselMassStreamReq thisArg 

{-|
The total maximum thrust that can be produced by the vessel's active
engines, in Newtons. This is computed by summing
<see cref="M:SpaceCenter.Engine.MaxThrust" /> for every active engine.
 -}
getVesselMaxThrustReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselMaxThrustReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]

getVesselMaxThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMaxThrust thisArg = simpleRequest $ getVesselMaxThrustReq thisArg

getVesselMaxThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMaxThrustStreamReq thisArg = makeStreamReq $ getVesselMaxThrustReq thisArg

getVesselMaxThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxThrustStream thisArg = requestAddStream $ getVesselMaxThrustStreamReq thisArg 

{-|
The total maximum thrust that can be produced by the vessel's active
engines when the vessel is in a vacuum, in Newtons. This is computed by
summing <see cref="M:SpaceCenter.Engine.MaxVacuumThrust" /> for every active engine.
 -}
getVesselMaxVacuumThrustReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselMaxVacuumThrustReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]

getVesselMaxVacuumThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMaxVacuumThrust thisArg = simpleRequest $ getVesselMaxVacuumThrustReq thisArg

getVesselMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMaxVacuumThrustStreamReq thisArg = makeStreamReq $ getVesselMaxVacuumThrustReq thisArg

getVesselMaxVacuumThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxVacuumThrustStream thisArg = requestAddStream $ getVesselMaxVacuumThrustStreamReq thisArg 

{-|
The moment of inertia of the vessel around its center of mass inkg.m^2.
The inertia values are around the pitch, roll and yaw directions respectively.
This corresponds to the vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 -}
getVesselMomentOfInertiaReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ((Double, Double, Double))
getVesselMomentOfInertiaReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]

getVesselMomentOfInertia :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselMomentOfInertia thisArg = simpleRequest $ getVesselMomentOfInertiaReq thisArg

getVesselMomentOfInertiaStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselMomentOfInertiaStreamReq thisArg = makeStreamReq $ getVesselMomentOfInertiaReq thisArg

getVesselMomentOfInertiaStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselMomentOfInertiaStream thisArg = requestAddStream $ getVesselMomentOfInertiaStreamReq thisArg 

{-|
The name of the vessel.
 -}
getVesselNameReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Data.Text.Text)
getVesselNameReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]

getVesselName :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Data.Text.Text)
getVesselName thisArg = simpleRequest $ getVesselNameReq thisArg

getVesselNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Data.Text.Text)
getVesselNameStreamReq thisArg = makeStreamReq $ getVesselNameReq thisArg

getVesselNameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Data.Text.Text))
getVesselNameStream thisArg = requestAddStream $ getVesselNameStreamReq thisArg 

{-|
The current orbit of the vessel.
 -}
getVesselOrbitReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.Orbit)
getVesselOrbitReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]

getVesselOrbit :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getVesselOrbit thisArg = simpleRequest $ getVesselOrbitReq thisArg

getVesselOrbitStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getVesselOrbitStreamReq thisArg = makeStreamReq $ getVesselOrbitReq thisArg

getVesselOrbitStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getVesselOrbitStream thisArg = requestAddStream $ getVesselOrbitStreamReq thisArg 

{-|
The reference frame that is fixed relative to the vessel, and orientated with the vessels
orbital prograde/normal/radial directions.
<list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the orbital prograde/normal/radial directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.Be careful not to confuse this with 'orbit' mode on the navball.
 -}
getVesselOrbitalReferenceFrameReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]

getVesselOrbitalReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrame thisArg = simpleRequest $ getVesselOrbitalReferenceFrameReq thisArg

getVesselOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrameStreamReq thisArg = makeStreamReq $ getVesselOrbitalReferenceFrameReq thisArg

getVesselOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselOrbitalReferenceFrameStream thisArg = requestAddStream $ getVesselOrbitalReferenceFrameStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Parts" /> object, that can used to interact with the parts that make up this vessel.
 -}
getVesselPartsReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.Parts)
getVesselPartsReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]

getVesselParts :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Parts)
getVesselParts thisArg = simpleRequest $ getVesselPartsReq thisArg

getVesselPartsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Parts)
getVesselPartsStreamReq thisArg = makeStreamReq $ getVesselPartsReq thisArg

getVesselPartsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Parts))
getVesselPartsStream thisArg = requestAddStream $ getVesselPartsStreamReq thisArg 

{-|
Whether the vessel is recoverable.
 -}
getVesselRecoverableReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Bool)
getVesselRecoverableReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Recoverable" [makeArgument 0 thisArg]

getVesselRecoverable :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
getVesselRecoverable thisArg = simpleRequest $ getVesselRecoverableReq thisArg

getVesselRecoverableStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Bool)
getVesselRecoverableStreamReq thisArg = makeStreamReq $ getVesselRecoverableReq thisArg

getVesselRecoverableStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Bool))
getVesselRecoverableStream thisArg = requestAddStream $ getVesselRecoverableStreamReq thisArg 

{-|
The reference frame that is fixed relative to the vessel, and orientated with the vessel.
<list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel.The x-axis points out to the right of the vessel.The y-axis points in the forward direction of the vessel.The z-axis points out of the bottom off the vessel.
 -}
getVesselReferenceFrameReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]

getVesselReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrame thisArg = simpleRequest $ getVesselReferenceFrameReq thisArg

getVesselReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrameStreamReq thisArg = makeStreamReq $ getVesselReferenceFrameReq thisArg

getVesselReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselReferenceFrameStream thisArg = requestAddStream $ getVesselReferenceFrameStreamReq thisArg 

{-|
A <see cref="T:SpaceCenter.Resources" /> object, that can used to get information
about resources stored in the vessel.
 -}
getVesselResourcesReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.Resources)
getVesselResourcesReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]

getVesselResources :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Resources)
getVesselResources thisArg = simpleRequest $ getVesselResourcesReq thisArg

getVesselResourcesStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
getVesselResourcesStreamReq thisArg = makeStreamReq $ getVesselResourcesReq thisArg

getVesselResourcesStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
getVesselResourcesStream thisArg = requestAddStream $ getVesselResourcesStreamReq thisArg 

{-|
The situation the vessel is in.
 -}
getVesselSituationReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituationReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]

getVesselSituation :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituation thisArg = simpleRequest $ getVesselSituationReq thisArg

getVesselSituationStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituationStreamReq thisArg = makeStreamReq $ getVesselSituationReq thisArg

getVesselSituationStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.VesselSituation))
getVesselSituationStream thisArg = requestAddStream $ getVesselSituationStreamReq thisArg 

{-|
The combined specific impulse of all active engines, in seconds. This is computed using the formula
<a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselSpecificImpulseReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]

getVesselSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselSpecificImpulse thisArg = simpleRequest $ getVesselSpecificImpulseReq thisArg

getVesselSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselSpecificImpulseStreamReq thisArg = makeStreamReq $ getVesselSpecificImpulseReq thisArg

getVesselSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselSpecificImpulseStream thisArg = requestAddStream $ getVesselSpecificImpulseStreamReq thisArg 

{-|
The reference frame that is fixed relative to the vessel, and orientated with the surface
of the body being orbited.
<list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the north and up directions on the surface of the body.The x-axis points in the <a href="https://en.wikipedia.org/wiki/Zenith">zenithdirection (upwards, normal to the body being orbited, from the center of the body towards the center of
mass of the vessel).The y-axis points northwards towards the
<a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon(north, and tangential to the
surface of the body -- the direction in which a compass would point when on the surface).The z-axis points eastwards towards the
<a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon(east, and tangential to the
surface of the body -- east on a compass when on the surface).Be careful not to confuse this with 'surface' mode on the navball.
 -}
getVesselSurfaceReferenceFrameReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [makeArgument 0 thisArg]

getVesselSurfaceReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrame thisArg = simpleRequest $ getVesselSurfaceReferenceFrameReq thisArg

getVesselSurfaceReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrameStreamReq thisArg = makeStreamReq $ getVesselSurfaceReferenceFrameReq thisArg

getVesselSurfaceReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceReferenceFrameStream thisArg = requestAddStream $ getVesselSurfaceReferenceFrameStreamReq thisArg 

{-|
The reference frame that is fixed relative to the vessel, and orientated with the velocity
vector of the vessel relative to the surface of the body being orbited.
<list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel's velocity vector.The y-axis points in the direction of the vessel's velocity vector,
relative to the surface of the body being orbited.The z-axis is in the plane of the
<a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon.The x-axis is orthogonal to the other two axes.
 -}
getVesselSurfaceVelocityReferenceFrameReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrameReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [makeArgument 0 thisArg]

getVesselSurfaceVelocityReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrame thisArg = simpleRequest $ getVesselSurfaceVelocityReferenceFrameReq thisArg

getVesselSurfaceVelocityReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrameStreamReq thisArg = makeStreamReq $ getVesselSurfaceVelocityReferenceFrameReq thisArg

getVesselSurfaceVelocityReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceVelocityReferenceFrameStream thisArg = requestAddStream $ getVesselSurfaceVelocityReferenceFrameStreamReq thisArg 

{-|
The total thrust currently being produced by the vessel's engines, in
Newtons. This is computed by summing <see cref="M:SpaceCenter.Engine.Thrust" /> for
every engine in the vessel.
 -}
getVesselThrustReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselThrustReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]

getVesselThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselThrust thisArg = simpleRequest $ getVesselThrustReq thisArg

getVesselThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselThrustStreamReq thisArg = makeStreamReq $ getVesselThrustReq thisArg

getVesselThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselThrustStream thisArg = requestAddStream $ getVesselThrustStreamReq thisArg 

{-|
The type of the vessel.
 -}
getVesselTypeReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (KRPCHS.SpaceCenter.VesselType)
getVesselTypeReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]

getVesselType :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.VesselType)
getVesselType thisArg = simpleRequest $ getVesselTypeReq thisArg

getVesselTypeStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.VesselType)
getVesselTypeStreamReq thisArg = makeStreamReq $ getVesselTypeReq thisArg

getVesselTypeStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.VesselType))
getVesselTypeStream thisArg = requestAddStream $ getVesselTypeStreamReq thisArg 

{-|
The combined vacuum specific impulse of all active engines, in seconds. This is computed using the formula
<a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselVacuumSpecificImpulseReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq (Float)
getVesselVacuumSpecificImpulseReq thisArg = makeCallReq "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]

getVesselVacuumSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselVacuumSpecificImpulse thisArg = simpleRequest $ getVesselVacuumSpecificImpulseReq thisArg

getVesselVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselVacuumSpecificImpulseStreamReq thisArg = makeStreamReq $ getVesselVacuumSpecificImpulseReq thisArg

getVesselVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselVacuumSpecificImpulseStream thisArg = requestAddStream $ getVesselVacuumSpecificImpulseStreamReq thisArg 

{-|
The name of the vessel.
 -}
setVesselNameReq :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> KRPCCallReq ()
setVesselNameReq thisArg valueArg = makeCallReq "SpaceCenter" "Vessel_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setVesselName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext ()
setVesselName thisArg valueArg = simpleRequest $ setVesselNameReq thisArg valueArg 

{-|
The type of the vessel.
 -}
setVesselTypeReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.VesselType -> KRPCCallReq ()
setVesselTypeReq thisArg valueArg = makeCallReq "SpaceCenter" "Vessel_set_Type" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setVesselType :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.VesselType -> RPCContext ()
setVesselType thisArg valueArg = simpleRequest $ setVesselTypeReq thisArg valueArg 

{-|
Uses time acceleration to warp forward to a time in the future, specified
by universal time <paramref name="ut" />. This call blocks until the desired
time is reached. Uses regular "on-rails" or physical time warp as appropriate.
For example, physical time warp is used when the active vessel is traveling
through an atmosphere. When using regular "on-rails" time warp, the warp
rate is limited by <paramref name="maxRailsRate" />, and when using physical
time warp, the warp rate is limited by <paramref name="maxPhysicsRate" />.<param name="ut">The universal time to warp to, in seconds.<param name="maxRailsRate">The maximum warp rate in regular "on-rails" time warp.<param name="maxPhysicsRate">The maximum warp rate in physical time warp.When the time warp is complete.
 -}
warpToReq :: Double -> Float -> Float -> KRPCCallReq ()
warpToReq utArg maxRailsRateArg maxPhysicsRateArg = makeCallReq "SpaceCenter" "WarpTo" [makeArgument 0 utArg, makeArgument 1 maxRailsRateArg, makeArgument 2 maxPhysicsRateArg]

warpTo :: Double -> Float -> Float -> RPCContext ()
warpTo utArg maxRailsRateArg maxPhysicsRateArg = simpleRequest $ warpToReq utArg maxRailsRateArg maxPhysicsRateArg 

{-|
Creates a waypoint at the given position at ground level, and returns a
<see cref="T:SpaceCenter.Waypoint" /> object that can be used to modify it.<param name="latitude">Latitude of the waypoint.<param name="longitude">Longitude of the waypoint.<param name="body">Celestial body the waypoint is attached to.<param name="name">Name of the waypoint.
 -}
waypointManagerAddWaypointReq :: KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> KRPCCallReq (KRPCHS.SpaceCenter.Waypoint)
waypointManagerAddWaypointReq thisArg latitudeArg longitudeArg bodyArg nameArg = makeCallReq "SpaceCenter" "WaypointManager_AddWaypoint" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 bodyArg, makeArgument 4 nameArg]

waypointManagerAddWaypoint :: KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> RPCContext (KRPCHS.SpaceCenter.Waypoint)
waypointManagerAddWaypoint thisArg latitudeArg longitudeArg bodyArg nameArg = simpleRequest $ waypointManagerAddWaypointReq thisArg latitudeArg longitudeArg bodyArg nameArg

waypointManagerAddWaypointStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.Waypoint)
waypointManagerAddWaypointStreamReq thisArg latitudeArg longitudeArg bodyArg nameArg = makeStreamReq $ waypointManagerAddWaypointReq thisArg latitudeArg longitudeArg bodyArg nameArg

waypointManagerAddWaypointStream :: KRPCHS.SpaceCenter.WaypointManager -> Double -> Double -> KRPCHS.SpaceCenter.CelestialBody -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Waypoint))
waypointManagerAddWaypointStream thisArg latitudeArg longitudeArg bodyArg nameArg = requestAddStream $ waypointManagerAddWaypointStreamReq thisArg latitudeArg longitudeArg bodyArg nameArg 

{-|
An example map of known color - seed pairs. 
Any other integers may be used as seed.
 -}
getWaypointManagerColorsReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCCallReq (Data.Map.Map (Data.Text.Text) (Data.Int.Int32))
getWaypointManagerColorsReq thisArg = makeCallReq "SpaceCenter" "WaypointManager_get_Colors" [makeArgument 0 thisArg]

getWaypointManagerColors :: KRPCHS.SpaceCenter.WaypointManager -> RPCContext (Data.Map.Map (Data.Text.Text) (Data.Int.Int32))
getWaypointManagerColors thisArg = simpleRequest $ getWaypointManagerColorsReq thisArg

getWaypointManagerColorsStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Data.Int.Int32))
getWaypointManagerColorsStreamReq thisArg = makeStreamReq $ getWaypointManagerColorsReq thisArg

getWaypointManagerColorsStream :: KRPCHS.SpaceCenter.WaypointManager -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Data.Int.Int32)))
getWaypointManagerColorsStream thisArg = requestAddStream $ getWaypointManagerColorsStreamReq thisArg 

{-|
Returns all available icons (from "GameData/Squad/Contracts/Icons/").
 -}
getWaypointManagerIconsReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCCallReq ([Data.Text.Text])
getWaypointManagerIconsReq thisArg = makeCallReq "SpaceCenter" "WaypointManager_get_Icons" [makeArgument 0 thisArg]

getWaypointManagerIcons :: KRPCHS.SpaceCenter.WaypointManager -> RPCContext ([Data.Text.Text])
getWaypointManagerIcons thisArg = simpleRequest $ getWaypointManagerIconsReq thisArg

getWaypointManagerIconsStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCStreamReq ([Data.Text.Text])
getWaypointManagerIconsStreamReq thisArg = makeStreamReq $ getWaypointManagerIconsReq thisArg

getWaypointManagerIconsStream :: KRPCHS.SpaceCenter.WaypointManager -> RPCContext (KRPCStream ([Data.Text.Text]))
getWaypointManagerIconsStream thisArg = requestAddStream $ getWaypointManagerIconsStreamReq thisArg 

{-|
A list of all existing waypoints.
 -}
getWaypointManagerWaypointsReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCCallReq ([KRPCHS.SpaceCenter.Waypoint])
getWaypointManagerWaypointsReq thisArg = makeCallReq "SpaceCenter" "WaypointManager_get_Waypoints" [makeArgument 0 thisArg]

getWaypointManagerWaypoints :: KRPCHS.SpaceCenter.WaypointManager -> RPCContext ([KRPCHS.SpaceCenter.Waypoint])
getWaypointManagerWaypoints thisArg = simpleRequest $ getWaypointManagerWaypointsReq thisArg

getWaypointManagerWaypointsStreamReq :: KRPCHS.SpaceCenter.WaypointManager -> KRPCStreamReq ([KRPCHS.SpaceCenter.Waypoint])
getWaypointManagerWaypointsStreamReq thisArg = makeStreamReq $ getWaypointManagerWaypointsReq thisArg

getWaypointManagerWaypointsStream :: KRPCHS.SpaceCenter.WaypointManager -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Waypoint]))
getWaypointManagerWaypointsStream thisArg = requestAddStream $ getWaypointManagerWaypointsStreamReq thisArg 

{-|
Removes the waypoint.
 -}
waypointRemoveReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq ()
waypointRemoveReq thisArg = makeCallReq "SpaceCenter" "Waypoint_Remove" [makeArgument 0 thisArg]

waypointRemove :: KRPCHS.SpaceCenter.Waypoint -> RPCContext ()
waypointRemove thisArg = simpleRequest $ waypointRemoveReq thisArg 

{-|
The altitude of the waypoint above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 -}
getWaypointBedrockAltitudeReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Double)
getWaypointBedrockAltitudeReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_BedrockAltitude" [makeArgument 0 thisArg]

getWaypointBedrockAltitude :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Double)
getWaypointBedrockAltitude thisArg = simpleRequest $ getWaypointBedrockAltitudeReq thisArg

getWaypointBedrockAltitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointBedrockAltitudeStreamReq thisArg = makeStreamReq $ getWaypointBedrockAltitudeReq thisArg

getWaypointBedrockAltitudeStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Double))
getWaypointBedrockAltitudeStream thisArg = requestAddStream $ getWaypointBedrockAltitudeStreamReq thisArg 

{-|
Celestial body the waypoint is attached to.
 -}
getWaypointBodyReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getWaypointBodyReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Body" [makeArgument 0 thisArg]

getWaypointBody :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getWaypointBody thisArg = simpleRequest $ getWaypointBodyReq thisArg

getWaypointBodyStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getWaypointBodyStreamReq thisArg = makeStreamReq $ getWaypointBodyReq thisArg

getWaypointBodyStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getWaypointBodyStream thisArg = requestAddStream $ getWaypointBodyStreamReq thisArg 

{-|
True if this waypoint is part of a set of clustered waypoints with greek letter names appended (Alpha, Beta, Gamma, etc). 
If true, there is a one-to-one correspondence with the greek letter name and the <see cref="M:SpaceCenter.Waypoint.Index" />.
 -}
getWaypointClusteredReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Bool)
getWaypointClusteredReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Clustered" [makeArgument 0 thisArg]

getWaypointClustered :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Bool)
getWaypointClustered thisArg = simpleRequest $ getWaypointClusteredReq thisArg

getWaypointClusteredStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointClusteredStreamReq thisArg = makeStreamReq $ getWaypointClusteredReq thisArg

getWaypointClusteredStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Bool))
getWaypointClusteredStream thisArg = requestAddStream $ getWaypointClusteredStreamReq thisArg 

{-|
The seed of the icon color. See <see cref="M:SpaceCenter.WaypointManager.Colors" /> for example colors.
 -}
getWaypointColorReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Data.Int.Int32)
getWaypointColorReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Color" [makeArgument 0 thisArg]

getWaypointColor :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Data.Int.Int32)
getWaypointColor thisArg = simpleRequest $ getWaypointColorReq thisArg

getWaypointColorStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Int.Int32)
getWaypointColorStreamReq thisArg = makeStreamReq $ getWaypointColorReq thisArg

getWaypointColorStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Data.Int.Int32))
getWaypointColorStream thisArg = requestAddStream $ getWaypointColorStreamReq thisArg 

{-|
The id of the associated contract.
Returns 0 if the waypoint does not belong to a contract.
 -}
getWaypointContractIdReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Data.Int.Int64)
getWaypointContractIdReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_ContractId" [makeArgument 0 thisArg]

getWaypointContractId :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Data.Int.Int64)
getWaypointContractId thisArg = simpleRequest $ getWaypointContractIdReq thisArg

getWaypointContractIdStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Int.Int64)
getWaypointContractIdStreamReq thisArg = makeStreamReq $ getWaypointContractIdReq thisArg

getWaypointContractIdStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Data.Int.Int64))
getWaypointContractIdStream thisArg = requestAddStream $ getWaypointContractIdStreamReq thisArg 

{-|
True if waypoint is actually glued to the ground.
 -}
getWaypointGroundedReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Bool)
getWaypointGroundedReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Grounded" [makeArgument 0 thisArg]

getWaypointGrounded :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Bool)
getWaypointGrounded thisArg = simpleRequest $ getWaypointGroundedReq thisArg

getWaypointGroundedStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointGroundedStreamReq thisArg = makeStreamReq $ getWaypointGroundedReq thisArg

getWaypointGroundedStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Bool))
getWaypointGroundedStream thisArg = requestAddStream $ getWaypointGroundedStreamReq thisArg 

{-|
Whether the waypoint belongs to a contract.
 -}
getWaypointHasContractReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Bool)
getWaypointHasContractReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_HasContract" [makeArgument 0 thisArg]

getWaypointHasContract :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Bool)
getWaypointHasContract thisArg = simpleRequest $ getWaypointHasContractReq thisArg

getWaypointHasContractStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointHasContractStreamReq thisArg = makeStreamReq $ getWaypointHasContractReq thisArg

getWaypointHasContractStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Bool))
getWaypointHasContractStream thisArg = requestAddStream $ getWaypointHasContractStreamReq thisArg 

{-|
The icon of the waypoint.
 -}
getWaypointIconReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Data.Text.Text)
getWaypointIconReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Icon" [makeArgument 0 thisArg]

getWaypointIcon :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Data.Text.Text)
getWaypointIcon thisArg = simpleRequest $ getWaypointIconReq thisArg

getWaypointIconStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Text.Text)
getWaypointIconStreamReq thisArg = makeStreamReq $ getWaypointIconReq thisArg

getWaypointIconStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Data.Text.Text))
getWaypointIconStream thisArg = requestAddStream $ getWaypointIconStreamReq thisArg 

{-|
The integer index of this waypoint amongst its cluster of sibling waypoints. 
In other words, when you have a cluster of waypoints called "Somewhere Alpha", "Somewhere Beta", and "Somewhere Gamma", 
then the alpha site has index 0, the beta site has index 1 and the gamma site has index 2. 
When <see cref="M:SpaceCenter.Waypoint.Clustered" /> is false, this value is zero but meaningless.
 -}
getWaypointIndexReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Data.Int.Int32)
getWaypointIndexReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Index" [makeArgument 0 thisArg]

getWaypointIndex :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Data.Int.Int32)
getWaypointIndex thisArg = simpleRequest $ getWaypointIndexReq thisArg

getWaypointIndexStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Int.Int32)
getWaypointIndexStreamReq thisArg = makeStreamReq $ getWaypointIndexReq thisArg

getWaypointIndexStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Data.Int.Int32))
getWaypointIndexStream thisArg = requestAddStream $ getWaypointIndexStreamReq thisArg 

{-|
The latitude of the waypoint.
 -}
getWaypointLatitudeReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Double)
getWaypointLatitudeReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Latitude" [makeArgument 0 thisArg]

getWaypointLatitude :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Double)
getWaypointLatitude thisArg = simpleRequest $ getWaypointLatitudeReq thisArg

getWaypointLatitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointLatitudeStreamReq thisArg = makeStreamReq $ getWaypointLatitudeReq thisArg

getWaypointLatitudeStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Double))
getWaypointLatitudeStream thisArg = requestAddStream $ getWaypointLatitudeStreamReq thisArg 

{-|
The longitude of the waypoint.
 -}
getWaypointLongitudeReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Double)
getWaypointLongitudeReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Longitude" [makeArgument 0 thisArg]

getWaypointLongitude :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Double)
getWaypointLongitude thisArg = simpleRequest $ getWaypointLongitudeReq thisArg

getWaypointLongitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointLongitudeStreamReq thisArg = makeStreamReq $ getWaypointLongitudeReq thisArg

getWaypointLongitudeStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Double))
getWaypointLongitudeStream thisArg = requestAddStream $ getWaypointLongitudeStreamReq thisArg 

{-|
The altitude of the waypoint above sea level, in meters.
 -}
getWaypointMeanAltitudeReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Double)
getWaypointMeanAltitudeReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_MeanAltitude" [makeArgument 0 thisArg]

getWaypointMeanAltitude :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Double)
getWaypointMeanAltitude thisArg = simpleRequest $ getWaypointMeanAltitudeReq thisArg

getWaypointMeanAltitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointMeanAltitudeStreamReq thisArg = makeStreamReq $ getWaypointMeanAltitudeReq thisArg

getWaypointMeanAltitudeStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Double))
getWaypointMeanAltitudeStream thisArg = requestAddStream $ getWaypointMeanAltitudeStreamReq thisArg 

{-|
Name of the waypoint as it appears on the map and the contract.
 -}
getWaypointNameReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Data.Text.Text)
getWaypointNameReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_Name" [makeArgument 0 thisArg]

getWaypointName :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Data.Text.Text)
getWaypointName thisArg = simpleRequest $ getWaypointNameReq thisArg

getWaypointNameStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Data.Text.Text)
getWaypointNameStreamReq thisArg = makeStreamReq $ getWaypointNameReq thisArg

getWaypointNameStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Data.Text.Text))
getWaypointNameStream thisArg = requestAddStream $ getWaypointNameStreamReq thisArg 

{-|
True if waypoint is a point near or on the body rather than high in orbit.
 -}
getWaypointNearSurfaceReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Bool)
getWaypointNearSurfaceReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_NearSurface" [makeArgument 0 thisArg]

getWaypointNearSurface :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Bool)
getWaypointNearSurface thisArg = simpleRequest $ getWaypointNearSurfaceReq thisArg

getWaypointNearSurfaceStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Bool)
getWaypointNearSurfaceStreamReq thisArg = makeStreamReq $ getWaypointNearSurfaceReq thisArg

getWaypointNearSurfaceStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Bool))
getWaypointNearSurfaceStream thisArg = requestAddStream $ getWaypointNearSurfaceStreamReq thisArg 

{-|
The altitude of the waypoint above the surface of the body or sea level, whichever is closer, in meters.
 -}
getWaypointSurfaceAltitudeReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCCallReq (Double)
getWaypointSurfaceAltitudeReq thisArg = makeCallReq "SpaceCenter" "Waypoint_get_SurfaceAltitude" [makeArgument 0 thisArg]

getWaypointSurfaceAltitude :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (Double)
getWaypointSurfaceAltitude thisArg = simpleRequest $ getWaypointSurfaceAltitudeReq thisArg

getWaypointSurfaceAltitudeStreamReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCStreamReq (Double)
getWaypointSurfaceAltitudeStreamReq thisArg = makeStreamReq $ getWaypointSurfaceAltitudeReq thisArg

getWaypointSurfaceAltitudeStream :: KRPCHS.SpaceCenter.Waypoint -> RPCContext (KRPCStream (Double))
getWaypointSurfaceAltitudeStream thisArg = requestAddStream $ getWaypointSurfaceAltitudeStreamReq thisArg 

{-|
The altitude of the waypoint above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 -}
setWaypointBedrockAltitudeReq :: KRPCHS.SpaceCenter.Waypoint -> Double -> KRPCCallReq ()
setWaypointBedrockAltitudeReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_BedrockAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointBedrockAltitude :: KRPCHS.SpaceCenter.Waypoint -> Double -> RPCContext ()
setWaypointBedrockAltitude thisArg valueArg = simpleRequest $ setWaypointBedrockAltitudeReq thisArg valueArg 

{-|
Celestial body the waypoint is attached to.
 -}
setWaypointBodyReq :: KRPCHS.SpaceCenter.Waypoint -> KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ()
setWaypointBodyReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_Body" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointBody :: KRPCHS.SpaceCenter.Waypoint -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setWaypointBody thisArg valueArg = simpleRequest $ setWaypointBodyReq thisArg valueArg 

{-|
The seed of the icon color. See <see cref="M:SpaceCenter.WaypointManager.Colors" /> for example colors.
 -}
setWaypointColorReq :: KRPCHS.SpaceCenter.Waypoint -> Data.Int.Int32 -> KRPCCallReq ()
setWaypointColorReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointColor :: KRPCHS.SpaceCenter.Waypoint -> Data.Int.Int32 -> RPCContext ()
setWaypointColor thisArg valueArg = simpleRequest $ setWaypointColorReq thisArg valueArg 

{-|
The icon of the waypoint.
 -}
setWaypointIconReq :: KRPCHS.SpaceCenter.Waypoint -> Data.Text.Text -> KRPCCallReq ()
setWaypointIconReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_Icon" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointIcon :: KRPCHS.SpaceCenter.Waypoint -> Data.Text.Text -> RPCContext ()
setWaypointIcon thisArg valueArg = simpleRequest $ setWaypointIconReq thisArg valueArg 

{-|
The latitude of the waypoint.
 -}
setWaypointLatitudeReq :: KRPCHS.SpaceCenter.Waypoint -> Double -> KRPCCallReq ()
setWaypointLatitudeReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_Latitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointLatitude :: KRPCHS.SpaceCenter.Waypoint -> Double -> RPCContext ()
setWaypointLatitude thisArg valueArg = simpleRequest $ setWaypointLatitudeReq thisArg valueArg 

{-|
The longitude of the waypoint.
 -}
setWaypointLongitudeReq :: KRPCHS.SpaceCenter.Waypoint -> Double -> KRPCCallReq ()
setWaypointLongitudeReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_Longitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointLongitude :: KRPCHS.SpaceCenter.Waypoint -> Double -> RPCContext ()
setWaypointLongitude thisArg valueArg = simpleRequest $ setWaypointLongitudeReq thisArg valueArg 

{-|
The altitude of the waypoint above sea level, in meters.
 -}
setWaypointMeanAltitudeReq :: KRPCHS.SpaceCenter.Waypoint -> Double -> KRPCCallReq ()
setWaypointMeanAltitudeReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_MeanAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointMeanAltitude :: KRPCHS.SpaceCenter.Waypoint -> Double -> RPCContext ()
setWaypointMeanAltitude thisArg valueArg = simpleRequest $ setWaypointMeanAltitudeReq thisArg valueArg 

{-|
Name of the waypoint as it appears on the map and the contract.
 -}
setWaypointNameReq :: KRPCHS.SpaceCenter.Waypoint -> Data.Text.Text -> KRPCCallReq ()
setWaypointNameReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointName :: KRPCHS.SpaceCenter.Waypoint -> Data.Text.Text -> RPCContext ()
setWaypointName thisArg valueArg = simpleRequest $ setWaypointNameReq thisArg valueArg 

{-|
The altitude of the waypoint above the surface of the body or sea level, whichever is closer, in meters.
 -}
setWaypointSurfaceAltitudeReq :: KRPCHS.SpaceCenter.Waypoint -> Double -> KRPCCallReq ()
setWaypointSurfaceAltitudeReq thisArg valueArg = makeCallReq "SpaceCenter" "Waypoint_set_SurfaceAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setWaypointSurfaceAltitude :: KRPCHS.SpaceCenter.Waypoint -> Double -> RPCContext ()
setWaypointSurfaceAltitude thisArg valueArg = simpleRequest $ setWaypointSurfaceAltitudeReq thisArg valueArg 

{-|
The currently active vessel.
 -}
getActiveVesselReq :: KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getActiveVesselReq  = makeCallReq "SpaceCenter" "get_ActiveVessel" []

getActiveVessel :: RPCContext (KRPCHS.SpaceCenter.Vessel)
getActiveVessel  = simpleRequest $ getActiveVesselReq 

getActiveVesselStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getActiveVesselStreamReq  = makeStreamReq $ getActiveVesselReq 

getActiveVesselStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getActiveVesselStream  = requestAddStream $ getActiveVesselStreamReq  

{-|
A dictionary of all celestial bodies (planets, moons, etc.) in the game,
keyed by the name of the body.
 -}
getBodiesReq :: KRPCCallReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodiesReq  = makeCallReq "SpaceCenter" "get_Bodies" []

getBodies :: RPCContext (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodies  = simpleRequest $ getBodiesReq 

getBodiesStreamReq :: KRPCStreamReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodiesStreamReq  = makeStreamReq $ getBodiesReq 

getBodiesStream :: RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody)))
getBodiesStream  = requestAddStream $ getBodiesStreamReq  

{-|
An object that can be used to control the camera.
 -}
getCameraReq :: KRPCCallReq (KRPCHS.SpaceCenter.Camera)
getCameraReq  = makeCallReq "SpaceCenter" "get_Camera" []

getCamera :: RPCContext (KRPCHS.SpaceCenter.Camera)
getCamera  = simpleRequest $ getCameraReq 

getCameraStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Camera)
getCameraStreamReq  = makeStreamReq $ getCameraReq 

getCameraStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Camera))
getCameraStream  = requestAddStream $ getCameraStreamReq  

{-|
Whether <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFARAvailableReq :: KRPCCallReq (Bool)
getFARAvailableReq  = makeCallReq "SpaceCenter" "get_FARAvailable" []

getFARAvailable :: RPCContext (Bool)
getFARAvailable  = simpleRequest $ getFARAvailableReq 

getFARAvailableStreamReq :: KRPCStreamReq (Bool)
getFARAvailableStreamReq  = makeStreamReq $ getFARAvailableReq 

getFARAvailableStream :: RPCContext (KRPCStream (Bool))
getFARAvailableStream  = requestAddStream $ getFARAvailableStreamReq  

{-|
The value of the <a href="https://en.wikipedia.org/wiki/Gravitational_constant">gravitational constantG inN(m/kg)^2.
 -}
getGReq :: KRPCCallReq (Float)
getGReq  = makeCallReq "SpaceCenter" "get_G" []

getG :: RPCContext (Float)
getG  = simpleRequest $ getGReq 

getGStreamReq :: KRPCStreamReq (Float)
getGStreamReq  = makeStreamReq $ getGReq 

getGStream :: RPCContext (KRPCStream (Float))
getGStream  = requestAddStream $ getGStreamReq  

{-|
The current maximum regular "on-rails" warp factor that can be set.
A value between 0 and 7 inclusive.  See
<a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">the KSP wikifor details.
 -}
getMaximumRailsWarpFactorReq :: KRPCCallReq (Data.Int.Int32)
getMaximumRailsWarpFactorReq  = makeCallReq "SpaceCenter" "get_MaximumRailsWarpFactor" []

getMaximumRailsWarpFactor :: RPCContext (Data.Int.Int32)
getMaximumRailsWarpFactor  = simpleRequest $ getMaximumRailsWarpFactorReq 

getMaximumRailsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getMaximumRailsWarpFactorStreamReq  = makeStreamReq $ getMaximumRailsWarpFactorReq 

getMaximumRailsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getMaximumRailsWarpFactorStream  = requestAddStream $ getMaximumRailsWarpFactorStreamReq  

{-|
The physical time warp rate. A value between 0 and 3 inclusive. 0 means
no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
getPhysicsWarpFactorReq :: KRPCCallReq (Data.Int.Int32)
getPhysicsWarpFactorReq  = makeCallReq "SpaceCenter" "get_PhysicsWarpFactor" []

getPhysicsWarpFactor :: RPCContext (Data.Int.Int32)
getPhysicsWarpFactor  = simpleRequest $ getPhysicsWarpFactorReq 

getPhysicsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getPhysicsWarpFactorStreamReq  = makeStreamReq $ getPhysicsWarpFactorReq 

getPhysicsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getPhysicsWarpFactorStream  = requestAddStream $ getPhysicsWarpFactorStreamReq  

{-|
The time warp rate, using regular "on-rails" time warp. A value between
0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
is active.

If requested time warp factor cannot be set, it will be set to the next
lowest possible value. For example, if the vessel is too close to a
planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
the KSP wikifor details.
 -}
getRailsWarpFactorReq :: KRPCCallReq (Data.Int.Int32)
getRailsWarpFactorReq  = makeCallReq "SpaceCenter" "get_RailsWarpFactor" []

getRailsWarpFactor :: RPCContext (Data.Int.Int32)
getRailsWarpFactor  = simpleRequest $ getRailsWarpFactorReq 

getRailsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getRailsWarpFactorStreamReq  = makeStreamReq $ getRailsWarpFactorReq 

getRailsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getRailsWarpFactorStream  = requestAddStream $ getRailsWarpFactorStreamReq  

{-|
The currently targeted celestial body.
 -}
getTargetBodyReq :: KRPCCallReq (KRPCHS.SpaceCenter.CelestialBody)
getTargetBodyReq  = makeCallReq "SpaceCenter" "get_TargetBody" []

getTargetBody :: RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getTargetBody  = simpleRequest $ getTargetBodyReq 

getTargetBodyStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getTargetBodyStreamReq  = makeStreamReq $ getTargetBodyReq 

getTargetBodyStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getTargetBodyStream  = requestAddStream $ getTargetBodyStreamReq  

{-|
The currently targeted docking port.
 -}
getTargetDockingPortReq :: KRPCCallReq (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPortReq  = makeCallReq "SpaceCenter" "get_TargetDockingPort" []

getTargetDockingPort :: RPCContext (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPort  = simpleRequest $ getTargetDockingPortReq 

getTargetDockingPortStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPortStreamReq  = makeStreamReq $ getTargetDockingPortReq 

getTargetDockingPortStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getTargetDockingPortStream  = requestAddStream $ getTargetDockingPortStreamReq  

{-|
The currently targeted vessel.
 -}
getTargetVesselReq :: KRPCCallReq (KRPCHS.SpaceCenter.Vessel)
getTargetVesselReq  = makeCallReq "SpaceCenter" "get_TargetVessel" []

getTargetVessel :: RPCContext (KRPCHS.SpaceCenter.Vessel)
getTargetVessel  = simpleRequest $ getTargetVesselReq 

getTargetVesselStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getTargetVesselStreamReq  = makeStreamReq $ getTargetVesselReq 

getTargetVesselStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getTargetVesselStream  = requestAddStream $ getTargetVesselStreamReq  

{-|
The current universal time in seconds.
 -}
getUTReq :: KRPCCallReq (Double)
getUTReq  = makeCallReq "SpaceCenter" "get_UT" []

getUT :: RPCContext (Double)
getUT  = simpleRequest $ getUTReq 

getUTStreamReq :: KRPCStreamReq (Double)
getUTStreamReq  = makeStreamReq $ getUTReq 

getUTStream :: RPCContext (KRPCStream (Double))
getUTStream  = requestAddStream $ getUTStreamReq  

{-|
A list of all the vessels in the game.
 -}
getVesselsReq :: KRPCCallReq ([KRPCHS.SpaceCenter.Vessel])
getVesselsReq  = makeCallReq "SpaceCenter" "get_Vessels" []

getVessels :: RPCContext ([KRPCHS.SpaceCenter.Vessel])
getVessels  = simpleRequest $ getVesselsReq 

getVesselsStreamReq :: KRPCStreamReq ([KRPCHS.SpaceCenter.Vessel])
getVesselsStreamReq  = makeStreamReq $ getVesselsReq 

getVesselsStream :: RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
getVesselsStream  = requestAddStream $ getVesselsStreamReq  

{-|
The current warp factor. This is the index of the rate at which time
is passing for either regular "on-rails" or physical time warp. Returns 0
if time warp is not active. When in on-rails time warp, this is equal to
<see cref="M:SpaceCenter.RailsWarpFactor" />, and in physics time warp, this is equal to
<see cref="M:SpaceCenter.PhysicsWarpFactor" />.
 -}
getWarpFactorReq :: KRPCCallReq (Float)
getWarpFactorReq  = makeCallReq "SpaceCenter" "get_WarpFactor" []

getWarpFactor :: RPCContext (Float)
getWarpFactor  = simpleRequest $ getWarpFactorReq 

getWarpFactorStreamReq :: KRPCStreamReq (Float)
getWarpFactorStreamReq  = makeStreamReq $ getWarpFactorReq 

getWarpFactorStream :: RPCContext (KRPCStream (Float))
getWarpFactorStream  = requestAddStream $ getWarpFactorStreamReq  

{-|
The current time warp mode. Returns <see cref="M:SpaceCenter.WarpMode.None" /> if time
warp is not active, <see cref="M:SpaceCenter.WarpMode.Rails" /> if regular "on-rails" time warp
is active, or <see cref="M:SpaceCenter.WarpMode.Physics" /> if physical time warp is active.
 -}
getWarpModeReq :: KRPCCallReq (KRPCHS.SpaceCenter.WarpMode)
getWarpModeReq  = makeCallReq "SpaceCenter" "get_WarpMode" []

getWarpMode :: RPCContext (KRPCHS.SpaceCenter.WarpMode)
getWarpMode  = simpleRequest $ getWarpModeReq 

getWarpModeStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.WarpMode)
getWarpModeStreamReq  = makeStreamReq $ getWarpModeReq 

getWarpModeStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.WarpMode))
getWarpModeStream  = requestAddStream $ getWarpModeStreamReq  

{-|
The current warp rate. This is the rate at which time is passing for
either on-rails or physical time warp. For example, a value of 10 means
time is passing 10x faster than normal. Returns 1 if time warp is not
active.
 -}
getWarpRateReq :: KRPCCallReq (Float)
getWarpRateReq  = makeCallReq "SpaceCenter" "get_WarpRate" []

getWarpRate :: RPCContext (Float)
getWarpRate  = simpleRequest $ getWarpRateReq 

getWarpRateStreamReq :: KRPCStreamReq (Float)
getWarpRateStreamReq  = makeStreamReq $ getWarpRateReq 

getWarpRateStream :: RPCContext (KRPCStream (Float))
getWarpRateStream  = requestAddStream $ getWarpRateStreamReq  

{-|
The waypoint manager.
 -}
getWaypointManagerReq :: KRPCCallReq (KRPCHS.SpaceCenter.WaypointManager)
getWaypointManagerReq  = makeCallReq "SpaceCenter" "get_WaypointManager" []

getWaypointManager :: RPCContext (KRPCHS.SpaceCenter.WaypointManager)
getWaypointManager  = simpleRequest $ getWaypointManagerReq 

getWaypointManagerStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.WaypointManager)
getWaypointManagerStreamReq  = makeStreamReq $ getWaypointManagerReq 

getWaypointManagerStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.WaypointManager))
getWaypointManagerStream  = requestAddStream $ getWaypointManagerStreamReq  

{-|
The currently active vessel.
 -}
setActiveVesselReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ()
setActiveVesselReq valueArg = makeCallReq "SpaceCenter" "set_ActiveVessel" [makeArgument 0 valueArg]

setActiveVessel :: KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setActiveVessel valueArg = simpleRequest $ setActiveVesselReq valueArg 

{-|
The physical time warp rate. A value between 0 and 3 inclusive. 0 means
no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
setPhysicsWarpFactorReq :: Data.Int.Int32 -> KRPCCallReq ()
setPhysicsWarpFactorReq valueArg = makeCallReq "SpaceCenter" "set_PhysicsWarpFactor" [makeArgument 0 valueArg]

setPhysicsWarpFactor :: Data.Int.Int32 -> RPCContext ()
setPhysicsWarpFactor valueArg = simpleRequest $ setPhysicsWarpFactorReq valueArg 

{-|
The time warp rate, using regular "on-rails" time warp. A value between
0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
is active.

If requested time warp factor cannot be set, it will be set to the next
lowest possible value. For example, if the vessel is too close to a
planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
the KSP wikifor details.
 -}
setRailsWarpFactorReq :: Data.Int.Int32 -> KRPCCallReq ()
setRailsWarpFactorReq valueArg = makeCallReq "SpaceCenter" "set_RailsWarpFactor" [makeArgument 0 valueArg]

setRailsWarpFactor :: Data.Int.Int32 -> RPCContext ()
setRailsWarpFactor valueArg = simpleRequest $ setRailsWarpFactorReq valueArg 

{-|
The currently targeted celestial body.
 -}
setTargetBodyReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCCallReq ()
setTargetBodyReq valueArg = makeCallReq "SpaceCenter" "set_TargetBody" [makeArgument 0 valueArg]

setTargetBody :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setTargetBody valueArg = simpleRequest $ setTargetBodyReq valueArg 

{-|
The currently targeted docking port.
 -}
setTargetDockingPortReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCCallReq ()
setTargetDockingPortReq valueArg = makeCallReq "SpaceCenter" "set_TargetDockingPort" [makeArgument 0 valueArg]

setTargetDockingPort :: KRPCHS.SpaceCenter.DockingPort -> RPCContext ()
setTargetDockingPort valueArg = simpleRequest $ setTargetDockingPortReq valueArg 

{-|
The currently targeted vessel.
 -}
setTargetVesselReq :: KRPCHS.SpaceCenter.Vessel -> KRPCCallReq ()
setTargetVesselReq valueArg = makeCallReq "SpaceCenter" "set_TargetVessel" [makeArgument 0 valueArg]

setTargetVessel :: KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setTargetVessel valueArg = simpleRequest $ setTargetVesselReq valueArg 

