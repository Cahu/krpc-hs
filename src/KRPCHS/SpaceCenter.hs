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
, Sensor
, SolarPanel
, Thruster
, Vessel
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
, getCelestialBodyEquatorialRadius
, getCelestialBodyEquatorialRadiusStream
, getCelestialBodyEquatorialRadiusStreamReq
, getCelestialBodyGravitationalParameter
, getCelestialBodyGravitationalParameterStream
, getCelestialBodyGravitationalParameterStreamReq
, getCelestialBodyHasAtmosphere
, getCelestialBodyHasAtmosphereStream
, getCelestialBodyHasAtmosphereStreamReq
, getCelestialBodyHasAtmosphericOxygen
, getCelestialBodyHasAtmosphericOxygenStream
, getCelestialBodyHasAtmosphericOxygenStreamReq
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
, getCelestialBodyRotationalPeriod
, getCelestialBodyRotationalPeriodStream
, getCelestialBodyRotationalPeriodStreamReq
, getCelestialBodyRotationalSpeed
, getCelestialBodyRotationalSpeedStream
, getCelestialBodyRotationalSpeedStreamReq
, getCelestialBodySatellites
, getCelestialBodySatellitesStream
, getCelestialBodySatellitesStreamReq
, getCelestialBodySphereOfInfluence
, getCelestialBodySphereOfInfluenceStream
, getCelestialBodySphereOfInfluenceStreamReq
, getCelestialBodySurfaceGravity
, getCelestialBodySurfaceGravityStream
, getCelestialBodySurfaceGravityStreamReq
, clearTarget
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
, getControlBrakes
, getControlBrakesStream
, getControlBrakesStreamReq
, getControlCurrentStage
, getControlCurrentStageStream
, getControlCurrentStageStreamReq
, getControlForward
, getControlForwardStream
, getControlForwardStreamReq
, getControlGear
, getControlGearStream
, getControlGearStreamReq
, getControlLights
, getControlLightsStream
, getControlLightsStreamReq
, getControlNodes
, getControlNodesStream
, getControlNodesStreamReq
, getControlPitch
, getControlPitchStream
, getControlPitchStreamReq
, getControlRCS
, getControlRCSStream
, getControlRCSStreamReq
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
, getControlSpeedMode
, getControlSpeedModeStream
, getControlSpeedModeStreamReq
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
, getControlYaw
, getControlYawStream
, getControlYawStreamReq
, setControlAbort
, setControlBrakes
, setControlForward
, setControlGear
, setControlLights
, setControlPitch
, setControlRCS
, setControlRight
, setControlRoll
, setControlSAS
, setControlSASMode
, setControlSpeedMode
, setControlThrottle
, setControlUp
, setControlWheelSteering
, setControlWheelThrottle
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
, getDockingPortName
, getDockingPortNameStream
, getDockingPortNameStreamReq
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
, setDockingPortName
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
, getFlightVelocity
, getFlightVelocityStream
, getFlightVelocityStreamReq
, getFlightVerticalSpeed
, getFlightVerticalSpeedStream
, getFlightVerticalSpeedStreamReq
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
, getLandingGearDeployable
, getLandingGearDeployableStream
, getLandingGearDeployableStreamReq
, getLandingGearDeployed
, getLandingGearDeployedStream
, getLandingGearDeployedStreamReq
, getLandingGearPart
, getLandingGearPartStream
, getLandingGearPartStreamReq
, getLandingGearState
, getLandingGearStateStream
, getLandingGearStateStreamReq
, setLandingGearDeployed
, getLandingLegDeployed
, getLandingLegDeployedStream
, getLandingLegDeployedStreamReq
, getLandingLegPart
, getLandingLegPartStream
, getLandingLegPartStreamReq
, getLandingLegState
, getLandingLegStateStream
, getLandingLegStateStreamReq
, setLandingLegDeployed
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
, orbitReferencePlaneDirection
, orbitReferencePlaneDirectionStream
, orbitReferencePlaneDirectionStreamReq
, orbitReferencePlaneNormal
, orbitReferencePlaneNormalStream
, orbitReferencePlaneNormalStreamReq
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
, parachuteDeploy
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
, partCenterOfMass
, partCenterOfMassStream
, partCenterOfMassStreamReq
, partDirection
, partDirectionStream
, partDirectionStreamReq
, partPosition
, partPositionStream
, partPositionStreamReq
, partRotation
, partRotationStream
, partRotationStreamReq
, partVelocity
, partVelocityStream
, partVelocityStreamReq
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
, getPartLandingGear
, getPartLandingGearStream
, getPartLandingGearStreamReq
, getPartLandingLeg
, getPartLandingLegStream
, getPartLandingLegStreamReq
, getPartLaunchClamp
, getPartLaunchClampStream
, getPartLaunchClampStreamReq
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
, partsDockingPortWithName
, partsDockingPortWithNameStream
, partsDockingPortWithNameStreamReq
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
, partsWithTitle
, partsWithTitleStream
, partsWithTitleStreamReq
, getPartsAll
, getPartsAllStream
, getPartsAllStreamReq
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
, getPartsLandingGear
, getPartsLandingGearStream
, getPartsLandingGearStreamReq
, getPartsLandingLegs
, getPartsLandingLegsStream
, getPartsLandingLegsStreamReq
, getPartsLaunchClamps
, getPartsLaunchClampsStream
, getPartsLaunchClampsStreamReq
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
, setPartsControlling
, getPropellantConnectedResources
, getPropellantConnectedResourcesStream
, getPropellantConnectedResourcesStreamReq
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
, getSensorActive
, getSensorActiveStream
, getSensorActiveStreamReq
, getSensorPart
, getSensorPartStream
, getSensorPartStreamReq
, getSensorPowerUsage
, getSensorPowerUsageStream
, getSensorPowerUsageStreamReq
, getSensorValue
, getSensorValueStream
, getSensorValueStreamReq
, setSensorActive
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
, getActiveVessel
, getActiveVesselStream
, getActiveVesselStreamReq
, getBodies
, getBodiesStream
, getBodiesStreamReq
, getCamera
, getCameraStream
, getCameraStreamReq
, getFARAvailable
, getFARAvailableStream
, getFARAvailableStreamReq
, getG
, getGStream
, getGStreamReq
, getMaximumRailsWarpFactor
, getMaximumRailsWarpFactorStream
, getMaximumRailsWarpFactorStreamReq
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
, setActiveVessel
, setPhysicsWarpFactor
, setRailsWarpFactor
, setTargetBody
, setTargetDockingPort
, setTargetVessel
) where

import qualified Data.Int
import qualified Data.Map
import qualified Data.Text
import qualified Data.Word

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


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
 - An air intake. Obtained by calling <see cref="M:SpaceCenter.Part.Intake" />.
 -}
newtype Intake = Intake { intakeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Intake where
    encodePb   = encodePb . intakeId
    decodePb b = Intake <$> decodePb b

instance KRPCResponseExtractable Intake

{-
 - Landing gear with wheels. Obtained by calling <see cref="M:SpaceCenter.Part.LandingGear" />.
 -}
newtype LandingGear = LandingGear { landingGearId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LandingGear where
    encodePb   = encodePb . landingGearId
    decodePb b = LandingGear <$> decodePb b

instance KRPCResponseExtractable LandingGear

{-
 - A landing leg. Obtained by calling <see cref="M:SpaceCenter.Part.LandingLeg" />.
 -}
newtype LandingLeg = LandingLeg { landingLegId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LandingLeg where
    encodePb   = encodePb . landingLegId
    decodePb b = LandingLeg <$> decodePb b

instance KRPCResponseExtractable LandingLeg

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
 - The state of a landing gear. See <see cref="M:SpaceCenter.LandingGear.State" />.
 -}
data LandingGearState
    = LandingGearState'Deployed
    | LandingGearState'Retracted
    | LandingGearState'Deploying
    | LandingGearState'Retracting
    | LandingGearState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable LandingGearState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable LandingGearState

{-
 - The state of a landing leg. See <see cref="M:SpaceCenter.LandingLeg.State" />.
 -}
data LandingLegState
    = LandingLegState'Deployed
    | LandingLegState'Retracted
    | LandingLegState'Deploying
    | LandingLegState'Retracting
    | LandingLegState'Broken
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable LandingLegState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable LandingLegState

{-
 - The state of a parachute. See <see cref="M:SpaceCenter.Parachute.State" />.
 -}
data ParachuteState
    = ParachuteState'Active
    | ParachuteState'Cut
    | ParachuteState'Deployed
    | ParachuteState'SemiDeployed
    | ParachuteState'Stowed
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
 - Disengage the auto-pilot.
 -}
autoPilotDisengage :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ()
autoPilotDisengage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Disengage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Engage the auto-pilot.
 -}
autoPilotEngage :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ()
autoPilotEngage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Engage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set target pitch and heading angles.<param name="pitch">Target pitch angle, in degrees between -90° and +90°.<param name="heading">Target heading angle, in degrees between 0° and 360°.
 -}
autoPilotTargetPitchAndHeading :: KRPCHS.SpaceCenter.AutoPilot -> Float -> Float -> RPCContext ()
autoPilotTargetPitchAndHeading thisArg pitchArg headingArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_TargetPitchAndHeading" [makeArgument 0 thisArg, makeArgument 1 pitchArg, makeArgument 2 headingArg]
    res <- sendRequest r
    processResponse res 

{-
 - Blocks until the vessel is pointing in the target direction and has the target roll (if set).
 -}
autoPilotWait :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ()
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
getAutoPilotAttenuationAngle :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotAttenuationAngle thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_AttenuationAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotAttenuationAngleStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotAttenuationAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_AttenuationAngle" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotAttenuationAngleStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotAttenuationAngleStream thisArg = requestStream $ getAutoPilotAttenuationAngleStreamReq thisArg 

{-
 - Whether the rotation rate controllers PID parameters should be automatically tuned using the
 - vessels moment of inertia and available torque. Defaults totrue.
 - See <see cref="M:SpaceCenter.AutoPilot.TimeToPeak" /> and  <see cref="M:SpaceCenter.AutoPilot.Overshoot" />.
 -}
getAutoPilotAutoTune :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
getAutoPilotAutoTune thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_AutoTune" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotAutoTuneStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Bool)
getAutoPilotAutoTuneStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_AutoTune" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotAutoTuneStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Bool))
getAutoPilotAutoTuneStream thisArg = requestStream $ getAutoPilotAutoTuneStreamReq thisArg 

{-
 - The time the vessel should take to come to a stop pointing in the target direction.
 - This determines the angular acceleration used to decelerate the vessel.
 - A vector of three times, in seconds, one for each of the pitch, roll and yaw axes.
 - Defaults to 5 seconds for each axis.
 -}
getAutoPilotDecelerationTime :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotDecelerationTime thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_DecelerationTime" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotDecelerationTimeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotDecelerationTimeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_DecelerationTime" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotDecelerationTimeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotDecelerationTimeStream thisArg = requestStream $ getAutoPilotDecelerationTimeStreamReq thisArg 

{-
 - The error, in degrees, between the direction the ship has been asked
 - to point in and the direction it is pointing in. Returns zero if the auto-pilot
 - has not been engaged and SAS is not enabled or is in stability assist mode.
 -}
getAutoPilotError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotErrorStream thisArg = requestStream $ getAutoPilotErrorStreamReq thisArg 

{-
 - The error, in degrees, between the vessels current and target heading.
 - Returns zero if the auto-pilot has not been engaged.
 -}
getAutoPilotHeadingError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotHeadingError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_HeadingError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotHeadingErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotHeadingErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_HeadingError" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotHeadingErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotHeadingErrorStream thisArg = requestStream $ getAutoPilotHeadingErrorStreamReq thisArg 

{-
 - The target overshoot percentage used to autotune the PID controllers.
 - A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
 - Defaults to 0.01 for each axis.
 -}
getAutoPilotOvershoot :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotOvershoot thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Overshoot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotOvershootStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotOvershootStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_Overshoot" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotOvershootStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotOvershootStream thisArg = requestStream $ getAutoPilotOvershootStreamReq thisArg 

{-
 - The error, in degrees, between the vessels current and target pitch.
 - Returns zero if the auto-pilot has not been engaged.
 -}
getAutoPilotPitchError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotPitchError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_PitchError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotPitchErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotPitchErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_PitchError" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotPitchErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotPitchErrorStream thisArg = requestStream $ getAutoPilotPitchErrorStreamReq thisArg 

{-
 - Gains for the pitch PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotPitchPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotPitchPIDGains thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_PitchPIDGains" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotPitchPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotPitchPIDGainsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_PitchPIDGains" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotPitchPIDGainsStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotPitchPIDGainsStream thisArg = requestStream $ getAutoPilotPitchPIDGainsStreamReq thisArg 

{-
 - The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).
 -}
getAutoPilotReferenceFrame :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotReferenceFrameStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotReferenceFrameStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getAutoPilotReferenceFrameStream thisArg = requestStream $ getAutoPilotReferenceFrameStreamReq thisArg 

{-
 - The error, in degrees, between the vessels current and target roll.
 - Returns zero if the auto-pilot has not been engaged or no target roll is set.
 -}
getAutoPilotRollError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotRollError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotRollErrorStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotRollErrorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotRollErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRollErrorStream thisArg = requestStream $ getAutoPilotRollErrorStreamReq thisArg 

{-
 - Gains for the roll PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotRollPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotRollPIDGains thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollPIDGains" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotRollPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotRollPIDGainsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_RollPIDGains" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotRollPIDGainsStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotRollPIDGainsStream thisArg = requestStream $ getAutoPilotRollPIDGainsStreamReq thisArg 

{-
 - The threshold at which the autopilot will try to match the target roll angle, if any.
 - Defaults to 5 degrees.
 -}
getAutoPilotRollThreshold :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Double)
getAutoPilotRollThreshold thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollThreshold" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotRollThresholdStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Double)
getAutoPilotRollThresholdStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_RollThreshold" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotRollThresholdStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Double))
getAutoPilotRollThresholdStream thisArg = requestStream $ getAutoPilotRollThresholdStreamReq thisArg 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
getAutoPilotSAS :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
getAutoPilotSAS thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotSASStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Bool)
getAutoPilotSASStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotSASStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Bool))
getAutoPilotSASStream thisArg = requestStream $ getAutoPilotSASStreamReq thisArg 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
getAutoPilotSASMode :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotSASModeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotSASModeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getAutoPilotSASModeStream thisArg = requestStream $ getAutoPilotSASModeStreamReq thisArg 

{-
 - The maximum amount of time that the vessel should need to come to a complete stop.
 - This determines the maximum angular velocity of the vessel.
 - A vector of three stopping times, in seconds, one for each of the pitch, roll and yaw axes.
 - Defaults to 0.5 seconds for each axis.
 -}
getAutoPilotStoppingTime :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotStoppingTime thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_StoppingTime" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotStoppingTimeStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotStoppingTimeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_StoppingTime" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotStoppingTimeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotStoppingTimeStream thisArg = requestStream $ getAutoPilotStoppingTimeStreamReq thisArg 

{-
 - Direction vector corresponding to the target pitch and heading.
 -}
getAutoPilotTargetDirection :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotTargetDirection thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetDirectionStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotTargetDirectionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetDirectionStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotTargetDirectionStream thisArg = requestStream $ getAutoPilotTargetDirectionStreamReq thisArg 

{-
 - The target heading, in degrees, between 0° and 360°.
 -}
getAutoPilotTargetHeading :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetHeading thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetHeading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetHeadingStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetHeadingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetHeading" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetHeadingStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetHeadingStream thisArg = requestStream $ getAutoPilotTargetHeadingStreamReq thisArg 

{-
 - The target pitch, in degrees, between -90° and +90°.
 -}
getAutoPilotTargetPitch :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetPitch thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetPitchStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetPitch" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetPitchStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetPitchStream thisArg = requestStream $ getAutoPilotTargetPitchStreamReq thisArg 

{-
 - The target roll, in degrees.NaNif no target roll is set.
 -}
getAutoPilotTargetRoll :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetRoll thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTargetRollStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq (Float)
getAutoPilotTargetRollStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTargetRollStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetRollStream thisArg = requestStream $ getAutoPilotTargetRollStreamReq thisArg 

{-
 - The target time to peak used to autotune the PID controllers.
 - A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
 - Defaults to 3 seconds for each axis.
 -}
getAutoPilotTimeToPeak :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotTimeToPeak thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TimeToPeak" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotTimeToPeakStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotTimeToPeakStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_TimeToPeak" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotTimeToPeakStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotTimeToPeakStream thisArg = requestStream $ getAutoPilotTimeToPeakStreamReq thisArg 

{-
 - Gains for the yaw PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
getAutoPilotYawPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotYawPIDGains thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_YawPIDGains" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getAutoPilotYawPIDGainsStreamReq :: KRPCHS.SpaceCenter.AutoPilot -> KRPCStreamReq ((Double, Double, Double))
getAutoPilotYawPIDGainsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "AutoPilot_get_YawPIDGains" [makeArgument 0 thisArg]
    in  makeStream req

getAutoPilotYawPIDGainsStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotYawPIDGainsStream thisArg = requestStream $ getAutoPilotYawPIDGainsStreamReq thisArg 

{-
 - The angle at which the autopilot considers the vessel to be pointing close to the target.
 - This determines the midpoint of the target velocity attenuation function.
 - A vector of three angles, in degrees, one for each of the pitch, roll and yaw axes.
 - Defaults to 1° for each axis.
 -}
setAutoPilotAttenuationAngle :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotAttenuationAngle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_AttenuationAngle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the rotation rate controllers PID parameters should be automatically tuned using the
 - vessels moment of inertia and available torque. Defaults totrue.
 - See <see cref="M:SpaceCenter.AutoPilot.TimeToPeak" /> and  <see cref="M:SpaceCenter.AutoPilot.Overshoot" />.
 -}
setAutoPilotAutoTune :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> RPCContext ()
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
setAutoPilotDecelerationTime :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotDecelerationTime thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_DecelerationTime" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target overshoot percentage used to autotune the PID controllers.
 - A vector of three values, between 0 and 1, for each of the pitch, roll and yaw axes.
 - Defaults to 0.01 for each axis.
 -}
setAutoPilotOvershoot :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotOvershoot thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_Overshoot" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Gains for the pitch PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotPitchPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotPitchPIDGains thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_PitchPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).
 -}
setAutoPilotReferenceFrame :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setAutoPilotReferenceFrame thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Gains for the roll PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotRollPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotRollPIDGains thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RollPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The threshold at which the autopilot will try to match the target roll angle, if any.
 - Defaults to 5 degrees.
 -}
setAutoPilotRollThreshold :: KRPCHS.SpaceCenter.AutoPilot -> Double -> RPCContext ()
setAutoPilotRollThreshold thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RollThreshold" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
setAutoPilotSAS :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> RPCContext ()
setAutoPilotSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
setAutoPilotSASMode :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.SASMode -> RPCContext ()
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
setAutoPilotStoppingTime :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotStoppingTime thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_StoppingTime" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Direction vector corresponding to the target pitch and heading.
 -}
setAutoPilotTargetDirection :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotTargetDirection thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetDirection" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target heading, in degrees, between 0° and 360°.
 -}
setAutoPilotTargetHeading :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext ()
setAutoPilotTargetHeading thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetHeading" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target pitch, in degrees, between -90° and +90°.
 -}
setAutoPilotTargetPitch :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext ()
setAutoPilotTargetPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetPitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target roll, in degrees.NaNif no target roll is set.
 -}
setAutoPilotTargetRoll :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext ()
setAutoPilotTargetRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetRoll" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The target time to peak used to autotune the PID controllers.
 - A vector of three times, in seconds, for each of the pitch, roll and yaw axes.
 - Defaults to 3 seconds for each axis.
 -}
setAutoPilotTimeToPeak :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotTimeToPeak thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TimeToPeak" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Gains for the yaw PID controller.When <see cref="M:SpaceCenter.AutoPilot.AutoTune" /> is true, these values are updated automatically, which will overwrite any manual changes.
 -}
setAutoPilotYawPIDGains :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext ()
setAutoPilotYawPIDGains thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_YawPIDGains" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Default distance from the camera to the subject.
 -}
getCameraDefaultDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraDefaultDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraDefaultDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraDefaultDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraDefaultDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraDefaultDistanceStream thisArg = requestStream $ getCameraDefaultDistanceStreamReq thisArg 

{-
 - The distance from the camera to the subject.
 - A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
getCameraDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraDistanceStream thisArg = requestStream $ getCameraDistanceStreamReq thisArg 

{-
 - In map mode, the celestial body that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a celestial body.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedBody :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBody thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraFocussedBodyStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBodyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]
    in  makeStream req

getCameraFocussedBodyStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getCameraFocussedBodyStream thisArg = requestStream $ getCameraFocussedBodyStreamReq thisArg 

{-
 - In map mode, the maneuver node that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a maneuver node.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedNode :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.Node)
getCameraFocussedNode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraFocussedNodeStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.Node)
getCameraFocussedNodeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]
    in  makeStream req

getCameraFocussedNodeStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Node))
getCameraFocussedNodeStream thisArg = requestStream $ getCameraFocussedNodeStreamReq thisArg 

{-
 - In map mode, the vessel that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a vessel.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedVessel :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraFocussedVesselStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVesselStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]
    in  makeStream req

getCameraFocussedVesselStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCameraFocussedVesselStream thisArg = requestStream $ getCameraFocussedVesselStreamReq thisArg 

{-
 - The heading of the camera, in degrees.
 -}
getCameraHeading :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraHeadingStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraHeadingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]
    in  makeStream req

getCameraHeadingStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraHeadingStream thisArg = requestStream $ getCameraHeadingStreamReq thisArg 

{-
 - Maximum distance from the camera to the subject.
 -}
getCameraMaxDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMaxDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMaxDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMaxDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMaxDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMaxDistanceStream thisArg = requestStream $ getCameraMaxDistanceStreamReq thisArg 

{-
 - The maximum pitch of the camera.
 -}
getCameraMaxPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMaxPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMaxPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMaxPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMaxPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMaxPitchStream thisArg = requestStream $ getCameraMaxPitchStreamReq thisArg 

{-
 - Minimum distance from the camera to the subject.
 -}
getCameraMinDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMinDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMinDistanceStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMinDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMinDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMinDistanceStream thisArg = requestStream $ getCameraMinDistanceStreamReq thisArg 

{-
 - The minimum pitch of the camera.
 -}
getCameraMinPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMinPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraMinPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraMinPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]
    in  makeStream req

getCameraMinPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMinPitchStream thisArg = requestStream $ getCameraMinPitchStreamReq thisArg 

{-
 - The current mode of the camera.
 -}
getCameraMode :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.CameraMode)
getCameraMode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraModeStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (KRPCHS.SpaceCenter.CameraMode)
getCameraModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]
    in  makeStream req

getCameraModeStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CameraMode))
getCameraModeStream thisArg = requestStream $ getCameraModeStreamReq thisArg 

{-
 - The pitch of the camera, in degrees.
 - A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
getCameraPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCameraPitchStreamReq :: KRPCHS.SpaceCenter.Camera -> KRPCStreamReq (Float)
getCameraPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]
    in  makeStream req

getCameraPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraPitchStream thisArg = requestStream $ getCameraPitchStreamReq thisArg 

{-
 - The distance from the camera to the subject.
 - A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
setCameraDistance :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext ()
setCameraDistance thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Distance" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - In map mode, the celestial body that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a celestial body.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedBody :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setCameraFocussedBody thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - In map mode, the maneuver node that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a maneuver node.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedNode :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Node -> RPCContext ()
setCameraFocussedNode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedNode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - In map mode, the vessel that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a vessel.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedVessel :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setCameraFocussedVessel thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The heading of the camera, in degrees.
 -}
setCameraHeading :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext ()
setCameraHeading thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Heading" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current mode of the camera.
 -}
setCameraMode :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CameraMode -> RPCContext ()
setCameraMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The pitch of the camera, in degrees.
 - A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
setCameraPitch :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext ()
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
canRailsWarpAt :: Data.Int.Int32 -> RPCContext (Bool)
canRailsWarpAt factorArg = do
    let r = makeRequest "SpaceCenter" "CanRailsWarpAt" [makeArgument 0 factorArg]
    res <- sendRequest r
    processResponse res

canRailsWarpAtStreamReq :: Data.Int.Int32 -> KRPCStreamReq (Bool)
canRailsWarpAtStreamReq factorArg =
    let req = makeRequest "SpaceCenter" "CanRailsWarpAt" [makeArgument 0 factorArg]
    in  makeStream req

canRailsWarpAtStream :: Data.Int.Int32 -> RPCContext (KRPCStream (Bool))
canRailsWarpAtStream factorArg = requestStream $ canRailsWarpAtStreamReq factorArg 

{-
 - Whether the cargo bay is open.
 -}
getCargoBayOpen :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (Bool)
getCargoBayOpen thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCargoBayOpenStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (Bool)
getCargoBayOpenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]
    in  makeStream req

getCargoBayOpenStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (Bool))
getCargoBayOpenStream thisArg = requestStream $ getCargoBayOpenStreamReq thisArg 

{-
 - The part object for this cargo bay.
 -}
getCargoBayPart :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCHS.SpaceCenter.Part)
getCargoBayPart thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCargoBayPartStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getCargoBayPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getCargoBayPartStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getCargoBayPartStream thisArg = requestStream $ getCargoBayPartStreamReq thisArg 

{-
 - The state of the cargo bay.
 -}
getCargoBayState :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayState thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCargoBayStateStreamReq :: KRPCHS.SpaceCenter.CargoBay -> KRPCStreamReq (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getCargoBayStateStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CargoBayState))
getCargoBayStateStream thisArg = requestStream $ getCargoBayStateStreamReq thisArg 

{-
 - Whether the cargo bay is open.
 -}
setCargoBayOpen :: KRPCHS.SpaceCenter.CargoBay -> Bool -> RPCContext ()
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
celestialBodyAngularVelocity :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyAngularVelocityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyAngularVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyAngularVelocityStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyAngularVelocityStream thisArg referenceFrameArg = requestStream $ celestialBodyAngularVelocityStreamReq thisArg referenceFrameArg 

{-
 - The height of the surface relative to mean sea level at the given position,
 - in meters. When over water, this is the height of the sea-bed and is therefore a
 - negative value.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodyBedrockHeight :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodyBedrockHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse res

celestialBodyBedrockHeightStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Double)
celestialBodyBedrockHeightStreamReq thisArg latitudeArg longitudeArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    in  makeStream req

celestialBodyBedrockHeightStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodyBedrockHeightStream thisArg latitudeArg longitudeArg = requestStream $ celestialBodyBedrockHeightStreamReq thisArg latitudeArg longitudeArg 

{-
 - The position of the surface at the given latitude and longitude, in the given
 - reference frame. When over water, this is the position at the bottom of the sea-bed.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyBedrockPosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyBedrockPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyBedrockPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyBedrockPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

celestialBodyBedrockPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyBedrockPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestStream $ celestialBodyBedrockPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-
 - Returns the direction in which the north pole of the celestial body is
 - pointing, as a unit vector, in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyDirection :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyDirectionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyDirectionStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyDirectionStream thisArg referenceFrameArg = requestStream $ celestialBodyDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position at mean sea level at the given latitude and longitude, in the given reference frame.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyMSLPosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyMSLPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyMSLPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyMSLPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

celestialBodyMSLPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyMSLPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestStream $ celestialBodyMSLPositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-
 - Returns the position vector of the center of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyPosition :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyPositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyPositionStream thisArg referenceFrameArg = requestStream $ celestialBodyPositionStreamReq thisArg referenceFrameArg 

{-
 - Returns the rotation of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyRotation :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
celestialBodyRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyRotationStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
celestialBodyRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyRotationStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
celestialBodyRotationStream thisArg referenceFrameArg = requestStream $ celestialBodyRotationStreamReq thisArg referenceFrameArg 

{-
 - The height of the surface relative to mean sea level at the given position,
 - in meters. When over water this is equal to 0.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodySurfaceHeight :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodySurfaceHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse res

celestialBodySurfaceHeightStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCStreamReq (Double)
celestialBodySurfaceHeightStreamReq thisArg latitudeArg longitudeArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    in  makeStream req

celestialBodySurfaceHeightStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodySurfaceHeightStream thisArg latitudeArg longitudeArg = requestStream $ celestialBodySurfaceHeightStreamReq thisArg latitudeArg longitudeArg 

{-
 - The position of the surface at the given latitude and longitude, in the given
 - reference frame. When over water, this is the position of the surface of the water.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodySurfacePosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodySurfacePosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodySurfacePositionStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodySurfacePositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    in  makeStream req

celestialBodySurfacePositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodySurfacePositionStream thisArg latitudeArg longitudeArg referenceFrameArg = requestStream $ celestialBodySurfacePositionStreamReq thisArg latitudeArg longitudeArg referenceFrameArg 

{-
 - Returns the velocity vector of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyVelocity :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

celestialBodyVelocityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
celestialBodyVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

celestialBodyVelocityStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyVelocityStream thisArg referenceFrameArg = requestStream $ celestialBodyVelocityStreamReq thisArg referenceFrameArg 

{-
 - The depth of the atmosphere, in meters.
 -}
getCelestialBodyAtmosphereDepth :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyAtmosphereDepth thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyAtmosphereDepthStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyAtmosphereDepthStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyAtmosphereDepthStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyAtmosphereDepthStream thisArg = requestStream $ getCelestialBodyAtmosphereDepthStreamReq thisArg 

{-
 - The equatorial radius of the body, in meters.
 -}
getCelestialBodyEquatorialRadius :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyEquatorialRadius thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyEquatorialRadiusStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyEquatorialRadiusStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyEquatorialRadiusStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyEquatorialRadiusStream thisArg = requestStream $ getCelestialBodyEquatorialRadiusStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Standard_gravitational_parameter">standard
 - gravitational parameterof the body inm^3s^{ -2}.
 -}
getCelestialBodyGravitationalParameter :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyGravitationalParameter thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyGravitationalParameterStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyGravitationalParameterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyGravitationalParameterStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyGravitationalParameterStream thisArg = requestStream $ getCelestialBodyGravitationalParameterStreamReq thisArg 

{-
 - trueif the body has an atmosphere.
 -}
getCelestialBodyHasAtmosphere :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphere thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyHasAtmosphereStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Bool)
getCelestialBodyHasAtmosphereStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyHasAtmosphereStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphereStream thisArg = requestStream $ getCelestialBodyHasAtmosphereStreamReq thisArg 

{-
 - trueif there is oxygen in the atmosphere, required for air-breathing engines.
 -}
getCelestialBodyHasAtmosphericOxygen :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphericOxygen thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyHasAtmosphericOxygenStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Bool)
getCelestialBodyHasAtmosphericOxygenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyHasAtmosphericOxygenStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphericOxygenStream thisArg = requestStream $ getCelestialBodyHasAtmosphericOxygenStreamReq thisArg 

{-
 - The mass of the body, in kilograms.
 -}
getCelestialBodyMass :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyMass thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyMassStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyMassStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyMassStream thisArg = requestStream $ getCelestialBodyMassStreamReq thisArg 

{-
 - The name of the body.
 -}
getCelestialBodyName :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Data.Text.Text)
getCelestialBodyName thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyNameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Data.Text.Text)
getCelestialBodyNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyNameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Data.Text.Text))
getCelestialBodyNameStream thisArg = requestStream $ getCelestialBodyNameStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this celestial body, and
 - orientated in a fixed direction (it does not rotate with the body).
 - <list type="bullet">The origin is at the center of the body.The axes do not rotate.The x-axis points in an arbitrary direction through the
 - equator.The y-axis points from the center of the body towards
 - the north pole.The z-axis points in an arbitrary direction through the
 - equator.
 -}
getCelestialBodyNonRotatingReferenceFrame :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyNonRotatingReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyNonRotatingReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyNonRotatingReferenceFrameStream thisArg = requestStream $ getCelestialBodyNonRotatingReferenceFrameStreamReq thisArg 

{-
 - The orbit of the body.
 -}
getCelestialBodyOrbit :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyOrbitStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyOrbitStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getCelestialBodyOrbitStream thisArg = requestStream $ getCelestialBodyOrbitStreamReq thisArg 

{-
 - Gets the reference frame that is fixed relative to this celestial body, but
 - orientated with the body's orbital prograde/normal/radial directions.
 - <list type="bullet">The origin is at the center of the body.The axes rotate with the orbital prograde/normal/radial
 - directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.
 -}
getCelestialBodyOrbitalReferenceFrame :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyOrbitalReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyOrbitalReferenceFrameStream thisArg = requestStream $ getCelestialBodyOrbitalReferenceFrameStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the celestial body.
 - <list type="bullet">The origin is at the center of the body.The axes rotate with the body.The x-axis points from the center of the body
 - towards the intersection of the prime meridian and equator (the
 - position at 0° longitude, 0° latitude).The y-axis points from the center of the body
 - towards the north pole.The z-axis points from the center of the body
 - towards the equator at 90°E longitude.
 -}
getCelestialBodyReferenceFrame :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyReferenceFrameStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getCelestialBodyReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyReferenceFrameStream thisArg = requestStream $ getCelestialBodyReferenceFrameStreamReq thisArg 

{-
 - The sidereal rotational period of the body, in seconds.
 -}
getCelestialBodyRotationalPeriod :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyRotationalPeriodStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyRotationalPeriodStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyRotationalPeriodStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalPeriodStream thisArg = requestStream $ getCelestialBodyRotationalPeriodStreamReq thisArg 

{-
 - The rotational speed of the body, in radians per second.
 -}
getCelestialBodyRotationalSpeed :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodyRotationalSpeedStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodyRotationalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodyRotationalSpeedStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalSpeedStream thisArg = requestStream $ getCelestialBodyRotationalSpeedStreamReq thisArg 

{-
 - A list of celestial bodies that are in orbit around this celestial body.
 -}
getCelestialBodySatellites :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellites thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySatellitesStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellitesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySatellitesStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.CelestialBody]))
getCelestialBodySatellitesStream thisArg = requestStream $ getCelestialBodySatellitesStreamReq thisArg 

{-
 - The radius of the sphere of influence of the body, in meters.
 -}
getCelestialBodySphereOfInfluence :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySphereOfInfluence thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySphereOfInfluenceStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySphereOfInfluenceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySphereOfInfluenceStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySphereOfInfluenceStream thisArg = requestStream $ getCelestialBodySphereOfInfluenceStreamReq thisArg 

{-
 - The acceleration due to gravity at sea level (mean altitude) on the body, inm/s^2.
 -}
getCelestialBodySurfaceGravity :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySurfaceGravity thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCelestialBodySurfaceGravityStreamReq :: KRPCHS.SpaceCenter.CelestialBody -> KRPCStreamReq (Float)
getCelestialBodySurfaceGravityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]
    in  makeStream req

getCelestialBodySurfaceGravityStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySurfaceGravityStream thisArg = requestStream $ getCelestialBodySurfaceGravityStreamReq thisArg 

{-
 - Clears the current target.
 -}
clearTarget :: RPCContext ()
clearTarget  = do
    let r = makeRequest "SpaceCenter" "ClearTarget" []
    res <- sendRequest r
    processResponse res 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getControlSurfaceAvailableTorque :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext ((Double, Double, Double))
getControlSurfaceAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq ((Double, Double, Double))
getControlSurfaceAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceAvailableTorqueStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream ((Double, Double, Double)))
getControlSurfaceAvailableTorqueStream thisArg = requestStream $ getControlSurfaceAvailableTorqueStreamReq thisArg 

{-
 - Whether the control surface has been fully deployed.
 -}
getControlSurfaceDeployed :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceDeployedStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceDeployedStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceDeployedStream thisArg = requestStream $ getControlSurfaceDeployedStreamReq thisArg 

{-
 - Whether the control surface movement is inverted.
 -}
getControlSurfaceInverted :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceInverted thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceInvertedStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceInvertedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceInvertedStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceInvertedStream thisArg = requestStream $ getControlSurfaceInvertedStreamReq thisArg 

{-
 - The part object for this control surface.
 -}
getControlSurfacePart :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCHS.SpaceCenter.Part)
getControlSurfacePart thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfacePartStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getControlSurfacePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfacePartStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getControlSurfacePartStream thisArg = requestStream $ getControlSurfacePartStreamReq thisArg 

{-
 - Whether the control surface has pitch control enabled.
 -}
getControlSurfacePitchEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfacePitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfacePitchEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfacePitchEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfacePitchEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfacePitchEnabledStream thisArg = requestStream $ getControlSurfacePitchEnabledStreamReq thisArg 

{-
 - Whether the control surface has roll control enabled.
 -}
getControlSurfaceRollEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceRollEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceRollEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceRollEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceRollEnabledStream thisArg = requestStream $ getControlSurfaceRollEnabledStreamReq thisArg 

{-
 - Surface area of the control surface inm^2.
 -}
getControlSurfaceSurfaceArea :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Float)
getControlSurfaceSurfaceArea thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceSurfaceAreaStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Float)
getControlSurfaceSurfaceAreaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceSurfaceAreaStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Float))
getControlSurfaceSurfaceAreaStream thisArg = requestStream $ getControlSurfaceSurfaceAreaStreamReq thisArg 

{-
 - Whether the control surface has yaw control enabled.
 -}
getControlSurfaceYawEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSurfaceYawEnabledStreamReq :: KRPCHS.SpaceCenter.ControlSurface -> KRPCStreamReq (Bool)
getControlSurfaceYawEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getControlSurfaceYawEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceYawEnabledStream thisArg = requestStream $ getControlSurfaceYawEnabledStreamReq thisArg 

{-
 - Whether the control surface has been fully deployed.
 -}
setControlSurfaceDeployed :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface movement is inverted.
 -}
setControlSurfaceInverted :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceInverted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Inverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has pitch control enabled.
 -}
setControlSurfacePitchEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfacePitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has roll control enabled.
 -}
setControlSurfaceRollEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the control surface has yaw control enabled.
 -}
setControlSurfaceYawEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext ()
setControlSurfaceYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Activates the next stage. Equivalent to pressing the space bar in-game.A list of vessel objects that are jettisoned from the active vessel.
 -}
controlActivateNextStage :: KRPCHS.SpaceCenter.Control -> RPCContext ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

controlActivateNextStageStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]
    in  makeStream req

controlActivateNextStageStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
controlActivateNextStageStream thisArg = requestStream $ controlActivateNextStageStreamReq thisArg 

{-
 - Creates a maneuver node at the given universal time, and returns a
 - <see cref="T:SpaceCenter.Node" /> object that can be used to modify it.
 - Optionally sets the magnitude of the delta-v for the maneuver node
 - in the prograde, normal and radial directions.<param name="ut">Universal time of the maneuver node.<param name="prograde">Delta-v in the prograde direction.<param name="normal">Delta-v in the normal direction.<param name="radial">Delta-v in the radial direction.
 -}
controlAddNode :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> RPCContext (KRPCHS.SpaceCenter.Node)
controlAddNode thisArg utArg progradeArg normalArg radialArg = do
    let r = makeRequest "SpaceCenter" "Control_AddNode" [makeArgument 0 thisArg, makeArgument 1 utArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg]
    res <- sendRequest r
    processResponse res

controlAddNodeStreamReq :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> KRPCStreamReq (KRPCHS.SpaceCenter.Node)
controlAddNodeStreamReq thisArg utArg progradeArg normalArg radialArg =
    let req = makeRequest "SpaceCenter" "Control_AddNode" [makeArgument 0 thisArg, makeArgument 1 utArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg]
    in  makeStream req

controlAddNodeStream :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Node))
controlAddNodeStream thisArg utArg progradeArg normalArg radialArg = requestStream $ controlAddNodeStreamReq thisArg utArg progradeArg normalArg radialArg 

{-
 - Returnstrueif the given action group is enabled.<param name="group">A number between 0 and 9 inclusive.
 -}
controlGetActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (Bool)
controlGetActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    res <- sendRequest r
    processResponse res

controlGetActionGroupStreamReq :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> KRPCStreamReq (Bool)
controlGetActionGroupStreamReq thisArg groupArg =
    let req = makeRequest "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    in  makeStream req

controlGetActionGroupStream :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (KRPCStream (Bool))
controlGetActionGroupStream thisArg groupArg = requestStream $ controlGetActionGroupStreamReq thisArg groupArg 

{-
 - Remove all maneuver nodes.
 -}
controlRemoveNodes :: KRPCHS.SpaceCenter.Control -> RPCContext ()
controlRemoveNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_RemoveNodes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Sets the state of the given action group (a value between 0 and 9
 - inclusive).<param name="group">A number between 0 and 9 inclusive.<param name="state">
 -}
controlSetActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> Bool -> RPCContext ()
controlSetActionGroup thisArg groupArg stateArg = do
    let r = makeRequest "SpaceCenter" "Control_SetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg, makeArgument 2 stateArg]
    res <- sendRequest r
    processResponse res 

{-
 - Toggles the state of the given action group.<param name="group">A number between 0 and 9 inclusive.
 -}
controlToggleActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext ()
controlToggleActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_ToggleActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the abort action group.
 -}
getControlAbort :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlAbort thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlAbortStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlAbortStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]
    in  makeStream req

getControlAbortStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlAbortStream thisArg = requestStream $ getControlAbortStreamReq thisArg 

{-
 - The state of the wheel brakes.
 -}
getControlBrakes :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlBrakes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlBrakesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlBrakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]
    in  makeStream req

getControlBrakesStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlBrakesStream thisArg = requestStream $ getControlBrakesStreamReq thisArg 

{-
 - The current stage of the vessel. Corresponds to the stage number in
 - the in-game UI.
 -}
getControlCurrentStage :: KRPCHS.SpaceCenter.Control -> RPCContext (Data.Int.Int32)
getControlCurrentStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlCurrentStageStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Data.Int.Int32)
getControlCurrentStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]
    in  makeStream req

getControlCurrentStageStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Data.Int.Int32))
getControlCurrentStageStream thisArg = requestStream $ getControlCurrentStageStreamReq thisArg 

{-
 - The state of the forward translational control.
 - A value between -1 and 1.
 - Equivalent to the h and n keys.
 -}
getControlForward :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlForward thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlForwardStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlForwardStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]
    in  makeStream req

getControlForwardStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlForwardStream thisArg = requestStream $ getControlForwardStreamReq thisArg 

{-
 - The state of the landing gear/legs.
 -}
getControlGear :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlGear thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlGearStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlGearStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]
    in  makeStream req

getControlGearStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlGearStream thisArg = requestStream $ getControlGearStreamReq thisArg 

{-
 - The state of the lights.
 -}
getControlLights :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlLights thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlLightsStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlLightsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]
    in  makeStream req

getControlLightsStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlLightsStream thisArg = requestStream $ getControlLightsStreamReq thisArg 

{-
 - Returns a list of all existing maneuver nodes, ordered by time from first to last.
 -}
getControlNodes :: KRPCHS.SpaceCenter.Control -> RPCContext ([KRPCHS.SpaceCenter.Node])
getControlNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlNodesStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq ([KRPCHS.SpaceCenter.Node])
getControlNodesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]
    in  makeStream req

getControlNodesStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Node]))
getControlNodesStream thisArg = requestStream $ getControlNodesStreamReq thisArg 

{-
 - The state of the pitch control.
 - A value between -1 and 1.
 - Equivalent to the w and s keys.
 -}
getControlPitch :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlPitchStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]
    in  makeStream req

getControlPitchStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlPitchStream thisArg = requestStream $ getControlPitchStreamReq thisArg 

{-
 - The state of RCS.
 -}
getControlRCS :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRCSStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlRCSStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]
    in  makeStream req

getControlRCSStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlRCSStream thisArg = requestStream $ getControlRCSStreamReq thisArg 

{-
 - The state of the right translational control.
 - A value between -1 and 1.
 - Equivalent to the j and l keys.
 -}
getControlRight :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlRight thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRightStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlRightStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]
    in  makeStream req

getControlRightStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlRightStream thisArg = requestStream $ getControlRightStreamReq thisArg 

{-
 - The state of the roll control.
 - A value between -1 and 1.
 - Equivalent to the q and e keys.
 -}
getControlRoll :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlRollStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlRollStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]
    in  makeStream req

getControlRollStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlRollStream thisArg = requestStream $ getControlRollStreamReq thisArg 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
getControlSAS :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlSAS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSASStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Bool)
getControlSASStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]
    in  makeStream req

getControlSASStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlSASStream thisArg = requestStream $ getControlSASStreamReq thisArg 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
getControlSASMode :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCHS.SpaceCenter.SASMode)
getControlSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSASModeStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.SASMode)
getControlSASModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]
    in  makeStream req

getControlSASModeStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getControlSASModeStream thisArg = requestStream $ getControlSASModeStreamReq thisArg 

{-
 - The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
 - This is the mode displayed next to the speed at the top of the navball.
 -}
getControlSpeedMode :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlSpeedModeStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]
    in  makeStream req

getControlSpeedModeStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SpeedMode))
getControlSpeedModeStream thisArg = requestStream $ getControlSpeedModeStreamReq thisArg 

{-
 - The state of the throttle. A value between 0 and 1.
 -}
getControlThrottle :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlThrottleStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlThrottleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]
    in  makeStream req

getControlThrottleStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlThrottleStream thisArg = requestStream $ getControlThrottleStreamReq thisArg 

{-
 - The state of the up translational control.
 - A value between -1 and 1.
 - Equivalent to the i and k keys.
 -}
getControlUp :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlUp thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlUpStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlUpStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]
    in  makeStream req

getControlUpStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlUpStream thisArg = requestStream $ getControlUpStreamReq thisArg 

{-
 - The state of the wheel steering.
 - A value between -1 and 1.
 - A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
getControlWheelSteering :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlWheelSteering thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlWheelSteeringStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlWheelSteeringStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]
    in  makeStream req

getControlWheelSteeringStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlWheelSteeringStream thisArg = requestStream $ getControlWheelSteeringStreamReq thisArg 

{-
 - The state of the wheel throttle.
 - A value between -1 and 1.
 - A value of 1 rotates the wheels forwards, a value of -1 rotates
 - the wheels backwards.
 -}
getControlWheelThrottle :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlWheelThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlWheelThrottleStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlWheelThrottleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [makeArgument 0 thisArg]
    in  makeStream req

getControlWheelThrottleStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlWheelThrottleStream thisArg = requestStream $ getControlWheelThrottleStreamReq thisArg 

{-
 - The state of the yaw control.
 - A value between -1 and 1.
 - Equivalent to the a and d keys.
 -}
getControlYaw :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlYaw thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getControlYawStreamReq :: KRPCHS.SpaceCenter.Control -> KRPCStreamReq (Float)
getControlYawStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]
    in  makeStream req

getControlYawStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlYawStream thisArg = requestStream $ getControlYawStreamReq thisArg 

{-
 - The state of the abort action group.
 -}
setControlAbort :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlAbort thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Abort" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the wheel brakes.
 -}
setControlBrakes :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlBrakes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Brakes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the forward translational control.
 - A value between -1 and 1.
 - Equivalent to the h and n keys.
 -}
setControlForward :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlForward thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Forward" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the landing gear/legs.
 -}
setControlGear :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlGear thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Gear" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the lights.
 -}
setControlLights :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlLights thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Lights" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the pitch control.
 - A value between -1 and 1.
 - Equivalent to the w and s keys.
 -}
setControlPitch :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of RCS.
 -}
setControlRCS :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlRCS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_RCS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the right translational control.
 - A value between -1 and 1.
 - Equivalent to the j and l keys.
 -}
setControlRight :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlRight thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Right" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the roll control.
 - A value between -1 and 1.
 - Equivalent to the q and e keys.
 -}
setControlRoll :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Roll" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
setControlSAS :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext ()
setControlSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
setControlSASMode :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SASMode -> RPCContext ()
setControlSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
 - This is the mode displayed next to the speed at the top of the navball.
 -}
setControlSpeedMode :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SpeedMode -> RPCContext ()
setControlSpeedMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SpeedMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the throttle. A value between 0 and 1.
 -}
setControlThrottle :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Throttle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the up translational control.
 - A value between -1 and 1.
 - Equivalent to the i and k keys.
 -}
setControlUp :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlUp thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Up" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the wheel steering.
 - A value between -1 and 1.
 - A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
setControlWheelSteering :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
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
setControlWheelThrottle :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlWheelThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelThrottle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the yaw control.
 - A value between -1 and 1.
 - Equivalent to the a and d keys.
 -}
setControlYaw :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext ()
setControlYaw thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Yaw" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Fires the decoupler. Returns the new vessel created when the decoupler fires.
 - Throws an exception if the decoupler has already fired.
 -}
decouplerDecouple :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCHS.SpaceCenter.Vessel)
decouplerDecouple thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

decouplerDecoupleStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
decouplerDecoupleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]
    in  makeStream req

decouplerDecoupleStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
decouplerDecoupleStream thisArg = requestStream $ decouplerDecoupleStreamReq thisArg 

{-
 - Whether the decoupler has fired.
 -}
getDecouplerDecoupled :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Bool)
getDecouplerDecoupled thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerDecoupledStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Bool)
getDecouplerDecoupledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerDecoupledStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Bool))
getDecouplerDecoupledStream thisArg = requestStream $ getDecouplerDecoupledStreamReq thisArg 

{-
 - The impulse that the decoupler imparts when it is fired, in Newton seconds.
 -}
getDecouplerImpulse :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Float)
getDecouplerImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerImpulseStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (Float)
getDecouplerImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerImpulseStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Float))
getDecouplerImpulseStream thisArg = requestStream $ getDecouplerImpulseStreamReq thisArg 

{-
 - The part object for this decoupler.
 -}
getDecouplerPart :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCHS.SpaceCenter.Part)
getDecouplerPart thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDecouplerPartStreamReq :: KRPCHS.SpaceCenter.Decoupler -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDecouplerPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getDecouplerPartStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDecouplerPartStream thisArg = requestStream $ getDecouplerPartStreamReq thisArg 

{-
 - The direction that docking port points in, in the given reference frame.
 -}
dockingPortDirection :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

dockingPortDirectionStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
dockingPortDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

dockingPortDirectionStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortDirectionStream thisArg referenceFrameArg = requestStream $ dockingPortDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position of the docking port in the given reference frame.
 -}
dockingPortPosition :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

dockingPortPositionStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
dockingPortPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

dockingPortPositionStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortPositionStream thisArg referenceFrameArg = requestStream $ dockingPortPositionStreamReq thisArg referenceFrameArg 

{-
 - The rotation of the docking port, in the given reference frame.
 -}
dockingPortRotation :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
dockingPortRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

dockingPortRotationStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
dockingPortRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

dockingPortRotationStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
dockingPortRotationStream thisArg referenceFrameArg = requestStream $ dockingPortRotationStreamReq thisArg referenceFrameArg 

{-
 - Undocks the docking port and returns the new <see cref="T:SpaceCenter.Vessel" /> that is created.
 - This method can be called for either docking port in a docked pair.
 - Throws an exception if the docking port is not docked to anything.After undocking, the active vessel may change. See <see cref="M:SpaceCenter.ActiveVessel" />.
 -}
dockingPortUndock :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Vessel)
dockingPortUndock thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

dockingPortUndockStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
dockingPortUndockStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]
    in  makeStream req

dockingPortUndockStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
dockingPortUndockStream thisArg = requestStream $ dockingPortUndockStreamReq thisArg 

{-
 - The part that this docking port is docked to. Returnsnullif this
 - docking port is not docked to anything.
 -}
getDockingPortDockedPart :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortDockedPartStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortDockedPartStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortDockedPartStream thisArg = requestStream $ getDockingPortDockedPartStreamReq thisArg 

{-
 - Whether the docking port has a shield.
 -}
getDockingPortHasShield :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Bool)
getDockingPortHasShield thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortHasShieldStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Bool)
getDockingPortHasShieldStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortHasShieldStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortHasShieldStream thisArg = requestStream $ getDockingPortHasShieldStreamReq thisArg 

{-
 - The port name of the docking port. This is the name of the port that can be set
 - in the right click menu, when the
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/40423-11-docking-port-alignment-indicator-version-621-beta-updated-04122016/">Docking Port Alignment Indicatormod is installed. If this mod is not installed, returns the title of the part
 - (<see cref="M:SpaceCenter.Part.Title" />).
 -}
getDockingPortName :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Data.Text.Text)
getDockingPortName thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortNameStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Data.Text.Text)
getDockingPortNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortNameStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Data.Text.Text))
getDockingPortNameStream thisArg = requestStream $ getDockingPortNameStreamReq thisArg 

{-
 - The part object for this docking port.
 -}
getDockingPortPart :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Part)
getDockingPortPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortPartStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getDockingPortPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortPartStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortPartStream thisArg = requestStream $ getDockingPortPartStreamReq thisArg 

{-
 - The distance a docking port must move away when it undocks before it
 - becomes ready to dock with another port, in meters.
 -}
getDockingPortReengageDistance :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Float)
getDockingPortReengageDistance thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortReengageDistanceStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Float)
getDockingPortReengageDistanceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortReengageDistanceStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Float))
getDockingPortReengageDistanceStream thisArg = requestStream $ getDockingPortReengageDistanceStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this docking port, and
 - oriented with the port.
 - <list type="bullet">The origin is at the position of the docking port.The axes rotate with the docking port.The x-axis points out to the right side of the docking port.The y-axis points in the direction the docking port is facing.The z-axis points out of the bottom off the docking port.This reference frame is not necessarily equivalent to the reference frame
 - for the part, returned by <see cref="M:SpaceCenter.Part.ReferenceFrame" />.
 -}
getDockingPortReferenceFrame :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortReferenceFrameStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getDockingPortReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortReferenceFrameStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getDockingPortReferenceFrameStream thisArg = requestStream $ getDockingPortReferenceFrameStreamReq thisArg 

{-
 - The state of the docking ports shield, if it has one.
 - Returnstrueif the docking port has a shield, and the shield is
 - closed. Otherwise returnsfalse. When set totrue, the shield is
 - closed, and when set tofalsethe shield is opened. If the docking
 - port does not have a shield, setting this attribute has no effect.
 -}
getDockingPortShielded :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Bool)
getDockingPortShielded thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortShieldedStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (Bool)
getDockingPortShieldedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortShieldedStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortShieldedStream thisArg = requestStream $ getDockingPortShieldedStreamReq thisArg 

{-
 - The current state of the docking port.
 -}
getDockingPortState :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortState thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getDockingPortStateStreamReq :: KRPCHS.SpaceCenter.DockingPort -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getDockingPortStateStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPortState))
getDockingPortStateStream thisArg = requestStream $ getDockingPortStateStreamReq thisArg 

{-
 - The port name of the docking port. This is the name of the port that can be set
 - in the right click menu, when the
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/40423-11-docking-port-alignment-indicator-version-621-beta-updated-04122016/">Docking Port Alignment Indicatormod is installed. If this mod is not installed, returns the title of the part
 - (<see cref="M:SpaceCenter.Part.Title" />).
 -}
setDockingPortName :: KRPCHS.SpaceCenter.DockingPort -> Data.Text.Text -> RPCContext ()
setDockingPortName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the docking ports shield, if it has one.
 - Returnstrueif the docking port has a shield, and the shield is
 - closed. Otherwise returnsfalse. When set totrue, the shield is
 - closed, and when set tofalsethe shield is opened. If the docking
 - port does not have a shield, setting this attribute has no effect.
 -}
setDockingPortShielded :: KRPCHS.SpaceCenter.DockingPort -> Bool -> RPCContext ()
setDockingPortShielded thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Shielded" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Toggle the current engine mode.
 -}
engineToggleMode :: KRPCHS.SpaceCenter.Engine -> RPCContext ()
engineToggleMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_ToggleMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the engine is active. Setting this attribute may have no effect,
 - depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
getEngineActive :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineActive thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineActiveStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getEngineActiveStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineActiveStream thisArg = requestStream $ getEngineActiveStreamReq thisArg 

{-
 - Whether the engine will automatically switch modes.
 -}
getEngineAutoModeSwitch :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineAutoModeSwitch thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineAutoModeSwitchStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineAutoModeSwitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]
    in  makeStream req

getEngineAutoModeSwitchStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineAutoModeSwitchStream thisArg = requestStream $ getEngineAutoModeSwitchStreamReq thisArg 

{-
 - The amount of thrust, in Newtons, that would be produced by the engine
 - when activated and with its throttle set to 100%.
 - Returns zero if the engine does not have any fuel.
 - Takes the engine's current <see cref="M:SpaceCenter.Engine.ThrustLimit" /> and atmospheric conditions into account.
 -}
getEngineAvailableThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineAvailableThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineAvailableThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineAvailableThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineAvailableThrustStream thisArg = requestStream $ getEngineAvailableThrustStreamReq thisArg 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the engine is inactive, or not gimballed.
 -}
getEngineAvailableTorque :: KRPCHS.SpaceCenter.Engine -> RPCContext ((Double, Double, Double))
getEngineAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ((Double, Double, Double))
getEngineAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getEngineAvailableTorqueStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ((Double, Double, Double)))
getEngineAvailableTorqueStream thisArg = requestStream $ getEngineAvailableTorqueStreamReq thisArg 

{-
 - Whether the engine can be restarted once shutdown. If the engine cannot be shutdown,
 - returnsfalse. For example, this istruefor liquid fueled rockets
 - andfalsefor solid rocket boosters.
 -}
getEngineCanRestart :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineCanRestart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineCanRestartStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineCanRestartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]
    in  makeStream req

getEngineCanRestartStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineCanRestartStream thisArg = requestStream $ getEngineCanRestartStreamReq thisArg 

{-
 - Whether the engine can be shutdown once activated. For example, this istruefor liquid fueled rockets andfalsefor solid rocket boosters.
 -}
getEngineCanShutdown :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineCanShutdown thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineCanShutdownStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineCanShutdownStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]
    in  makeStream req

getEngineCanShutdownStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineCanShutdownStream thisArg = requestStream $ getEngineCanShutdownStreamReq thisArg 

{-
 - The gimbal limiter of the engine. A value between 0 and 1.
 - Returns 0 if the gimbal is locked.
 -}
getEngineGimbalLimit :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineGimbalLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimbalLimitStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineGimbalLimitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimbalLimitStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalLimitStream thisArg = requestStream $ getEngineGimbalLimitStreamReq thisArg 

{-
 - Whether the engines gimbal is locked in place. Setting this attribute has
 - no effect if the engine is not gimballed.
 -}
getEngineGimbalLocked :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineGimbalLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimbalLockedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineGimbalLockedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimbalLockedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineGimbalLockedStream thisArg = requestStream $ getEngineGimbalLockedStreamReq thisArg 

{-
 - The range over which the gimbal can move, in degrees.
 - Returns 0 if the engine is not gimballed.
 -}
getEngineGimbalRange :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineGimbalRange thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimbalRangeStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineGimbalRangeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimbalRangeStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalRangeStream thisArg = requestStream $ getEngineGimbalRangeStreamReq thisArg 

{-
 - Whether the engine is gimballed.
 -}
getEngineGimballed :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineGimballedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineGimballedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]
    in  makeStream req

getEngineGimballedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineGimballedStream thisArg = requestStream $ getEngineGimballedStreamReq thisArg 

{-
 - Whether the engine has any fuel available.The engine must be activated for this property to update correctly.
 -}
getEngineHasFuel :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineHasFuelStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineHasFuelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]
    in  makeStream req

getEngineHasFuelStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineHasFuelStream thisArg = requestStream $ getEngineHasFuelStreamReq thisArg 

{-
 - Whether the engine has multiple modes of operation.
 -}
getEngineHasModes :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineHasModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineHasModesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineHasModesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]
    in  makeStream req

getEngineHasModesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineHasModesStream thisArg = requestStream $ getEngineHasModesStreamReq thisArg 

{-
 - The specific impulse of the engine at sea level on Kerbin, in seconds.
 -}
getEngineKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineKerbinSeaLevelSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getEngineKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineKerbinSeaLevelSpecificImpulseStream thisArg = requestStream $ getEngineKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-
 - The amount of thrust, in Newtons, that would be produced by the engine
 - when activated and fueled, with its throttle and throttle limiter set to 100%.
 -}
getEngineMaxThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineMaxThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineMaxThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineMaxThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineMaxThrustStream thisArg = requestStream $ getEngineMaxThrustStreamReq thisArg 

{-
 - The maximum amount of thrust that can be produced by the engine in a
 - vacuum, in Newtons. This is the amount of thrust produced by the engine
 - when activated, <see cref="M:SpaceCenter.Engine.ThrustLimit" /> is set to 100%, the main
 - vessel's throttle is set to 100% and the engine is in a vacuum.
 -}
getEngineMaxVacuumThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineMaxVacuumThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineMaxVacuumThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineMaxVacuumThrustStream thisArg = requestStream $ getEngineMaxVacuumThrustStreamReq thisArg 

{-
 - The name of the current engine mode.
 -}
getEngineMode :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Text.Text)
getEngineMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineModeStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Text.Text)
getEngineModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]
    in  makeStream req

getEngineModeStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Text.Text))
getEngineModeStream thisArg = requestStream $ getEngineModeStreamReq thisArg 

{-
 - The available modes for the engine.
 - A dictionary mapping mode names to <see cref="T:SpaceCenter.Engine" /> objects.
 -}
getEngineModes :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineModesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]
    in  makeStream req

getEngineModesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine)))
getEngineModesStream thisArg = requestStream $ getEngineModesStreamReq thisArg 

{-
 - The part object for this engine.
 -}
getEnginePart :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCHS.SpaceCenter.Part)
getEnginePart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePartStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getEnginePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePartStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getEnginePartStream thisArg = requestStream $ getEnginePartStreamReq thisArg 

{-
 - The names of the propellants that the engine consumes.
 -}
getEnginePropellantNames :: KRPCHS.SpaceCenter.Engine -> RPCContext ([Data.Text.Text])
getEnginePropellantNames thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantNames" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePropellantNamesStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([Data.Text.Text])
getEnginePropellantNamesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_PropellantNames" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePropellantNamesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([Data.Text.Text]))
getEnginePropellantNamesStream thisArg = requestStream $ getEnginePropellantNamesStreamReq thisArg 

{-
 - The ratio of resources that the engine consumes. A dictionary mapping resource names
 - to the ratio at which they are consumed by the engine.For example, if the ratios are 0.6 for LiquidFuel and 0.4 for Oxidizer, then for every 0.6 units of
 - LiquidFuel that the engine burns, it will burn 0.4 units of Oxidizer.
 -}
getEnginePropellantRatios :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePropellantRatiosStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatiosStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePropellantRatiosStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getEnginePropellantRatiosStream thisArg = requestStream $ getEnginePropellantRatiosStreamReq thisArg 

{-
 - The propellants that the engine consumes.
 -}
getEnginePropellants :: KRPCHS.SpaceCenter.Engine -> RPCContext ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellants thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEnginePropellantsStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([KRPCHS.SpaceCenter.Propellant])
getEnginePropellantsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]
    in  makeStream req

getEnginePropellantsStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Propellant]))
getEnginePropellantsStream thisArg = requestStream $ getEnginePropellantsStreamReq thisArg 

{-
 - The current specific impulse of the engine, in seconds. Returns zero
 - if the engine is not active.
 -}
getEngineSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getEngineSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineSpecificImpulseStream thisArg = requestStream $ getEngineSpecificImpulseStreamReq thisArg 

{-
 - The current throttle setting for the engine. A value between 0 and 1.
 - This is not necessarily the same as the vessel's main throttle
 - setting, as some engines take time to adjust their throttle
 - (such as jet engines).
 -}
getEngineThrottle :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Throttle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrottleStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrottleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Throttle" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrottleStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrottleStream thisArg = requestStream $ getEngineThrottleStreamReq thisArg 

{-
 - Whether the <see cref="M:SpaceCenter.Control.Throttle" /> affects the engine. For example,
 - this istruefor liquid fueled rockets, andfalsefor solid rocket
 - boosters.
 -}
getEngineThrottleLocked :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineThrottleLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrottleLockedStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Bool)
getEngineThrottleLockedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrottleLockedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineThrottleLockedStream thisArg = requestStream $ getEngineThrottleLockedStreamReq thisArg 

{-
 - The current amount of thrust being produced by the engine, in Newtons.
 -}
getEngineThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrustStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrustStream thisArg = requestStream $ getEngineThrustStreamReq thisArg 

{-
 - The thrust limiter of the engine. A value between 0 and 1. Setting this
 - attribute may have no effect, for example the thrust limit for a solid
 - rocket booster cannot be changed in flight.
 -}
getEngineThrustLimit :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrustLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrustLimitStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineThrustLimitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrustLimitStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrustLimitStream thisArg = requestStream $ getEngineThrustLimitStreamReq thisArg 

{-
 - The components of the engine that generate thrust.For example, this corresponds to the rocket nozzel on a solid rocket booster,
 - or the individual nozzels on a RAPIER engine.
 - The overall thrust produced by the engine, as reported by <see cref="M:SpaceCenter.Engine.AvailableThrust" />,
 - <see cref="M:SpaceCenter.Engine.MaxThrust" /> and others, is the sum of the thrust generated by each thruster.
 -}
getEngineThrusters :: KRPCHS.SpaceCenter.Engine -> RPCContext ([KRPCHS.SpaceCenter.Thruster])
getEngineThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrusters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineThrustersStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq ([KRPCHS.SpaceCenter.Thruster])
getEngineThrustersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_Thrusters" [makeArgument 0 thisArg]
    in  makeStream req

getEngineThrustersStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getEngineThrustersStream thisArg = requestStream $ getEngineThrustersStreamReq thisArg 

{-
 - The vacuum specific impulse of the engine, in seconds.
 -}
getEngineVacuumSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getEngineVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Engine -> KRPCStreamReq (Float)
getEngineVacuumSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getEngineVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineVacuumSpecificImpulseStream thisArg = requestStream $ getEngineVacuumSpecificImpulseStreamReq thisArg 

{-
 - Whether the engine is active. Setting this attribute may have no effect,
 - depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
setEngineActive :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext ()
setEngineActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the engine will automatically switch modes.
 -}
setEngineAutoModeSwitch :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext ()
setEngineAutoModeSwitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_AutoModeSwitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The gimbal limiter of the engine. A value between 0 and 1.
 - Returns 0 if the gimbal is locked.
 -}
setEngineGimbalLimit :: KRPCHS.SpaceCenter.Engine -> Float -> RPCContext ()
setEngineGimbalLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the engines gimbal is locked in place. Setting this attribute has
 - no effect if the engine is not gimballed.
 -}
setEngineGimbalLocked :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext ()
setEngineGimbalLocked thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The name of the current engine mode.
 -}
setEngineMode :: KRPCHS.SpaceCenter.Engine -> Data.Text.Text -> RPCContext ()
setEngineMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The thrust limiter of the engine. A value between 0 and 1. Setting this
 - attribute may have no effect, for example the thrust limit for a solid
 - rocket booster cannot be changed in flight.
 -}
setEngineThrustLimit :: KRPCHS.SpaceCenter.Engine -> Float -> RPCContext ()
setEngineThrustLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_ThrustLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Dump the experimental data contained by the experiment.
 -}
experimentDump :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentDump thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Dump" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reset the experiment.
 -}
experimentReset :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentReset thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Reset" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Run the experiment.
 -}
experimentRun :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentRun thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Run" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Transmit all experimental data contained by this part.
 -}
experimentTransmit :: KRPCHS.SpaceCenter.Experiment -> RPCContext ()
experimentTransmit thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_Transmit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The data contained in this experiment.
 -}
getExperimentData :: KRPCHS.SpaceCenter.Experiment -> RPCContext ([KRPCHS.SpaceCenter.ScienceData])
getExperimentData thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Data" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentDataStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq ([KRPCHS.SpaceCenter.ScienceData])
getExperimentDataStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Data" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentDataStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ScienceData]))
getExperimentDataStream thisArg = requestStream $ getExperimentDataStreamReq thisArg 

{-
 - Whether the experiment has been deployed.
 -}
getExperimentDeployed :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentDeployedStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentDeployedStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentDeployedStream thisArg = requestStream $ getExperimentDeployedStreamReq thisArg 

{-
 - Whether the experiment contains data.
 -}
getExperimentHasData :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentHasData thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_HasData" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentHasDataStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentHasDataStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_HasData" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentHasDataStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentHasDataStream thisArg = requestStream $ getExperimentHasDataStreamReq thisArg 

{-
 - Whether the experiment is inoperable.
 -}
getExperimentInoperable :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentInoperable thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Inoperable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentInoperableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentInoperableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Inoperable" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentInoperableStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentInoperableStream thisArg = requestStream $ getExperimentInoperableStreamReq thisArg 

{-
 - The part object for this experiment.
 -}
getExperimentPart :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCHS.SpaceCenter.Part)
getExperimentPart thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentPartStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getExperimentPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentPartStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getExperimentPartStream thisArg = requestStream $ getExperimentPartStreamReq thisArg 

{-
 - Whether the experiment can be re-run.
 -}
getExperimentRerunnable :: KRPCHS.SpaceCenter.Experiment -> RPCContext (Bool)
getExperimentRerunnable thisArg = do
    let r = makeRequest "SpaceCenter" "Experiment_get_Rerunnable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getExperimentRerunnableStreamReq :: KRPCHS.SpaceCenter.Experiment -> KRPCStreamReq (Bool)
getExperimentRerunnableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Experiment_get_Rerunnable" [makeArgument 0 thisArg]
    in  makeStream req

getExperimentRerunnableStream :: KRPCHS.SpaceCenter.Experiment -> RPCContext (KRPCStream (Bool))
getExperimentRerunnableStream thisArg = requestStream $ getExperimentRerunnableStreamReq thisArg 

{-
 - Jettison the fairing. Has no effect if it has already been jettisoned.
 -}
fairingJettison :: KRPCHS.SpaceCenter.Fairing -> RPCContext ()
fairingJettison thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_Jettison" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the fairing has been jettisoned.
 -}
getFairingJettisoned :: KRPCHS.SpaceCenter.Fairing -> RPCContext (Bool)
getFairingJettisoned thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFairingJettisonedStreamReq :: KRPCHS.SpaceCenter.Fairing -> KRPCStreamReq (Bool)
getFairingJettisonedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]
    in  makeStream req

getFairingJettisonedStream :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCStream (Bool))
getFairingJettisonedStream thisArg = requestStream $ getFairingJettisonedStreamReq thisArg 

{-
 - The part object for this fairing.
 -}
getFairingPart :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCHS.SpaceCenter.Part)
getFairingPart thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFairingPartStreamReq :: KRPCHS.SpaceCenter.Fairing -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getFairingPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getFairingPartStream :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getFairingPartStream thisArg = requestStream $ getFairingPartStreamReq thisArg 

{-
 - The total aerodynamic forces acting on the vessel, as a vector pointing in the direction of the force, with its
 - magnitude equal to the strength of the force in Newtons.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightAerodynamicForce :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAerodynamicForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAerodynamicForceStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAerodynamicForceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAerodynamicForceStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAerodynamicForceStream thisArg = requestStream $ getFlightAerodynamicForceStreamReq thisArg 

{-
 - Gets the pitch angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightAngleOfAttack :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightAngleOfAttack thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAngleOfAttackStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightAngleOfAttackStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAngleOfAttackStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightAngleOfAttackStream thisArg = requestStream $ getFlightAngleOfAttackStreamReq thisArg 

{-
 - The unit direction vector pointing in the anti-normal direction.
 -}
getFlightAntiNormal :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAntiNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAntiNormalStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAntiNormalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAntiNormalStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiNormalStream thisArg = requestStream $ getFlightAntiNormalStreamReq thisArg 

{-
 - The unit direction vector pointing in the anti-radial direction.
 -}
getFlightAntiRadial :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAntiRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAntiRadialStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightAntiRadialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAntiRadialStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiRadialStream thisArg = requestStream $ getFlightAntiRadialStreamReq thisArg 

{-
 - The current density of the atmosphere around the vessel, inkg/m^3.
 -}
getFlightAtmosphereDensity :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightAtmosphereDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightAtmosphereDensityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightAtmosphereDensityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]
    in  makeStream req

getFlightAtmosphereDensityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightAtmosphereDensityStream thisArg = requestStream $ getFlightAtmosphereDensityStreamReq thisArg 

{-
 - Gets the <a href="https://en.wikipedia.org/wiki/Ballistic_coefficient">ballistic coefficient.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightBallisticCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightBallisticCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightBallisticCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightBallisticCoefficientStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]
    in  makeStream req

getFlightBallisticCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightBallisticCoefficientStream thisArg = requestStream $ getFlightBallisticCoefficientStreamReq thisArg 

{-
 - The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 - Measured from the center of mass of the vessel.
 -}
getFlightBedrockAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightBedrockAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightBedrockAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightBedrockAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightBedrockAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightBedrockAltitudeStream thisArg = requestStream $ getFlightBedrockAltitudeStreamReq thisArg 

{-
 - The position of the center of mass of the vessel.
 -}
getFlightCenterOfMass :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightCenterOfMass thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightCenterOfMassStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightCenterOfMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]
    in  makeStream req

getFlightCenterOfMassStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightCenterOfMassStream thisArg = requestStream $ getFlightCenterOfMassStreamReq thisArg 

{-
 - The direction vector that the vessel is pointing in.
 -}
getFlightDirection :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightDirection thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDirectionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightDirectionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDirectionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDirectionStream thisArg = requestStream $ getFlightDirectionStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic dragcurrently acting on the vessel,
 - as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightDrag :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightDrag thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDragStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightDragStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDragStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDragStream thisArg = requestStream $ getFlightDragStreamReq thisArg 

{-
 - Gets the coefficient of drag. This is the amount of drag produced by the vessel. It depends on air speed,
 - air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightDragCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightDragCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDragCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightDragCoefficientStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDragCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightDragCoefficientStream thisArg = requestStream $ getFlightDragCoefficientStreamReq thisArg 

{-
 - The dynamic pressure acting on the vessel, in Pascals. This is a measure of the strength of the
 - aerodynamic forces. It is equal to\frac{1}{2} . \mbox{air density} .  \mbox{velocity}^2.
 - It is commonly denoted asQ.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model, or
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchif it is installed.
 -}
getFlightDynamicPressure :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightDynamicPressureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightDynamicPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [makeArgument 0 thisArg]
    in  makeStream req

getFlightDynamicPressureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightDynamicPressureStream thisArg = requestStream $ getFlightDynamicPressureStreamReq thisArg 

{-
 - The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level,
 - and is negative when the vessel is over the sea.
 -}
getFlightElevation :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightElevation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightElevationStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightElevationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]
    in  makeStream req

getFlightElevationStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightElevationStream thisArg = requestStream $ getFlightElevationStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Equivalent_airspeed">equivalent air speedof the vessel, inm/s.Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightEquivalentAirSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightEquivalentAirSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightEquivalentAirSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightEquivalentAirSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightEquivalentAirSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightEquivalentAirSpeedStream thisArg = requestStream $ getFlightEquivalentAirSpeedStreamReq thisArg 

{-
 - The current G force acting on the vessel inm/s^2.
 -}
getFlightGForce :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightGForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightGForceStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightGForceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]
    in  makeStream req

getFlightGForceStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightGForceStream thisArg = requestStream $ getFlightGForceStreamReq thisArg 

{-
 - The heading angle of the vessel relative to north, in degrees. A value between 0° and 360°.
 -}
getFlightHeading :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightHeadingStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightHeadingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]
    in  makeStream req

getFlightHeadingStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightHeadingStream thisArg = requestStream $ getFlightHeadingStreamReq thisArg 

{-
 - The horizontal speed of the vessel in meters per second.
 -}
getFlightHorizontalSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightHorizontalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightHorizontalSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightHorizontalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightHorizontalSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightHorizontalSpeedStream thisArg = requestStream $ getFlightHorizontalSpeedStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Latitude">latitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLatitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightLatitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLatitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightLatitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLatitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightLatitudeStream thisArg = requestStream $ getFlightLatitudeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic liftcurrently acting on the vessel,
 - as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightLift :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightLift thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLiftStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightLiftStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLiftStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightLiftStream thisArg = requestStream $ getFlightLiftStreamReq thisArg 

{-
 - Gets the coefficient of lift. This is the amount of lift produced by the vessel, and depends on air speed, air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightLiftCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightLiftCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLiftCoefficientStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightLiftCoefficientStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLiftCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightLiftCoefficientStream thisArg = requestStream $ getFlightLiftCoefficientStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Longitude">longitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLongitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightLongitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightLongitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightLongitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightLongitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightLongitudeStream thisArg = requestStream $ getFlightLongitudeStreamReq thisArg 

{-
 - The speed of the vessel, in multiples of the speed of sound.Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightMach :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightMach thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightMachStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightMachStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]
    in  makeStream req

getFlightMachStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightMachStream thisArg = requestStream $ getFlightMachStreamReq thisArg 

{-
 - The altitude above sea level, in meters.
 - Measured from the center of mass of the vessel.
 -}
getFlightMeanAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightMeanAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightMeanAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightMeanAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightMeanAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightMeanAltitudeStream thisArg = requestStream $ getFlightMeanAltitudeStreamReq thisArg 

{-
 - The unit direction vector pointing in the normal direction.
 -}
getFlightNormal :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightNormalStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightNormalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]
    in  makeStream req

getFlightNormalStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightNormalStream thisArg = requestStream $ getFlightNormalStreamReq thisArg 

{-
 - The pitch angle of the vessel relative to the horizon, in degrees. A value between -90° and +90°.
 -}
getFlightPitch :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightPitchStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightPitchStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]
    in  makeStream req

getFlightPitchStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightPitchStream thisArg = requestStream $ getFlightPitchStreamReq thisArg 

{-
 - The unit direction vector pointing in the prograde direction.
 -}
getFlightPrograde :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightPrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightProgradeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightProgradeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]
    in  makeStream req

getFlightProgradeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightProgradeStream thisArg = requestStream $ getFlightProgradeStreamReq thisArg 

{-
 - The unit direction vector pointing in the radial direction.
 -}
getFlightRadial :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRadialStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightRadialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRadialStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRadialStream thisArg = requestStream $ getFlightRadialStreamReq thisArg 

{-
 - The unit direction vector pointing in the retrograde direction.
 -}
getFlightRetrograde :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightRetrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRetrogradeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightRetrogradeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRetrogradeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRetrogradeStream thisArg = requestStream $ getFlightRetrogradeStreamReq thisArg 

{-
 - The roll angle of the vessel relative to the horizon, in degrees. A value between -180° and +180°.
 -}
getFlightRoll :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRollStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightRollStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRollStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightRollStream thisArg = requestStream $ getFlightRollStreamReq thisArg 

{-
 - The rotation of the vessel.
 -}
getFlightRotation :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double, Double))
getFlightRotation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightRotationStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double, Double))
getFlightRotationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]
    in  makeStream req

getFlightRotationStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getFlightRotationStream thisArg = requestStream $ getFlightRotationStreamReq thisArg 

{-
 - Gets the yaw angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightSideslipAngle :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightSideslipAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSideslipAngleStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightSideslipAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSideslipAngleStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightSideslipAngleStream thisArg = requestStream $ getFlightSideslipAngleStreamReq thisArg 

{-
 - The speed of the vessel in meters per second.
 -}
getFlightSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightSpeedStream thisArg = requestStream $ getFlightSpeedStreamReq thisArg 

{-
 - The speed of sound, in the atmosphere around the vessel, inm/s.Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightSpeedOfSound :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightSpeedOfSound thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSpeedOfSoundStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightSpeedOfSoundStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSpeedOfSoundStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightSpeedOfSoundStream thisArg = requestStream $ getFlightSpeedOfSoundStreamReq thisArg 

{-
 - Gets the current amount of stall, between 0 and 1. A value greater than 0.005 indicates a minor stall
 - and a value greater than 0.5 indicates a large-scale stall.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightStallFraction :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStallFraction thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStallFractionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStallFractionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStallFractionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStallFractionStream thisArg = requestStream $ getFlightStallFractionStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">static (ambient) temperatureof the
 - atmosphere around the vessel, in Kelvin.
 -}
getFlightStaticAirTemperature :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStaticAirTemperatureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticAirTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStaticAirTemperatureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticAirTemperatureStream thisArg = requestStream $ getFlightStaticAirTemperatureStreamReq thisArg 

{-
 - The static atmospheric pressure acting on the vessel, in Pascals.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightStaticPressure :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightStaticPressureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightStaticPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]
    in  makeStream req

getFlightStaticPressureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticPressureStream thisArg = requestStream $ getFlightStaticPressureStreamReq thisArg 

{-
 - The altitude above the surface of the body or sea level, whichever is closer, in meters.
 - Measured from the center of mass of the vessel.
 -}
getFlightSurfaceAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightSurfaceAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightSurfaceAltitudeStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightSurfaceAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getFlightSurfaceAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightSurfaceAltitudeStream thisArg = requestStream $ getFlightSurfaceAltitudeStreamReq thisArg 

{-
 - An estimate of the current terminal velocity of the vessel, inm/s.
 - This is the speed at which the drag forces cancel out the force of gravity.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model, or
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchif it is installed.
 -}
getFlightTerminalVelocity :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTerminalVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightTerminalVelocityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTerminalVelocityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]
    in  makeStream req

getFlightTerminalVelocityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTerminalVelocityStream thisArg = requestStream $ getFlightTerminalVelocityStreamReq thisArg 

{-
 - Gets the thrust specific fuel consumption for the jet engines on the vessel. This is a measure of the
 - efficiency of the engines, with a lower value indicating a more efficient vessel. This value is the
 - number of Newtons of fuel that are burned, per hour, to product one newton of thrust.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightThrustSpecificFuelConsumption :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightThrustSpecificFuelConsumption thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightThrustSpecificFuelConsumptionStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightThrustSpecificFuelConsumptionStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]
    in  makeStream req

getFlightThrustSpecificFuelConsumptionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightThrustSpecificFuelConsumptionStream thisArg = requestStream $ getFlightThrustSpecificFuelConsumptionStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">total air temperatureof the atmosphere
 - around the vessel, in Kelvin. This temperature includes the <see cref="M:SpaceCenter.Flight.StaticAirTemperature" /> and the vessel's kinetic energy.
 -}
getFlightTotalAirTemperature :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTotalAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightTotalAirTemperatureStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Float)
getFlightTotalAirTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getFlightTotalAirTemperatureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTotalAirTemperatureStream thisArg = requestStream $ getFlightTotalAirTemperatureStreamReq thisArg 

{-
 - The velocity vector of the vessel. The magnitude of the vector is the speed of the vessel in meters per second.
 - The direction of the vector is the direction of the vessels motion.
 -}
getFlightVelocity :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightVelocityStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq ((Double, Double, Double))
getFlightVelocityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]
    in  makeStream req

getFlightVelocityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightVelocityStream thisArg = requestStream $ getFlightVelocityStreamReq thisArg 

{-
 - The vertical speed of the vessel in meters per second.
 -}
getFlightVerticalSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightVerticalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getFlightVerticalSpeedStreamReq :: KRPCHS.SpaceCenter.Flight -> KRPCStreamReq (Double)
getFlightVerticalSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]
    in  makeStream req

getFlightVerticalSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightVerticalSpeedStream thisArg = requestStream $ getFlightVerticalSpeedStreamReq thisArg 

{-
 - The area of the intake's opening, in square meters.
 -}
getIntakeArea :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeArea thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeAreaStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeAreaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeAreaStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeAreaStream thisArg = requestStream $ getIntakeAreaStreamReq thisArg 

{-
 - The rate of flow into the intake, in units of resource per second.
 -}
getIntakeFlow :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeFlow thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeFlowStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeFlowStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeFlowStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeFlowStream thisArg = requestStream $ getIntakeFlowStreamReq thisArg 

{-
 - Whether the intake is open.
 -}
getIntakeOpen :: KRPCHS.SpaceCenter.Intake -> RPCContext (Bool)
getIntakeOpen thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeOpenStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Bool)
getIntakeOpenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeOpenStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Bool))
getIntakeOpenStream thisArg = requestStream $ getIntakeOpenStreamReq thisArg 

{-
 - The part object for this intake.
 -}
getIntakePart :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCHS.SpaceCenter.Part)
getIntakePart thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakePartStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getIntakePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getIntakePartStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getIntakePartStream thisArg = requestStream $ getIntakePartStreamReq thisArg 

{-
 - Speed of the flow into the intake, inm/s.
 -}
getIntakeSpeed :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getIntakeSpeedStreamReq :: KRPCHS.SpaceCenter.Intake -> KRPCStreamReq (Float)
getIntakeSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getIntakeSpeedStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeSpeedStream thisArg = requestStream $ getIntakeSpeedStreamReq thisArg 

{-
 - Whether the intake is open.
 -}
setIntakeOpen :: KRPCHS.SpaceCenter.Intake -> Bool -> RPCContext ()
setIntakeOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Intake_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the landing gear is deployable.
 -}
getLandingGearDeployable :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (Bool)
getLandingGearDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingGearDeployableStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (Bool)
getLandingGearDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingGear_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getLandingGearDeployableStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployableStream thisArg = requestStream $ getLandingGearDeployableStreamReq thisArg 

{-
 - Whether the landing gear is deployed.Fixed landing gear are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
getLandingGearDeployed :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (Bool)
getLandingGearDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingGearDeployedStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (Bool)
getLandingGearDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingGear_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getLandingGearDeployedStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployedStream thisArg = requestStream $ getLandingGearDeployedStreamReq thisArg 

{-
 - The part object for this landing gear.
 -}
getLandingGearPart :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCHS.SpaceCenter.Part)
getLandingGearPart thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingGearPartStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLandingGearPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingGear_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLandingGearPartStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLandingGearPartStream thisArg = requestStream $ getLandingGearPartStreamReq thisArg 

{-
 - Gets the current state of the landing gear.Fixed landing gear are always deployed.
 -}
getLandingGearState :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCHS.SpaceCenter.LandingGearState)
getLandingGearState thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingGearStateStreamReq :: KRPCHS.SpaceCenter.LandingGear -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingGearState)
getLandingGearStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingGear_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getLandingGearStateStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingGearState))
getLandingGearStateStream thisArg = requestStream $ getLandingGearStateStreamReq thisArg 

{-
 - Whether the landing gear is deployed.Fixed landing gear are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
setLandingGearDeployed :: KRPCHS.SpaceCenter.LandingGear -> Bool -> RPCContext ()
setLandingGearDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the landing leg is deployed.Fixed landing legs are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
getLandingLegDeployed :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (Bool)
getLandingLegDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingLegDeployedStreamReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCStreamReq (Bool)
getLandingLegDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingLeg_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getLandingLegDeployedStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (Bool))
getLandingLegDeployedStream thisArg = requestStream $ getLandingLegDeployedStreamReq thisArg 

{-
 - The part object for this landing leg.
 -}
getLandingLegPart :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCHS.SpaceCenter.Part)
getLandingLegPart thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingLegPartStreamReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLandingLegPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingLeg_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLandingLegPartStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLandingLegPartStream thisArg = requestStream $ getLandingLegPartStreamReq thisArg 

{-
 - The current state of the landing leg.
 -}
getLandingLegState :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCHS.SpaceCenter.LandingLegState)
getLandingLegState thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLandingLegStateStreamReq :: KRPCHS.SpaceCenter.LandingLeg -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingLegState)
getLandingLegStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LandingLeg_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getLandingLegStateStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingLegState))
getLandingLegStateStream thisArg = requestStream $ getLandingLegStateStreamReq thisArg 

{-
 - Whether the landing leg is deployed.Fixed landing legs are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
setLandingLegDeployed :: KRPCHS.SpaceCenter.LandingLeg -> Bool -> RPCContext ()
setLandingLegDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Releases the docking clamp. Has no effect if the clamp has already been released.
 -}
launchClampRelease :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext ()
launchClampRelease thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_Release" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The part object for this launch clamp.
 -}
getLaunchClampPart :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (KRPCHS.SpaceCenter.Part)
getLaunchClampPart thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLaunchClampPartStreamReq :: KRPCHS.SpaceCenter.LaunchClamp -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLaunchClampPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLaunchClampPartStream :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLaunchClampPartStream thisArg = requestStream $ getLaunchClampPartStreamReq thisArg 

{-
 - Launch a vessel.<param name="craftDirectory">Name of the directory in the current saves "Ships" directory, that contains the craft file. For example"VAB"or"SPH".<param name="name">Name of the vessel to launch. This is the name of the ".craft" file in the save directory, without the ".craft" file extension.<param name="launchSite">Name of the launch site. For example"LaunchPad"or"Runway".
 -}
launchVessel :: Data.Text.Text -> Data.Text.Text -> Data.Text.Text -> RPCContext ()
launchVessel craftDirectoryArg nameArg launchSiteArg = do
    let r = makeRequest "SpaceCenter" "LaunchVessel" [makeArgument 0 craftDirectoryArg, makeArgument 1 nameArg, makeArgument 2 launchSiteArg]
    res <- sendRequest r
    processResponse res 

{-
 - Launch a new vessel from the SPH onto the runway.<param name="name">Name of the vessel to launch.This is equivalent to calling <see cref="M:SpaceCenter.LaunchVessel" /> with the craft directory set to "SPH" and the launch site set to "Runway".
 -}
launchVesselFromSPH :: Data.Text.Text -> RPCContext ()
launchVesselFromSPH nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromSPH" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Launch a new vessel from the VAB onto the launchpad.<param name="name">Name of the vessel to launch.This is equivalent to calling <see cref="M:SpaceCenter.LaunchVessel" /> with the craft directory set to "VAB" and the launch site set to "LaunchPad".
 -}
launchVesselFromVAB :: Data.Text.Text -> RPCContext ()
launchVesselFromVAB nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromVAB" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns a list of vessels from the given <paramref name="craftDirectory" /> that can be launched.<param name="craftDirectory">Name of the directory in the current saves "Ships" directory. For example"VAB"or"SPH".
 -}
launchableVessels :: Data.Text.Text -> RPCContext ([Data.Text.Text])
launchableVessels craftDirectoryArg = do
    let r = makeRequest "SpaceCenter" "LaunchableVessels" [makeArgument 0 craftDirectoryArg]
    res <- sendRequest r
    processResponse res

launchableVesselsStreamReq :: Data.Text.Text -> KRPCStreamReq ([Data.Text.Text])
launchableVesselsStreamReq craftDirectoryArg =
    let req = makeRequest "SpaceCenter" "LaunchableVessels" [makeArgument 0 craftDirectoryArg]
    in  makeStream req

launchableVesselsStream :: Data.Text.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
launchableVesselsStream craftDirectoryArg = requestStream $ launchableVesselsStreamReq craftDirectoryArg 

{-
 - Whether the light is switched on.
 -}
getLightActive :: KRPCHS.SpaceCenter.Light -> RPCContext (Bool)
getLightActive thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightActiveStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (Bool)
getLightActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getLightActiveStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (Bool))
getLightActiveStream thisArg = requestStream $ getLightActiveStreamReq thisArg 

{-
 - The color of the light, as an RGB triple.
 -}
getLightColor :: KRPCHS.SpaceCenter.Light -> RPCContext ((Float, Float, Float))
getLightColor thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightColorStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq ((Float, Float, Float))
getLightColorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]
    in  makeStream req

getLightColorStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream ((Float, Float, Float)))
getLightColorStream thisArg = requestStream $ getLightColorStreamReq thisArg 

{-
 - The part object for this light.
 -}
getLightPart :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCHS.SpaceCenter.Part)
getLightPart thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightPartStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getLightPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getLightPartStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLightPartStream thisArg = requestStream $ getLightPartStreamReq thisArg 

{-
 - The current power usage, in units of charge per second.
 -}
getLightPowerUsage :: KRPCHS.SpaceCenter.Light -> RPCContext (Float)
getLightPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLightPowerUsageStreamReq :: KRPCHS.SpaceCenter.Light -> KRPCStreamReq (Float)
getLightPowerUsageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]
    in  makeStream req

getLightPowerUsageStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (Float))
getLightPowerUsageStream thisArg = requestStream $ getLightPowerUsageStreamReq thisArg 

{-
 - Whether the light is switched on.
 -}
setLightActive :: KRPCHS.SpaceCenter.Light -> Bool -> RPCContext ()
setLightActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The color of the light, as an RGB triple.
 -}
setLightColor :: KRPCHS.SpaceCenter.Light -> (Float, Float, Float) -> RPCContext ()
setLightColor thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Load the game with the given name.
 - This will create a load a save file calledname.sfsfrom the folder of the current save game.
 -}
load :: Data.Text.Text -> RPCContext ()
load nameArg = do
    let r = makeRequest "SpaceCenter" "Load" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns the value of a field.<param name="name">Name of the field.
 -}
moduleGetField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Data.Text.Text)
moduleGetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleGetFieldStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Data.Text.Text)
moduleGetFieldStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleGetFieldStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Data.Text.Text))
moduleGetFieldStream thisArg nameArg = requestStream $ moduleGetFieldStreamReq thisArg nameArg 

{-
 - trueif the part has an action with the given name.<param name="name">
 -}
moduleHasAction :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasAction thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleHasActionStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasActionStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleHasActionStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasActionStream thisArg nameArg = requestStream $ moduleHasActionStreamReq thisArg nameArg 

{-
 - trueif the module has an event with the given name.<param name="name">
 -}
moduleHasEvent :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleHasEventStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasEventStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleHasEventStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasEventStream thisArg nameArg = requestStream $ moduleHasEventStreamReq thisArg nameArg 

{-
 - Returnstrueif the module has a field with the given name.<param name="name">Name of the field.
 -}
moduleHasField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

moduleHasFieldStreamReq :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> KRPCStreamReq (Bool)
moduleHasFieldStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

moduleHasFieldStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasFieldStream thisArg nameArg = requestStream $ moduleHasFieldStreamReq thisArg nameArg 

{-
 - Set the value of a field to its original value.<param name="name">Name of the field.
 -}
moduleResetField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext ()
moduleResetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_ResetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of an action with the given name.<param name="name"><param name="value">
 -}
moduleSetAction :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Bool -> RPCContext ()
moduleSetAction thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetAction" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of a field to the given floating point number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldFloat :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Float -> RPCContext ()
moduleSetFieldFloat thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldFloat" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of a field to the given integer number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldInt :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Int.Int32 -> RPCContext ()
moduleSetFieldInt thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldInt" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the value of a field to the given string.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldString :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Text.Text -> RPCContext ()
moduleSetFieldString thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldString" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Trigger the named event. Equivalent to clicking the button in the right-click menu of the part.<param name="name">
 -}
moduleTriggerEvent :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext ()
moduleTriggerEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_TriggerEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - A list of all the names of the modules actions. These are the parts actions that can be assigned
 - to action groups in the in-game editor.
 -}
getModuleActions :: KRPCHS.SpaceCenter.Module -> RPCContext ([Data.Text.Text])
getModuleActions thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleActionsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq ([Data.Text.Text])
getModuleActionsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]
    in  makeStream req

getModuleActionsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream ([Data.Text.Text]))
getModuleActionsStream thisArg = requestStream $ getModuleActionsStreamReq thisArg 

{-
 - A list of the names of all of the modules events. Events are the clickable buttons
 - visible in the right-click menu of the part.
 -}
getModuleEvents :: KRPCHS.SpaceCenter.Module -> RPCContext ([Data.Text.Text])
getModuleEvents thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleEventsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq ([Data.Text.Text])
getModuleEventsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]
    in  makeStream req

getModuleEventsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream ([Data.Text.Text]))
getModuleEventsStream thisArg = requestStream $ getModuleEventsStreamReq thisArg 

{-
 - The modules field names and their associated values, as a dictionary.
 - These are the values visible in the right-click menu of the part.
 -}
getModuleFields :: KRPCHS.SpaceCenter.Module -> RPCContext (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFields thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleFieldsStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFieldsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]
    in  makeStream req

getModuleFieldsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Data.Text.Text)))
getModuleFieldsStream thisArg = requestStream $ getModuleFieldsStreamReq thisArg 

{-
 - Name of the PartModule. For example, "ModuleEngines".
 -}
getModuleName :: KRPCHS.SpaceCenter.Module -> RPCContext (Data.Text.Text)
getModuleName thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModuleNameStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (Data.Text.Text)
getModuleNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getModuleNameStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (Data.Text.Text))
getModuleNameStream thisArg = requestStream $ getModuleNameStreamReq thisArg 

{-
 - The part that contains this module.
 -}
getModulePart :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCHS.SpaceCenter.Part)
getModulePart thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getModulePartStreamReq :: KRPCHS.SpaceCenter.Module -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getModulePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getModulePartStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getModulePartStream thisArg = requestStream $ getModulePartStreamReq thisArg 

{-
 - Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
 - is the delta-v of the burn in m/s.<param name="referenceFrame">Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingBurnVector" />.
 -}
nodeBurnVector :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodeBurnVectorStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeBurnVectorStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodeBurnVectorStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeBurnVectorStream thisArg referenceFrameArg = requestStream $ nodeBurnVectorStreamReq thisArg referenceFrameArg 

{-
 - Returns the unit direction vector of the maneuver nodes burn in the given reference frame.<param name="referenceFrame">
 -}
nodeDirection :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodeDirectionStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodeDirectionStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeDirectionStream thisArg referenceFrameArg = requestStream $ nodeDirectionStreamReq thisArg referenceFrameArg 

{-
 - Returns the position vector of the maneuver node in the given reference frame.<param name="referenceFrame">
 -}
nodePosition :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodePosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodePositionStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodePositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodePositionStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodePositionStream thisArg referenceFrameArg = requestStream $ nodePositionStreamReq thisArg referenceFrameArg 

{-
 - Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
 - is the delta-v of the burn in m/s. The direction and magnitude change as the burn is executed.<param name="referenceFrame">
 -}
nodeRemainingBurnVector :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeRemainingBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

nodeRemainingBurnVectorStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
nodeRemainingBurnVectorStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

nodeRemainingBurnVectorStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeRemainingBurnVectorStream thisArg referenceFrameArg = requestStream $ nodeRemainingBurnVectorStreamReq thisArg referenceFrameArg 

{-
 - Removes the maneuver node.
 -}
nodeRemove :: KRPCHS.SpaceCenter.Node -> RPCContext ()
nodeRemove thisArg = do
    let r = makeRequest "SpaceCenter" "Node_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
getNodeDeltaV :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeDeltaVStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeDeltaVStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]
    in  makeStream req

getNodeDeltaVStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeDeltaVStream thisArg = requestStream $ getNodeDeltaVStreamReq thisArg 

{-
 - The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
getNodeNormal :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeNormalStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeNormalStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]
    in  makeStream req

getNodeNormalStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeNormalStream thisArg = requestStream $ getNodeNormalStreamReq thisArg 

{-
 - The orbit that results from executing the maneuver node.
 -}
getNodeOrbit :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getNodeOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeOrbitStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getNodeOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]
    in  makeStream req

getNodeOrbitStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
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
getNodeOrbitalReferenceFrame :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeOrbitalReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getNodeOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeOrbitalReferenceFrameStream thisArg = requestStream $ getNodeOrbitalReferenceFrameStreamReq thisArg 

{-
 - The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
getNodePrograde :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodePrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeProgradeStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeProgradeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]
    in  makeStream req

getNodeProgradeStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeProgradeStream thisArg = requestStream $ getNodeProgradeStreamReq thisArg 

{-
 - The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
getNodeRadial :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeRadialStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeRadialStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]
    in  makeStream req

getNodeRadialStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeRadialStream thisArg = requestStream $ getNodeRadialStreamReq thisArg 

{-
 - Gets the reference frame that is fixed relative to the maneuver node's burn.
 - <list type="bullet">The origin is at the position of the maneuver node.The y-axis points in the direction of the burn.The x-axis and z-axis point in arbitrary but fixed directions.
 -}
getNodeReferenceFrame :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getNodeReferenceFrameStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeReferenceFrameStream thisArg = requestStream $ getNodeReferenceFrameStreamReq thisArg 

{-
 - Gets the remaining delta-v of the maneuver node, in meters per second. Changes as the node
 - is executed. This is equivalent to the delta-v reported in-game.
 -}
getNodeRemainingDeltaV :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeRemainingDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeRemainingDeltaVStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Float)
getNodeRemainingDeltaVStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]
    in  makeStream req

getNodeRemainingDeltaVStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeRemainingDeltaVStream thisArg = requestStream $ getNodeRemainingDeltaVStreamReq thisArg 

{-
 - The time until the maneuver node will be encountered, in seconds.
 -}
getNodeTimeTo :: KRPCHS.SpaceCenter.Node -> RPCContext (Double)
getNodeTimeTo thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeTimeToStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeTimeToStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]
    in  makeStream req

getNodeTimeToStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Double))
getNodeTimeToStream thisArg = requestStream $ getNodeTimeToStreamReq thisArg 

{-
 - The universal time at which the maneuver will occur, in seconds.
 -}
getNodeUT :: KRPCHS.SpaceCenter.Node -> RPCContext (Double)
getNodeUT thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getNodeUTStreamReq :: KRPCHS.SpaceCenter.Node -> KRPCStreamReq (Double)
getNodeUTStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]
    in  makeStream req

getNodeUTStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Double))
getNodeUTStream thisArg = requestStream $ getNodeUTStreamReq thisArg 

{-
 - The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
setNodeDeltaV :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodeDeltaV thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_DeltaV" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
setNodeNormal :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodeNormal thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Normal" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
setNodePrograde :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodePrograde thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Prograde" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
setNodeRadial :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext ()
setNodeRadial thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Radial" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The universal time at which the maneuver will occur, in seconds.
 -}
setNodeUT :: KRPCHS.SpaceCenter.Node -> Double -> RPCContext ()
setNodeUT thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_UT" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The unit direction vector from which the orbits longitude of ascending node is measured,
 - in the given reference frame.<param name="referenceFrame">
 -}
orbitReferencePlaneDirection :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitReferencePlaneDirection referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]
    res <- sendRequest r
    processResponse res

orbitReferencePlaneDirectionStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
orbitReferencePlaneDirectionStreamReq referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]
    in  makeStream req

orbitReferencePlaneDirectionStream :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneDirectionStream referenceFrameArg = requestStream $ orbitReferencePlaneDirectionStreamReq referenceFrameArg 

{-
 - The unit direction vector that is normal to the orbits reference plane, in the given
 - reference frame. The reference plane is the plane from which the orbits inclination is measured.<param name="referenceFrame">
 -}
orbitReferencePlaneNormal :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitReferencePlaneNormal referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]
    res <- sendRequest r
    processResponse res

orbitReferencePlaneNormalStreamReq :: KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
orbitReferencePlaneNormalStreamReq referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]
    in  makeStream req

orbitReferencePlaneNormalStream :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneNormalStream referenceFrameArg = requestStream $ orbitReferencePlaneNormalStreamReq referenceFrameArg 

{-
 - Gets the apoapsis of the orbit, in meters, from the center of mass of the body being orbited.For the apoapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.ApoapsisAltitude" />.
 -}
getOrbitApoapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitApoapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitApoapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitApoapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisStream thisArg = requestStream $ getOrbitApoapsisStreamReq thisArg 

{-
 - The apoapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Apoapsis" /> minus the equatorial radius of the body.
 -}
getOrbitApoapsisAltitude :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitApoapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitApoapsisAltitudeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitApoapsisAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitApoapsisAltitudeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisAltitudeStream thisArg = requestStream $ getOrbitApoapsisAltitudeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Argument_of_periapsis">argument of periapsis, in radians.
 -}
getOrbitArgumentOfPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitArgumentOfPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitArgumentOfPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitArgumentOfPeriapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitArgumentOfPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitArgumentOfPeriapsisStream thisArg = requestStream $ getOrbitArgumentOfPeriapsisStreamReq thisArg 

{-
 - The celestial body (e.g. planet or moon) around which the object is orbiting.
 -}
getOrbitBody :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBody thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitBodyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBodyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitBodyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getOrbitBodyStream thisArg = requestStream $ getOrbitBodyStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Eccentric_anomaly">eccentric anomaly.
 -}
getOrbitEccentricAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEccentricAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitEccentricAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEccentricAnomalyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitEccentricAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricAnomalyStream thisArg = requestStream $ getOrbitEccentricAnomalyStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Orbital_eccentricity">eccentricityof the orbit.
 -}
getOrbitEccentricity :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEccentricity thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitEccentricityStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEccentricityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitEccentricityStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricityStream thisArg = requestStream $ getOrbitEccentricityStreamReq thisArg 

{-
 - The time since the epoch (the point at which the
 - <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epochwas measured, in seconds.
 -}
getOrbitEpoch :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitEpochStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitEpochStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitEpochStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEpochStream thisArg = requestStream $ getOrbitEpochStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Orbital_inclination">inclinationof the orbit,
 - in radians.
 -}
getOrbitInclination :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitInclination thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitInclinationStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitInclinationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitInclinationStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitInclinationStream thisArg = requestStream $ getOrbitInclinationStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Longitude_of_the_ascending_node">longitude of the
 - ascending node, in radians.
 -}
getOrbitLongitudeOfAscendingNode :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitLongitudeOfAscendingNode thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitLongitudeOfAscendingNodeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitLongitudeOfAscendingNodeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitLongitudeOfAscendingNodeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitLongitudeOfAscendingNodeStream thisArg = requestStream $ getOrbitLongitudeOfAscendingNodeStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly.
 -}
getOrbitMeanAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitMeanAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitMeanAnomalyStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitMeanAnomalyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitMeanAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyStream thisArg = requestStream $ getOrbitMeanAnomalyStreamReq thisArg 

{-
 - The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epoch.
 -}
getOrbitMeanAnomalyAtEpoch :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitMeanAnomalyAtEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitMeanAnomalyAtEpochStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitMeanAnomalyAtEpochStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitMeanAnomalyAtEpochStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyAtEpochStream thisArg = requestStream $ getOrbitMeanAnomalyAtEpochStreamReq thisArg 

{-
 - If the object is going to change sphere of influence in the future, returns the new orbit
 - after the change. Otherwise returnsnull.
 -}
getOrbitNextOrbit :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitNextOrbitStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitNextOrbitStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getOrbitNextOrbitStream thisArg = requestStream $ getOrbitNextOrbitStreamReq thisArg 

{-
 - The periapsis of the orbit, in meters, from the center of mass of the body being orbited.For the periapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.PeriapsisAltitude" />.
 -}
getOrbitPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisStream thisArg = requestStream $ getOrbitPeriapsisStreamReq thisArg 

{-
 - The periapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Periapsis" /> minus the equatorial radius of the body.
 -}
getOrbitPeriapsisAltitude :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitPeriapsisAltitudeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriapsisAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitPeriapsisAltitudeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisAltitudeStream thisArg = requestStream $ getOrbitPeriapsisAltitudeStreamReq thisArg 

{-
 - The orbital period, in seconds.
 -}
getOrbitPeriod :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitPeriodStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitPeriodStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitPeriodStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriodStream thisArg = requestStream $ getOrbitPeriodStreamReq thisArg 

{-
 - The current radius of the orbit, in meters. This is the distance between the center
 - of mass of the object in orbit, and the center of mass of the body around which it is orbiting.This value will change over time if the orbit is elliptical.
 -}
getOrbitRadius :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitRadius thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitRadiusStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitRadiusStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitRadiusStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitRadiusStream thisArg = requestStream $ getOrbitRadiusStreamReq thisArg 

{-
 - The semi-major axis of the orbit, in meters.
 -}
getOrbitSemiMajorAxis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSemiMajorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitSemiMajorAxisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSemiMajorAxisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitSemiMajorAxisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMajorAxisStream thisArg = requestStream $ getOrbitSemiMajorAxisStreamReq thisArg 

{-
 - The semi-minor axis of the orbit, in meters.
 -}
getOrbitSemiMinorAxis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSemiMinorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitSemiMinorAxisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSemiMinorAxisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitSemiMinorAxisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMinorAxisStream thisArg = requestStream $ getOrbitSemiMinorAxisStreamReq thisArg 

{-
 - The current orbital speed of the object in meters per second.This value will change over time if the orbit is elliptical.
 -}
getOrbitSpeed :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitSpeedStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitSpeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitSpeedStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSpeedStream thisArg = requestStream $ getOrbitSpeedStreamReq thisArg 

{-
 - The time until the object reaches apoapsis, in seconds.
 -}
getOrbitTimeToApoapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTimeToApoapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToApoapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTimeToApoapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToApoapsisStream thisArg = requestStream $ getOrbitTimeToApoapsisStreamReq thisArg 

{-
 - The time until the object reaches periapsis, in seconds.
 -}
getOrbitTimeToPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTimeToPeriapsisStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToPeriapsisStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTimeToPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToPeriapsisStream thisArg = requestStream $ getOrbitTimeToPeriapsisStreamReq thisArg 

{-
 - The time until the object changes sphere of influence, in seconds. ReturnsNaNif the
 - object is not going to change sphere of influence.
 -}
getOrbitTimeToSOIChange :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToSOIChange thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getOrbitTimeToSOIChangeStreamReq :: KRPCHS.SpaceCenter.Orbit -> KRPCStreamReq (Double)
getOrbitTimeToSOIChangeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]
    in  makeStream req

getOrbitTimeToSOIChangeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToSOIChangeStream thisArg = requestStream $ getOrbitTimeToSOIChangeStreamReq thisArg 

{-
 - Deploys the parachute. This has no effect if the parachute has already
 - been deployed.
 -}
parachuteDeploy :: KRPCHS.SpaceCenter.Parachute -> RPCContext ()
parachuteDeploy thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_Deploy" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The altitude at which the parachute will full deploy, in meters.
 -}
getParachuteDeployAltitude :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Float)
getParachuteDeployAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteDeployAltitudeStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Float)
getParachuteDeployAltitudeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteDeployAltitudeStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployAltitudeStream thisArg = requestStream $ getParachuteDeployAltitudeStreamReq thisArg 

{-
 - The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 -}
getParachuteDeployMinPressure :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Float)
getParachuteDeployMinPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteDeployMinPressureStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Float)
getParachuteDeployMinPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteDeployMinPressureStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployMinPressureStream thisArg = requestStream $ getParachuteDeployMinPressureStreamReq thisArg 

{-
 - Whether the parachute has been deployed.
 -}
getParachuteDeployed :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Bool)
getParachuteDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteDeployedStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (Bool)
getParachuteDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteDeployedStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Bool))
getParachuteDeployedStream thisArg = requestStream $ getParachuteDeployedStreamReq thisArg 

{-
 - The part object for this parachute.
 -}
getParachutePart :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCHS.SpaceCenter.Part)
getParachutePart thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachutePartStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getParachutePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getParachutePartStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getParachutePartStream thisArg = requestStream $ getParachutePartStreamReq thisArg 

{-
 - The current state of the parachute.
 -}
getParachuteState :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCHS.SpaceCenter.ParachuteState)
getParachuteState thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getParachuteStateStreamReq :: KRPCHS.SpaceCenter.Parachute -> KRPCStreamReq (KRPCHS.SpaceCenter.ParachuteState)
getParachuteStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getParachuteStateStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ParachuteState))
getParachuteStateStream thisArg = requestStream $ getParachuteStateStreamReq thisArg 

{-
 - The altitude at which the parachute will full deploy, in meters.
 -}
setParachuteDeployAltitude :: KRPCHS.SpaceCenter.Parachute -> Float -> RPCContext ()
setParachuteDeployAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 -}
setParachuteDeployMinPressure :: KRPCHS.SpaceCenter.Parachute -> Float -> RPCContext ()
setParachuteDeployMinPressure thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployMinPressure" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The position of the parts center of mass in the given reference frame.
 - If the part is physicsless, this is equivalent to <see cref="M:SpaceCenter.Part.Position" />.<param name="referenceFrame">
 -}
partCenterOfMass :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partCenterOfMass thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partCenterOfMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partCenterOfMassStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partCenterOfMassStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partCenterOfMassStream thisArg referenceFrameArg = requestStream $ partCenterOfMassStreamReq thisArg referenceFrameArg 

{-
 - The direction of the part in the given reference frame.<param name="referenceFrame">
 -}
partDirection :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partDirectionStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partDirectionStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partDirectionStream thisArg referenceFrameArg = requestStream $ partDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position of the part in the given reference frame.This is a fixed position in the part, defined by the parts model.
 - It s not necessarily the same as the parts center of mass.
 - Use <see cref="M:SpaceCenter.Part.CenterOfMass" /> to get the parts center of mass.<param name="referenceFrame">
 -}
partPosition :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partPositionStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partPositionStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partPositionStream thisArg referenceFrameArg = requestStream $ partPositionStreamReq thisArg referenceFrameArg 

{-
 - The rotation of the part in the given reference frame.<param name="referenceFrame">
 -}
partRotation :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
partRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partRotationStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
partRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partRotationStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
partRotationStream thisArg referenceFrameArg = requestStream $ partRotationStreamReq thisArg referenceFrameArg 

{-
 - The velocity of the part in the given reference frame.<param name="referenceFrame">
 -}
partVelocity :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

partVelocityStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
partVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

partVelocityStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partVelocityStream thisArg referenceFrameArg = requestStream $ partVelocityStreamReq thisArg referenceFrameArg 

{-
 - Whether the part is axially attached to its parent, i.e. on the top
 - or bottom of its parent. If the part has no parent, returnsfalse.
 -}
getPartAxiallyAttached :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartAxiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartAxiallyAttachedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartAxiallyAttachedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]
    in  makeStream req

getPartAxiallyAttachedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartAxiallyAttachedStream thisArg = requestStream $ getPartAxiallyAttachedStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.CargoBay" /> if the part is a cargo bay, otherwisenull.
 -}
getPartCargoBay :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBay thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCargoBayStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBayStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]
    in  makeStream req

getPartCargoBayStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CargoBay))
getPartCargoBayStream thisArg = requestStream $ getPartCargoBayStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this part, and centered on its center of mass.
 - <list type="bullet">The origin is at the center of mass of the part, as returned by <see cref="M:SpaceCenter.Part.CenterOfMass" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
 - for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartCenterOfMassReferenceFrame :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCenterOfMassReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getPartCenterOfMassReferenceFrameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartCenterOfMassReferenceFrameStream thisArg = requestStream $ getPartCenterOfMassReferenceFrameStreamReq thisArg 

{-
 - The parts children. Returns an empty list if the part has no children.
 - This, in combination with <see cref="M:SpaceCenter.Part.Parent" />, can be used to traverse the vessels parts tree.
 -}
getPartChildren :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartChildren thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartChildrenStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartChildrenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]
    in  makeStream req

getPartChildrenStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartChildrenStream thisArg = requestStream $ getPartChildrenStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ControlSurface" /> if the part is an aerodynamic control surface, otherwisenull.
 -}
getPartControlSurface :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurface thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartControlSurfaceStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurfaceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]
    in  makeStream req

getPartControlSurfaceStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ControlSurface))
getPartControlSurfaceStream thisArg = requestStream $ getPartControlSurfaceStreamReq thisArg 

{-
 - The cost of the part, in units of funds.
 -}
getPartCost :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartCost thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCostStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartCostStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]
    in  makeStream req

getPartCostStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartCostStream thisArg = requestStream $ getPartCostStreamReq thisArg 

{-
 - Whether this part is crossfeed capable.
 -}
getPartCrossfeed :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartCrossfeed thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartCrossfeedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartCrossfeedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]
    in  makeStream req

getPartCrossfeedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartCrossfeedStream thisArg = requestStream $ getPartCrossfeedStreamReq thisArg 

{-
 - The stage in which this part will be decoupled. Returns -1 if the part is never decoupled from the vessel.
 -}
getPartDecoupleStage :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Int.Int32)
getPartDecoupleStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Int.Int32)
getPartDecoupleStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]
    in  makeStream req

getPartDecoupleStageStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Int.Int32))
getPartDecoupleStageStream thisArg = requestStream $ getPartDecoupleStageStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Decoupler" /> if the part is a decoupler, otherwisenull.
 -}
getPartDecoupler :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Decoupler)
getPartDecoupler thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDecouplerStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Decoupler)
getPartDecouplerStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]
    in  makeStream req

getPartDecouplerStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Decoupler))
getPartDecouplerStream thisArg = requestStream $ getPartDecouplerStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.DockingPort" /> if the part is a docking port, otherwisenull.
 -}
getPartDockingPort :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPort thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDockingPortStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPortStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]
    in  makeStream req

getPartDockingPortStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getPartDockingPortStream thisArg = requestStream $ getPartDockingPortStreamReq thisArg 

{-
 - The mass of the part, not including any resources it contains, in kilograms. Returns zero if the part is massless.
 -}
getPartDryMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDryMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartDryMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartDryMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartDryMassStream thisArg = requestStream $ getPartDryMassStreamReq thisArg 

{-
 - The dynamic pressure acting on the part, in Pascals.
 -}
getPartDynamicPressure :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartDynamicPressureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartDynamicPressureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]
    in  makeStream req

getPartDynamicPressureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartDynamicPressureStream thisArg = requestStream $ getPartDynamicPressureStreamReq thisArg 

{-
 - An <see cref="T:SpaceCenter.Engine" /> if the part is an engine, otherwisenull.
 -}
getPartEngine :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Engine)
getPartEngine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartEngineStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Engine)
getPartEngineStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]
    in  makeStream req

getPartEngineStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Engine))
getPartEngineStream thisArg = requestStream $ getPartEngineStreamReq thisArg 

{-
 - An <see cref="T:SpaceCenter.Experiment" /> if the part is a science experiment, otherwisenull.
 -}
getPartExperiment :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Experiment)
getPartExperiment thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Experiment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartExperimentStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Experiment)
getPartExperimentStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Experiment" [makeArgument 0 thisArg]
    in  makeStream req

getPartExperimentStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Experiment))
getPartExperimentStream thisArg = requestStream $ getPartExperimentStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Fairing" /> if the part is a fairing, otherwisenull.
 -}
getPartFairing :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Fairing)
getPartFairing thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartFairingStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Fairing)
getPartFairingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]
    in  makeStream req

getPartFairingStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Fairing))
getPartFairingStream thisArg = requestStream $ getPartFairingStreamReq thisArg 

{-
 - The parts that are connected to this part via fuel lines, where the direction of the fuel line is into this part.
 -}
getPartFuelLinesFrom :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFrom thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartFuelLinesFromStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFromStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]
    in  makeStream req

getPartFuelLinesFromStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesFromStream thisArg = requestStream $ getPartFuelLinesFromStreamReq thisArg 

{-
 - The parts that are connected to this part via fuel lines, where the direction of the fuel line is out of this part.
 -}
getPartFuelLinesTo :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesTo thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartFuelLinesToStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesToStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]
    in  makeStream req

getPartFuelLinesToStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesToStream thisArg = requestStream $ getPartFuelLinesToStreamReq thisArg 

{-
 - The impact tolerance of the part, in meters per second.
 -}
getPartImpactTolerance :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartImpactTolerance thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartImpactToleranceStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartImpactToleranceStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]
    in  makeStream req

getPartImpactToleranceStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartImpactToleranceStream thisArg = requestStream $ getPartImpactToleranceStreamReq thisArg 

{-
 - The inertia tensor of the part in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getPartInertiaTensor :: KRPCHS.SpaceCenter.Part -> RPCContext ([Double])
getPartInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartInertiaTensorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([Double])
getPartInertiaTensorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]
    in  makeStream req

getPartInertiaTensorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([Double]))
getPartInertiaTensorStream thisArg = requestStream $ getPartInertiaTensorStreamReq thisArg 

{-
 - An <see cref="T:SpaceCenter.Intake" /> if the part is an intake, otherwisenull.This includes any part that generates thrust. This covers many different types of engine,
 - including liquid fuel rockets, solid rocket boosters and jet engines.
 - For RCS thrusters see <see cref="T:SpaceCenter.RCS" />.
 -}
getPartIntake :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Intake)
getPartIntake thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartIntakeStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Intake)
getPartIntakeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]
    in  makeStream req

getPartIntakeStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Intake))
getPartIntakeStream thisArg = requestStream $ getPartIntakeStreamReq thisArg 

{-
 - Whether this part is a fuel line.
 -}
getPartIsFuelLine :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartIsFuelLine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartIsFuelLineStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartIsFuelLineStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]
    in  makeStream req

getPartIsFuelLineStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartIsFuelLineStream thisArg = requestStream $ getPartIsFuelLineStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.LandingGear" /> if the part is a landing gear, otherwisenull.
 -}
getPartLandingGear :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LandingGear)
getPartLandingGear thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingGear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLandingGearStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingGear)
getPartLandingGearStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_LandingGear" [makeArgument 0 thisArg]
    in  makeStream req

getPartLandingGearStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingGear))
getPartLandingGearStream thisArg = requestStream $ getPartLandingGearStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.LandingLeg" /> if the part is a landing leg, otherwisenull.
 -}
getPartLandingLeg :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LandingLeg)
getPartLandingLeg thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingLeg" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLandingLegStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LandingLeg)
getPartLandingLegStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_LandingLeg" [makeArgument 0 thisArg]
    in  makeStream req

getPartLandingLegStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingLeg))
getPartLandingLegStream thisArg = requestStream $ getPartLandingLegStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.LaunchClamp" /> if the part is a launch clamp, otherwisenull.
 -}
getPartLaunchClamp :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClamp thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLaunchClampStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClampStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]
    in  makeStream req

getPartLaunchClampStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LaunchClamp))
getPartLaunchClampStream thisArg = requestStream $ getPartLaunchClampStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Light" /> if the part is a light, otherwisenull.
 -}
getPartLight :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Light)
getPartLight thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartLightStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Light)
getPartLightStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]
    in  makeStream req

getPartLightStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Light))
getPartLightStream thisArg = requestStream $ getPartLightStreamReq thisArg 

{-
 - The current mass of the part, including resources it contains, in kilograms.
 - Returns zero if the part is massless.
 -}
getPartMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]
    in  makeStream req

getPartMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMassStream thisArg = requestStream $ getPartMassStreamReq thisArg 

{-
 - Whether the part is <a href="http://wiki.kerbalspaceprogram.com/wiki/Massless_part">massless.
 -}
getPartMassless :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartMassless thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMasslessStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartMasslessStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]
    in  makeStream req

getPartMasslessStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartMasslessStream thisArg = requestStream $ getPartMasslessStreamReq thisArg 

{-
 - Maximum temperature that the skin of the part can survive, in Kelvin.
 -}
getPartMaxSkinTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMaxSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMaxSkinTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMaxSkinTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartMaxSkinTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMaxSkinTemperatureStream thisArg = requestStream $ getPartMaxSkinTemperatureStreamReq thisArg 

{-
 - Maximum temperature that the part can survive, in Kelvin.
 -}
getPartMaxTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMaxTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMaxTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartMaxTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartMaxTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMaxTemperatureStream thisArg = requestStream $ getPartMaxTemperatureStreamReq thisArg 

{-
 - The modules for this part.
 -}
getPartModules :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Module])
getPartModules thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartModulesStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ([KRPCHS.SpaceCenter.Module])
getPartModulesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]
    in  makeStream req

getPartModulesStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Module]))
getPartModulesStream thisArg = requestStream $ getPartModulesStreamReq thisArg 

{-
 - The moment of inertia of the part inkg.m^2around its center of mass
 - in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 -}
getPartMomentOfInertia :: KRPCHS.SpaceCenter.Part -> RPCContext ((Double, Double, Double))
getPartMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartMomentOfInertiaStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq ((Double, Double, Double))
getPartMomentOfInertiaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]
    in  makeStream req

getPartMomentOfInertiaStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ((Double, Double, Double)))
getPartMomentOfInertiaStream thisArg = requestStream $ getPartMomentOfInertiaStreamReq thisArg 

{-
 - Internal name of the part, as used in
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation">part cfg files.
 - For example "Mark1-2Pod".
 -}
getPartName :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartName thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartNameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getPartNameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartNameStream thisArg = requestStream $ getPartNameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Parachute" /> if the part is a parachute, otherwisenull.
 -}
getPartParachute :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Parachute)
getPartParachute thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartParachuteStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Parachute)
getPartParachuteStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]
    in  makeStream req

getPartParachuteStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Parachute))
getPartParachuteStream thisArg = requestStream $ getPartParachuteStreamReq thisArg 

{-
 - The parts parent. Returnsnullif the part does not have a parent.
 - This, in combination with <see cref="M:SpaceCenter.Part.Children" />, can be used to traverse the vessels parts tree.
 -}
getPartParent :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartParent thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartParentStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartParentStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]
    in  makeStream req

getPartParentStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartParentStream thisArg = requestStream $ getPartParentStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.RCS" /> if the part is an RCS block/thruster, otherwisenull.
 -}
getPartRCS :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.RCS)
getPartRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartRCSStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.RCS)
getPartRCSStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]
    in  makeStream req

getPartRCSStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.RCS))
getPartRCSStream thisArg = requestStream $ getPartRCSStreamReq thisArg 

{-
 - Whether the part is radially attached to its parent, i.e. on the side of its parent.
 - If the part has no parent, returnsfalse.
 -}
getPartRadiallyAttached :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartRadiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartRadiallyAttachedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartRadiallyAttachedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]
    in  makeStream req

getPartRadiallyAttachedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartRadiallyAttachedStream thisArg = requestStream $ getPartRadiallyAttachedStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Radiator" /> if the part is a radiator, otherwisenull.
 -}
getPartRadiator :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Radiator)
getPartRadiator thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartRadiatorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Radiator)
getPartRadiatorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]
    in  makeStream req

getPartRadiatorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Radiator))
getPartRadiatorStream thisArg = requestStream $ getPartRadiatorStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ReactionWheel" /> if the part is a reaction wheel, otherwisenull.
 -}
getPartReactionWheel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartReactionWheelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]
    in  makeStream req

getPartReactionWheelStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReactionWheel))
getPartReactionWheelStream thisArg = requestStream $ getPartReactionWheelStreamReq thisArg 

{-
 - The reference frame that is fixed relative to this part, and centered on a fixed position within the part, defined by the parts model.
 - <list type="bullet">The origin is at the position of the part, as returned by <see cref="M:SpaceCenter.Part.Position" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
 - for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartReferenceFrame :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getPartReferenceFrameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartReferenceFrameStream thisArg = requestStream $ getPartReferenceFrameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ResourceConverter" /> if the part is a resource converter, otherwisenull.
 -}
getPartResourceConverter :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverter thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartResourceConverterStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]
    in  makeStream req

getPartResourceConverterStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceConverter))
getPartResourceConverterStream thisArg = requestStream $ getPartResourceConverterStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.ResourceHarvester" /> if the part is a resource harvester, otherwisenull.
 -}
getPartResourceHarvester :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvester thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartResourceHarvesterStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvesterStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]
    in  makeStream req

getPartResourceHarvesterStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvester))
getPartResourceHarvesterStream thisArg = requestStream $ getPartResourceHarvesterStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Resources" /> object for the part.
 -}
getPartResources :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Resources)
getPartResources thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartResourcesStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
getPartResourcesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]
    in  makeStream req

getPartResourcesStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
getPartResourcesStream thisArg = requestStream $ getPartResourcesStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Sensor" /> if the part is a sensor, otherwisenull.
 -}
getPartSensor :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Sensor)
getPartSensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartSensorStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Sensor)
getPartSensorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]
    in  makeStream req

getPartSensorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Sensor))
getPartSensorStream thisArg = requestStream $ getPartSensorStreamReq thisArg 

{-
 - Whether the part is shielded from the exterior of the vessel, for example by a fairing.
 -}
getPartShielded :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartShielded thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartShieldedStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Bool)
getPartShieldedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]
    in  makeStream req

getPartShieldedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartShieldedStream thisArg = requestStream $ getPartShieldedStreamReq thisArg 

{-
 - Temperature of the skin of the part, in Kelvin.
 -}
getPartSkinTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartSkinTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartSkinTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartSkinTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartSkinTemperatureStream thisArg = requestStream $ getPartSkinTemperatureStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.SolarPanel" /> if the part is a solar panel, otherwisenull.
 -}
getPartSolarPanel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartSolarPanelStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]
    in  makeStream req

getPartSolarPanelStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SolarPanel))
getPartSolarPanelStream thisArg = requestStream $ getPartSolarPanelStreamReq thisArg 

{-
 - The stage in which this part will be activated. Returns -1 if the part is not activated by staging.
 -}
getPartStage :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Int.Int32)
getPartStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartStageStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Int.Int32)
getPartStageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]
    in  makeStream req

getPartStageStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Int.Int32))
getPartStageStream thisArg = requestStream $ getPartStageStreamReq thisArg 

{-
 - Temperature of the part, in Kelvin.
 -}
getPartTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartTemperatureStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Double)
getPartTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]
    in  makeStream req

getPartTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartTemperatureStream thisArg = requestStream $ getPartTemperatureStreamReq thisArg 

{-
 - The rate at which heat energy is conducting into or out of the part via contact with other parts.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConductionFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalConductionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalConductionFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalConductionFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalConductionFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalConductionFluxStream thisArg = requestStream $ getPartThermalConductionFluxStreamReq thisArg 

{-
 - The rate at which heat energy is convecting into or out of the part from the surrounding atmosphere.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConvectionFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalConvectionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalConvectionFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalConvectionFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalConvectionFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalConvectionFluxStream thisArg = requestStream $ getPartThermalConvectionFluxStreamReq thisArg 

{-
 - The rate at which heat energy is begin generated by the part.
 - For example, some engines generate heat by combusting fuel.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalInternalFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalInternalFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalInternalFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalInternalFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalInternalFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalInternalFluxStream thisArg = requestStream $ getPartThermalInternalFluxStreamReq thisArg 

{-
 - A measure of how much energy it takes to increase the internal temperature of the part, in Joules per Kelvin.
 -}
getPartThermalMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalMassStream thisArg = requestStream $ getPartThermalMassStreamReq thisArg 

{-
 - The rate at which heat energy is radiating into or out of the part from the surrounding environment.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalRadiationFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalRadiationFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalRadiationFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalRadiationFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalRadiationFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalRadiationFluxStream thisArg = requestStream $ getPartThermalRadiationFluxStreamReq thisArg 

{-
 - A measure of how much energy it takes to increase the temperature of the resources contained in the part, in Joules per Kelvin.
 -}
getPartThermalResourceMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalResourceMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalResourceMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalResourceMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalResourceMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalResourceMassStream thisArg = requestStream $ getPartThermalResourceMassStreamReq thisArg 

{-
 - A measure of how much energy it takes to increase the skin temperature of the part, in Joules per Kelvin.
 -}
getPartThermalSkinMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalSkinMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalSkinMassStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalSkinMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalSkinMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinMassStream thisArg = requestStream $ getPartThermalSkinMassStreamReq thisArg 

{-
 - The rate at which heat energy is transferring between the part's skin and its internals.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part's internals are gaining heat energy,
 - and negative means its skin is gaining heat energy.
 -}
getPartThermalSkinToInternalFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalSkinToInternalFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartThermalSkinToInternalFluxStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Float)
getPartThermalSkinToInternalFluxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [makeArgument 0 thisArg]
    in  makeStream req

getPartThermalSkinToInternalFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinToInternalFluxStream thisArg = requestStream $ getPartThermalSkinToInternalFluxStreamReq thisArg 

{-
 - Title of the part, as shown when the part is right clicked in-game. For example "Mk1-2 Command Pod".
 -}
getPartTitle :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartTitle thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartTitleStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (Data.Text.Text)
getPartTitleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]
    in  makeStream req

getPartTitleStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartTitleStream thisArg = requestStream $ getPartTitleStreamReq thisArg 

{-
 - The vessel that contains this part.
 -}
getPartVessel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getPartVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartVesselStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getPartVesselStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]
    in  makeStream req

getPartVesselStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getPartVesselStream thisArg = requestStream $ getPartVesselStreamReq thisArg 

{-
 - The first docking port in the vessel with the given port name, as returned by <see cref="M:SpaceCenter.DockingPort.Name" />.
 - Returnsnullif there are no such docking ports.<param name="name">
 -}
partsDockingPortWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCHS.SpaceCenter.DockingPort)
partsDockingPortWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_DockingPortWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

partsDockingPortWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
partsDockingPortWithNameStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Parts_DockingPortWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

partsDockingPortWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
partsDockingPortWithNameStream thisArg nameArg = requestStream $ partsDockingPortWithNameStreamReq thisArg nameArg 

{-
 - A list of all parts that are decoupled in the given <paramref name="stage" />.<param name="stage">
 -}
partsInDecoupleStage :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    res <- sendRequest r
    processResponse res

partsInDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStageStreamReq thisArg stageArg =
    let req = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    in  makeStream req

partsInDecoupleStageStream :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInDecoupleStageStream thisArg stageArg = requestStream $ partsInDecoupleStageStreamReq thisArg stageArg 

{-
 - A list of all parts that are activated in the given <paramref name="stage" />.<param name="stage">
 -}
partsInStage :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsInStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    res <- sendRequest r
    processResponse res

partsInStageStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsInStageStreamReq thisArg stageArg =
    let req = makeRequest "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    in  makeStream req

partsInStageStream :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInStageStream thisArg stageArg = requestStream $ partsInStageStreamReq thisArg stageArg 

{-
 - A list of modules (combined across all parts in the vessel) whose
 - <see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsModulesWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Module])
partsModulesWithName thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    res <- sendRequest r
    processResponse res

partsModulesWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Module])
partsModulesWithNameStreamReq thisArg moduleNameArg =
    let req = makeRequest "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    in  makeStream req

partsModulesWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Module]))
partsModulesWithNameStream thisArg moduleNameArg = requestStream $ partsModulesWithNameStreamReq thisArg moduleNameArg 

{-
 - A list of all parts that contain a <see cref="T:SpaceCenter.Module" /> whose
 - <see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsWithModule :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithModule thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    res <- sendRequest r
    processResponse res

partsWithModuleStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithModuleStreamReq thisArg moduleNameArg =
    let req = makeRequest "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    in  makeStream req

partsWithModuleStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithModuleStream thisArg moduleNameArg = requestStream $ partsWithModuleStreamReq thisArg moduleNameArg 

{-
 - A list of parts whose <see cref="M:SpaceCenter.Part.Name" /> is <paramref name="name" />.<param name="name">
 -}
partsWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

partsWithNameStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithNameStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

partsWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithNameStream thisArg nameArg = requestStream $ partsWithNameStreamReq thisArg nameArg 

{-
 - A list of all parts whose <see cref="M:SpaceCenter.Part.Title" /> is <paramref name="title" />.<param name="title">
 -}
partsWithTitle :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithTitle thisArg titleArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]
    res <- sendRequest r
    processResponse res

partsWithTitleStreamReq :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
partsWithTitleStreamReq thisArg titleArg =
    let req = makeRequest "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]
    in  makeStream req

partsWithTitleStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithTitleStream thisArg titleArg = requestStream $ partsWithTitleStreamReq thisArg titleArg 

{-
 - A list of all of the vessels parts.
 -}
getPartsAll :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartsAll thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsAllStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Part])
getPartsAllStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]
    in  makeStream req

getPartsAllStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartsAllStream thisArg = requestStream $ getPartsAllStreamReq thisArg 

{-
 - A list of all cargo bays in the vessel.
 -}
getPartsCargoBays :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBays thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsCargoBaysStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBaysStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]
    in  makeStream req

getPartsCargoBaysStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.CargoBay]))
getPartsCargoBaysStream thisArg = requestStream $ getPartsCargoBaysStreamReq thisArg 

{-
 - A list of all control surfaces in the vessel.
 -}
getPartsControlSurfaces :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfaces thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsControlSurfacesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfacesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]
    in  makeStream req

getPartsControlSurfacesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ControlSurface]))
getPartsControlSurfacesStream thisArg = requestStream $ getPartsControlSurfacesStreamReq thisArg 

{-
 - The part from which the vessel is controlled.
 -}
getPartsControlling :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartsControlling thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsControllingStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartsControllingStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]
    in  makeStream req

getPartsControllingStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsControllingStream thisArg = requestStream $ getPartsControllingStreamReq thisArg 

{-
 - A list of all decouplers in the vessel.
 -}
getPartsDecouplers :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplers thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsDecouplersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]
    in  makeStream req

getPartsDecouplersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Decoupler]))
getPartsDecouplersStream thisArg = requestStream $ getPartsDecouplersStreamReq thisArg 

{-
 - A list of all docking ports in the vessel.
 -}
getPartsDockingPorts :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPorts thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsDockingPortsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPortsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]
    in  makeStream req

getPartsDockingPortsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.DockingPort]))
getPartsDockingPortsStream thisArg = requestStream $ getPartsDockingPortsStreamReq thisArg 

{-
 - A list of all engines in the vessel.This includes any part that generates thrust. This covers many different types of engine,
 - including liquid fuel rockets, solid rocket boosters, jet engines and RCS thrusters.
 -}
getPartsEngines :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Engine])
getPartsEngines thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsEnginesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Engine])
getPartsEnginesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]
    in  makeStream req

getPartsEnginesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Engine]))
getPartsEnginesStream thisArg = requestStream $ getPartsEnginesStreamReq thisArg 

{-
 - A list of all science experiments in the vessel.
 -}
getPartsExperiments :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Experiment])
getPartsExperiments thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Experiments" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsExperimentsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Experiment])
getPartsExperimentsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Experiments" [makeArgument 0 thisArg]
    in  makeStream req

getPartsExperimentsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Experiment]))
getPartsExperimentsStream thisArg = requestStream $ getPartsExperimentsStreamReq thisArg 

{-
 - A list of all fairings in the vessel.
 -}
getPartsFairings :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Fairing])
getPartsFairings thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsFairingsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Fairing])
getPartsFairingsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]
    in  makeStream req

getPartsFairingsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Fairing]))
getPartsFairingsStream thisArg = requestStream $ getPartsFairingsStreamReq thisArg 

{-
 - A list of all intakes in the vessel.
 -}
getPartsIntakes :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Intake])
getPartsIntakes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsIntakesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Intake])
getPartsIntakesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]
    in  makeStream req

getPartsIntakesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Intake]))
getPartsIntakesStream thisArg = requestStream $ getPartsIntakesStreamReq thisArg 

{-
 - A list of all landing gear attached to the vessel.
 -}
getPartsLandingGear :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LandingGear])
getPartsLandingGear thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingGear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLandingGearStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LandingGear])
getPartsLandingGearStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_LandingGear" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLandingGearStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LandingGear]))
getPartsLandingGearStream thisArg = requestStream $ getPartsLandingGearStreamReq thisArg 

{-
 - A list of all landing legs attached to the vessel.
 -}
getPartsLandingLegs :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LandingLeg])
getPartsLandingLegs thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingLegs" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLandingLegsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LandingLeg])
getPartsLandingLegsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_LandingLegs" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLandingLegsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LandingLeg]))
getPartsLandingLegsStream thisArg = requestStream $ getPartsLandingLegsStreamReq thisArg 

{-
 - A list of all launch clamps attached to the vessel.
 -}
getPartsLaunchClamps :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClamps thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLaunchClampsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClampsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLaunchClampsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LaunchClamp]))
getPartsLaunchClampsStream thisArg = requestStream $ getPartsLaunchClampsStreamReq thisArg 

{-
 - A list of all lights in the vessel.
 -}
getPartsLights :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Light])
getPartsLights thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsLightsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Light])
getPartsLightsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]
    in  makeStream req

getPartsLightsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Light]))
getPartsLightsStream thisArg = requestStream $ getPartsLightsStreamReq thisArg 

{-
 - A list of all parachutes in the vessel.
 -}
getPartsParachutes :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsParachutesStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]
    in  makeStream req

getPartsParachutesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Parachute]))
getPartsParachutesStream thisArg = requestStream $ getPartsParachutesStreamReq thisArg 

{-
 - A list of all RCS blocks/thrusters in the vessel.
 -}
getPartsRCS :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.RCS])
getPartsRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsRCSStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.RCS])
getPartsRCSStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]
    in  makeStream req

getPartsRCSStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.RCS]))
getPartsRCSStream thisArg = requestStream $ getPartsRCSStreamReq thisArg 

{-
 - A list of all radiators in the vessel.
 -}
getPartsRadiators :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiators thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsRadiatorsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiatorsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]
    in  makeStream req

getPartsRadiatorsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Radiator]))
getPartsRadiatorsStream thisArg = requestStream $ getPartsRadiatorsStreamReq thisArg 

{-
 - A list of all reaction wheels in the vessel.
 -}
getPartsReactionWheels :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsReactionWheelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]
    in  makeStream req

getPartsReactionWheelsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ReactionWheel]))
getPartsReactionWheelsStream thisArg = requestStream $ getPartsReactionWheelsStreamReq thisArg 

{-
 - A list of all resource converters in the vessel.
 -}
getPartsResourceConverters :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConverters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsResourceConvertersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConvertersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]
    in  makeStream req

getPartsResourceConvertersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ResourceConverter]))
getPartsResourceConvertersStream thisArg = requestStream $ getPartsResourceConvertersStreamReq thisArg 

{-
 - A list of all resource harvesters in the vessel.
 -}
getPartsResourceHarvesters :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvesters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsResourceHarvestersStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvestersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]
    in  makeStream req

getPartsResourceHarvestersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ResourceHarvester]))
getPartsResourceHarvestersStream thisArg = requestStream $ getPartsResourceHarvestersStreamReq thisArg 

{-
 - The vessels root part.
 -}
getPartsRoot :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartsRoot thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsRootStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getPartsRootStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]
    in  makeStream req

getPartsRootStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsRootStream thisArg = requestStream $ getPartsRootStreamReq thisArg 

{-
 - A list of all sensors in the vessel.
 -}
getPartsSensors :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Sensor])
getPartsSensors thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsSensorsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.Sensor])
getPartsSensorsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]
    in  makeStream req

getPartsSensorsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Sensor]))
getPartsSensorsStream thisArg = requestStream $ getPartsSensorsStreamReq thisArg 

{-
 - A list of all solar panels in the vessel.
 -}
getPartsSolarPanels :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPartsSolarPanelsStreamReq :: KRPCHS.SpaceCenter.Parts -> KRPCStreamReq ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanelsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]
    in  makeStream req

getPartsSolarPanelsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.SolarPanel]))
getPartsSolarPanelsStream thisArg = requestStream $ getPartsSolarPanelsStreamReq thisArg 

{-
 - The part from which the vessel is controlled.
 -}
setPartsControlling :: KRPCHS.SpaceCenter.Parts -> KRPCHS.SpaceCenter.Part -> RPCContext ()
setPartsControlling thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parts_set_Controlling" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The reachable resources connected to this propellant.
 -}
getPropellantConnectedResources :: KRPCHS.SpaceCenter.Propellant -> RPCContext ([KRPCHS.SpaceCenter.Resource])
getPropellantConnectedResources thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_ConnectedResources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantConnectedResourcesStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
getPropellantConnectedResourcesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_ConnectedResources" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantConnectedResourcesStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
getPropellantConnectedResourcesStream thisArg = requestStream $ getPropellantConnectedResourcesStreamReq thisArg 

{-
 - The current amount of propellant.
 -}
getPropellantCurrentAmount :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantCurrentAmount thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_CurrentAmount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantCurrentAmountStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantCurrentAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_CurrentAmount" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantCurrentAmountStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantCurrentAmountStream thisArg = requestStream $ getPropellantCurrentAmountStreamReq thisArg 

{-
 - The required amount of propellant.
 -}
getPropellantCurrentRequirement :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantCurrentRequirement thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_CurrentRequirement" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantCurrentRequirementStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantCurrentRequirementStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_CurrentRequirement" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantCurrentRequirementStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantCurrentRequirementStream thisArg = requestStream $ getPropellantCurrentRequirementStreamReq thisArg 

{-
 - If this propellant has a stack gauge or not.
 -}
getPropellantDrawStackGauge :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantDrawStackGauge thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_DrawStackGauge" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantDrawStackGaugeStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantDrawStackGaugeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_DrawStackGauge" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantDrawStackGaugeStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantDrawStackGaugeStream thisArg = requestStream $ getPropellantDrawStackGaugeStreamReq thisArg 

{-
 - If this propellant should be ignored when calculating required mass flow given specific impulse.
 -}
getPropellantIgnoreForIsp :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantIgnoreForIsp thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_IgnoreForIsp" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantIgnoreForIspStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIgnoreForIspStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_IgnoreForIsp" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantIgnoreForIspStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantIgnoreForIspStream thisArg = requestStream $ getPropellantIgnoreForIspStreamReq thisArg 

{-
 - If this propellant should be ignored for thrust curve calculations.
 -}
getPropellantIgnoreForThrustCurve :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantIgnoreForThrustCurve thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_IgnoreForThrustCurve" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantIgnoreForThrustCurveStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIgnoreForThrustCurveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_IgnoreForThrustCurve" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantIgnoreForThrustCurveStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantIgnoreForThrustCurveStream thisArg = requestStream $ getPropellantIgnoreForThrustCurveStreamReq thisArg 

{-
 - If this propellant is deprived.
 -}
getPropellantIsDeprived :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Bool)
getPropellantIsDeprived thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_IsDeprived" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantIsDeprivedStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Bool)
getPropellantIsDeprivedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_IsDeprived" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantIsDeprivedStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Bool))
getPropellantIsDeprivedStream thisArg = requestStream $ getPropellantIsDeprivedStreamReq thisArg 

{-
 - The name of the propellant.
 -}
getPropellantName :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Data.Text.Text)
getPropellantName thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantNameStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Data.Text.Text)
getPropellantNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantNameStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Data.Text.Text))
getPropellantNameStream thisArg = requestStream $ getPropellantNameStreamReq thisArg 

{-
 - The propellant ratio.
 -}
getPropellantRatio :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Float)
getPropellantRatio thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_Ratio" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantRatioStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Float)
getPropellantRatioStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_Ratio" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantRatioStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Float))
getPropellantRatioStream thisArg = requestStream $ getPropellantRatioStreamReq thisArg 

{-
 - The total amount of the underlying resource currently reachable given resource flow rules.
 -}
getPropellantTotalResourceAvailable :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantTotalResourceAvailable thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_TotalResourceAvailable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantTotalResourceAvailableStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantTotalResourceAvailableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_TotalResourceAvailable" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantTotalResourceAvailableStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantTotalResourceAvailableStream thisArg = requestStream $ getPropellantTotalResourceAvailableStreamReq thisArg 

{-
 - The total vehicle capacity for the underlying propellant resource, restricted by resource flow rules.
 -}
getPropellantTotalResourceCapacity :: KRPCHS.SpaceCenter.Propellant -> RPCContext (Double)
getPropellantTotalResourceCapacity thisArg = do
    let r = makeRequest "SpaceCenter" "Propellant_get_TotalResourceCapacity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPropellantTotalResourceCapacityStreamReq :: KRPCHS.SpaceCenter.Propellant -> KRPCStreamReq (Double)
getPropellantTotalResourceCapacityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Propellant_get_TotalResourceCapacity" [makeArgument 0 thisArg]
    in  makeStream req

getPropellantTotalResourceCapacityStream :: KRPCHS.SpaceCenter.Propellant -> RPCContext (KRPCStream (Double))
getPropellantTotalResourceCapacityStream thisArg = requestStream $ getPropellantTotalResourceCapacityStreamReq thisArg 

{-
 - Load a quicksave.This is the same as calling <see cref="M:SpaceCenter.Load" /> with the name "quicksave".
 -}
quickload :: RPCContext ()
quickload  = do
    let r = makeRequest "SpaceCenter" "Quickload" []
    res <- sendRequest r
    processResponse res 

{-
 - Save a quicksave.This is the same as calling <see cref="M:SpaceCenter.Save" /> with the name "quicksave".
 -}
quicksave :: RPCContext ()
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
getRCSActive :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSActive thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSActiveStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getRCSActiveStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSActiveStream thisArg = requestStream $ getRCSActiveStreamReq thisArg 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the RCS is inactive.
 -}
getRCSAvailableTorque :: KRPCHS.SpaceCenter.RCS -> RPCContext ((Double, Double, Double))
getRCSAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ((Double, Double, Double))
getRCSAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getRCSAvailableTorqueStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ((Double, Double, Double)))
getRCSAvailableTorqueStream thisArg = requestStream $ getRCSAvailableTorqueStreamReq thisArg 

{-
 - Whether the RCS thrusters are enabled.
 -}
getRCSEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSEnabledStream thisArg = requestStream $ getRCSEnabledStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSForwardEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSForwardEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSForwardEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSForwardEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSForwardEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSForwardEnabledStream thisArg = requestStream $ getRCSForwardEnabledStreamReq thisArg 

{-
 - Whether the RCS has fuel available.The RCS thruster must be activated for this property to update correctly.
 -}
getRCSHasFuel :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSHasFuelStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSHasFuelStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]
    in  makeStream req

getRCSHasFuelStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSHasFuelStream thisArg = requestStream $ getRCSHasFuelStreamReq thisArg 

{-
 - The specific impulse of the RCS at sea level on Kerbin, in seconds.
 -}
getRCSKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSKerbinSeaLevelSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getRCSKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSKerbinSeaLevelSpecificImpulseStream thisArg = requestStream $ getRCSKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-
 - The maximum amount of thrust that can be produced by the RCS thrusters when active, in Newtons.
 -}
getRCSMaxThrust :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSMaxThrustStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSMaxThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]
    in  makeStream req

getRCSMaxThrustStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSMaxThrustStream thisArg = requestStream $ getRCSMaxThrustStreamReq thisArg 

{-
 - The maximum amount of thrust that can be produced by the RCS thrusters when active in a vacuum, in Newtons.
 -}
getRCSMaxVacuumThrust :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSMaxVacuumThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    in  makeStream req

getRCSMaxVacuumThrustStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSMaxVacuumThrustStream thisArg = requestStream $ getRCSMaxVacuumThrustStreamReq thisArg 

{-
 - The part object for this RCS.
 -}
getRCSPart :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCHS.SpaceCenter.Part)
getRCSPart thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPartStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getRCSPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPartStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getRCSPartStream thisArg = requestStream $ getRCSPartStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSPitchEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSPitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPitchEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSPitchEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPitchEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSPitchEnabledStream thisArg = requestStream $ getRCSPitchEnabledStreamReq thisArg 

{-
 - The ratios of resources that the RCS consumes. A dictionary mapping resource names
 - to the ratios at which they are consumed by the RCS.
 -}
getRCSPropellantRatios :: KRPCHS.SpaceCenter.RCS -> RPCContext (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPropellantRatiosStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatiosStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPropellantRatiosStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getRCSPropellantRatiosStream thisArg = requestStream $ getRCSPropellantRatiosStreamReq thisArg 

{-
 - The names of resources that the RCS consumes.
 -}
getRCSPropellants :: KRPCHS.SpaceCenter.RCS -> RPCContext ([Data.Text.Text])
getRCSPropellants thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSPropellantsStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ([Data.Text.Text])
getRCSPropellantsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]
    in  makeStream req

getRCSPropellantsStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ([Data.Text.Text]))
getRCSPropellantsStream thisArg = requestStream $ getRCSPropellantsStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRightEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSRightEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSRightEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSRightEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSRightEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSRightEnabledStream thisArg = requestStream $ getRCSRightEnabledStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRollEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSRollEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSRollEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSRollEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSRollEnabledStream thisArg = requestStream $ getRCSRollEnabledStreamReq thisArg 

{-
 - The current specific impulse of the RCS, in seconds. Returns zero
 - if the RCS is not active.
 -}
getRCSSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getRCSSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSSpecificImpulseStream thisArg = requestStream $ getRCSSpecificImpulseStreamReq thisArg 

{-
 - A list of thrusters, one of each nozzel in the RCS part.
 -}
getRCSThrusters :: KRPCHS.SpaceCenter.RCS -> RPCContext ([KRPCHS.SpaceCenter.Thruster])
getRCSThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSThrustersStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq ([KRPCHS.SpaceCenter.Thruster])
getRCSThrustersStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]
    in  makeStream req

getRCSThrustersStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getRCSThrustersStream thisArg = requestStream $ getRCSThrustersStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSUpEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSUpEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSUpEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSUpEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSUpEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSUpEnabledStream thisArg = requestStream $ getRCSUpEnabledStreamReq thisArg 

{-
 - The vacuum specific impulse of the RCS, in seconds.
 -}
getRCSVacuumSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Float)
getRCSVacuumSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getRCSVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSVacuumSpecificImpulseStream thisArg = requestStream $ getRCSVacuumSpecificImpulseStreamReq thisArg 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSYawEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRCSYawEnabledStreamReq :: KRPCHS.SpaceCenter.RCS -> KRPCStreamReq (Bool)
getRCSYawEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]
    in  makeStream req

getRCSYawEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSYawEnabledStream thisArg = requestStream $ getRCSYawEnabledStreamReq thisArg 

{-
 - Whether the RCS thrusters are enabled.
 -}
setRCSEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSForwardEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSForwardEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_ForwardEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSPitchEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSPitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRightEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSRightEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RightEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRollEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSUpEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSUpEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_UpEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSYawEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext ()
setRCSYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the radiator is deployable.
 -}
getRadiatorDeployable :: KRPCHS.SpaceCenter.Radiator -> RPCContext (Bool)
getRadiatorDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorDeployableStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (Bool)
getRadiatorDeployableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorDeployableStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployableStream thisArg = requestStream $ getRadiatorDeployableStreamReq thisArg 

{-
 - For a deployable radiator,trueif the radiator is extended.
 - If the radiator is not deployable, this is alwaystrue.
 -}
getRadiatorDeployed :: KRPCHS.SpaceCenter.Radiator -> RPCContext (Bool)
getRadiatorDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorDeployedStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (Bool)
getRadiatorDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorDeployedStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployedStream thisArg = requestStream $ getRadiatorDeployedStreamReq thisArg 

{-
 - The part object for this radiator.
 -}
getRadiatorPart :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCHS.SpaceCenter.Part)
getRadiatorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorPartStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getRadiatorPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorPartStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getRadiatorPartStream thisArg = requestStream $ getRadiatorPartStreamReq thisArg 

{-
 - The current state of the radiator.A fixed radiator is always <see cref="M:SpaceCenter.RadiatorState.Extended" />.
 -}
getRadiatorState :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorState thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRadiatorStateStreamReq :: KRPCHS.SpaceCenter.Radiator -> KRPCStreamReq (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getRadiatorStateStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.RadiatorState))
getRadiatorStateStream thisArg = requestStream $ getRadiatorStateStreamReq thisArg 

{-
 - For a deployable radiator,trueif the radiator is extended.
 - If the radiator is not deployable, this is alwaystrue.
 -}
setRadiatorDeployed :: KRPCHS.SpaceCenter.Radiator -> Bool -> RPCContext ()
setRadiatorDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Radiator_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the reaction wheel is active.
 -}
getReactionWheelActive :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (Bool)
getReactionWheelActive thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelActiveStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (Bool)
getReactionWheelActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelActiveStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelActiveStream thisArg = requestStream $ getReactionWheelActiveStreamReq thisArg 

{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the reaction wheel is inactive or broken.
 -}
getReactionWheelAvailableTorque :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq ((Double, Double, Double))
getReactionWheelAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelAvailableTorqueStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelAvailableTorqueStream thisArg = requestStream $ getReactionWheelAvailableTorqueStreamReq thisArg 

{-
 - Whether the reaction wheel is broken.
 -}
getReactionWheelBroken :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (Bool)
getReactionWheelBroken thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelBrokenStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (Bool)
getReactionWheelBrokenStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelBrokenStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelBrokenStream thisArg = requestStream $ getReactionWheelBrokenStreamReq thisArg 

{-
 - The maximum torque the reaction wheel can provide, is it active,
 - in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getReactionWheelMaxTorque :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelMaxTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelMaxTorqueStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq ((Double, Double, Double))
getReactionWheelMaxTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelMaxTorqueStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelMaxTorqueStream thisArg = requestStream $ getReactionWheelMaxTorqueStreamReq thisArg 

{-
 - The part object for this reaction wheel.
 -}
getReactionWheelPart :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCHS.SpaceCenter.Part)
getReactionWheelPart thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getReactionWheelPartStreamReq :: KRPCHS.SpaceCenter.ReactionWheel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getReactionWheelPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getReactionWheelPartStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getReactionWheelPartStream thisArg = requestStream $ getReactionWheelPartStreamReq thisArg 

{-
 - Whether the reaction wheel is active.
 -}
setReactionWheelActive :: KRPCHS.SpaceCenter.ReactionWheel -> Bool -> RPCContext ()
setReactionWheelActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - True if the specified converter is active.<param name="index">Index of the converter.
 -}
resourceConverterActive :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Bool)
resourceConverterActive thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterActiveStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Bool)
resourceConverterActiveStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterActiveStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Bool))
resourceConverterActiveStream thisArg indexArg = requestStream $ resourceConverterActiveStreamReq thisArg indexArg 

{-
 - List of the names of resources consumed by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterInputs :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ([Data.Text.Text])
resourceConverterInputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterInputsStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq ([Data.Text.Text])
resourceConverterInputsStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterInputsStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream ([Data.Text.Text]))
resourceConverterInputsStream thisArg indexArg = requestStream $ resourceConverterInputsStreamReq thisArg indexArg 

{-
 - The name of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterName :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Data.Text.Text)
resourceConverterName thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterNameStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Data.Text.Text)
resourceConverterNameStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterNameStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Data.Text.Text))
resourceConverterNameStream thisArg indexArg = requestStream $ resourceConverterNameStreamReq thisArg indexArg 

{-
 - List of the names of resources produced by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterOutputs :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ([Data.Text.Text])
resourceConverterOutputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterOutputsStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq ([Data.Text.Text])
resourceConverterOutputsStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterOutputsStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream ([Data.Text.Text]))
resourceConverterOutputsStream thisArg indexArg = requestStream $ resourceConverterOutputsStreamReq thisArg indexArg 

{-
 - Start the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStart :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ()
resourceConverterStart thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Start" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res 

{-
 - The state of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterState :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterState thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterStateStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterStateStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterStateStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceConverterState))
resourceConverterStateStream thisArg indexArg = requestStream $ resourceConverterStateStreamReq thisArg indexArg 

{-
 - Status information for the specified converter.
 - This is the full status message shown in the in-game UI.<param name="index">Index of the converter.
 -}
resourceConverterStatusInfo :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Data.Text.Text)
resourceConverterStatusInfo thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res

resourceConverterStatusInfoStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> KRPCStreamReq (Data.Text.Text)
resourceConverterStatusInfoStreamReq thisArg indexArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    in  makeStream req

resourceConverterStatusInfoStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Data.Text.Text))
resourceConverterStatusInfoStream thisArg indexArg = requestStream $ resourceConverterStatusInfoStreamReq thisArg indexArg 

{-
 - Stop the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStop :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ()
resourceConverterStop thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Stop" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse res 

{-
 - The number of converters in the part.
 -}
getResourceConverterCount :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (Data.Int.Int32)
getResourceConverterCount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceConverterCountStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCStreamReq (Data.Int.Int32)
getResourceConverterCountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]
    in  makeStream req

getResourceConverterCountStream :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCStream (Data.Int.Int32))
getResourceConverterCountStream thisArg = requestStream $ getResourceConverterCountStreamReq thisArg 

{-
 - The part object for this converter.
 -}
getResourceConverterPart :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourceConverterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceConverterPartStreamReq :: KRPCHS.SpaceCenter.ResourceConverter -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourceConverterPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getResourceConverterPartStream :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceConverterPartStream thisArg = requestStream $ getResourceConverterPartStreamReq thisArg 

{-
 - Whether the harvester is actively drilling.
 -}
getResourceHarvesterActive :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterActive thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterActiveStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Bool)
getResourceHarvesterActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterActiveStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterActiveStream thisArg = requestStream $ getResourceHarvesterActiveStreamReq thisArg 

{-
 - The core temperature of the drill, in Kelvin.
 -}
getResourceHarvesterCoreTemperature :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterCoreTemperatureStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterCoreTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterCoreTemperatureStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterCoreTemperatureStream thisArg = requestStream $ getResourceHarvesterCoreTemperatureStreamReq thisArg 

{-
 - Whether the harvester is deployed.
 -}
getResourceHarvesterDeployed :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterDeployedStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Bool)
getResourceHarvesterDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterDeployedStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterDeployedStream thisArg = requestStream $ getResourceHarvesterDeployedStreamReq thisArg 

{-
 - The rate at which the drill is extracting ore, in units per second.
 -}
getResourceHarvesterExtractionRate :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterExtractionRate thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterExtractionRateStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterExtractionRateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterExtractionRateStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterExtractionRateStream thisArg = requestStream $ getResourceHarvesterExtractionRateStreamReq thisArg 

{-
 - The core temperature at which the drill will operate with peak efficiency, in Kelvin.
 -}
getResourceHarvesterOptimumCoreTemperature :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterOptimumCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterOptimumCoreTemperatureStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterOptimumCoreTemperatureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterOptimumCoreTemperatureStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterOptimumCoreTemperatureStream thisArg = requestStream $ getResourceHarvesterOptimumCoreTemperatureStreamReq thisArg 

{-
 - The part object for this harvester.
 -}
getResourceHarvesterPart :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterPartStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterPartStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceHarvesterPartStream thisArg = requestStream $ getResourceHarvesterPartStreamReq thisArg 

{-
 - The state of the harvester.
 -}
getResourceHarvesterState :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterState thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterStateStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterStateStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvesterState))
getResourceHarvesterStateStream thisArg = requestStream $ getResourceHarvesterStateStreamReq thisArg 

{-
 - The thermal efficiency of the drill, as a percentage of its maximum.
 -}
getResourceHarvesterThermalEfficiency :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterThermalEfficiency thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceHarvesterThermalEfficiencyStreamReq :: KRPCHS.SpaceCenter.ResourceHarvester -> KRPCStreamReq (Float)
getResourceHarvesterThermalEfficiencyStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]
    in  makeStream req

getResourceHarvesterThermalEfficiencyStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterThermalEfficiencyStream thisArg = requestStream $ getResourceHarvesterThermalEfficiencyStreamReq thisArg 

{-
 - Whether the harvester is actively drilling.
 -}
setResourceHarvesterActive :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> RPCContext ()
setResourceHarvesterActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the harvester is deployed.
 -}
setResourceHarvesterDeployed :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> RPCContext ()
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
resourceTransferStart :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> RPCContext (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStart fromPartArg toPartArg resourceArg maxAmountArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_Start" [makeArgument 0 fromPartArg, makeArgument 1 toPartArg, makeArgument 2 resourceArg, makeArgument 3 maxAmountArg]
    res <- sendRequest r
    processResponse res

resourceTransferStartStreamReq :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceTransfer)
resourceTransferStartStreamReq fromPartArg toPartArg resourceArg maxAmountArg =
    let req = makeRequest "SpaceCenter" "ResourceTransfer_Start" [makeArgument 0 fromPartArg, makeArgument 1 toPartArg, makeArgument 2 resourceArg, makeArgument 3 maxAmountArg]
    in  makeStream req

resourceTransferStartStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceTransfer))
resourceTransferStartStream fromPartArg toPartArg resourceArg maxAmountArg = requestStream $ resourceTransferStartStreamReq fromPartArg toPartArg resourceArg maxAmountArg 

{-
 - The amount of the resource that has been transferred.
 -}
getResourceTransferAmount :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (Float)
getResourceTransferAmount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceTransferAmountStreamReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCStreamReq (Float)
getResourceTransferAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]
    in  makeStream req

getResourceTransferAmountStream :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (KRPCStream (Float))
getResourceTransferAmountStream thisArg = requestStream $ getResourceTransferAmountStreamReq thisArg 

{-
 - Whether the transfer has completed.
 -}
getResourceTransferComplete :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (Bool)
getResourceTransferComplete thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceTransferCompleteStreamReq :: KRPCHS.SpaceCenter.ResourceTransfer -> KRPCStreamReq (Bool)
getResourceTransferCompleteStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]
    in  makeStream req

getResourceTransferCompleteStream :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (KRPCStream (Bool))
getResourceTransferCompleteStream thisArg = requestStream $ getResourceTransferCompleteStreamReq thisArg 

{-
 - The amount of the resource that is currently stored in the part.
 -}
getResourceAmount :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceAmount thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceAmountStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]
    in  makeStream req

getResourceAmountStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceAmountStream thisArg = requestStream $ getResourceAmountStreamReq thisArg 

{-
 - The density of the resource, inkg/l.
 -}
getResourceDensity :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceDensityStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceDensityStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]
    in  makeStream req

getResourceDensityStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceDensityStream thisArg = requestStream $ getResourceDensityStreamReq thisArg 

{-
 - Whether use of this resource is enabled.
 -}
getResourceEnabled :: KRPCHS.SpaceCenter.Resource -> RPCContext (Bool)
getResourceEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceEnabledStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Bool)
getResourceEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]
    in  makeStream req

getResourceEnabledStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Bool))
getResourceEnabledStream thisArg = requestStream $ getResourceEnabledStreamReq thisArg 

{-
 - The flow mode of the resource.
 -}
getResourceFlowMode :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowMode thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceFlowModeStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowModeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]
    in  makeStream req

getResourceFlowModeStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
getResourceFlowModeStream thisArg = requestStream $ getResourceFlowModeStreamReq thisArg 

{-
 - The total amount of the resource that can be stored in the part.
 -}
getResourceMax :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceMax thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceMaxStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Float)
getResourceMaxStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]
    in  makeStream req

getResourceMaxStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceMaxStream thisArg = requestStream $ getResourceMaxStreamReq thisArg 

{-
 - The name of the resource.
 -}
getResourceName :: KRPCHS.SpaceCenter.Resource -> RPCContext (Data.Text.Text)
getResourceName thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourceNameStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (Data.Text.Text)
getResourceNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getResourceNameStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Data.Text.Text))
getResourceNameStream thisArg = requestStream $ getResourceNameStreamReq thisArg 

{-
 - The part containing the resource.
 -}
getResourcePart :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourcePart thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcePartStreamReq :: KRPCHS.SpaceCenter.Resource -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getResourcePartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getResourcePartStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourcePartStream thisArg = requestStream $ getResourcePartStreamReq thisArg 

{-
 - Whether use of this resource is enabled.
 -}
setResourceEnabled :: KRPCHS.SpaceCenter.Resource -> Bool -> RPCContext ()
setResourceEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Resource_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Returns the amount of a resource that is currently stored.<param name="name">The name of the resource.
 -}
resourcesAmount :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Float)
resourcesAmount thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesAmountStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Float)
resourcesAmountStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesAmountStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesAmountStream thisArg nameArg = requestStream $ resourcesAmountStreamReq thisArg nameArg 

{-
 - Returns the density of a resource, in kg/l.<param name="name">The name of the resource.
 -}
resourcesDensity :: Data.Text.Text -> RPCContext (Float)
resourcesDensity nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Density" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res

resourcesDensityStreamReq :: Data.Text.Text -> KRPCStreamReq (Float)
resourcesDensityStreamReq nameArg =
    let req = makeRequest "SpaceCenter" "Resources_Density" [makeArgument 0 nameArg]
    in  makeStream req

resourcesDensityStream :: Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesDensityStream nameArg = requestStream $ resourcesDensityStreamReq nameArg 

{-
 - Returns the flow mode of a resource.<param name="name">The name of the resource.
 -}
resourcesFlowMode :: Data.Text.Text -> RPCContext (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesFlowMode nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_FlowMode" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res

resourcesFlowModeStreamReq :: Data.Text.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesFlowModeStreamReq nameArg =
    let req = makeRequest "SpaceCenter" "Resources_FlowMode" [makeArgument 0 nameArg]
    in  makeStream req

resourcesFlowModeStream :: Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
resourcesFlowModeStream nameArg = requestStream $ resourcesFlowModeStreamReq nameArg 

{-
 - Check whether the named resource can be stored.<param name="name">The name of the resource.
 -}
resourcesHasResource :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Bool)
resourcesHasResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesHasResourceStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Bool)
resourcesHasResourceStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesHasResourceStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
resourcesHasResourceStream thisArg nameArg = requestStream $ resourcesHasResourceStreamReq thisArg nameArg 

{-
 - Returns the amount of a resource that can be stored.<param name="name">The name of the resource.
 -}
resourcesMax :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Float)
resourcesMax thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesMaxStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq (Float)
resourcesMaxStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesMaxStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesMaxStream thisArg nameArg = requestStream $ resourcesMaxStreamReq thisArg nameArg 

{-
 - All the individual resources with the given name that can be stored.
 -}
resourcesWithResource :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Resource])
resourcesWithResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse res

resourcesWithResourceStreamReq :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
resourcesWithResourceStreamReq thisArg nameArg =
    let req = makeRequest "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    in  makeStream req

resourcesWithResourceStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
resourcesWithResourceStream thisArg nameArg = requestStream $ resourcesWithResourceStreamReq thisArg nameArg 

{-
 - All the individual resources that can be stored.
 -}
getResourcesAll :: KRPCHS.SpaceCenter.Resources -> RPCContext ([KRPCHS.SpaceCenter.Resource])
getResourcesAll thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcesAllStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq ([KRPCHS.SpaceCenter.Resource])
getResourcesAllStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]
    in  makeStream req

getResourcesAllStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
getResourcesAllStream thisArg = requestStream $ getResourcesAllStreamReq thisArg 

{-
 - Whether use of all the resources are enabled.This is true if all of the resources are enabled. If any of the resources are not enabled, this is false.
 -}
getResourcesEnabled :: KRPCHS.SpaceCenter.Resources -> RPCContext (Bool)
getResourcesEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcesEnabledStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq (Bool)
getResourcesEnabledStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resources_get_Enabled" [makeArgument 0 thisArg]
    in  makeStream req

getResourcesEnabledStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream (Bool))
getResourcesEnabledStream thisArg = requestStream $ getResourcesEnabledStreamReq thisArg 

{-
 - A list of resource names that can be stored.
 -}
getResourcesNames :: KRPCHS.SpaceCenter.Resources -> RPCContext ([Data.Text.Text])
getResourcesNames thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getResourcesNamesStreamReq :: KRPCHS.SpaceCenter.Resources -> KRPCStreamReq ([Data.Text.Text])
getResourcesNamesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]
    in  makeStream req

getResourcesNamesStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream ([Data.Text.Text]))
getResourcesNamesStream thisArg = requestStream $ getResourcesNamesStreamReq thisArg 

{-
 - Whether use of all the resources are enabled.This is true if all of the resources are enabled. If any of the resources are not enabled, this is false.
 -}
setResourcesEnabled :: KRPCHS.SpaceCenter.Resources -> Bool -> RPCContext ()
setResourcesEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Resources_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Save the game with a given name.
 - This will create a save file calledname.sfsin the folder of the current save game.
 -}
save :: Data.Text.Text -> RPCContext ()
save nameArg = do
    let r = makeRequest "SpaceCenter" "Save" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse res 

{-
 - Data amount.
 -}
getScienceDataDataAmount :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (Float)
getScienceDataDataAmount thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceData_get_DataAmount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceDataDataAmountStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataDataAmountStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceData_get_DataAmount" [makeArgument 0 thisArg]
    in  makeStream req

getScienceDataDataAmountStream :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (KRPCStream (Float))
getScienceDataDataAmountStream thisArg = requestStream $ getScienceDataDataAmountStreamReq thisArg 

{-
 - Science value.
 -}
getScienceDataScienceValue :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (Float)
getScienceDataScienceValue thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceData_get_ScienceValue" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceDataScienceValueStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataScienceValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceData_get_ScienceValue" [makeArgument 0 thisArg]
    in  makeStream req

getScienceDataScienceValueStream :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (KRPCStream (Float))
getScienceDataScienceValueStream thisArg = requestStream $ getScienceDataScienceValueStreamReq thisArg 

{-
 - Transmit value.
 -}
getScienceDataTransmitValue :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (Float)
getScienceDataTransmitValue thisArg = do
    let r = makeRequest "SpaceCenter" "ScienceData_get_TransmitValue" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getScienceDataTransmitValueStreamReq :: KRPCHS.SpaceCenter.ScienceData -> KRPCStreamReq (Float)
getScienceDataTransmitValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "ScienceData_get_TransmitValue" [makeArgument 0 thisArg]
    in  makeStream req

getScienceDataTransmitValueStream :: KRPCHS.SpaceCenter.ScienceData -> RPCContext (KRPCStream (Float))
getScienceDataTransmitValueStream thisArg = requestStream $ getScienceDataTransmitValueStreamReq thisArg 

{-
 - Whether the sensor is active.
 -}
getSensorActive :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Bool)
getSensorActive thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorActiveStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Bool)
getSensorActiveStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]
    in  makeStream req

getSensorActiveStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Bool))
getSensorActiveStream thisArg = requestStream $ getSensorActiveStreamReq thisArg 

{-
 - The part object for this sensor.
 -}
getSensorPart :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCHS.SpaceCenter.Part)
getSensorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorPartStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getSensorPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getSensorPartStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getSensorPartStream thisArg = requestStream $ getSensorPartStreamReq thisArg 

{-
 - The current power usage of the sensor, in units of charge per second.
 -}
getSensorPowerUsage :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Float)
getSensorPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_PowerUsage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorPowerUsageStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Float)
getSensorPowerUsageStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_PowerUsage" [makeArgument 0 thisArg]
    in  makeStream req

getSensorPowerUsageStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Float))
getSensorPowerUsageStream thisArg = requestStream $ getSensorPowerUsageStreamReq thisArg 

{-
 - The current value of the sensor.
 -}
getSensorValue :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Data.Text.Text)
getSensorValue thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSensorValueStreamReq :: KRPCHS.SpaceCenter.Sensor -> KRPCStreamReq (Data.Text.Text)
getSensorValueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]
    in  makeStream req

getSensorValueStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Data.Text.Text))
getSensorValueStream thisArg = requestStream $ getSensorValueStreamReq thisArg 

{-
 - Whether the sensor is active.
 -}
setSensorActive :: KRPCHS.SpaceCenter.Sensor -> Bool -> RPCContext ()
setSensorActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Sensor_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the solar panel is extended.
 -}
getSolarPanelDeployed :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Bool)
getSolarPanelDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelDeployedStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Bool)
getSolarPanelDeployedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelDeployedStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Bool))
getSolarPanelDeployedStream thisArg = requestStream $ getSolarPanelDeployedStreamReq thisArg 

{-
 - The current amount of energy being generated by the solar panel, in
 - units of charge per second.
 -}
getSolarPanelEnergyFlow :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Float)
getSolarPanelEnergyFlow thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelEnergyFlowStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Float)
getSolarPanelEnergyFlowStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelEnergyFlowStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelEnergyFlowStream thisArg = requestStream $ getSolarPanelEnergyFlowStreamReq thisArg 

{-
 - The part object for this solar panel.
 -}
getSolarPanelPart :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCHS.SpaceCenter.Part)
getSolarPanelPart thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelPartStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getSolarPanelPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelPartStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getSolarPanelPartStream thisArg = requestStream $ getSolarPanelPartStreamReq thisArg 

{-
 - The current state of the solar panel.
 -}
getSolarPanelState :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelState thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelStateStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelStateStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelStateStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SolarPanelState))
getSolarPanelStateStream thisArg = requestStream $ getSolarPanelStateStreamReq thisArg 

{-
 - The current amount of sunlight that is incident on the solar panel,
 - as a percentage. A value between 0 and 1.
 -}
getSolarPanelSunExposure :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Float)
getSolarPanelSunExposure thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getSolarPanelSunExposureStreamReq :: KRPCHS.SpaceCenter.SolarPanel -> KRPCStreamReq (Float)
getSolarPanelSunExposureStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]
    in  makeStream req

getSolarPanelSunExposureStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelSunExposureStream thisArg = requestStream $ getSolarPanelSunExposureStreamReq thisArg 

{-
 - Whether the solar panel is extended.
 -}
setSolarPanelDeployed :: KRPCHS.SpaceCenter.SolarPanel -> Bool -> RPCContext ()
setSolarPanelDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position around which the gimbal pivots.
 -}
thrusterGimbalPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterGimbalPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterGimbalPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterGimbalPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterGimbalPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterGimbalPositionStream thisArg referenceFrameArg = requestStream $ thrusterGimbalPositionStreamReq thisArg referenceFrameArg 

{-
 - The direction of the force generated by the thruster, when the engine is in its
 - initial position (no gimballing), in the given reference frame.
 - This is opposite to the direction in which the thruster expels propellant.<param name="referenceFrame">
 -}
thrusterInitialThrustDirection :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterInitialThrustDirectionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterInitialThrustDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterInitialThrustDirectionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustDirectionStream thisArg referenceFrameArg = requestStream $ thrusterInitialThrustDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position at which the thruster generates thrust, when the engine is in its
 - initial position (no gimballing), in the given reference frame.<param name="referenceFrame">This position can move when the gimbal rotates. This is because the thrust position and
 - gimbal position are not necessarily the same.
 -}
thrusterInitialThrustPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterInitialThrustPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterInitialThrustPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterInitialThrustPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustPositionStream thisArg referenceFrameArg = requestStream $ thrusterInitialThrustPositionStreamReq thisArg referenceFrameArg 

{-
 - The direction of the force generated by the thruster, in the given reference frame.
 - This is opposite to the direction in which the thruster expels propellant.
 - For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustDirection :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterThrustDirectionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterThrustDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterThrustDirectionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustDirectionStream thisArg referenceFrameArg = requestStream $ thrusterThrustDirectionStreamReq thisArg referenceFrameArg 

{-
 - The position at which the thruster generates thrust, in the given reference frame.
 - For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

thrusterThrustPositionStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
thrusterThrustPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

thrusterThrustPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustPositionStream thisArg referenceFrameArg = requestStream $ thrusterThrustPositionStreamReq thisArg referenceFrameArg 

{-
 - The current gimbal angle in the pitch, roll and yaw axes.
 -}
getThrusterGimbalAngle :: KRPCHS.SpaceCenter.Thruster -> RPCContext ((Double, Double, Double))
getThrusterGimbalAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterGimbalAngleStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq ((Double, Double, Double))
getThrusterGimbalAngleStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterGimbalAngleStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream ((Double, Double, Double)))
getThrusterGimbalAngleStream thisArg = requestStream $ getThrusterGimbalAngleStreamReq thisArg 

{-
 - Whether the thruster is gimballed.
 -}
getThrusterGimballed :: KRPCHS.SpaceCenter.Thruster -> RPCContext (Bool)
getThrusterGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterGimballedStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (Bool)
getThrusterGimballedStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterGimballedStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (Bool))
getThrusterGimballedStream thisArg = requestStream $ getThrusterGimballedStreamReq thisArg 

{-
 - The <see cref="T:SpaceCenter.Part" /> that contains this thruster.
 -}
getThrusterPart :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCHS.SpaceCenter.Part)
getThrusterPart thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterPartStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (KRPCHS.SpaceCenter.Part)
getThrusterPartStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterPartStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getThrusterPartStream thisArg = requestStream $ getThrusterPartStreamReq thisArg 

{-
 - A reference frame that is fixed relative to the thruster and orientated with
 - its thrust direction (<see cref="M:SpaceCenter.Thruster.ThrustDirection" />).
 - For gimballed engines, this takes into account the current rotation of the gimbal.
 - <list type="bullet">The origin is at the position of thrust for this thruster (<see cref="M:SpaceCenter.Thruster.ThrustPosition" />).The axes rotate with the thrust direction.
 - This is the direction in which the thruster expels propellant, including any gimballing.The y-axis points along the thrust direction.The x-axis and z-axis are perpendicular to the thrust direction.
 -}
getThrusterThrustReferenceFrame :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getThrusterThrustReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Thruster -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getThrusterThrustReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getThrusterThrustReferenceFrameStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getThrusterThrustReferenceFrameStream thisArg = requestStream $ getThrusterThrustReferenceFrameStreamReq thisArg 

{-
 - Converts a direction vector from one reference frame to another.<param name="direction">Direction vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the direction vector is in.<param name="to">The reference frame to covert the direction vector to.The corresponding direction vector in reference frame <paramref name="to" />.
 -}
transformDirection :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformDirection directionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse res

transformDirectionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformDirectionStreamReq directionArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    in  makeStream req

transformDirectionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformDirectionStream directionArg fromArg toArg = requestStream $ transformDirectionStreamReq directionArg fromArg toArg 

{-
 - Converts a position vector from one reference frame to another.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position vector is in.<param name="to">The reference frame to covert the position vector to.The corresponding position vector in reference frame <paramref name="to" />.
 -}
transformPosition :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformPosition positionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse res

transformPositionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformPositionStreamReq positionArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    in  makeStream req

transformPositionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformPositionStream positionArg fromArg toArg = requestStream $ transformPositionStreamReq positionArg fromArg toArg 

{-
 - Converts a rotation from one reference frame to another.<param name="rotation">Rotation in reference frame <paramref name="from" />.<param name="from">The reference frame that the rotation is in.<param name="to">The corresponding rotation in reference frame <paramref name="to" />.The corresponding rotation in reference frame <paramref name="to" />.
 -}
transformRotation :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
transformRotation rotationArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse res

transformRotationStreamReq :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
transformRotationStreamReq rotationArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    in  makeStream req

transformRotationStream :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
transformRotationStream rotationArg fromArg toArg = requestStream $ transformRotationStreamReq rotationArg fromArg toArg 

{-
 - Converts a velocity vector (acting at the specified position vector) from one
 - reference frame to another. The position vector is required to take the
 - relative angular velocity of the reference frames into account.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="velocity">Velocity vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position and velocity vectors are in.<param name="to">The reference frame to covert the velocity vector to.The corresponding velocity in reference frame <paramref name="to" />.
 -}
transformVelocity :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformVelocity positionArg velocityArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]
    res <- sendRequest r
    processResponse res

transformVelocityStreamReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
transformVelocityStreamReq positionArg velocityArg fromArg toArg =
    let req = makeRequest "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]
    in  makeStream req

transformVelocityStream :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformVelocityStream positionArg velocityArg fromArg toArg = requestStream $ transformVelocityStreamReq positionArg velocityArg fromArg toArg 

{-
 - Returns the angular velocity of the vessel in the given reference frame. The magnitude of the returned
 - vector is the rotational speed in radians per second, and the direction of the vector indicates the
 - axis of rotation (using the right hand rule).<param name="referenceFrame">
 -}
vesselAngularVelocity :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselAngularVelocityStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselAngularVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselAngularVelocityStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselAngularVelocityStream thisArg referenceFrameArg = requestStream $ vesselAngularVelocityStreamReq thisArg referenceFrameArg 

{-
 - Returns the direction in which the vessel is pointing, as a unit vector, in the given reference frame.<param name="referenceFrame">
 -}
vesselDirection :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselDirectionStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselDirectionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselDirectionStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselDirectionStream thisArg referenceFrameArg = requestStream $ vesselDirectionStreamReq thisArg referenceFrameArg 

{-
 - Returns a <see cref="T:SpaceCenter.Flight" /> object that can be used to get flight
 - telemetry for the vessel, in the specified reference frame.<param name="referenceFrame">
 - Reference frame. Defaults to the vessel's surface reference frame (<see cref="M:SpaceCenter.Vessel.SurfaceReferenceFrame" />).
 -}
vesselFlight :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCHS.SpaceCenter.Flight)
vesselFlight thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselFlightStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq (KRPCHS.SpaceCenter.Flight)
vesselFlightStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselFlightStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Flight))
vesselFlightStream thisArg referenceFrameArg = requestStream $ vesselFlightStreamReq thisArg referenceFrameArg 

{-
 - Returns the position vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselPosition :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselPositionStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselPositionStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselPositionStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselPositionStream thisArg referenceFrameArg = requestStream $ vesselPositionStreamReq thisArg referenceFrameArg 

{-
 - Recover the vessel.
 -}
vesselRecover :: KRPCHS.SpaceCenter.Vessel -> RPCContext ()
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
vesselResourcesInDecoupleStage :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> RPCContext (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStage thisArg stageArg cumulativeArg = do
    let r = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg, makeArgument 2 cumulativeArg]
    res <- sendRequest r
    processResponse res

vesselResourcesInDecoupleStageStreamReq :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
vesselResourcesInDecoupleStageStreamReq thisArg stageArg cumulativeArg =
    let req = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg, makeArgument 2 cumulativeArg]
    in  makeStream req

vesselResourcesInDecoupleStageStream :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
vesselResourcesInDecoupleStageStream thisArg stageArg cumulativeArg = requestStream $ vesselResourcesInDecoupleStageStreamReq thisArg stageArg cumulativeArg 

{-
 - Returns the rotation of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselRotation :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
vesselRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselRotationStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double, Double))
vesselRotationStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselRotationStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
vesselRotationStream thisArg referenceFrameArg = requestStream $ vesselRotationStreamReq thisArg referenceFrameArg 

{-
 - Returns the velocity vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselVelocity :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse res

vesselVelocityStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCStreamReq ((Double, Double, Double))
vesselVelocityStreamReq thisArg referenceFrameArg =
    let req = makeRequest "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    in  makeStream req

vesselVelocityStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselVelocityStream thisArg referenceFrameArg = requestStream $ vesselVelocityStreamReq thisArg referenceFrameArg 

{-
 - An <see cref="T:SpaceCenter.AutoPilot" /> object, that can be used to perform
 - simple auto-piloting of the vessel.
 -}
getVesselAutoPilot :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilot thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAutoPilotStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilotStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAutoPilotStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.AutoPilot))
getVesselAutoPilotStream thisArg = requestStream $ getVesselAutoPilotStreamReq thisArg 

{-
 - The maximum torque that the aerodynamic control surfaces can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableControlSurfaceTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableControlSurfaceTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableControlSurfaceTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableControlSurfaceTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableControlSurfaceTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableControlSurfaceTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableControlSurfaceTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableControlSurfaceTorqueStream thisArg = requestStream $ getVesselAvailableControlSurfaceTorqueStreamReq thisArg 

{-
 - The maximum torque that the currently active and gimballed engines can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableEngineTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableEngineTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableEngineTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableEngineTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableEngineTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableEngineTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableEngineTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableEngineTorqueStream thisArg = requestStream $ getVesselAvailableEngineTorqueStreamReq thisArg 

{-
 - The maximum torque that the currently active RCS thrusters can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableRCSTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableRCSTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableRCSTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableRCSTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableRCSTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableRCSTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableRCSTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableRCSTorqueStream thisArg = requestStream $ getVesselAvailableRCSTorqueStreamReq thisArg 

{-
 - The maximum torque that the currently active and powered reaction wheels can generate.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableReactionWheelTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableReactionWheelTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableReactionWheelTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableReactionWheelTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableReactionWheelTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableReactionWheelTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableReactionWheelTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableReactionWheelTorqueStream thisArg = requestStream $ getVesselAvailableReactionWheelTorqueStreamReq thisArg 

{-
 - Gets the total available thrust that can be produced by the vessel's
 - active engines, in Newtons. This is computed by summing
 - <see cref="M:SpaceCenter.Engine.AvailableThrust" /> for every active engine in the vessel.
 -}
getVesselAvailableThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselAvailableThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselAvailableThrustStream thisArg = requestStream $ getVesselAvailableThrustStreamReq thisArg 

{-
 - The maximum torque that the vessel generate. Includes contributions from reaction wheels,
 - RCS, gimballed engines and aerodynamic control surfaces.
 - Returns the torques inN.maround each of the coordinate axes of the
 - vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - These axes are equivalent to the pitch, roll and yaw axes of the vessel.
 -}
getVesselAvailableTorque :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselAvailableTorqueStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselAvailableTorqueStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_AvailableTorque" [makeArgument 0 thisArg]
    in  makeStream req

getVesselAvailableTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableTorqueStream thisArg = requestStream $ getVesselAvailableTorqueStreamReq thisArg 

{-
 - Returns a <see cref="T:SpaceCenter.Control" /> object that can be used to manipulate
 - the vessel's control inputs. For example, its pitch/yaw/roll controls,
 - RCS and thrust.
 -}
getVesselControl :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Control)
getVesselControl thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselControlStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Control)
getVesselControlStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]
    in  makeStream req

getVesselControlStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Control))
getVesselControlStream thisArg = requestStream $ getVesselControlStreamReq thisArg 

{-
 - The total mass of the vessel, excluding resources, in kg.
 -}
getVesselDryMass :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselDryMassStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselDryMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]
    in  makeStream req

getVesselDryMassStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselDryMassStream thisArg = requestStream $ getVesselDryMassStreamReq thisArg 

{-
 - The inertia tensor of the vessel around its center of mass, in the vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getVesselInertiaTensor :: KRPCHS.SpaceCenter.Vessel -> RPCContext ([Double])
getVesselInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselInertiaTensorStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ([Double])
getVesselInertiaTensorStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]
    in  makeStream req

getVesselInertiaTensorStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ([Double]))
getVesselInertiaTensorStream thisArg = requestStream $ getVesselInertiaTensorStreamReq thisArg 

{-
 - The combined specific impulse of all active engines at sea level on Kerbin, in seconds.
 - This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselKerbinSeaLevelSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselKerbinSeaLevelSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getVesselKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselKerbinSeaLevelSpecificImpulseStream thisArg = requestStream $ getVesselKerbinSeaLevelSpecificImpulseStreamReq thisArg 

{-
 - The mission elapsed time in seconds.
 -}
getVesselMET :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Double)
getVesselMET thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMETStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Double)
getVesselMETStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMETStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Double))
getVesselMETStream thisArg = requestStream $ getVesselMETStreamReq thisArg 

{-
 - The total mass of the vessel, including resources, in kg.
 -}
getVesselMass :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMassStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMassStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMassStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMassStream thisArg = requestStream $ getVesselMassStreamReq thisArg 

{-
 - The total maximum thrust that can be produced by the vessel's active
 - engines, in Newtons. This is computed by summing
 - <see cref="M:SpaceCenter.Engine.MaxThrust" /> for every active engine.
 -}
getVesselMaxThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMaxThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMaxThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMaxThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxThrustStream thisArg = requestStream $ getVesselMaxThrustStreamReq thisArg 

{-
 - The total maximum thrust that can be produced by the vessel's active
 - engines when the vessel is in a vacuum, in Newtons. This is computed by
 - summing <see cref="M:SpaceCenter.Engine.MaxVacuumThrust" /> for every active engine.
 -}
getVesselMaxVacuumThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMaxVacuumThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselMaxVacuumThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMaxVacuumThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxVacuumThrustStream thisArg = requestStream $ getVesselMaxVacuumThrustStreamReq thisArg 

{-
 - The moment of inertia of the vessel around its center of mass inkg.m^2.
 - The inertia values are around the pitch, roll and yaw directions respectively.
 - This corresponds to the vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 -}
getVesselMomentOfInertia :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselMomentOfInertiaStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq ((Double, Double, Double))
getVesselMomentOfInertiaStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]
    in  makeStream req

getVesselMomentOfInertiaStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselMomentOfInertiaStream thisArg = requestStream $ getVesselMomentOfInertiaStreamReq thisArg 

{-
 - The name of the vessel.
 -}
getVesselName :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Data.Text.Text)
getVesselName thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselNameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Data.Text.Text)
getVesselNameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]
    in  makeStream req

getVesselNameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Data.Text.Text))
getVesselNameStream thisArg = requestStream $ getVesselNameStreamReq thisArg 

{-
 - The current orbit of the vessel.
 -}
getVesselOrbit :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getVesselOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselOrbitStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Orbit)
getVesselOrbitStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]
    in  makeStream req

getVesselOrbitStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getVesselOrbitStream thisArg = requestStream $ getVesselOrbitStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the vessels
 - orbital prograde/normal/radial directions.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the orbital prograde/normal/radial directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.Be careful not to confuse this with 'orbit' mode on the navball.
 -}
getVesselOrbitalReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselOrbitalReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselOrbitalReferenceFrameStream thisArg = requestStream $ getVesselOrbitalReferenceFrameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Parts" /> object, that can used to interact with the parts that make up this vessel.
 -}
getVesselParts :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Parts)
getVesselParts thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselPartsStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Parts)
getVesselPartsStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]
    in  makeStream req

getVesselPartsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Parts))
getVesselPartsStream thisArg = requestStream $ getVesselPartsStreamReq thisArg 

{-
 - Whether the vessel is recoverable.
 -}
getVesselRecoverable :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
getVesselRecoverable thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Recoverable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselRecoverableStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Bool)
getVesselRecoverableStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Recoverable" [makeArgument 0 thisArg]
    in  makeStream req

getVesselRecoverableStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Bool))
getVesselRecoverableStream thisArg = requestStream $ getVesselRecoverableStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the vessel.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel.The x-axis points out to the right of the vessel.The y-axis points in the forward direction of the vessel.The z-axis points out of the bottom off the vessel.
 -}
getVesselReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselReferenceFrameStream thisArg = requestStream $ getVesselReferenceFrameStreamReq thisArg 

{-
 - A <see cref="T:SpaceCenter.Resources" /> object, that can used to get information
 - about resources stored in the vessel.
 -}
getVesselResources :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Resources)
getVesselResources thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselResourcesStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.Resources)
getVesselResourcesStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]
    in  makeStream req

getVesselResourcesStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
getVesselResourcesStream thisArg = requestStream $ getVesselResourcesStreamReq thisArg 

{-
 - The situation the vessel is in.
 -}
getVesselSituation :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituation thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSituationStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituationStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSituationStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.VesselSituation))
getVesselSituationStream thisArg = requestStream $ getVesselSituationStreamReq thisArg 

{-
 - The combined specific impulse of all active engines, in seconds. This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
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
getVesselSurfaceReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSurfaceReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSurfaceReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceReferenceFrameStream thisArg = requestStream $ getVesselSurfaceReferenceFrameStreamReq thisArg 

{-
 - The reference frame that is fixed relative to the vessel, and orientated with the velocity
 - vector of the vessel relative to the surface of the body being orbited.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel's velocity vector.The y-axis points in the direction of the vessel's velocity vector,
 - relative to the surface of the body being orbited.The z-axis is in the plane of the
 - <a href="https://en.wikipedia.org/wiki/Horizon">astronomical horizon.The x-axis is orthogonal to the other two axes.
 -}
getVesselSurfaceVelocityReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselSurfaceVelocityReferenceFrameStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselSurfaceVelocityReferenceFrameStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getVesselSurfaceVelocityReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceVelocityReferenceFrameStream thisArg = requestStream $ getVesselSurfaceVelocityReferenceFrameStreamReq thisArg 

{-
 - The total thrust currently being produced by the vessel's engines, in
 - Newtons. This is computed by summing <see cref="M:SpaceCenter.Engine.Thrust" /> for
 - every engine in the vessel.
 -}
getVesselThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselThrustStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselThrustStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]
    in  makeStream req

getVesselThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselThrustStream thisArg = requestStream $ getVesselThrustStreamReq thisArg 

{-
 - The type of the vessel.
 -}
getVesselType :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.VesselType)
getVesselType thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselTypeStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (KRPCHS.SpaceCenter.VesselType)
getVesselTypeStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]
    in  makeStream req

getVesselTypeStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.VesselType))
getVesselTypeStream thisArg = requestStream $ getVesselTypeStreamReq thisArg 

{-
 - The combined vacuum specific impulse of all active engines, in seconds. This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselVacuumSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getVesselVacuumSpecificImpulseStreamReq :: KRPCHS.SpaceCenter.Vessel -> KRPCStreamReq (Float)
getVesselVacuumSpecificImpulseStreamReq thisArg =
    let req = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    in  makeStream req

getVesselVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselVacuumSpecificImpulseStream thisArg = requestStream $ getVesselVacuumSpecificImpulseStreamReq thisArg 

{-
 - The name of the vessel.
 -}
setVesselName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext ()
setVesselName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The type of the vessel.
 -}
setVesselType :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.VesselType -> RPCContext ()
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
warpTo :: Double -> Float -> Float -> RPCContext ()
warpTo utArg maxRailsRateArg maxPhysicsRateArg = do
    let r = makeRequest "SpaceCenter" "WarpTo" [makeArgument 0 utArg, makeArgument 1 maxRailsRateArg, makeArgument 2 maxPhysicsRateArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently active vessel.
 -}
getActiveVessel :: RPCContext (KRPCHS.SpaceCenter.Vessel)
getActiveVessel  = do
    let r = makeRequest "SpaceCenter" "get_ActiveVessel" []
    res <- sendRequest r
    processResponse res

getActiveVesselStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getActiveVesselStreamReq  =
    let req = makeRequest "SpaceCenter" "get_ActiveVessel" []
    in  makeStream req

getActiveVesselStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getActiveVesselStream  = requestStream $ getActiveVesselStreamReq  

{-
 - A dictionary of all celestial bodies (planets, moons, etc.) in the game,
 - keyed by the name of the body.
 -}
getBodies :: RPCContext (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodies  = do
    let r = makeRequest "SpaceCenter" "get_Bodies" []
    res <- sendRequest r
    processResponse res

getBodiesStreamReq :: KRPCStreamReq (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodiesStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Bodies" []
    in  makeStream req

getBodiesStream :: RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody)))
getBodiesStream  = requestStream $ getBodiesStreamReq  

{-
 - An object that can be used to control the camera.
 -}
getCamera :: RPCContext (KRPCHS.SpaceCenter.Camera)
getCamera  = do
    let r = makeRequest "SpaceCenter" "get_Camera" []
    res <- sendRequest r
    processResponse res

getCameraStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Camera)
getCameraStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Camera" []
    in  makeStream req

getCameraStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Camera))
getCameraStream  = requestStream $ getCameraStreamReq  

{-
 - Whether <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFARAvailable :: RPCContext (Bool)
getFARAvailable  = do
    let r = makeRequest "SpaceCenter" "get_FARAvailable" []
    res <- sendRequest r
    processResponse res

getFARAvailableStreamReq :: KRPCStreamReq (Bool)
getFARAvailableStreamReq  =
    let req = makeRequest "SpaceCenter" "get_FARAvailable" []
    in  makeStream req

getFARAvailableStream :: RPCContext (KRPCStream (Bool))
getFARAvailableStream  = requestStream $ getFARAvailableStreamReq  

{-
 - The value of the <a href="https://en.wikipedia.org/wiki/Gravitational_constant">gravitational constantG inN(m/kg)^2.
 -}
getG :: RPCContext (Float)
getG  = do
    let r = makeRequest "SpaceCenter" "get_G" []
    res <- sendRequest r
    processResponse res

getGStreamReq :: KRPCStreamReq (Float)
getGStreamReq  =
    let req = makeRequest "SpaceCenter" "get_G" []
    in  makeStream req

getGStream :: RPCContext (KRPCStream (Float))
getGStream  = requestStream $ getGStreamReq  

{-
 - The current maximum regular "on-rails" warp factor that can be set.
 - A value between 0 and 7 inclusive.  See
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">the KSP wikifor details.
 -}
getMaximumRailsWarpFactor :: RPCContext (Data.Int.Int32)
getMaximumRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" []
    res <- sendRequest r
    processResponse res

getMaximumRailsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getMaximumRailsWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" []
    in  makeStream req

getMaximumRailsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getMaximumRailsWarpFactorStream  = requestStream $ getMaximumRailsWarpFactorStreamReq  

{-
 - The physical time warp rate. A value between 0 and 3 inclusive. 0 means
 - no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
getPhysicsWarpFactor :: RPCContext (Data.Int.Int32)
getPhysicsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" []
    res <- sendRequest r
    processResponse res

getPhysicsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getPhysicsWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" []
    in  makeStream req

getPhysicsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getPhysicsWarpFactorStream  = requestStream $ getPhysicsWarpFactorStreamReq  

{-
 - The time warp rate, using regular "on-rails" time warp. A value between
 - 0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
 - is active.
 - If requested time warp factor cannot be set, it will be set to the next
 - lowest possible value. For example, if the vessel is too close to a
 - planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
 - the KSP wikifor details.
 -}
getRailsWarpFactor :: RPCContext (Data.Int.Int32)
getRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_RailsWarpFactor" []
    res <- sendRequest r
    processResponse res

getRailsWarpFactorStreamReq :: KRPCStreamReq (Data.Int.Int32)
getRailsWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_RailsWarpFactor" []
    in  makeStream req

getRailsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getRailsWarpFactorStream  = requestStream $ getRailsWarpFactorStreamReq  

{-
 - The currently targeted celestial body.
 -}
getTargetBody :: RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getTargetBody  = do
    let r = makeRequest "SpaceCenter" "get_TargetBody" []
    res <- sendRequest r
    processResponse res

getTargetBodyStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.CelestialBody)
getTargetBodyStreamReq  =
    let req = makeRequest "SpaceCenter" "get_TargetBody" []
    in  makeStream req

getTargetBodyStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getTargetBodyStream  = requestStream $ getTargetBodyStreamReq  

{-
 - The currently targeted docking port.
 -}
getTargetDockingPort :: RPCContext (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPort  = do
    let r = makeRequest "SpaceCenter" "get_TargetDockingPort" []
    res <- sendRequest r
    processResponse res

getTargetDockingPortStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPortStreamReq  =
    let req = makeRequest "SpaceCenter" "get_TargetDockingPort" []
    in  makeStream req

getTargetDockingPortStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getTargetDockingPortStream  = requestStream $ getTargetDockingPortStreamReq  

{-
 - The currently targeted vessel.
 -}
getTargetVessel :: RPCContext (KRPCHS.SpaceCenter.Vessel)
getTargetVessel  = do
    let r = makeRequest "SpaceCenter" "get_TargetVessel" []
    res <- sendRequest r
    processResponse res

getTargetVesselStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.Vessel)
getTargetVesselStreamReq  =
    let req = makeRequest "SpaceCenter" "get_TargetVessel" []
    in  makeStream req

getTargetVesselStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getTargetVesselStream  = requestStream $ getTargetVesselStreamReq  

{-
 - The current universal time in seconds.
 -}
getUT :: RPCContext (Double)
getUT  = do
    let r = makeRequest "SpaceCenter" "get_UT" []
    res <- sendRequest r
    processResponse res

getUTStreamReq :: KRPCStreamReq (Double)
getUTStreamReq  =
    let req = makeRequest "SpaceCenter" "get_UT" []
    in  makeStream req

getUTStream :: RPCContext (KRPCStream (Double))
getUTStream  = requestStream $ getUTStreamReq  

{-
 - A list of all the vessels in the game.
 -}
getVessels :: RPCContext ([KRPCHS.SpaceCenter.Vessel])
getVessels  = do
    let r = makeRequest "SpaceCenter" "get_Vessels" []
    res <- sendRequest r
    processResponse res

getVesselsStreamReq :: KRPCStreamReq ([KRPCHS.SpaceCenter.Vessel])
getVesselsStreamReq  =
    let req = makeRequest "SpaceCenter" "get_Vessels" []
    in  makeStream req

getVesselsStream :: RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
getVesselsStream  = requestStream $ getVesselsStreamReq  

{-
 - The current warp factor. This is the index of the rate at which time
 - is passing for either regular "on-rails" or physical time warp. Returns 0
 - if time warp is not active. When in on-rails time warp, this is equal to
 - <see cref="M:SpaceCenter.RailsWarpFactor" />, and in physics time warp, this is equal to
 - <see cref="M:SpaceCenter.PhysicsWarpFactor" />.
 -}
getWarpFactor :: RPCContext (Float)
getWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_WarpFactor" []
    res <- sendRequest r
    processResponse res

getWarpFactorStreamReq :: KRPCStreamReq (Float)
getWarpFactorStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WarpFactor" []
    in  makeStream req

getWarpFactorStream :: RPCContext (KRPCStream (Float))
getWarpFactorStream  = requestStream $ getWarpFactorStreamReq  

{-
 - The current time warp mode. Returns <see cref="M:SpaceCenter.WarpMode.None" /> if time
 - warp is not active, <see cref="M:SpaceCenter.WarpMode.Rails" /> if regular "on-rails" time warp
 - is active, or <see cref="M:SpaceCenter.WarpMode.Physics" /> if physical time warp is active.
 -}
getWarpMode :: RPCContext (KRPCHS.SpaceCenter.WarpMode)
getWarpMode  = do
    let r = makeRequest "SpaceCenter" "get_WarpMode" []
    res <- sendRequest r
    processResponse res

getWarpModeStreamReq :: KRPCStreamReq (KRPCHS.SpaceCenter.WarpMode)
getWarpModeStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WarpMode" []
    in  makeStream req

getWarpModeStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.WarpMode))
getWarpModeStream  = requestStream $ getWarpModeStreamReq  

{-
 - The current warp rate. This is the rate at which time is passing for
 - either on-rails or physical time warp. For example, a value of 10 means
 - time is passing 10x faster than normal. Returns 1 if time warp is not
 - active.
 -}
getWarpRate :: RPCContext (Float)
getWarpRate  = do
    let r = makeRequest "SpaceCenter" "get_WarpRate" []
    res <- sendRequest r
    processResponse res

getWarpRateStreamReq :: KRPCStreamReq (Float)
getWarpRateStreamReq  =
    let req = makeRequest "SpaceCenter" "get_WarpRate" []
    in  makeStream req

getWarpRateStream :: RPCContext (KRPCStream (Float))
getWarpRateStream  = requestStream $ getWarpRateStreamReq  

{-
 - The currently active vessel.
 -}
setActiveVessel :: KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setActiveVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_ActiveVessel" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The physical time warp rate. A value between 0 and 3 inclusive. 0 means
 - no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
setPhysicsWarpFactor :: Data.Int.Int32 -> RPCContext ()
setPhysicsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_PhysicsWarpFactor" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The time warp rate, using regular "on-rails" time warp. A value between
 - 0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
 - is active.
 - If requested time warp factor cannot be set, it will be set to the next
 - lowest possible value. For example, if the vessel is too close to a
 - planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
 - the KSP wikifor details.
 -}
setRailsWarpFactor :: Data.Int.Int32 -> RPCContext ()
setRailsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_RailsWarpFactor" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently targeted celestial body.
 -}
setTargetBody :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext ()
setTargetBody valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetBody" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently targeted docking port.
 -}
setTargetDockingPort :: KRPCHS.SpaceCenter.DockingPort -> RPCContext ()
setTargetDockingPort valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetDockingPort" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The currently targeted vessel.
 -}
setTargetVessel :: KRPCHS.SpaceCenter.Vessel -> RPCContext ()
setTargetVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetVessel" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse res 

