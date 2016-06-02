{-# LANGUAGE RecordWildCards #-}
module KRPCHS.SpaceCenter
( AutoPilot(..)
, Camera(..)
, CelestialBody(..)
, Comms(..)
, Control(..)
, Flight(..)
, Node(..)
, Orbit(..)
, CargoBay(..)
, ControlSurface(..)
, Decoupler(..)
, DockingPort(..)
, Engine(..)
, Fairing(..)
, Intake(..)
, LandingGear(..)
, LandingLeg(..)
, LaunchClamp(..)
, Light(..)
, Module(..)
, Parachute(..)
, Part(..)
, Parts(..)
, RCS(..)
, Radiator(..)
, ReactionWheel(..)
, ResourceConverter(..)
, ResourceHarvester(..)
, Sensor(..)
, SolarPanel(..)
, Thruster(..)
, ReferenceFrame(..)
, Resource(..)
, ResourceTransfer(..)
, Resources(..)
, Vessel(..)
, CameraMode(..)
, CargoBayState(..)
, DockingPortState(..)
, LandingGearState(..)
, LandingLegState(..)
, ParachuteState(..)
, RadiatorState(..)
, ResourceConverterState(..)
, ResourceHarvesterState(..)
, SolarPanelState(..)
, ResourceFlowMode(..)
, SASMode(..)
, SpeedMode(..)
, VesselSituation(..)
, VesselType(..)
, WarpMode(..)
, clearTarget
, launchVesselFromVAB
, launchVesselFromSPH
, save
, load
, quicksave
, quickload
, canRailsWarpAt
, warpTo
, transformPosition
, transformDirection
, transformRotation
, transformVelocity
, drawDirection
, drawLine
, clearDrawing
, getActiveVessel
, setActiveVessel
, getVessels
, getBodies
, getTargetBody
, setTargetBody
, getTargetVessel
, setTargetVessel
, getTargetDockingPort
, setTargetDockingPort
, getCamera
, getUT
, getG
, getWarpMode
, getWarpRate
, getWarpFactor
, getRailsWarpFactor
, setRailsWarpFactor
, getPhysicsWarpFactor
, setPhysicsWarpFactor
, getMaximumRailsWarpFactor
, getFARAvailable
, getRemoteTechAvailable
, autoPilotEngage
, autoPilotDisengage
, autoPilotWait
, autoPilotTargetPitchAndHeading
, autoPilotSetPIDParameters
, getAutoPilotError
, getAutoPilotRollError
, getAutoPilotReferenceFrame
, setAutoPilotReferenceFrame
, getAutoPilotTargetDirection
, setAutoPilotTargetDirection
, getAutoPilotTargetRoll
, setAutoPilotTargetRoll
, getAutoPilotSAS
, setAutoPilotSAS
, getAutoPilotSASMode
, setAutoPilotSASMode
, getAutoPilotRotationSpeedMultiplier
, setAutoPilotRotationSpeedMultiplier
, getAutoPilotMaxRotationSpeed
, setAutoPilotMaxRotationSpeed
, getAutoPilotRollSpeedMultiplier
, setAutoPilotRollSpeedMultiplier
, getAutoPilotMaxRollSpeed
, setAutoPilotMaxRollSpeed
, getCameraMode
, setCameraMode
, getCameraPitch
, setCameraPitch
, getCameraHeading
, setCameraHeading
, getCameraDistance
, setCameraDistance
, getCameraMinPitch
, getCameraMaxPitch
, getCameraMinDistance
, getCameraMaxDistance
, getCameraDefaultDistance
, getCameraFocussedBody
, setCameraFocussedBody
, getCameraFocussedVessel
, setCameraFocussedVessel
, getCameraFocussedNode
, setCameraFocussedNode
, celestialBodySurfaceHeight
, celestialBodyBedrockHeight
, celestialBodyMSLPosition
, celestialBodySurfacePosition
, celestialBodyBedrockPosition
, celestialBodyPosition
, celestialBodyVelocity
, celestialBodyRotation
, celestialBodyDirection
, celestialBodyAngularVelocity
, getCelestialBodyName
, getCelestialBodySatellites
, getCelestialBodyMass
, getCelestialBodyGravitationalParameter
, getCelestialBodySurfaceGravity
, getCelestialBodyRotationalPeriod
, getCelestialBodyRotationalSpeed
, getCelestialBodyEquatorialRadius
, getCelestialBodySphereOfInfluence
, getCelestialBodyOrbit
, getCelestialBodyHasAtmosphere
, getCelestialBodyAtmosphereDepth
, getCelestialBodyHasAtmosphericOxygen
, getCelestialBodyReferenceFrame
, getCelestialBodyNonRotatingReferenceFrame
, getCelestialBodyOrbitalReferenceFrame
, commsSignalDelayToVessel
, getCommsHasLocalControl
, getCommsHasFlightComputer
, getCommsHasConnection
, getCommsHasConnectionToGroundStation
, getCommsSignalDelay
, getCommsSignalDelayToGroundStation
, controlActivateNextStage
, controlGetActionGroup
, controlSetActionGroup
, controlToggleActionGroup
, controlAddNode
, controlRemoveNodes
, getControlSAS
, setControlSAS
, getControlSASMode
, setControlSASMode
, getControlSpeedMode
, setControlSpeedMode
, getControlRCS
, setControlRCS
, getControlGear
, setControlGear
, getControlLights
, setControlLights
, getControlBrakes
, setControlBrakes
, getControlAbort
, setControlAbort
, getControlThrottle
, setControlThrottle
, getControlPitch
, setControlPitch
, getControlYaw
, setControlYaw
, getControlRoll
, setControlRoll
, getControlForward
, setControlForward
, getControlUp
, setControlUp
, getControlRight
, setControlRight
, getControlWheelThrottle
, setControlWheelThrottle
, getControlWheelSteering
, setControlWheelSteering
, getControlCurrentStage
, getControlNodes
, getFlightGForce
, getFlightMeanAltitude
, getFlightSurfaceAltitude
, getFlightBedrockAltitude
, getFlightElevation
, getFlightLatitude
, getFlightLongitude
, getFlightVelocity
, getFlightSpeed
, getFlightHorizontalSpeed
, getFlightVerticalSpeed
, getFlightCenterOfMass
, getFlightRotation
, getFlightDirection
, getFlightPitch
, getFlightHeading
, getFlightRoll
, getFlightPrograde
, getFlightRetrograde
, getFlightNormal
, getFlightAntiNormal
, getFlightRadial
, getFlightAntiRadial
, getFlightAtmosphereDensity
, getFlightDynamicPressure
, getFlightStaticPressure
, getFlightAerodynamicForce
, getFlightLift
, getFlightDrag
, getFlightSpeedOfSound
, getFlightMach
, getFlightEquivalentAirSpeed
, getFlightTerminalVelocity
, getFlightAngleOfAttack
, getFlightSideslipAngle
, getFlightTotalAirTemperature
, getFlightStaticAirTemperature
, getFlightStallFraction
, getFlightDragCoefficient
, getFlightLiftCoefficient
, getFlightBallisticCoefficient
, getFlightThrustSpecificFuelConsumption
, nodeBurnVector
, nodeRemainingBurnVector
, nodeRemove
, nodePosition
, nodeDirection
, getNodePrograde
, setNodePrograde
, getNodeNormal
, setNodeNormal
, getNodeRadial
, setNodeRadial
, getNodeDeltaV
, setNodeDeltaV
, getNodeRemainingDeltaV
, getNodeUT
, setNodeUT
, getNodeTimeTo
, getNodeOrbit
, getNodeReferenceFrame
, getNodeOrbitalReferenceFrame
, orbitReferencePlaneNormal
, orbitReferencePlaneDirection
, getOrbitBody
, getOrbitApoapsis
, getOrbitPeriapsis
, getOrbitApoapsisAltitude
, getOrbitPeriapsisAltitude
, getOrbitSemiMajorAxis
, getOrbitSemiMinorAxis
, getOrbitRadius
, getOrbitSpeed
, getOrbitPeriod
, getOrbitTimeToApoapsis
, getOrbitTimeToPeriapsis
, getOrbitEccentricity
, getOrbitInclination
, getOrbitLongitudeOfAscendingNode
, getOrbitArgumentOfPeriapsis
, getOrbitMeanAnomalyAtEpoch
, getOrbitEpoch
, getOrbitMeanAnomaly
, getOrbitEccentricAnomaly
, getOrbitNextOrbit
, getOrbitTimeToSOIChange
, getCargoBayPart
, getCargoBayState
, getCargoBayOpen
, setCargoBayOpen
, getControlSurfacePart
, getControlSurfacePitchEnabled
, setControlSurfacePitchEnabled
, getControlSurfaceYawEnabled
, setControlSurfaceYawEnabled
, getControlSurfaceRollEnabled
, setControlSurfaceRollEnabled
, getControlSurfaceInverted
, setControlSurfaceInverted
, getControlSurfaceDeployed
, setControlSurfaceDeployed
, getControlSurfaceSurfaceArea
, decouplerDecouple
, getDecouplerPart
, getDecouplerDecoupled
, getDecouplerImpulse
, dockingPortUndock
, dockingPortPosition
, dockingPortDirection
, dockingPortRotation
, getDockingPortPart
, getDockingPortName
, setDockingPortName
, getDockingPortState
, getDockingPortDockedPart
, getDockingPortReengageDistance
, getDockingPortHasShield
, getDockingPortShielded
, setDockingPortShielded
, getDockingPortReferenceFrame
, engineToggleMode
, getEnginePart
, getEngineActive
, setEngineActive
, getEngineThrust
, getEngineAvailableThrust
, getEngineMaxThrust
, getEngineMaxVacuumThrust
, getEngineThrustLimit
, setEngineThrustLimit
, getEngineThrusters
, getEngineSpecificImpulse
, getEngineVacuumSpecificImpulse
, getEngineKerbinSeaLevelSpecificImpulse
, getEnginePropellants
, getEnginePropellantRatios
, getEngineHasFuel
, getEngineThrottle
, getEngineThrottleLocked
, getEngineCanRestart
, getEngineCanShutdown
, getEngineHasModes
, getEngineMode
, setEngineMode
, getEngineModes
, getEngineAutoModeSwitch
, setEngineAutoModeSwitch
, getEngineGimballed
, getEngineGimbalRange
, getEngineGimbalLocked
, setEngineGimbalLocked
, getEngineGimbalLimit
, setEngineGimbalLimit
, fairingJettison
, getFairingPart
, getFairingJettisoned
, getIntakePart
, getIntakeOpen
, setIntakeOpen
, getIntakeSpeed
, getIntakeFlow
, getIntakeArea
, getLandingGearPart
, getLandingGearDeployable
, getLandingGearState
, getLandingGearDeployed
, setLandingGearDeployed
, getLandingLegPart
, getLandingLegState
, getLandingLegDeployed
, setLandingLegDeployed
, launchClampRelease
, getLaunchClampPart
, getLightPart
, getLightActive
, setLightActive
, getLightPowerUsage
, moduleHasField
, moduleGetField
, moduleHasEvent
, moduleTriggerEvent
, moduleHasAction
, moduleSetAction
, getModuleName
, getModulePart
, getModuleFields
, getModuleEvents
, getModuleActions
, parachuteDeploy
, getParachutePart
, getParachuteDeployed
, getParachuteState
, getParachuteDeployAltitude
, setParachuteDeployAltitude
, getParachuteDeployMinPressure
, setParachuteDeployMinPressure
, partPosition
, partCenterOfMass
, partDirection
, partVelocity
, partRotation
, getPartName
, getPartTitle
, getPartCost
, getPartVessel
, getPartParent
, getPartChildren
, getPartAxiallyAttached
, getPartRadiallyAttached
, getPartStage
, getPartDecoupleStage
, getPartMassless
, getPartMass
, getPartDryMass
, getPartShielded
, getPartDynamicPressure
, getPartImpactTolerance
, getPartTemperature
, getPartSkinTemperature
, getPartMaxTemperature
, getPartMaxSkinTemperature
, getPartThermalMass
, getPartThermalSkinMass
, getPartThermalResourceMass
, getPartThermalInternalFlux
, getPartThermalConductionFlux
, getPartThermalConvectionFlux
, getPartThermalRadiationFlux
, getPartThermalSkinToInternalFlux
, getPartResources
, getPartCrossfeed
, getPartIsFuelLine
, getPartFuelLinesFrom
, getPartFuelLinesTo
, getPartModules
, getPartCargoBay
, getPartControlSurface
, getPartDecoupler
, getPartDockingPort
, getPartEngine
, getPartFairing
, getPartIntake
, getPartLandingGear
, getPartLandingLeg
, getPartLaunchClamp
, getPartLight
, getPartParachute
, getPartRadiator
, getPartRCS
, getPartReactionWheel
, getPartResourceConverter
, getPartResourceHarvester
, getPartSensor
, getPartSolarPanel
, getPartMomentOfInertia
, getPartInertiaTensor
, getPartReferenceFrame
, getPartCenterOfMassReferenceFrame
, partsWithName
, partsWithTitle
, partsWithModule
, partsInStage
, partsInDecoupleStage
, partsModulesWithName
, partsDockingPortWithName
, getPartsAll
, getPartsRoot
, getPartsControlling
, setPartsControlling
, getPartsControlSurfaces
, getPartsCargoBays
, getPartsDecouplers
, getPartsDockingPorts
, getPartsEngines
, getPartsFairings
, getPartsIntakes
, getPartsLandingGear
, getPartsLandingLegs
, getPartsLaunchClamps
, getPartsLights
, getPartsParachutes
, getPartsRadiators
, getPartsRCS
, getPartsReactionWheels
, getPartsResourceConverters
, getPartsResourceHarvesters
, getPartsSensors
, getPartsSolarPanels
, getRCSPart
, getRCSActive
, getRCSEnabled
, setRCSEnabled
, getRCSPitchEnabled
, setRCSPitchEnabled
, getRCSYawEnabled
, setRCSYawEnabled
, getRCSRollEnabled
, setRCSRollEnabled
, getRCSForwardEnabled
, setRCSForwardEnabled
, getRCSUpEnabled
, setRCSUpEnabled
, getRCSRightEnabled
, setRCSRightEnabled
, getRCSMaxThrust
, getRCSMaxVacuumThrust
, getRCSThrusters
, getRCSSpecificImpulse
, getRCSVacuumSpecificImpulse
, getRCSKerbinSeaLevelSpecificImpulse
, getRCSPropellants
, getRCSPropellantRatios
, getRCSHasFuel
, getRadiatorPart
, getRadiatorDeployable
, getRadiatorDeployed
, setRadiatorDeployed
, getRadiatorState
, getReactionWheelPart
, getReactionWheelActive
, setReactionWheelActive
, getReactionWheelBroken
, getReactionWheelTorque
, resourceConverterActive
, resourceConverterName
, resourceConverterStart
, resourceConverterStop
, resourceConverterState
, resourceConverterStatusInfo
, resourceConverterInputs
, resourceConverterOutputs
, getResourceConverterPart
, getResourceConverterCount
, getResourceHarvesterPart
, getResourceHarvesterState
, getResourceHarvesterDeployed
, setResourceHarvesterDeployed
, getResourceHarvesterActive
, setResourceHarvesterActive
, getResourceHarvesterExtractionRate
, getResourceHarvesterThermalEfficiency
, getResourceHarvesterCoreTemperature
, getResourceHarvesterOptimumCoreTemperature
, getSensorPart
, getSensorActive
, setSensorActive
, getSensorValue
, getSensorPowerUsage
, getSolarPanelPart
, getSolarPanelDeployed
, setSolarPanelDeployed
, getSolarPanelState
, getSolarPanelEnergyFlow
, getSolarPanelSunExposure
, thrusterThrustPosition
, thrusterThrustDirection
, thrusterInitialThrustPosition
, thrusterInitialThrustDirection
, thrusterGimbalPosition
, getThrusterPart
, getThrusterThrustReferenceFrame
, getThrusterGimballed
, getThrusterGimbalAngle
, getResourceName
, getResourcePart
, getResourceMax
, getResourceAmount
, getResourceDensity
, getResourceFlowMode
, getResourceEnabled
, setResourceEnabled
, resourceTransferStart
, getResourceTransferComplete
, getResourceTransferAmount
, resourcesWithResource
, resourcesHasResource
, resourcesMax
, resourcesAmount
, resourcesDensity
, resourcesFlowMode
, getResourcesAll
, getResourcesNames
, vesselFlight
, vesselResourcesInDecoupleStage
, vesselPosition
, vesselVelocity
, vesselRotation
, vesselDirection
, vesselAngularVelocity
, getVesselName
, setVesselName
, getVesselType
, setVesselType
, getVesselSituation
, getVesselMET
, getVesselOrbit
, getVesselControl
, getVesselAutoPilot
, getVesselResources
, getVesselParts
, getVesselComms
, getVesselMass
, getVesselDryMass
, getVesselThrust
, getVesselAvailableThrust
, getVesselMaxThrust
, getVesselMaxVacuumThrust
, getVesselSpecificImpulse
, getVesselVacuumSpecificImpulse
, getVesselKerbinSeaLevelSpecificImpulse
, getVesselMomentOfInertia
, getVesselInertiaTensor
, getVesselReactionWheelTorque
, getVesselReferenceFrame
, getVesselOrbitalReferenceFrame
, getVesselSurfaceReferenceFrame
, getVesselSurfaceVelocityReferenceFrame
, canRailsWarpAtStream
, transformPositionStream
, transformDirectionStream
, transformRotationStream
, transformVelocityStream
, getActiveVesselStream
, getVesselsStream
, getBodiesStream
, getTargetBodyStream
, getTargetVesselStream
, getTargetDockingPortStream
, getCameraStream
, getUTStream
, getGStream
, getWarpModeStream
, getWarpRateStream
, getWarpFactorStream
, getRailsWarpFactorStream
, getPhysicsWarpFactorStream
, getMaximumRailsWarpFactorStream
, getFARAvailableStream
, getRemoteTechAvailableStream
, getAutoPilotErrorStream
, getAutoPilotRollErrorStream
, getAutoPilotReferenceFrameStream
, getAutoPilotTargetDirectionStream
, getAutoPilotTargetRollStream
, getAutoPilotSASStream
, getAutoPilotSASModeStream
, getAutoPilotRotationSpeedMultiplierStream
, getAutoPilotMaxRotationSpeedStream
, getAutoPilotRollSpeedMultiplierStream
, getAutoPilotMaxRollSpeedStream
, getCameraModeStream
, getCameraPitchStream
, getCameraHeadingStream
, getCameraDistanceStream
, getCameraMinPitchStream
, getCameraMaxPitchStream
, getCameraMinDistanceStream
, getCameraMaxDistanceStream
, getCameraDefaultDistanceStream
, getCameraFocussedBodyStream
, getCameraFocussedVesselStream
, getCameraFocussedNodeStream
, celestialBodySurfaceHeightStream
, celestialBodyBedrockHeightStream
, celestialBodyMSLPositionStream
, celestialBodySurfacePositionStream
, celestialBodyBedrockPositionStream
, celestialBodyPositionStream
, celestialBodyVelocityStream
, celestialBodyRotationStream
, celestialBodyDirectionStream
, celestialBodyAngularVelocityStream
, getCelestialBodyNameStream
, getCelestialBodySatellitesStream
, getCelestialBodyMassStream
, getCelestialBodyGravitationalParameterStream
, getCelestialBodySurfaceGravityStream
, getCelestialBodyRotationalPeriodStream
, getCelestialBodyRotationalSpeedStream
, getCelestialBodyEquatorialRadiusStream
, getCelestialBodySphereOfInfluenceStream
, getCelestialBodyOrbitStream
, getCelestialBodyHasAtmosphereStream
, getCelestialBodyAtmosphereDepthStream
, getCelestialBodyHasAtmosphericOxygenStream
, getCelestialBodyReferenceFrameStream
, getCelestialBodyNonRotatingReferenceFrameStream
, getCelestialBodyOrbitalReferenceFrameStream
, commsSignalDelayToVesselStream
, getCommsHasLocalControlStream
, getCommsHasFlightComputerStream
, getCommsHasConnectionStream
, getCommsHasConnectionToGroundStationStream
, getCommsSignalDelayStream
, getCommsSignalDelayToGroundStationStream
, controlActivateNextStageStream
, controlGetActionGroupStream
, controlAddNodeStream
, getControlSASStream
, getControlSASModeStream
, getControlSpeedModeStream
, getControlRCSStream
, getControlGearStream
, getControlLightsStream
, getControlBrakesStream
, getControlAbortStream
, getControlThrottleStream
, getControlPitchStream
, getControlYawStream
, getControlRollStream
, getControlForwardStream
, getControlUpStream
, getControlRightStream
, getControlWheelThrottleStream
, getControlWheelSteeringStream
, getControlCurrentStageStream
, getControlNodesStream
, getFlightGForceStream
, getFlightMeanAltitudeStream
, getFlightSurfaceAltitudeStream
, getFlightBedrockAltitudeStream
, getFlightElevationStream
, getFlightLatitudeStream
, getFlightLongitudeStream
, getFlightVelocityStream
, getFlightSpeedStream
, getFlightHorizontalSpeedStream
, getFlightVerticalSpeedStream
, getFlightCenterOfMassStream
, getFlightRotationStream
, getFlightDirectionStream
, getFlightPitchStream
, getFlightHeadingStream
, getFlightRollStream
, getFlightProgradeStream
, getFlightRetrogradeStream
, getFlightNormalStream
, getFlightAntiNormalStream
, getFlightRadialStream
, getFlightAntiRadialStream
, getFlightAtmosphereDensityStream
, getFlightDynamicPressureStream
, getFlightStaticPressureStream
, getFlightAerodynamicForceStream
, getFlightLiftStream
, getFlightDragStream
, getFlightSpeedOfSoundStream
, getFlightMachStream
, getFlightEquivalentAirSpeedStream
, getFlightTerminalVelocityStream
, getFlightAngleOfAttackStream
, getFlightSideslipAngleStream
, getFlightTotalAirTemperatureStream
, getFlightStaticAirTemperatureStream
, getFlightStallFractionStream
, getFlightDragCoefficientStream
, getFlightLiftCoefficientStream
, getFlightBallisticCoefficientStream
, getFlightThrustSpecificFuelConsumptionStream
, nodeBurnVectorStream
, nodeRemainingBurnVectorStream
, nodePositionStream
, nodeDirectionStream
, getNodeProgradeStream
, getNodeNormalStream
, getNodeRadialStream
, getNodeDeltaVStream
, getNodeRemainingDeltaVStream
, getNodeUTStream
, getNodeTimeToStream
, getNodeOrbitStream
, getNodeReferenceFrameStream
, getNodeOrbitalReferenceFrameStream
, orbitReferencePlaneNormalStream
, orbitReferencePlaneDirectionStream
, getOrbitBodyStream
, getOrbitApoapsisStream
, getOrbitPeriapsisStream
, getOrbitApoapsisAltitudeStream
, getOrbitPeriapsisAltitudeStream
, getOrbitSemiMajorAxisStream
, getOrbitSemiMinorAxisStream
, getOrbitRadiusStream
, getOrbitSpeedStream
, getOrbitPeriodStream
, getOrbitTimeToApoapsisStream
, getOrbitTimeToPeriapsisStream
, getOrbitEccentricityStream
, getOrbitInclinationStream
, getOrbitLongitudeOfAscendingNodeStream
, getOrbitArgumentOfPeriapsisStream
, getOrbitMeanAnomalyAtEpochStream
, getOrbitEpochStream
, getOrbitMeanAnomalyStream
, getOrbitEccentricAnomalyStream
, getOrbitNextOrbitStream
, getOrbitTimeToSOIChangeStream
, getCargoBayPartStream
, getCargoBayStateStream
, getCargoBayOpenStream
, getControlSurfacePartStream
, getControlSurfacePitchEnabledStream
, getControlSurfaceYawEnabledStream
, getControlSurfaceRollEnabledStream
, getControlSurfaceInvertedStream
, getControlSurfaceDeployedStream
, getControlSurfaceSurfaceAreaStream
, decouplerDecoupleStream
, getDecouplerPartStream
, getDecouplerDecoupledStream
, getDecouplerImpulseStream
, dockingPortUndockStream
, dockingPortPositionStream
, dockingPortDirectionStream
, dockingPortRotationStream
, getDockingPortPartStream
, getDockingPortNameStream
, getDockingPortStateStream
, getDockingPortDockedPartStream
, getDockingPortReengageDistanceStream
, getDockingPortHasShieldStream
, getDockingPortShieldedStream
, getDockingPortReferenceFrameStream
, getEnginePartStream
, getEngineActiveStream
, getEngineThrustStream
, getEngineAvailableThrustStream
, getEngineMaxThrustStream
, getEngineMaxVacuumThrustStream
, getEngineThrustLimitStream
, getEngineThrustersStream
, getEngineSpecificImpulseStream
, getEngineVacuumSpecificImpulseStream
, getEngineKerbinSeaLevelSpecificImpulseStream
, getEnginePropellantsStream
, getEnginePropellantRatiosStream
, getEngineHasFuelStream
, getEngineThrottleStream
, getEngineThrottleLockedStream
, getEngineCanRestartStream
, getEngineCanShutdownStream
, getEngineHasModesStream
, getEngineModeStream
, getEngineModesStream
, getEngineAutoModeSwitchStream
, getEngineGimballedStream
, getEngineGimbalRangeStream
, getEngineGimbalLockedStream
, getEngineGimbalLimitStream
, getFairingPartStream
, getFairingJettisonedStream
, getIntakePartStream
, getIntakeOpenStream
, getIntakeSpeedStream
, getIntakeFlowStream
, getIntakeAreaStream
, getLandingGearPartStream
, getLandingGearDeployableStream
, getLandingGearStateStream
, getLandingGearDeployedStream
, getLandingLegPartStream
, getLandingLegStateStream
, getLandingLegDeployedStream
, getLaunchClampPartStream
, getLightPartStream
, getLightActiveStream
, getLightPowerUsageStream
, moduleHasFieldStream
, moduleGetFieldStream
, moduleHasEventStream
, moduleHasActionStream
, getModuleNameStream
, getModulePartStream
, getModuleFieldsStream
, getModuleEventsStream
, getModuleActionsStream
, getParachutePartStream
, getParachuteDeployedStream
, getParachuteStateStream
, getParachuteDeployAltitudeStream
, getParachuteDeployMinPressureStream
, partPositionStream
, partCenterOfMassStream
, partDirectionStream
, partVelocityStream
, partRotationStream
, getPartNameStream
, getPartTitleStream
, getPartCostStream
, getPartVesselStream
, getPartParentStream
, getPartChildrenStream
, getPartAxiallyAttachedStream
, getPartRadiallyAttachedStream
, getPartStageStream
, getPartDecoupleStageStream
, getPartMasslessStream
, getPartMassStream
, getPartDryMassStream
, getPartShieldedStream
, getPartDynamicPressureStream
, getPartImpactToleranceStream
, getPartTemperatureStream
, getPartSkinTemperatureStream
, getPartMaxTemperatureStream
, getPartMaxSkinTemperatureStream
, getPartThermalMassStream
, getPartThermalSkinMassStream
, getPartThermalResourceMassStream
, getPartThermalInternalFluxStream
, getPartThermalConductionFluxStream
, getPartThermalConvectionFluxStream
, getPartThermalRadiationFluxStream
, getPartThermalSkinToInternalFluxStream
, getPartResourcesStream
, getPartCrossfeedStream
, getPartIsFuelLineStream
, getPartFuelLinesFromStream
, getPartFuelLinesToStream
, getPartModulesStream
, getPartCargoBayStream
, getPartControlSurfaceStream
, getPartDecouplerStream
, getPartDockingPortStream
, getPartEngineStream
, getPartFairingStream
, getPartIntakeStream
, getPartLandingGearStream
, getPartLandingLegStream
, getPartLaunchClampStream
, getPartLightStream
, getPartParachuteStream
, getPartRadiatorStream
, getPartRCSStream
, getPartReactionWheelStream
, getPartResourceConverterStream
, getPartResourceHarvesterStream
, getPartSensorStream
, getPartSolarPanelStream
, getPartMomentOfInertiaStream
, getPartInertiaTensorStream
, getPartReferenceFrameStream
, getPartCenterOfMassReferenceFrameStream
, partsWithNameStream
, partsWithTitleStream
, partsWithModuleStream
, partsInStageStream
, partsInDecoupleStageStream
, partsModulesWithNameStream
, partsDockingPortWithNameStream
, getPartsAllStream
, getPartsRootStream
, getPartsControllingStream
, getPartsControlSurfacesStream
, getPartsCargoBaysStream
, getPartsDecouplersStream
, getPartsDockingPortsStream
, getPartsEnginesStream
, getPartsFairingsStream
, getPartsIntakesStream
, getPartsLandingGearStream
, getPartsLandingLegsStream
, getPartsLaunchClampsStream
, getPartsLightsStream
, getPartsParachutesStream
, getPartsRadiatorsStream
, getPartsRCSStream
, getPartsReactionWheelsStream
, getPartsResourceConvertersStream
, getPartsResourceHarvestersStream
, getPartsSensorsStream
, getPartsSolarPanelsStream
, getRCSPartStream
, getRCSActiveStream
, getRCSEnabledStream
, getRCSPitchEnabledStream
, getRCSYawEnabledStream
, getRCSRollEnabledStream
, getRCSForwardEnabledStream
, getRCSUpEnabledStream
, getRCSRightEnabledStream
, getRCSMaxThrustStream
, getRCSMaxVacuumThrustStream
, getRCSThrustersStream
, getRCSSpecificImpulseStream
, getRCSVacuumSpecificImpulseStream
, getRCSKerbinSeaLevelSpecificImpulseStream
, getRCSPropellantsStream
, getRCSPropellantRatiosStream
, getRCSHasFuelStream
, getRadiatorPartStream
, getRadiatorDeployableStream
, getRadiatorDeployedStream
, getRadiatorStateStream
, getReactionWheelPartStream
, getReactionWheelActiveStream
, getReactionWheelBrokenStream
, getReactionWheelTorqueStream
, resourceConverterActiveStream
, resourceConverterNameStream
, resourceConverterStateStream
, resourceConverterStatusInfoStream
, resourceConverterInputsStream
, resourceConverterOutputsStream
, getResourceConverterPartStream
, getResourceConverterCountStream
, getResourceHarvesterPartStream
, getResourceHarvesterStateStream
, getResourceHarvesterDeployedStream
, getResourceHarvesterActiveStream
, getResourceHarvesterExtractionRateStream
, getResourceHarvesterThermalEfficiencyStream
, getResourceHarvesterCoreTemperatureStream
, getResourceHarvesterOptimumCoreTemperatureStream
, getSensorPartStream
, getSensorActiveStream
, getSensorValueStream
, getSensorPowerUsageStream
, getSolarPanelPartStream
, getSolarPanelDeployedStream
, getSolarPanelStateStream
, getSolarPanelEnergyFlowStream
, getSolarPanelSunExposureStream
, thrusterThrustPositionStream
, thrusterThrustDirectionStream
, thrusterInitialThrustPositionStream
, thrusterInitialThrustDirectionStream
, thrusterGimbalPositionStream
, getThrusterPartStream
, getThrusterThrustReferenceFrameStream
, getThrusterGimballedStream
, getThrusterGimbalAngleStream
, getResourceNameStream
, getResourcePartStream
, getResourceMaxStream
, getResourceAmountStream
, getResourceDensityStream
, getResourceFlowModeStream
, getResourceEnabledStream
, resourceTransferStartStream
, getResourceTransferCompleteStream
, getResourceTransferAmountStream
, resourcesWithResourceStream
, resourcesHasResourceStream
, resourcesMaxStream
, resourcesAmountStream
, resourcesDensityStream
, resourcesFlowModeStream
, getResourcesAllStream
, getResourcesNamesStream
, vesselFlightStream
, vesselResourcesInDecoupleStageStream
, vesselPositionStream
, vesselVelocityStream
, vesselRotationStream
, vesselDirectionStream
, vesselAngularVelocityStream
, getVesselNameStream
, getVesselTypeStream
, getVesselSituationStream
, getVesselMETStream
, getVesselOrbitStream
, getVesselControlStream
, getVesselAutoPilotStream
, getVesselResourcesStream
, getVesselPartsStream
, getVesselCommsStream
, getVesselMassStream
, getVesselDryMassStream
, getVesselThrustStream
, getVesselAvailableThrustStream
, getVesselMaxThrustStream
, getVesselMaxVacuumThrustStream
, getVesselSpecificImpulseStream
, getVesselVacuumSpecificImpulseStream
, getVesselKerbinSeaLevelSpecificImpulseStream
, getVesselMomentOfInertiaStream
, getVesselInertiaTensorStream
, getVesselReactionWheelTorqueStream
, getVesselReferenceFrameStream
, getVesselOrbitalReferenceFrameStream
, getVesselSurfaceReferenceFrameStream
, getVesselSurfaceVelocityReferenceFrameStream
) where


import Data.Map
import Data.Int
import Data.Word
import Data.Text

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


newtype AutoPilot = AutoPilot { autoPilotId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable AutoPilot where
    encodePb   = encodePb . autoPilotId
    decodePb b = AutoPilot <$> decodePb b

newtype Camera = Camera { cameraId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Camera where
    encodePb   = encodePb . cameraId
    decodePb b = Camera <$> decodePb b

newtype CelestialBody = CelestialBody { celestialBodyId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CelestialBody where
    encodePb   = encodePb . celestialBodyId
    decodePb b = CelestialBody <$> decodePb b

newtype Comms = Comms { commsId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Comms where
    encodePb   = encodePb . commsId
    decodePb b = Comms <$> decodePb b

newtype Control = Control { controlId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Control where
    encodePb   = encodePb . controlId
    decodePb b = Control <$> decodePb b

newtype Flight = Flight { flightId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Flight where
    encodePb   = encodePb . flightId
    decodePb b = Flight <$> decodePb b

newtype Node = Node { nodeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Node where
    encodePb   = encodePb . nodeId
    decodePb b = Node <$> decodePb b

newtype Orbit = Orbit { orbitId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Orbit where
    encodePb   = encodePb . orbitId
    decodePb b = Orbit <$> decodePb b

newtype CargoBay = CargoBay { cargoBayId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CargoBay where
    encodePb   = encodePb . cargoBayId
    decodePb b = CargoBay <$> decodePb b

newtype ControlSurface = ControlSurface { controlSurfaceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ControlSurface where
    encodePb   = encodePb . controlSurfaceId
    decodePb b = ControlSurface <$> decodePb b

newtype Decoupler = Decoupler { decouplerId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Decoupler where
    encodePb   = encodePb . decouplerId
    decodePb b = Decoupler <$> decodePb b

newtype DockingPort = DockingPort { dockingPortId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable DockingPort where
    encodePb   = encodePb . dockingPortId
    decodePb b = DockingPort <$> decodePb b

newtype Engine = Engine { engineId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Engine where
    encodePb   = encodePb . engineId
    decodePb b = Engine <$> decodePb b

newtype Fairing = Fairing { fairingId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Fairing where
    encodePb   = encodePb . fairingId
    decodePb b = Fairing <$> decodePb b

newtype Intake = Intake { intakeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Intake where
    encodePb   = encodePb . intakeId
    decodePb b = Intake <$> decodePb b

newtype LandingGear = LandingGear { landingGearId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LandingGear where
    encodePb   = encodePb . landingGearId
    decodePb b = LandingGear <$> decodePb b

newtype LandingLeg = LandingLeg { landingLegId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LandingLeg where
    encodePb   = encodePb . landingLegId
    decodePb b = LandingLeg <$> decodePb b

newtype LaunchClamp = LaunchClamp { launchClampId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LaunchClamp where
    encodePb   = encodePb . launchClampId
    decodePb b = LaunchClamp <$> decodePb b

newtype Light = Light { lightId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Light where
    encodePb   = encodePb . lightId
    decodePb b = Light <$> decodePb b

newtype Module = Module { moduleId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Module where
    encodePb   = encodePb . moduleId
    decodePb b = Module <$> decodePb b

newtype Parachute = Parachute { parachuteId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Parachute where
    encodePb   = encodePb . parachuteId
    decodePb b = Parachute <$> decodePb b

newtype Part = Part { partId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Part where
    encodePb   = encodePb . partId
    decodePb b = Part <$> decodePb b

newtype Parts = Parts { partsId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Parts where
    encodePb   = encodePb . partsId
    decodePb b = Parts <$> decodePb b

newtype RCS = RCS { rCSId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable RCS where
    encodePb   = encodePb . rCSId
    decodePb b = RCS <$> decodePb b

newtype Radiator = Radiator { radiatorId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Radiator where
    encodePb   = encodePb . radiatorId
    decodePb b = Radiator <$> decodePb b

newtype ReactionWheel = ReactionWheel { reactionWheelId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ReactionWheel where
    encodePb   = encodePb . reactionWheelId
    decodePb b = ReactionWheel <$> decodePb b

newtype ResourceConverter = ResourceConverter { resourceConverterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceConverter where
    encodePb   = encodePb . resourceConverterId
    decodePb b = ResourceConverter <$> decodePb b

newtype ResourceHarvester = ResourceHarvester { resourceHarvesterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceHarvester where
    encodePb   = encodePb . resourceHarvesterId
    decodePb b = ResourceHarvester <$> decodePb b

newtype Sensor = Sensor { sensorId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Sensor where
    encodePb   = encodePb . sensorId
    decodePb b = Sensor <$> decodePb b

newtype SolarPanel = SolarPanel { solarPanelId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable SolarPanel where
    encodePb   = encodePb . solarPanelId
    decodePb b = SolarPanel <$> decodePb b

newtype Thruster = Thruster { thrusterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Thruster where
    encodePb   = encodePb . thrusterId
    decodePb b = Thruster <$> decodePb b

newtype ReferenceFrame = ReferenceFrame { referenceFrameId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ReferenceFrame where
    encodePb   = encodePb . referenceFrameId
    decodePb b = ReferenceFrame <$> decodePb b

newtype Resource = Resource { resourceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Resource where
    encodePb   = encodePb . resourceId
    decodePb b = Resource <$> decodePb b

newtype ResourceTransfer = ResourceTransfer { resourceTransferId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceTransfer where
    encodePb   = encodePb . resourceTransferId
    decodePb b = ResourceTransfer <$> decodePb b

newtype Resources = Resources { resourcesId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Resources where
    encodePb   = encodePb . resourcesId
    decodePb b = Resources <$> decodePb b

newtype Vessel = Vessel { vesselId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Vessel where
    encodePb   = encodePb . vesselId
    decodePb b = Vessel <$> decodePb b

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

data CargoBayState
    = CargoBayState'Open
    | CargoBayState'Closed
    | CargoBayState'Opening
    | CargoBayState'Closing
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable CargoBayState where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

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

data ResourceFlowMode
    = ResourceFlowMode'Vessel
    | ResourceFlowMode'Stage
    | ResourceFlowMode'Adjacent
    | ResourceFlowMode'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable ResourceFlowMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

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

data SpeedMode
    = SpeedMode'Orbit
    | SpeedMode'Surface
    | SpeedMode'Target
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable SpeedMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

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

data WarpMode
    = WarpMode'Rails
    | WarpMode'Physics
    | WarpMode'None
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable WarpMode where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

clearTarget :: RPCContext (Bool)
clearTarget  = do
    let r = makeRequest "SpaceCenter" "ClearTarget" [  ]
    res <- sendRequest r
    processResponse extractNothing res

launchVesselFromVAB :: Text -> RPCContext (Bool)
launchVesselFromVAB nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromVAB" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractNothing res

launchVesselFromSPH :: Text -> RPCContext (Bool)
launchVesselFromSPH nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromSPH" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractNothing res

save :: Text -> RPCContext (Bool)
save nameArg = do
    let r = makeRequest "SpaceCenter" "Save" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractNothing res

load :: Text -> RPCContext (Bool)
load nameArg = do
    let r = makeRequest "SpaceCenter" "Load" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractNothing res

quicksave :: RPCContext (Bool)
quicksave  = do
    let r = makeRequest "SpaceCenter" "Quicksave" [  ]
    res <- sendRequest r
    processResponse extractNothing res

quickload :: RPCContext (Bool)
quickload  = do
    let r = makeRequest "SpaceCenter" "Quickload" [  ]
    res <- sendRequest r
    processResponse extractNothing res

canRailsWarpAt :: Int32 -> RPCContext (Bool)
canRailsWarpAt factorArg = do
    let r = makeRequest "SpaceCenter" "CanRailsWarpAt" [ makeArgument 0 factorArg ]
    res <- sendRequest r
    processResponse extractValue res

canRailsWarpAtStream :: Int32 -> RPCContext (KRPCStream (Bool))
canRailsWarpAtStream factorArg = do
    let r = makeRequest "SpaceCenter" "CanRailsWarpAt" [ makeArgument 0 factorArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

warpTo :: Double -> Float -> Float -> RPCContext (Bool)
warpTo uTArg maxRailsRateArg maxPhysicsRateArg = do
    let r = makeRequest "SpaceCenter" "WarpTo" [ makeArgument 0 uTArg, makeArgument 1 maxRailsRateArg, makeArgument 2 maxPhysicsRateArg ]
    res <- sendRequest r
    processResponse extractNothing res

transformPosition :: (Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext ((Double, Double, Double))
transformPosition positionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformPosition" [ makeArgumentTuple3 0 positionArg, makeArgument 1 (referenceFrameId fromArg), makeArgument 2 (referenceFrameId toArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

transformPositionStream :: (Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformPositionStream positionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformPosition" [ makeArgumentTuple3 0 positionArg, makeArgument 1 (referenceFrameId fromArg), makeArgument 2 (referenceFrameId toArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

transformDirection :: (Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext ((Double, Double, Double))
transformDirection directionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformDirection" [ makeArgumentTuple3 0 directionArg, makeArgument 1 (referenceFrameId fromArg), makeArgument 2 (referenceFrameId toArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

transformDirectionStream :: (Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformDirectionStream directionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformDirection" [ makeArgumentTuple3 0 directionArg, makeArgument 1 (referenceFrameId fromArg), makeArgument 2 (referenceFrameId toArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

transformRotation :: (Double,Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
transformRotation rotationArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformRotation" [ makeArgumentTuple4 0 rotationArg, makeArgument 1 (referenceFrameId fromArg), makeArgument 2 (referenceFrameId toArg) ]
    res <- sendRequest r
    processResponse extractTuple4 res

transformRotationStream :: (Double,Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
transformRotationStream rotationArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformRotation" [ makeArgumentTuple4 0 rotationArg, makeArgument 1 (referenceFrameId fromArg), makeArgument 2 (referenceFrameId toArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple4

transformVelocity :: (Double,Double,Double) -> (Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext ((Double, Double, Double))
transformVelocity positionArg velocityArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformVelocity" [ makeArgumentTuple3 0 positionArg, makeArgumentTuple3 1 velocityArg, makeArgument 2 (referenceFrameId fromArg), makeArgument 3 (referenceFrameId toArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

transformVelocityStream :: (Double,Double,Double) -> (Double,Double,Double) -> ReferenceFrame -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformVelocityStream positionArg velocityArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformVelocity" [ makeArgumentTuple3 0 positionArg, makeArgumentTuple3 1 velocityArg, makeArgument 2 (referenceFrameId fromArg), makeArgument 3 (referenceFrameId toArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

drawDirection :: (Double,Double,Double) -> ReferenceFrame -> (Double,Double,Double) -> Float -> RPCContext (Bool)
drawDirection directionArg referenceFrameArg colorArg lengthArg = do
    let r = makeRequest "SpaceCenter" "DrawDirection" [ makeArgumentTuple3 0 directionArg, makeArgument 1 (referenceFrameId referenceFrameArg), makeArgumentTuple3 2 colorArg, makeArgument 3 lengthArg ]
    res <- sendRequest r
    processResponse extractNothing res

drawLine :: (Double,Double,Double) -> (Double,Double,Double) -> ReferenceFrame -> (Double,Double,Double) -> RPCContext (Bool)
drawLine startArg endArg referenceFrameArg colorArg = do
    let r = makeRequest "SpaceCenter" "DrawLine" [ makeArgumentTuple3 0 startArg, makeArgumentTuple3 1 endArg, makeArgument 2 (referenceFrameId referenceFrameArg), makeArgumentTuple3 3 colorArg ]
    res <- sendRequest r
    processResponse extractNothing res

clearDrawing :: RPCContext (Bool)
clearDrawing  = do
    let r = makeRequest "SpaceCenter" "ClearDrawing" [  ]
    res <- sendRequest r
    processResponse extractNothing res

getActiveVessel :: RPCContext (Vessel)
getActiveVessel  = do
    let r = makeRequest "SpaceCenter" "get_ActiveVessel" [  ]
    res <- sendRequest r
    processResponse extractValue res

getActiveVesselStream :: RPCContext (KRPCStream (Vessel))
getActiveVesselStream  = do
    let r = makeRequest "SpaceCenter" "get_ActiveVessel" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setActiveVessel :: Vessel -> RPCContext (Bool)
setActiveVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_ActiveVessel" [ makeArgument 0 (vesselId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getVessels :: RPCContext ([Vessel])
getVessels  = do
    let r = makeRequest "SpaceCenter" "get_Vessels" [  ]
    res <- sendRequest r
    processResponse extractList res

getVesselsStream :: RPCContext (KRPCStream ([Vessel]))
getVesselsStream  = do
    let r = makeRequest "SpaceCenter" "get_Vessels" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getBodies :: RPCContext (Map Text CelestialBody)
getBodies  = do
    let r = makeRequest "SpaceCenter" "get_Bodies" [  ]
    res <- sendRequest r
    processResponse extractMap res

getBodiesStream :: RPCContext (KRPCStream (Map Text CelestialBody))
getBodiesStream  = do
    let r = makeRequest "SpaceCenter" "get_Bodies" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractMap

getTargetBody :: RPCContext (CelestialBody)
getTargetBody  = do
    let r = makeRequest "SpaceCenter" "get_TargetBody" [  ]
    res <- sendRequest r
    processResponse extractValue res

getTargetBodyStream :: RPCContext (KRPCStream (CelestialBody))
getTargetBodyStream  = do
    let r = makeRequest "SpaceCenter" "get_TargetBody" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setTargetBody :: CelestialBody -> RPCContext (Bool)
setTargetBody valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetBody" [ makeArgument 0 (celestialBodyId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getTargetVessel :: RPCContext (Vessel)
getTargetVessel  = do
    let r = makeRequest "SpaceCenter" "get_TargetVessel" [  ]
    res <- sendRequest r
    processResponse extractValue res

getTargetVesselStream :: RPCContext (KRPCStream (Vessel))
getTargetVesselStream  = do
    let r = makeRequest "SpaceCenter" "get_TargetVessel" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setTargetVessel :: Vessel -> RPCContext (Bool)
setTargetVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetVessel" [ makeArgument 0 (vesselId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getTargetDockingPort :: RPCContext (DockingPort)
getTargetDockingPort  = do
    let r = makeRequest "SpaceCenter" "get_TargetDockingPort" [  ]
    res <- sendRequest r
    processResponse extractValue res

getTargetDockingPortStream :: RPCContext (KRPCStream (DockingPort))
getTargetDockingPortStream  = do
    let r = makeRequest "SpaceCenter" "get_TargetDockingPort" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setTargetDockingPort :: DockingPort -> RPCContext (Bool)
setTargetDockingPort valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetDockingPort" [ makeArgument 0 (dockingPortId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getCamera :: RPCContext (Camera)
getCamera  = do
    let r = makeRequest "SpaceCenter" "get_Camera" [  ]
    res <- sendRequest r
    processResponse extractValue res

getCameraStream :: RPCContext (KRPCStream (Camera))
getCameraStream  = do
    let r = makeRequest "SpaceCenter" "get_Camera" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getUT :: RPCContext (Double)
getUT  = do
    let r = makeRequest "SpaceCenter" "get_UT" [  ]
    res <- sendRequest r
    processResponse extractValue res

getUTStream :: RPCContext (KRPCStream (Double))
getUTStream  = do
    let r = makeRequest "SpaceCenter" "get_UT" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getG :: RPCContext (Float)
getG  = do
    let r = makeRequest "SpaceCenter" "get_G" [  ]
    res <- sendRequest r
    processResponse extractValue res

getGStream :: RPCContext (KRPCStream (Float))
getGStream  = do
    let r = makeRequest "SpaceCenter" "get_G" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getWarpMode :: RPCContext (WarpMode)
getWarpMode  = do
    let r = makeRequest "SpaceCenter" "get_WarpMode" [  ]
    res <- sendRequest r
    processResponse extractValue res

getWarpModeStream :: RPCContext (KRPCStream (WarpMode))
getWarpModeStream  = do
    let r = makeRequest "SpaceCenter" "get_WarpMode" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getWarpRate :: RPCContext (Float)
getWarpRate  = do
    let r = makeRequest "SpaceCenter" "get_WarpRate" [  ]
    res <- sendRequest r
    processResponse extractValue res

getWarpRateStream :: RPCContext (KRPCStream (Float))
getWarpRateStream  = do
    let r = makeRequest "SpaceCenter" "get_WarpRate" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getWarpFactor :: RPCContext (Float)
getWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_WarpFactor" [  ]
    res <- sendRequest r
    processResponse extractValue res

getWarpFactorStream :: RPCContext (KRPCStream (Float))
getWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_WarpFactor" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRailsWarpFactor :: RPCContext (Int32)
getRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_RailsWarpFactor" [  ]
    res <- sendRequest r
    processResponse extractValue res

getRailsWarpFactorStream :: RPCContext (KRPCStream (Int32))
getRailsWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_RailsWarpFactor" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRailsWarpFactor :: Int32 -> RPCContext (Bool)
setRailsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_RailsWarpFactor" [ makeArgument 0 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getPhysicsWarpFactor :: RPCContext (Int32)
getPhysicsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" [  ]
    res <- sendRequest r
    processResponse extractValue res

getPhysicsWarpFactorStream :: RPCContext (KRPCStream (Int32))
getPhysicsWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setPhysicsWarpFactor :: Int32 -> RPCContext (Bool)
setPhysicsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_PhysicsWarpFactor" [ makeArgument 0 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getMaximumRailsWarpFactor :: RPCContext (Int32)
getMaximumRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" [  ]
    res <- sendRequest r
    processResponse extractValue res

getMaximumRailsWarpFactorStream :: RPCContext (KRPCStream (Int32))
getMaximumRailsWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFARAvailable :: RPCContext (Bool)
getFARAvailable  = do
    let r = makeRequest "SpaceCenter" "get_FARAvailable" [  ]
    res <- sendRequest r
    processResponse extractValue res

getFARAvailableStream :: RPCContext (KRPCStream (Bool))
getFARAvailableStream  = do
    let r = makeRequest "SpaceCenter" "get_FARAvailable" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRemoteTechAvailable :: RPCContext (Bool)
getRemoteTechAvailable  = do
    let r = makeRequest "SpaceCenter" "get_RemoteTechAvailable" [  ]
    res <- sendRequest r
    processResponse extractValue res

getRemoteTechAvailableStream :: RPCContext (KRPCStream (Bool))
getRemoteTechAvailableStream  = do
    let r = makeRequest "SpaceCenter" "get_RemoteTechAvailable" [  ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

autoPilotEngage :: AutoPilot -> RPCContext (Bool)
autoPilotEngage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Engage" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

autoPilotDisengage :: AutoPilot -> RPCContext (Bool)
autoPilotDisengage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Disengage" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

autoPilotWait :: AutoPilot -> RPCContext (Bool)
autoPilotWait thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Wait" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

autoPilotTargetPitchAndHeading :: AutoPilot -> Float -> Float -> RPCContext (Bool)
autoPilotTargetPitchAndHeading thisArg pitchArg headingArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_TargetPitchAndHeading" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 pitchArg, makeArgument 2 headingArg ]
    res <- sendRequest r
    processResponse extractNothing res

autoPilotSetPIDParameters :: AutoPilot -> Float -> Float -> Float -> RPCContext (Bool)
autoPilotSetPIDParameters thisArg kpArg kiArg kdArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_SetPIDParameters" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 kpArg, makeArgument 2 kiArg, makeArgument 3 kdArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotError :: AutoPilot -> RPCContext (Float)
getAutoPilotError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Error" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotErrorStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotErrorStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Error" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getAutoPilotRollError :: AutoPilot -> RPCContext (Float)
getAutoPilotRollError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotRollErrorStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRollErrorStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getAutoPilotReferenceFrame :: AutoPilot -> RPCContext (ReferenceFrame)
getAutoPilotReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotReferenceFrameStream :: AutoPilot -> RPCContext (KRPCStream (ReferenceFrame))
getAutoPilotReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotReferenceFrame :: AutoPilot -> ReferenceFrame -> RPCContext (Bool)
setAutoPilotReferenceFrame thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_ReferenceFrame" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 (referenceFrameId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotTargetDirection :: AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotTargetDirection thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getAutoPilotTargetDirectionStream :: AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotTargetDirectionStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

setAutoPilotTargetDirection :: AutoPilot -> (Double,Double,Double) -> RPCContext (Bool)
setAutoPilotTargetDirection thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetDirection" [ makeArgument 0 (autoPilotId thisArg), makeArgumentTuple3 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotTargetRoll :: AutoPilot -> RPCContext (Float)
getAutoPilotTargetRoll thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotTargetRollStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetRollStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotTargetRoll :: AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotTargetRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetRoll" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotSAS :: AutoPilot -> RPCContext (Bool)
getAutoPilotSAS thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotSASStream :: AutoPilot -> RPCContext (KRPCStream (Bool))
getAutoPilotSASStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotSAS :: AutoPilot -> Bool -> RPCContext (Bool)
setAutoPilotSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SAS" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotSASMode :: AutoPilot -> RPCContext (SASMode)
getAutoPilotSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotSASModeStream :: AutoPilot -> RPCContext (KRPCStream (SASMode))
getAutoPilotSASModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotSASMode :: AutoPilot -> SASMode -> RPCContext (Bool)
setAutoPilotSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SASMode" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotRotationSpeedMultiplier :: AutoPilot -> RPCContext (Float)
getAutoPilotRotationSpeedMultiplier thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RotationSpeedMultiplier" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotRotationSpeedMultiplierStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRotationSpeedMultiplierStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RotationSpeedMultiplier" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotRotationSpeedMultiplier :: AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotRotationSpeedMultiplier thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RotationSpeedMultiplier" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotMaxRotationSpeed :: AutoPilot -> RPCContext (Float)
getAutoPilotMaxRotationSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRotationSpeed" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotMaxRotationSpeedStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotMaxRotationSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRotationSpeed" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotMaxRotationSpeed :: AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotMaxRotationSpeed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_MaxRotationSpeed" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotRollSpeedMultiplier :: AutoPilot -> RPCContext (Float)
getAutoPilotRollSpeedMultiplier thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollSpeedMultiplier" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotRollSpeedMultiplierStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRollSpeedMultiplierStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollSpeedMultiplier" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotRollSpeedMultiplier :: AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotRollSpeedMultiplier thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RollSpeedMultiplier" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getAutoPilotMaxRollSpeed :: AutoPilot -> RPCContext (Float)
getAutoPilotMaxRollSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRollSpeed" [ makeArgument 0 (autoPilotId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getAutoPilotMaxRollSpeedStream :: AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotMaxRollSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRollSpeed" [ makeArgument 0 (autoPilotId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setAutoPilotMaxRollSpeed :: AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotMaxRollSpeed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_MaxRollSpeed" [ makeArgument 0 (autoPilotId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraMode :: Camera -> RPCContext (CameraMode)
getCameraMode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Mode" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraModeStream :: Camera -> RPCContext (KRPCStream (CameraMode))
getCameraModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Mode" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraMode :: Camera -> CameraMode -> RPCContext (Bool)
setCameraMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Mode" [ makeArgument 0 (cameraId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraPitch :: Camera -> RPCContext (Float)
getCameraPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Pitch" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraPitchStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Pitch" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraPitch :: Camera -> Float -> RPCContext (Bool)
setCameraPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Pitch" [ makeArgument 0 (cameraId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraHeading :: Camera -> RPCContext (Float)
getCameraHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Heading" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraHeadingStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraHeadingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Heading" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraHeading :: Camera -> Float -> RPCContext (Bool)
setCameraHeading thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Heading" [ makeArgument 0 (cameraId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraDistance :: Camera -> RPCContext (Float)
getCameraDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Distance" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraDistanceStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Distance" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraDistance :: Camera -> Float -> RPCContext (Bool)
setCameraDistance thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Distance" [ makeArgument 0 (cameraId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraMinPitch :: Camera -> RPCContext (Float)
getCameraMinPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinPitch" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraMinPitchStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraMinPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinPitch" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCameraMaxPitch :: Camera -> RPCContext (Float)
getCameraMaxPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraMaxPitchStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraMaxPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCameraMinDistance :: Camera -> RPCContext (Float)
getCameraMinDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinDistance" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraMinDistanceStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraMinDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinDistance" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCameraMaxDistance :: Camera -> RPCContext (Float)
getCameraMaxDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraMaxDistanceStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraMaxDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCameraDefaultDistance :: Camera -> RPCContext (Float)
getCameraDefaultDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraDefaultDistanceStream :: Camera -> RPCContext (KRPCStream (Float))
getCameraDefaultDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCameraFocussedBody :: Camera -> RPCContext (CelestialBody)
getCameraFocussedBody thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraFocussedBodyStream :: Camera -> RPCContext (KRPCStream (CelestialBody))
getCameraFocussedBodyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraFocussedBody :: Camera -> CelestialBody -> RPCContext (Bool)
setCameraFocussedBody thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedBody" [ makeArgument 0 (cameraId thisArg), makeArgument 1 (celestialBodyId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraFocussedVessel :: Camera -> RPCContext (Vessel)
getCameraFocussedVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraFocussedVesselStream :: Camera -> RPCContext (KRPCStream (Vessel))
getCameraFocussedVesselStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraFocussedVessel :: Camera -> Vessel -> RPCContext (Bool)
setCameraFocussedVessel thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedVessel" [ makeArgument 0 (cameraId thisArg), makeArgument 1 (vesselId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getCameraFocussedNode :: Camera -> RPCContext (Node)
getCameraFocussedNode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [ makeArgument 0 (cameraId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCameraFocussedNodeStream :: Camera -> RPCContext (KRPCStream (Node))
getCameraFocussedNodeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [ makeArgument 0 (cameraId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCameraFocussedNode :: Camera -> Node -> RPCContext (Bool)
setCameraFocussedNode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedNode" [ makeArgument 0 (cameraId thisArg), makeArgument 1 (nodeId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

celestialBodySurfaceHeight :: CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodySurfaceHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg ]
    res <- sendRequest r
    processResponse extractValue res

celestialBodySurfaceHeightStream :: CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodySurfaceHeightStream thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

celestialBodyBedrockHeight :: CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodyBedrockHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg ]
    res <- sendRequest r
    processResponse extractValue res

celestialBodyBedrockHeightStream :: CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodyBedrockHeightStream thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

celestialBodyMSLPosition :: CelestialBody -> Double -> Double -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyMSLPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodyMSLPositionStream :: CelestialBody -> Double -> Double -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyMSLPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

celestialBodySurfacePosition :: CelestialBody -> Double -> Double -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodySurfacePosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodySurfacePositionStream :: CelestialBody -> Double -> Double -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodySurfacePositionStream thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

celestialBodyBedrockPosition :: CelestialBody -> Double -> Double -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyBedrockPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodyBedrockPositionStream :: CelestialBody -> Double -> Double -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyBedrockPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

celestialBodyPosition :: CelestialBody -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Position" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodyPositionStream :: CelestialBody -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Position" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

celestialBodyVelocity :: CelestialBody -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Velocity" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodyVelocityStream :: CelestialBody -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Velocity" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

celestialBodyRotation :: CelestialBody -> ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
celestialBodyRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Rotation" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple4 res

celestialBodyRotationStream :: CelestialBody -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
celestialBodyRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Rotation" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple4

celestialBodyDirection :: CelestialBody -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Direction" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodyDirectionStream :: CelestialBody -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Direction" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

celestialBodyAngularVelocity :: CelestialBody -> ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

celestialBodyAngularVelocityStream :: CelestialBody -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyAngularVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [ makeArgument 0 (celestialBodyId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getCelestialBodyName :: CelestialBody -> RPCContext (Text)
getCelestialBodyName thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Name" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyNameStream :: CelestialBody -> RPCContext (KRPCStream (Text))
getCelestialBodyNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Name" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodySatellites :: CelestialBody -> RPCContext ([CelestialBody])
getCelestialBodySatellites thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getCelestialBodySatellitesStream :: CelestialBody -> RPCContext (KRPCStream ([CelestialBody]))
getCelestialBodySatellitesStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getCelestialBodyMass :: CelestialBody -> RPCContext (Float)
getCelestialBodyMass thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyMassStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyGravitationalParameter :: CelestialBody -> RPCContext (Float)
getCelestialBodyGravitationalParameter thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyGravitationalParameterStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyGravitationalParameterStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodySurfaceGravity :: CelestialBody -> RPCContext (Float)
getCelestialBodySurfaceGravity thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodySurfaceGravityStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySurfaceGravityStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyRotationalPeriod :: CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyRotationalPeriodStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalPeriodStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyRotationalSpeed :: CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyRotationalSpeedStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyEquatorialRadius :: CelestialBody -> RPCContext (Float)
getCelestialBodyEquatorialRadius thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyEquatorialRadiusStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyEquatorialRadiusStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodySphereOfInfluence :: CelestialBody -> RPCContext (Float)
getCelestialBodySphereOfInfluence thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodySphereOfInfluenceStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySphereOfInfluenceStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyOrbit :: CelestialBody -> RPCContext (Orbit)
getCelestialBodyOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyOrbitStream :: CelestialBody -> RPCContext (KRPCStream (Orbit))
getCelestialBodyOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyHasAtmosphere :: CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphere thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyHasAtmosphereStream :: CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphereStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyAtmosphereDepth :: CelestialBody -> RPCContext (Float)
getCelestialBodyAtmosphereDepth thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyAtmosphereDepthStream :: CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyAtmosphereDepthStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyHasAtmosphericOxygen :: CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphericOxygen thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyHasAtmosphericOxygenStream :: CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphericOxygenStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyReferenceFrame :: CelestialBody -> RPCContext (ReferenceFrame)
getCelestialBodyReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyReferenceFrameStream :: CelestialBody -> RPCContext (KRPCStream (ReferenceFrame))
getCelestialBodyReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyNonRotatingReferenceFrame :: CelestialBody -> RPCContext (ReferenceFrame)
getCelestialBodyNonRotatingReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyNonRotatingReferenceFrameStream :: CelestialBody -> RPCContext (KRPCStream (ReferenceFrame))
getCelestialBodyNonRotatingReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCelestialBodyOrbitalReferenceFrame :: CelestialBody -> RPCContext (ReferenceFrame)
getCelestialBodyOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [ makeArgument 0 (celestialBodyId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCelestialBodyOrbitalReferenceFrameStream :: CelestialBody -> RPCContext (KRPCStream (ReferenceFrame))
getCelestialBodyOrbitalReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [ makeArgument 0 (celestialBodyId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

commsSignalDelayToVessel :: Comms -> Vessel -> RPCContext (Double)
commsSignalDelayToVessel thisArg otherArg = do
    let r = makeRequest "SpaceCenter" "Comms_SignalDelayToVessel" [ makeArgument 0 (commsId thisArg), makeArgument 1 (vesselId otherArg) ]
    res <- sendRequest r
    processResponse extractValue res

commsSignalDelayToVesselStream :: Comms -> Vessel -> RPCContext (KRPCStream (Double))
commsSignalDelayToVesselStream thisArg otherArg = do
    let r = makeRequest "SpaceCenter" "Comms_SignalDelayToVessel" [ makeArgument 0 (commsId thisArg), makeArgument 1 (vesselId otherArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCommsHasLocalControl :: Comms -> RPCContext (Bool)
getCommsHasLocalControl thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasLocalControl" [ makeArgument 0 (commsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCommsHasLocalControlStream :: Comms -> RPCContext (KRPCStream (Bool))
getCommsHasLocalControlStream thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasLocalControl" [ makeArgument 0 (commsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCommsHasFlightComputer :: Comms -> RPCContext (Bool)
getCommsHasFlightComputer thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasFlightComputer" [ makeArgument 0 (commsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCommsHasFlightComputerStream :: Comms -> RPCContext (KRPCStream (Bool))
getCommsHasFlightComputerStream thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasFlightComputer" [ makeArgument 0 (commsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCommsHasConnection :: Comms -> RPCContext (Bool)
getCommsHasConnection thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasConnection" [ makeArgument 0 (commsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCommsHasConnectionStream :: Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasConnection" [ makeArgument 0 (commsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCommsHasConnectionToGroundStation :: Comms -> RPCContext (Bool)
getCommsHasConnectionToGroundStation thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasConnectionToGroundStation" [ makeArgument 0 (commsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCommsHasConnectionToGroundStationStream :: Comms -> RPCContext (KRPCStream (Bool))
getCommsHasConnectionToGroundStationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_HasConnectionToGroundStation" [ makeArgument 0 (commsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCommsSignalDelay :: Comms -> RPCContext (Double)
getCommsSignalDelay thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_SignalDelay" [ makeArgument 0 (commsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCommsSignalDelayStream :: Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayStream thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_SignalDelay" [ makeArgument 0 (commsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCommsSignalDelayToGroundStation :: Comms -> RPCContext (Double)
getCommsSignalDelayToGroundStation thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_SignalDelayToGroundStation" [ makeArgument 0 (commsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCommsSignalDelayToGroundStationStream :: Comms -> RPCContext (KRPCStream (Double))
getCommsSignalDelayToGroundStationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Comms_get_SignalDelayToGroundStation" [ makeArgument 0 (commsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

controlActivateNextStage :: Control -> RPCContext ([Vessel])
controlActivateNextStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_ActivateNextStage" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

controlActivateNextStageStream :: Control -> RPCContext (KRPCStream ([Vessel]))
controlActivateNextStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_ActivateNextStage" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

controlGetActionGroup :: Control -> Word32 -> RPCContext (Bool)
controlGetActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_GetActionGroup" [ makeArgument 0 (controlId thisArg), makeArgument 1 groupArg ]
    res <- sendRequest r
    processResponse extractValue res

controlGetActionGroupStream :: Control -> Word32 -> RPCContext (KRPCStream (Bool))
controlGetActionGroupStream thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_GetActionGroup" [ makeArgument 0 (controlId thisArg), makeArgument 1 groupArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

controlSetActionGroup :: Control -> Word32 -> Bool -> RPCContext (Bool)
controlSetActionGroup thisArg groupArg stateArg = do
    let r = makeRequest "SpaceCenter" "Control_SetActionGroup" [ makeArgument 0 (controlId thisArg), makeArgument 1 groupArg, makeArgument 2 stateArg ]
    res <- sendRequest r
    processResponse extractNothing res

controlToggleActionGroup :: Control -> Word32 -> RPCContext (Bool)
controlToggleActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_ToggleActionGroup" [ makeArgument 0 (controlId thisArg), makeArgument 1 groupArg ]
    res <- sendRequest r
    processResponse extractNothing res

controlAddNode :: Control -> Double -> Float -> Float -> Float -> RPCContext (Node)
controlAddNode thisArg uTArg progradeArg normalArg radialArg = do
    let r = makeRequest "SpaceCenter" "Control_AddNode" [ makeArgument 0 (controlId thisArg), makeArgument 1 uTArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg ]
    res <- sendRequest r
    processResponse extractValue res

controlAddNodeStream :: Control -> Double -> Float -> Float -> Float -> RPCContext (KRPCStream (Node))
controlAddNodeStream thisArg uTArg progradeArg normalArg radialArg = do
    let r = makeRequest "SpaceCenter" "Control_AddNode" [ makeArgument 0 (controlId thisArg), makeArgument 1 uTArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

controlRemoveNodes :: Control -> RPCContext (Bool)
controlRemoveNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_RemoveNodes" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSAS :: Control -> RPCContext (Bool)
getControlSAS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SAS" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSASStream :: Control -> RPCContext (KRPCStream (Bool))
getControlSASStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SAS" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSAS :: Control -> Bool -> RPCContext (Bool)
setControlSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SAS" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSASMode :: Control -> RPCContext (SASMode)
getControlSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SASMode" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSASModeStream :: Control -> RPCContext (KRPCStream (SASMode))
getControlSASModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SASMode" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSASMode :: Control -> SASMode -> RPCContext (Bool)
setControlSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SASMode" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSpeedMode :: Control -> RPCContext (SpeedMode)
getControlSpeedMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SpeedMode" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSpeedModeStream :: Control -> RPCContext (KRPCStream (SpeedMode))
getControlSpeedModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SpeedMode" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSpeedMode :: Control -> SpeedMode -> RPCContext (Bool)
setControlSpeedMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SpeedMode" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlRCS :: Control -> RPCContext (Bool)
getControlRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_RCS" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlRCSStream :: Control -> RPCContext (KRPCStream (Bool))
getControlRCSStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_RCS" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlRCS :: Control -> Bool -> RPCContext (Bool)
setControlRCS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_RCS" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlGear :: Control -> RPCContext (Bool)
getControlGear thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Gear" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlGearStream :: Control -> RPCContext (KRPCStream (Bool))
getControlGearStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Gear" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlGear :: Control -> Bool -> RPCContext (Bool)
setControlGear thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Gear" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlLights :: Control -> RPCContext (Bool)
getControlLights thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Lights" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlLightsStream :: Control -> RPCContext (KRPCStream (Bool))
getControlLightsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Lights" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlLights :: Control -> Bool -> RPCContext (Bool)
setControlLights thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Lights" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlBrakes :: Control -> RPCContext (Bool)
getControlBrakes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Brakes" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlBrakesStream :: Control -> RPCContext (KRPCStream (Bool))
getControlBrakesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Brakes" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlBrakes :: Control -> Bool -> RPCContext (Bool)
setControlBrakes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Brakes" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlAbort :: Control -> RPCContext (Bool)
getControlAbort thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Abort" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlAbortStream :: Control -> RPCContext (KRPCStream (Bool))
getControlAbortStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Abort" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlAbort :: Control -> Bool -> RPCContext (Bool)
setControlAbort thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Abort" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlThrottle :: Control -> RPCContext (Float)
getControlThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Throttle" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlThrottleStream :: Control -> RPCContext (KRPCStream (Float))
getControlThrottleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Throttle" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlThrottle :: Control -> Float -> RPCContext (Bool)
setControlThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Throttle" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlPitch :: Control -> RPCContext (Float)
getControlPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Pitch" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlPitchStream :: Control -> RPCContext (KRPCStream (Float))
getControlPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Pitch" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlPitch :: Control -> Float -> RPCContext (Bool)
setControlPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Pitch" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlYaw :: Control -> RPCContext (Float)
getControlYaw thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Yaw" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlYawStream :: Control -> RPCContext (KRPCStream (Float))
getControlYawStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Yaw" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlYaw :: Control -> Float -> RPCContext (Bool)
setControlYaw thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Yaw" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlRoll :: Control -> RPCContext (Float)
getControlRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Roll" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlRollStream :: Control -> RPCContext (KRPCStream (Float))
getControlRollStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Roll" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlRoll :: Control -> Float -> RPCContext (Bool)
setControlRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Roll" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlForward :: Control -> RPCContext (Float)
getControlForward thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Forward" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlForwardStream :: Control -> RPCContext (KRPCStream (Float))
getControlForwardStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Forward" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlForward :: Control -> Float -> RPCContext (Bool)
setControlForward thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Forward" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlUp :: Control -> RPCContext (Float)
getControlUp thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Up" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlUpStream :: Control -> RPCContext (KRPCStream (Float))
getControlUpStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Up" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlUp :: Control -> Float -> RPCContext (Bool)
setControlUp thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Up" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlRight :: Control -> RPCContext (Float)
getControlRight thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Right" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlRightStream :: Control -> RPCContext (KRPCStream (Float))
getControlRightStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Right" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlRight :: Control -> Float -> RPCContext (Bool)
setControlRight thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Right" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlWheelThrottle :: Control -> RPCContext (Float)
getControlWheelThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlWheelThrottleStream :: Control -> RPCContext (KRPCStream (Float))
getControlWheelThrottleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlWheelThrottle :: Control -> Float -> RPCContext (Bool)
setControlWheelThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelThrottle" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlWheelSteering :: Control -> RPCContext (Float)
getControlWheelSteering thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelSteering" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlWheelSteeringStream :: Control -> RPCContext (KRPCStream (Float))
getControlWheelSteeringStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelSteering" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlWheelSteering :: Control -> Float -> RPCContext (Bool)
setControlWheelSteering thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelSteering" [ makeArgument 0 (controlId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlCurrentStage :: Control -> RPCContext (Int32)
getControlCurrentStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CurrentStage" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlCurrentStageStream :: Control -> RPCContext (KRPCStream (Int32))
getControlCurrentStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CurrentStage" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getControlNodes :: Control -> RPCContext ([Node])
getControlNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Nodes" [ makeArgument 0 (controlId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getControlNodesStream :: Control -> RPCContext (KRPCStream ([Node]))
getControlNodesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Nodes" [ makeArgument 0 (controlId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getFlightGForce :: Flight -> RPCContext (Float)
getFlightGForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_GForce" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightGForceStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightGForceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_GForce" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightMeanAltitude :: Flight -> RPCContext (Double)
getFlightMeanAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightMeanAltitudeStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightMeanAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightSurfaceAltitude :: Flight -> RPCContext (Double)
getFlightSurfaceAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightSurfaceAltitudeStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightSurfaceAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightBedrockAltitude :: Flight -> RPCContext (Double)
getFlightBedrockAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightBedrockAltitudeStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightBedrockAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightElevation :: Flight -> RPCContext (Double)
getFlightElevation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Elevation" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightElevationStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightElevationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Elevation" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightLatitude :: Flight -> RPCContext (Double)
getFlightLatitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Latitude" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightLatitudeStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightLatitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Latitude" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightLongitude :: Flight -> RPCContext (Double)
getFlightLongitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Longitude" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightLongitudeStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightLongitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Longitude" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightVelocity :: Flight -> RPCContext ((Double, Double, Double))
getFlightVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Velocity" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightVelocityStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightVelocityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Velocity" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightSpeed :: Flight -> RPCContext (Double)
getFlightSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Speed" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightSpeedStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Speed" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightHorizontalSpeed :: Flight -> RPCContext (Double)
getFlightHorizontalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightHorizontalSpeedStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightHorizontalSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightVerticalSpeed :: Flight -> RPCContext (Double)
getFlightVerticalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightVerticalSpeedStream :: Flight -> RPCContext (KRPCStream (Double))
getFlightVerticalSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightCenterOfMass :: Flight -> RPCContext ((Double, Double, Double))
getFlightCenterOfMass thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightCenterOfMassStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightCenterOfMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightRotation :: Flight -> RPCContext ((Double, Double, Double, Double))
getFlightRotation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Rotation" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple4 res

getFlightRotationStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getFlightRotationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Rotation" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple4

getFlightDirection :: Flight -> RPCContext ((Double, Double, Double))
getFlightDirection thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Direction" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightDirectionStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDirectionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Direction" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightPitch :: Flight -> RPCContext (Float)
getFlightPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Pitch" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightPitchStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Pitch" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightHeading :: Flight -> RPCContext (Float)
getFlightHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Heading" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightHeadingStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightHeadingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Heading" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightRoll :: Flight -> RPCContext (Float)
getFlightRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Roll" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightRollStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightRollStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Roll" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightPrograde :: Flight -> RPCContext ((Double, Double, Double))
getFlightPrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Prograde" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightProgradeStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightProgradeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Prograde" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightRetrograde :: Flight -> RPCContext ((Double, Double, Double))
getFlightRetrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Retrograde" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightRetrogradeStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRetrogradeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Retrograde" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightNormal :: Flight -> RPCContext ((Double, Double, Double))
getFlightNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Normal" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightNormalStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightNormalStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Normal" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightAntiNormal :: Flight -> RPCContext ((Double, Double, Double))
getFlightAntiNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightAntiNormalStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiNormalStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightRadial :: Flight -> RPCContext ((Double, Double, Double))
getFlightRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Radial" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightRadialStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRadialStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Radial" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightAntiRadial :: Flight -> RPCContext ((Double, Double, Double))
getFlightAntiRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightAntiRadialStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiRadialStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightAtmosphereDensity :: Flight -> RPCContext (Float)
getFlightAtmosphereDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightAtmosphereDensityStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightAtmosphereDensityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightDynamicPressure :: Flight -> RPCContext (Float)
getFlightDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightDynamicPressureStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightDynamicPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightStaticPressure :: Flight -> RPCContext (Float)
getFlightStaticPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightStaticPressureStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightStaticPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightAerodynamicForce :: Flight -> RPCContext ((Double, Double, Double))
getFlightAerodynamicForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightAerodynamicForceStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAerodynamicForceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightLift :: Flight -> RPCContext ((Double, Double, Double))
getFlightLift thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Lift" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightLiftStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightLiftStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Lift" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightDrag :: Flight -> RPCContext ((Double, Double, Double))
getFlightDrag thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Drag" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getFlightDragStream :: Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDragStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Drag" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getFlightSpeedOfSound :: Flight -> RPCContext (Float)
getFlightSpeedOfSound thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightSpeedOfSoundStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightSpeedOfSoundStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightMach :: Flight -> RPCContext (Float)
getFlightMach thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Mach" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightMachStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightMachStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Mach" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightEquivalentAirSpeed :: Flight -> RPCContext (Float)
getFlightEquivalentAirSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightEquivalentAirSpeedStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightEquivalentAirSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightTerminalVelocity :: Flight -> RPCContext (Float)
getFlightTerminalVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightTerminalVelocityStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightTerminalVelocityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightAngleOfAttack :: Flight -> RPCContext (Float)
getFlightAngleOfAttack thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightAngleOfAttackStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightAngleOfAttackStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightSideslipAngle :: Flight -> RPCContext (Float)
getFlightSideslipAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightSideslipAngleStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightSideslipAngleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightTotalAirTemperature :: Flight -> RPCContext (Float)
getFlightTotalAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightTotalAirTemperatureStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightTotalAirTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightStaticAirTemperature :: Flight -> RPCContext (Float)
getFlightStaticAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightStaticAirTemperatureStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightStaticAirTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightStallFraction :: Flight -> RPCContext (Float)
getFlightStallFraction thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StallFraction" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightStallFractionStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightStallFractionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StallFraction" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightDragCoefficient :: Flight -> RPCContext (Float)
getFlightDragCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightDragCoefficientStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightDragCoefficientStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightLiftCoefficient :: Flight -> RPCContext (Float)
getFlightLiftCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightLiftCoefficientStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightLiftCoefficientStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightBallisticCoefficient :: Flight -> RPCContext (Float)
getFlightBallisticCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightBallisticCoefficientStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightBallisticCoefficientStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFlightThrustSpecificFuelConsumption :: Flight -> RPCContext (Float)
getFlightThrustSpecificFuelConsumption thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [ makeArgument 0 (flightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFlightThrustSpecificFuelConsumptionStream :: Flight -> RPCContext (KRPCStream (Float))
getFlightThrustSpecificFuelConsumptionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [ makeArgument 0 (flightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

nodeBurnVector :: Node -> ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_BurnVector" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

nodeBurnVectorStream :: Node -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeBurnVectorStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_BurnVector" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

nodeRemainingBurnVector :: Node -> ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeRemainingBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

nodeRemainingBurnVectorStream :: Node -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeRemainingBurnVectorStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

nodeRemove :: Node -> RPCContext (Bool)
nodeRemove thisArg = do
    let r = makeRequest "SpaceCenter" "Node_Remove" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

nodePosition :: Node -> ReferenceFrame -> RPCContext ((Double, Double, Double))
nodePosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Position" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

nodePositionStream :: Node -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodePositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Position" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

nodeDirection :: Node -> ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Direction" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

nodeDirectionStream :: Node -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Direction" [ makeArgument 0 (nodeId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getNodePrograde :: Node -> RPCContext (Float)
getNodePrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Prograde" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeProgradeStream :: Node -> RPCContext (KRPCStream (Float))
getNodeProgradeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Prograde" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setNodePrograde :: Node -> Float -> RPCContext (Bool)
setNodePrograde thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Prograde" [ makeArgument 0 (nodeId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getNodeNormal :: Node -> RPCContext (Float)
getNodeNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Normal" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeNormalStream :: Node -> RPCContext (KRPCStream (Float))
getNodeNormalStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Normal" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setNodeNormal :: Node -> Float -> RPCContext (Bool)
setNodeNormal thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Normal" [ makeArgument 0 (nodeId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getNodeRadial :: Node -> RPCContext (Float)
getNodeRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Radial" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeRadialStream :: Node -> RPCContext (KRPCStream (Float))
getNodeRadialStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Radial" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setNodeRadial :: Node -> Float -> RPCContext (Bool)
setNodeRadial thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Radial" [ makeArgument 0 (nodeId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getNodeDeltaV :: Node -> RPCContext (Float)
getNodeDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_DeltaV" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeDeltaVStream :: Node -> RPCContext (KRPCStream (Float))
getNodeDeltaVStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_DeltaV" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setNodeDeltaV :: Node -> Float -> RPCContext (Bool)
setNodeDeltaV thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_DeltaV" [ makeArgument 0 (nodeId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getNodeRemainingDeltaV :: Node -> RPCContext (Float)
getNodeRemainingDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeRemainingDeltaVStream :: Node -> RPCContext (KRPCStream (Float))
getNodeRemainingDeltaVStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getNodeUT :: Node -> RPCContext (Double)
getNodeUT thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_UT" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeUTStream :: Node -> RPCContext (KRPCStream (Double))
getNodeUTStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_UT" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setNodeUT :: Node -> Double -> RPCContext (Bool)
setNodeUT thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_UT" [ makeArgument 0 (nodeId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getNodeTimeTo :: Node -> RPCContext (Double)
getNodeTimeTo thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_TimeTo" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeTimeToStream :: Node -> RPCContext (KRPCStream (Double))
getNodeTimeToStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_TimeTo" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getNodeOrbit :: Node -> RPCContext (Orbit)
getNodeOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Orbit" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeOrbitStream :: Node -> RPCContext (KRPCStream (Orbit))
getNodeOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Orbit" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getNodeReferenceFrame :: Node -> RPCContext (ReferenceFrame)
getNodeReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeReferenceFrameStream :: Node -> RPCContext (KRPCStream (ReferenceFrame))
getNodeReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getNodeOrbitalReferenceFrame :: Node -> RPCContext (ReferenceFrame)
getNodeOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [ makeArgument 0 (nodeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getNodeOrbitalReferenceFrameStream :: Node -> RPCContext (KRPCStream (ReferenceFrame))
getNodeOrbitalReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [ makeArgument 0 (nodeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

orbitReferencePlaneNormal :: ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitReferencePlaneNormal referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [ makeArgument 0 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

orbitReferencePlaneNormalStream :: ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneNormalStream referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [ makeArgument 0 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

orbitReferencePlaneDirection :: ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitReferencePlaneDirection referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [ makeArgument 0 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

orbitReferencePlaneDirectionStream :: ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneDirectionStream referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [ makeArgument 0 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getOrbitBody :: Orbit -> RPCContext (CelestialBody)
getOrbitBody thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Body" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitBodyStream :: Orbit -> RPCContext (KRPCStream (CelestialBody))
getOrbitBodyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Body" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitApoapsis :: Orbit -> RPCContext (Double)
getOrbitApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitApoapsisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitPeriapsis :: Orbit -> RPCContext (Double)
getOrbitPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitPeriapsisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitApoapsisAltitude :: Orbit -> RPCContext (Double)
getOrbitApoapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitApoapsisAltitudeStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitPeriapsisAltitude :: Orbit -> RPCContext (Double)
getOrbitPeriapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitPeriapsisAltitudeStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitSemiMajorAxis :: Orbit -> RPCContext (Double)
getOrbitSemiMajorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitSemiMajorAxisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMajorAxisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitSemiMinorAxis :: Orbit -> RPCContext (Double)
getOrbitSemiMinorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitSemiMinorAxisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMinorAxisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitRadius :: Orbit -> RPCContext (Double)
getOrbitRadius thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Radius" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitRadiusStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitRadiusStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Radius" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitSpeed :: Orbit -> RPCContext (Double)
getOrbitSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Speed" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitSpeedStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Speed" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitPeriod :: Orbit -> RPCContext (Double)
getOrbitPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Period" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitPeriodStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriodStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Period" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitTimeToApoapsis :: Orbit -> RPCContext (Double)
getOrbitTimeToApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitTimeToApoapsisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToApoapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitTimeToPeriapsis :: Orbit -> RPCContext (Double)
getOrbitTimeToPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitTimeToPeriapsisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToPeriapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitEccentricity :: Orbit -> RPCContext (Double)
getOrbitEccentricity thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitEccentricityStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitInclination :: Orbit -> RPCContext (Double)
getOrbitInclination thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Inclination" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitInclinationStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitInclinationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Inclination" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitLongitudeOfAscendingNode :: Orbit -> RPCContext (Double)
getOrbitLongitudeOfAscendingNode thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitLongitudeOfAscendingNodeStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitLongitudeOfAscendingNodeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitArgumentOfPeriapsis :: Orbit -> RPCContext (Double)
getOrbitArgumentOfPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitArgumentOfPeriapsisStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitArgumentOfPeriapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitMeanAnomalyAtEpoch :: Orbit -> RPCContext (Double)
getOrbitMeanAnomalyAtEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitMeanAnomalyAtEpochStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyAtEpochStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitEpoch :: Orbit -> RPCContext (Double)
getOrbitEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Epoch" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitEpochStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitEpochStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Epoch" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitMeanAnomaly :: Orbit -> RPCContext (Double)
getOrbitMeanAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitMeanAnomalyStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitEccentricAnomaly :: Orbit -> RPCContext (Double)
getOrbitEccentricAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitEccentricAnomalyStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricAnomalyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitNextOrbit :: Orbit -> RPCContext (Orbit)
getOrbitNextOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitNextOrbitStream :: Orbit -> RPCContext (KRPCStream (Orbit))
getOrbitNextOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getOrbitTimeToSOIChange :: Orbit -> RPCContext (Double)
getOrbitTimeToSOIChange thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [ makeArgument 0 (orbitId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getOrbitTimeToSOIChangeStream :: Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToSOIChangeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [ makeArgument 0 (orbitId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCargoBayPart :: CargoBay -> RPCContext (Part)
getCargoBayPart thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Part" [ makeArgument 0 (cargoBayId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCargoBayPartStream :: CargoBay -> RPCContext (KRPCStream (Part))
getCargoBayPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Part" [ makeArgument 0 (cargoBayId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCargoBayState :: CargoBay -> RPCContext (CargoBayState)
getCargoBayState thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_State" [ makeArgument 0 (cargoBayId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCargoBayStateStream :: CargoBay -> RPCContext (KRPCStream (CargoBayState))
getCargoBayStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_State" [ makeArgument 0 (cargoBayId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getCargoBayOpen :: CargoBay -> RPCContext (Bool)
getCargoBayOpen thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Open" [ makeArgument 0 (cargoBayId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getCargoBayOpenStream :: CargoBay -> RPCContext (KRPCStream (Bool))
getCargoBayOpenStream thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Open" [ makeArgument 0 (cargoBayId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setCargoBayOpen :: CargoBay -> Bool -> RPCContext (Bool)
setCargoBayOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_set_Open" [ makeArgument 0 (cargoBayId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSurfacePart :: ControlSurface -> RPCContext (Part)
getControlSurfacePart thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Part" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfacePartStream :: ControlSurface -> RPCContext (KRPCStream (Part))
getControlSurfacePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Part" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getControlSurfacePitchEnabled :: ControlSurface -> RPCContext (Bool)
getControlSurfacePitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfacePitchEnabledStream :: ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfacePitchEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSurfacePitchEnabled :: ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfacePitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_PitchEnabled" [ makeArgument 0 (controlSurfaceId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSurfaceYawEnabled :: ControlSurface -> RPCContext (Bool)
getControlSurfaceYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfaceYawEnabledStream :: ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceYawEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSurfaceYawEnabled :: ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_YawEnabled" [ makeArgument 0 (controlSurfaceId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSurfaceRollEnabled :: ControlSurface -> RPCContext (Bool)
getControlSurfaceRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfaceRollEnabledStream :: ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceRollEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSurfaceRollEnabled :: ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_RollEnabled" [ makeArgument 0 (controlSurfaceId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSurfaceInverted :: ControlSurface -> RPCContext (Bool)
getControlSurfaceInverted thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfaceInvertedStream :: ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceInvertedStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSurfaceInverted :: ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceInverted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Inverted" [ makeArgument 0 (controlSurfaceId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSurfaceDeployed :: ControlSurface -> RPCContext (Bool)
getControlSurfaceDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfaceDeployedStream :: ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setControlSurfaceDeployed :: ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Deployed" [ makeArgument 0 (controlSurfaceId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getControlSurfaceSurfaceArea :: ControlSurface -> RPCContext (Float)
getControlSurfaceSurfaceArea thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [ makeArgument 0 (controlSurfaceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getControlSurfaceSurfaceAreaStream :: ControlSurface -> RPCContext (KRPCStream (Float))
getControlSurfaceSurfaceAreaStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [ makeArgument 0 (controlSurfaceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

decouplerDecouple :: Decoupler -> RPCContext (Vessel)
decouplerDecouple thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_Decouple" [ makeArgument 0 (decouplerId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

decouplerDecoupleStream :: Decoupler -> RPCContext (KRPCStream (Vessel))
decouplerDecoupleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_Decouple" [ makeArgument 0 (decouplerId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDecouplerPart :: Decoupler -> RPCContext (Part)
getDecouplerPart thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Part" [ makeArgument 0 (decouplerId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDecouplerPartStream :: Decoupler -> RPCContext (KRPCStream (Part))
getDecouplerPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Part" [ makeArgument 0 (decouplerId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDecouplerDecoupled :: Decoupler -> RPCContext (Bool)
getDecouplerDecoupled thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [ makeArgument 0 (decouplerId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDecouplerDecoupledStream :: Decoupler -> RPCContext (KRPCStream (Bool))
getDecouplerDecoupledStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [ makeArgument 0 (decouplerId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDecouplerImpulse :: Decoupler -> RPCContext (Float)
getDecouplerImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [ makeArgument 0 (decouplerId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDecouplerImpulseStream :: Decoupler -> RPCContext (KRPCStream (Float))
getDecouplerImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [ makeArgument 0 (decouplerId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

dockingPortUndock :: DockingPort -> RPCContext (Vessel)
dockingPortUndock thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Undock" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

dockingPortUndockStream :: DockingPort -> RPCContext (KRPCStream (Vessel))
dockingPortUndockStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Undock" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

dockingPortPosition :: DockingPort -> ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Position" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

dockingPortPositionStream :: DockingPort -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Position" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

dockingPortDirection :: DockingPort -> ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Direction" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

dockingPortDirectionStream :: DockingPort -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Direction" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

dockingPortRotation :: DockingPort -> ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
dockingPortRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Rotation" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple4 res

dockingPortRotationStream :: DockingPort -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
dockingPortRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Rotation" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple4

getDockingPortPart :: DockingPort -> RPCContext (Part)
getDockingPortPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Part" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortPartStream :: DockingPort -> RPCContext (KRPCStream (Part))
getDockingPortPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Part" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDockingPortName :: DockingPort -> RPCContext (Text)
getDockingPortName thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Name" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortNameStream :: DockingPort -> RPCContext (KRPCStream (Text))
getDockingPortNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Name" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setDockingPortName :: DockingPort -> Text -> RPCContext (Bool)
setDockingPortName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Name" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getDockingPortState :: DockingPort -> RPCContext (DockingPortState)
getDockingPortState thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_State" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortStateStream :: DockingPort -> RPCContext (KRPCStream (DockingPortState))
getDockingPortStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_State" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDockingPortDockedPart :: DockingPort -> RPCContext (Part)
getDockingPortDockedPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortDockedPartStream :: DockingPort -> RPCContext (KRPCStream (Part))
getDockingPortDockedPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDockingPortReengageDistance :: DockingPort -> RPCContext (Float)
getDockingPortReengageDistance thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortReengageDistanceStream :: DockingPort -> RPCContext (KRPCStream (Float))
getDockingPortReengageDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDockingPortHasShield :: DockingPort -> RPCContext (Bool)
getDockingPortHasShield thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortHasShieldStream :: DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortHasShieldStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getDockingPortShielded :: DockingPort -> RPCContext (Bool)
getDockingPortShielded thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortShieldedStream :: DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortShieldedStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setDockingPortShielded :: DockingPort -> Bool -> RPCContext (Bool)
setDockingPortShielded thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Shielded" [ makeArgument 0 (dockingPortId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getDockingPortReferenceFrame :: DockingPort -> RPCContext (ReferenceFrame)
getDockingPortReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [ makeArgument 0 (dockingPortId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getDockingPortReferenceFrameStream :: DockingPort -> RPCContext (KRPCStream (ReferenceFrame))
getDockingPortReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [ makeArgument 0 (dockingPortId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

engineToggleMode :: Engine -> RPCContext (Bool)
engineToggleMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_ToggleMode" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getEnginePart :: Engine -> RPCContext (Part)
getEnginePart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Part" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEnginePartStream :: Engine -> RPCContext (KRPCStream (Part))
getEnginePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Part" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineActive :: Engine -> RPCContext (Bool)
getEngineActive thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Active" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineActiveStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Active" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setEngineActive :: Engine -> Bool -> RPCContext (Bool)
setEngineActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Active" [ makeArgument 0 (engineId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getEngineThrust :: Engine -> RPCContext (Float)
getEngineThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrust" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineThrustStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrust" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineAvailableThrust :: Engine -> RPCContext (Float)
getEngineAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineAvailableThrustStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineAvailableThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineMaxThrust :: Engine -> RPCContext (Float)
getEngineMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineMaxThrustStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineMaxThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineMaxVacuumThrust :: Engine -> RPCContext (Float)
getEngineMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineMaxVacuumThrustStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineMaxVacuumThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineThrustLimit :: Engine -> RPCContext (Float)
getEngineThrustLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineThrustLimitStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineThrustLimitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setEngineThrustLimit :: Engine -> Float -> RPCContext (Bool)
setEngineThrustLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_ThrustLimit" [ makeArgument 0 (engineId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getEngineThrusters :: Engine -> RPCContext ([Thruster])
getEngineThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrusters" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getEngineThrustersStream :: Engine -> RPCContext (KRPCStream ([Thruster]))
getEngineThrustersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrusters" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getEngineSpecificImpulse :: Engine -> RPCContext (Float)
getEngineSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineSpecificImpulseStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineVacuumSpecificImpulse :: Engine -> RPCContext (Float)
getEngineVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineVacuumSpecificImpulseStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineVacuumSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineKerbinSeaLevelSpecificImpulse :: Engine -> RPCContext (Float)
getEngineKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineKerbinSeaLevelSpecificImpulseStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineKerbinSeaLevelSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEnginePropellants :: Engine -> RPCContext ([Text])
getEnginePropellants thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Propellants" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getEnginePropellantsStream :: Engine -> RPCContext (KRPCStream ([Text]))
getEnginePropellantsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Propellants" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getEnginePropellantRatios :: Engine -> RPCContext (Map Text Float)
getEnginePropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractMap res

getEnginePropellantRatiosStream :: Engine -> RPCContext (KRPCStream (Map Text Float))
getEnginePropellantRatiosStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractMap

getEngineHasFuel :: Engine -> RPCContext (Bool)
getEngineHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasFuel" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineHasFuelStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineHasFuelStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasFuel" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineThrottle :: Engine -> RPCContext (Float)
getEngineThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Throttle" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineThrottleStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineThrottleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Throttle" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineThrottleLocked :: Engine -> RPCContext (Bool)
getEngineThrottleLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineThrottleLockedStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineThrottleLockedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineCanRestart :: Engine -> RPCContext (Bool)
getEngineCanRestart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanRestart" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineCanRestartStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineCanRestartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanRestart" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineCanShutdown :: Engine -> RPCContext (Bool)
getEngineCanShutdown thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineCanShutdownStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineCanShutdownStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineHasModes :: Engine -> RPCContext (Bool)
getEngineHasModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasModes" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineHasModesStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineHasModesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasModes" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineMode :: Engine -> RPCContext (Text)
getEngineMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Mode" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineModeStream :: Engine -> RPCContext (KRPCStream (Text))
getEngineModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Mode" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setEngineMode :: Engine -> Text -> RPCContext (Bool)
setEngineMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Mode" [ makeArgument 0 (engineId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getEngineModes :: Engine -> RPCContext (Map Text Engine)
getEngineModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Modes" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractMap res

getEngineModesStream :: Engine -> RPCContext (KRPCStream (Map Text Engine))
getEngineModesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Modes" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractMap

getEngineAutoModeSwitch :: Engine -> RPCContext (Bool)
getEngineAutoModeSwitch thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineAutoModeSwitchStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineAutoModeSwitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setEngineAutoModeSwitch :: Engine -> Bool -> RPCContext (Bool)
setEngineAutoModeSwitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_AutoModeSwitch" [ makeArgument 0 (engineId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getEngineGimballed :: Engine -> RPCContext (Bool)
getEngineGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Gimballed" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineGimballedStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineGimballedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Gimballed" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineGimbalRange :: Engine -> RPCContext (Float)
getEngineGimbalRange thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineGimbalRangeStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalRangeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getEngineGimbalLocked :: Engine -> RPCContext (Bool)
getEngineGimbalLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineGimbalLockedStream :: Engine -> RPCContext (KRPCStream (Bool))
getEngineGimbalLockedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setEngineGimbalLocked :: Engine -> Bool -> RPCContext (Bool)
setEngineGimbalLocked thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLocked" [ makeArgument 0 (engineId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getEngineGimbalLimit :: Engine -> RPCContext (Float)
getEngineGimbalLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [ makeArgument 0 (engineId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getEngineGimbalLimitStream :: Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalLimitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [ makeArgument 0 (engineId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setEngineGimbalLimit :: Engine -> Float -> RPCContext (Bool)
setEngineGimbalLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLimit" [ makeArgument 0 (engineId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

fairingJettison :: Fairing -> RPCContext (Bool)
fairingJettison thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_Jettison" [ makeArgument 0 (fairingId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getFairingPart :: Fairing -> RPCContext (Part)
getFairingPart thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Part" [ makeArgument 0 (fairingId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFairingPartStream :: Fairing -> RPCContext (KRPCStream (Part))
getFairingPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Part" [ makeArgument 0 (fairingId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getFairingJettisoned :: Fairing -> RPCContext (Bool)
getFairingJettisoned thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [ makeArgument 0 (fairingId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getFairingJettisonedStream :: Fairing -> RPCContext (KRPCStream (Bool))
getFairingJettisonedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [ makeArgument 0 (fairingId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getIntakePart :: Intake -> RPCContext (Part)
getIntakePart thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Part" [ makeArgument 0 (intakeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getIntakePartStream :: Intake -> RPCContext (KRPCStream (Part))
getIntakePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Part" [ makeArgument 0 (intakeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getIntakeOpen :: Intake -> RPCContext (Bool)
getIntakeOpen thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Open" [ makeArgument 0 (intakeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getIntakeOpenStream :: Intake -> RPCContext (KRPCStream (Bool))
getIntakeOpenStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Open" [ makeArgument 0 (intakeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setIntakeOpen :: Intake -> Bool -> RPCContext (Bool)
setIntakeOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Intake_set_Open" [ makeArgument 0 (intakeId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getIntakeSpeed :: Intake -> RPCContext (Float)
getIntakeSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Speed" [ makeArgument 0 (intakeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getIntakeSpeedStream :: Intake -> RPCContext (KRPCStream (Float))
getIntakeSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Speed" [ makeArgument 0 (intakeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getIntakeFlow :: Intake -> RPCContext (Float)
getIntakeFlow thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Flow" [ makeArgument 0 (intakeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getIntakeFlowStream :: Intake -> RPCContext (KRPCStream (Float))
getIntakeFlowStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Flow" [ makeArgument 0 (intakeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getIntakeArea :: Intake -> RPCContext (Float)
getIntakeArea thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Area" [ makeArgument 0 (intakeId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getIntakeAreaStream :: Intake -> RPCContext (KRPCStream (Float))
getIntakeAreaStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Area" [ makeArgument 0 (intakeId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLandingGearPart :: LandingGear -> RPCContext (Part)
getLandingGearPart thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Part" [ makeArgument 0 (landingGearId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingGearPartStream :: LandingGear -> RPCContext (KRPCStream (Part))
getLandingGearPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Part" [ makeArgument 0 (landingGearId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLandingGearDeployable :: LandingGear -> RPCContext (Bool)
getLandingGearDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployable" [ makeArgument 0 (landingGearId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingGearDeployableStream :: LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployableStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployable" [ makeArgument 0 (landingGearId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLandingGearState :: LandingGear -> RPCContext (LandingGearState)
getLandingGearState thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_State" [ makeArgument 0 (landingGearId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingGearStateStream :: LandingGear -> RPCContext (KRPCStream (LandingGearState))
getLandingGearStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_State" [ makeArgument 0 (landingGearId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLandingGearDeployed :: LandingGear -> RPCContext (Bool)
getLandingGearDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployed" [ makeArgument 0 (landingGearId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingGearDeployedStream :: LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployed" [ makeArgument 0 (landingGearId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setLandingGearDeployed :: LandingGear -> Bool -> RPCContext (Bool)
setLandingGearDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_set_Deployed" [ makeArgument 0 (landingGearId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getLandingLegPart :: LandingLeg -> RPCContext (Part)
getLandingLegPart thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Part" [ makeArgument 0 (landingLegId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingLegPartStream :: LandingLeg -> RPCContext (KRPCStream (Part))
getLandingLegPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Part" [ makeArgument 0 (landingLegId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLandingLegState :: LandingLeg -> RPCContext (LandingLegState)
getLandingLegState thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_State" [ makeArgument 0 (landingLegId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingLegStateStream :: LandingLeg -> RPCContext (KRPCStream (LandingLegState))
getLandingLegStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_State" [ makeArgument 0 (landingLegId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLandingLegDeployed :: LandingLeg -> RPCContext (Bool)
getLandingLegDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Deployed" [ makeArgument 0 (landingLegId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLandingLegDeployedStream :: LandingLeg -> RPCContext (KRPCStream (Bool))
getLandingLegDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Deployed" [ makeArgument 0 (landingLegId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setLandingLegDeployed :: LandingLeg -> Bool -> RPCContext (Bool)
setLandingLegDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_set_Deployed" [ makeArgument 0 (landingLegId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

launchClampRelease :: LaunchClamp -> RPCContext (Bool)
launchClampRelease thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_Release" [ makeArgument 0 (launchClampId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getLaunchClampPart :: LaunchClamp -> RPCContext (Part)
getLaunchClampPart thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [ makeArgument 0 (launchClampId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLaunchClampPartStream :: LaunchClamp -> RPCContext (KRPCStream (Part))
getLaunchClampPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [ makeArgument 0 (launchClampId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLightPart :: Light -> RPCContext (Part)
getLightPart thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Part" [ makeArgument 0 (lightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLightPartStream :: Light -> RPCContext (KRPCStream (Part))
getLightPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Part" [ makeArgument 0 (lightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getLightActive :: Light -> RPCContext (Bool)
getLightActive thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Active" [ makeArgument 0 (lightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLightActiveStream :: Light -> RPCContext (KRPCStream (Bool))
getLightActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Active" [ makeArgument 0 (lightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setLightActive :: Light -> Bool -> RPCContext (Bool)
setLightActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Active" [ makeArgument 0 (lightId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getLightPowerUsage :: Light -> RPCContext (Float)
getLightPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_PowerUsage" [ makeArgument 0 (lightId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getLightPowerUsageStream :: Light -> RPCContext (KRPCStream (Float))
getLightPowerUsageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_PowerUsage" [ makeArgument 0 (lightId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

moduleHasField :: Module -> Text -> RPCContext (Bool)
moduleHasField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasField" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

moduleHasFieldStream :: Module -> Text -> RPCContext (KRPCStream (Bool))
moduleHasFieldStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasField" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

moduleGetField :: Module -> Text -> RPCContext (Text)
moduleGetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_GetField" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

moduleGetFieldStream :: Module -> Text -> RPCContext (KRPCStream (Text))
moduleGetFieldStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_GetField" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

moduleHasEvent :: Module -> Text -> RPCContext (Bool)
moduleHasEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasEvent" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

moduleHasEventStream :: Module -> Text -> RPCContext (KRPCStream (Bool))
moduleHasEventStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasEvent" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

moduleTriggerEvent :: Module -> Text -> RPCContext (Bool)
moduleTriggerEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_TriggerEvent" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractNothing res

moduleHasAction :: Module -> Text -> RPCContext (Bool)
moduleHasAction thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasAction" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

moduleHasActionStream :: Module -> Text -> RPCContext (KRPCStream (Bool))
moduleHasActionStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasAction" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

moduleSetAction :: Module -> Text -> Bool -> RPCContext (Bool)
moduleSetAction thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetAction" [ makeArgument 0 (moduleId thisArg), makeArgument 1 nameArg, makeArgument 2 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getModuleName :: Module -> RPCContext (Text)
getModuleName thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Name" [ makeArgument 0 (moduleId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getModuleNameStream :: Module -> RPCContext (KRPCStream (Text))
getModuleNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Name" [ makeArgument 0 (moduleId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getModulePart :: Module -> RPCContext (Part)
getModulePart thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Part" [ makeArgument 0 (moduleId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getModulePartStream :: Module -> RPCContext (KRPCStream (Part))
getModulePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Part" [ makeArgument 0 (moduleId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getModuleFields :: Module -> RPCContext (Map Text Text)
getModuleFields thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Fields" [ makeArgument 0 (moduleId thisArg) ]
    res <- sendRequest r
    processResponse extractMap res

getModuleFieldsStream :: Module -> RPCContext (KRPCStream (Map Text Text))
getModuleFieldsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Fields" [ makeArgument 0 (moduleId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractMap

getModuleEvents :: Module -> RPCContext ([Text])
getModuleEvents thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Events" [ makeArgument 0 (moduleId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getModuleEventsStream :: Module -> RPCContext (KRPCStream ([Text]))
getModuleEventsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Events" [ makeArgument 0 (moduleId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getModuleActions :: Module -> RPCContext ([Text])
getModuleActions thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Actions" [ makeArgument 0 (moduleId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getModuleActionsStream :: Module -> RPCContext (KRPCStream ([Text]))
getModuleActionsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Actions" [ makeArgument 0 (moduleId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

parachuteDeploy :: Parachute -> RPCContext (Bool)
parachuteDeploy thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_Deploy" [ makeArgument 0 (parachuteId thisArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getParachutePart :: Parachute -> RPCContext (Part)
getParachutePart thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Part" [ makeArgument 0 (parachuteId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getParachutePartStream :: Parachute -> RPCContext (KRPCStream (Part))
getParachutePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Part" [ makeArgument 0 (parachuteId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getParachuteDeployed :: Parachute -> RPCContext (Bool)
getParachuteDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Deployed" [ makeArgument 0 (parachuteId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getParachuteDeployedStream :: Parachute -> RPCContext (KRPCStream (Bool))
getParachuteDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Deployed" [ makeArgument 0 (parachuteId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getParachuteState :: Parachute -> RPCContext (ParachuteState)
getParachuteState thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_State" [ makeArgument 0 (parachuteId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getParachuteStateStream :: Parachute -> RPCContext (KRPCStream (ParachuteState))
getParachuteStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_State" [ makeArgument 0 (parachuteId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getParachuteDeployAltitude :: Parachute -> RPCContext (Float)
getParachuteDeployAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [ makeArgument 0 (parachuteId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getParachuteDeployAltitudeStream :: Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [ makeArgument 0 (parachuteId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setParachuteDeployAltitude :: Parachute -> Float -> RPCContext (Bool)
setParachuteDeployAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployAltitude" [ makeArgument 0 (parachuteId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getParachuteDeployMinPressure :: Parachute -> RPCContext (Float)
getParachuteDeployMinPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [ makeArgument 0 (parachuteId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getParachuteDeployMinPressureStream :: Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployMinPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [ makeArgument 0 (parachuteId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setParachuteDeployMinPressure :: Parachute -> Float -> RPCContext (Bool)
setParachuteDeployMinPressure thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployMinPressure" [ makeArgument 0 (parachuteId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

partPosition :: Part -> ReferenceFrame -> RPCContext ((Double, Double, Double))
partPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Position" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

partPositionStream :: Part -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Position" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

partCenterOfMass :: Part -> ReferenceFrame -> RPCContext ((Double, Double, Double))
partCenterOfMass thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_CenterOfMass" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

partCenterOfMassStream :: Part -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partCenterOfMassStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_CenterOfMass" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

partDirection :: Part -> ReferenceFrame -> RPCContext ((Double, Double, Double))
partDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Direction" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

partDirectionStream :: Part -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Direction" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

partVelocity :: Part -> ReferenceFrame -> RPCContext ((Double, Double, Double))
partVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Velocity" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

partVelocityStream :: Part -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Velocity" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

partRotation :: Part -> ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
partRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Rotation" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple4 res

partRotationStream :: Part -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
partRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Rotation" [ makeArgument 0 (partId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple4

getPartName :: Part -> RPCContext (Text)
getPartName thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Name" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartNameStream :: Part -> RPCContext (KRPCStream (Text))
getPartNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Name" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartTitle :: Part -> RPCContext (Text)
getPartTitle thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Title" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartTitleStream :: Part -> RPCContext (KRPCStream (Text))
getPartTitleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Title" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartCost :: Part -> RPCContext (Double)
getPartCost thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Cost" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartCostStream :: Part -> RPCContext (KRPCStream (Double))
getPartCostStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Cost" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartVessel :: Part -> RPCContext (Vessel)
getPartVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Vessel" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartVesselStream :: Part -> RPCContext (KRPCStream (Vessel))
getPartVesselStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Vessel" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartParent :: Part -> RPCContext (Part)
getPartParent thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parent" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartParentStream :: Part -> RPCContext (KRPCStream (Part))
getPartParentStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parent" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartChildren :: Part -> RPCContext ([Part])
getPartChildren thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Children" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartChildrenStream :: Part -> RPCContext (KRPCStream ([Part]))
getPartChildrenStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Children" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartAxiallyAttached :: Part -> RPCContext (Bool)
getPartAxiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartAxiallyAttachedStream :: Part -> RPCContext (KRPCStream (Bool))
getPartAxiallyAttachedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartRadiallyAttached :: Part -> RPCContext (Bool)
getPartRadiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartRadiallyAttachedStream :: Part -> RPCContext (KRPCStream (Bool))
getPartRadiallyAttachedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartStage :: Part -> RPCContext (Int32)
getPartStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Stage" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartStageStream :: Part -> RPCContext (KRPCStream (Int32))
getPartStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Stage" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartDecoupleStage :: Part -> RPCContext (Int32)
getPartDecoupleStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartDecoupleStageStream :: Part -> RPCContext (KRPCStream (Int32))
getPartDecoupleStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartMassless :: Part -> RPCContext (Bool)
getPartMassless thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Massless" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartMasslessStream :: Part -> RPCContext (KRPCStream (Bool))
getPartMasslessStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Massless" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartMass :: Part -> RPCContext (Double)
getPartMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Mass" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartMassStream :: Part -> RPCContext (KRPCStream (Double))
getPartMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Mass" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartDryMass :: Part -> RPCContext (Double)
getPartDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DryMass" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartDryMassStream :: Part -> RPCContext (KRPCStream (Double))
getPartDryMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DryMass" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartShielded :: Part -> RPCContext (Bool)
getPartShielded thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Shielded" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartShieldedStream :: Part -> RPCContext (KRPCStream (Bool))
getPartShieldedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Shielded" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartDynamicPressure :: Part -> RPCContext (Float)
getPartDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartDynamicPressureStream :: Part -> RPCContext (KRPCStream (Float))
getPartDynamicPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartImpactTolerance :: Part -> RPCContext (Double)
getPartImpactTolerance thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartImpactToleranceStream :: Part -> RPCContext (KRPCStream (Double))
getPartImpactToleranceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartTemperature :: Part -> RPCContext (Double)
getPartTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Temperature" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartTemperatureStream :: Part -> RPCContext (KRPCStream (Double))
getPartTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Temperature" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartSkinTemperature :: Part -> RPCContext (Double)
getPartSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartSkinTemperatureStream :: Part -> RPCContext (KRPCStream (Double))
getPartSkinTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartMaxTemperature :: Part -> RPCContext (Double)
getPartMaxTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartMaxTemperatureStream :: Part -> RPCContext (KRPCStream (Double))
getPartMaxTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartMaxSkinTemperature :: Part -> RPCContext (Double)
getPartMaxSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartMaxSkinTemperatureStream :: Part -> RPCContext (KRPCStream (Double))
getPartMaxSkinTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalMass :: Part -> RPCContext (Float)
getPartThermalMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalMass" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalMassStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalMass" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalSkinMass :: Part -> RPCContext (Float)
getPartThermalSkinMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalSkinMassStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalResourceMass :: Part -> RPCContext (Float)
getPartThermalResourceMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalResourceMassStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalResourceMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalInternalFlux :: Part -> RPCContext (Float)
getPartThermalInternalFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalInternalFluxStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalInternalFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalConductionFlux :: Part -> RPCContext (Float)
getPartThermalConductionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalConductionFluxStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalConductionFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalConvectionFlux :: Part -> RPCContext (Float)
getPartThermalConvectionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalConvectionFluxStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalConvectionFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalRadiationFlux :: Part -> RPCContext (Float)
getPartThermalRadiationFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalRadiationFluxStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalRadiationFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartThermalSkinToInternalFlux :: Part -> RPCContext (Float)
getPartThermalSkinToInternalFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartThermalSkinToInternalFluxStream :: Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinToInternalFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartResources :: Part -> RPCContext (Resources)
getPartResources thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Resources" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartResourcesStream :: Part -> RPCContext (KRPCStream (Resources))
getPartResourcesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Resources" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartCrossfeed :: Part -> RPCContext (Bool)
getPartCrossfeed thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Crossfeed" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartCrossfeedStream :: Part -> RPCContext (KRPCStream (Bool))
getPartCrossfeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Crossfeed" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartIsFuelLine :: Part -> RPCContext (Bool)
getPartIsFuelLine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartIsFuelLineStream :: Part -> RPCContext (KRPCStream (Bool))
getPartIsFuelLineStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartFuelLinesFrom :: Part -> RPCContext ([Part])
getPartFuelLinesFrom thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartFuelLinesFromStream :: Part -> RPCContext (KRPCStream ([Part]))
getPartFuelLinesFromStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartFuelLinesTo :: Part -> RPCContext ([Part])
getPartFuelLinesTo thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartFuelLinesToStream :: Part -> RPCContext (KRPCStream ([Part]))
getPartFuelLinesToStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartModules :: Part -> RPCContext ([Module])
getPartModules thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Modules" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartModulesStream :: Part -> RPCContext (KRPCStream ([Module]))
getPartModulesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Modules" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartCargoBay :: Part -> RPCContext (CargoBay)
getPartCargoBay thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CargoBay" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartCargoBayStream :: Part -> RPCContext (KRPCStream (CargoBay))
getPartCargoBayStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CargoBay" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartControlSurface :: Part -> RPCContext (ControlSurface)
getPartControlSurface thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ControlSurface" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartControlSurfaceStream :: Part -> RPCContext (KRPCStream (ControlSurface))
getPartControlSurfaceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ControlSurface" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartDecoupler :: Part -> RPCContext (Decoupler)
getPartDecoupler thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Decoupler" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartDecouplerStream :: Part -> RPCContext (KRPCStream (Decoupler))
getPartDecouplerStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Decoupler" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartDockingPort :: Part -> RPCContext (DockingPort)
getPartDockingPort thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DockingPort" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartDockingPortStream :: Part -> RPCContext (KRPCStream (DockingPort))
getPartDockingPortStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DockingPort" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartEngine :: Part -> RPCContext (Engine)
getPartEngine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Engine" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartEngineStream :: Part -> RPCContext (KRPCStream (Engine))
getPartEngineStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Engine" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartFairing :: Part -> RPCContext (Fairing)
getPartFairing thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Fairing" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartFairingStream :: Part -> RPCContext (KRPCStream (Fairing))
getPartFairingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Fairing" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartIntake :: Part -> RPCContext (Intake)
getPartIntake thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Intake" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartIntakeStream :: Part -> RPCContext (KRPCStream (Intake))
getPartIntakeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Intake" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartLandingGear :: Part -> RPCContext (LandingGear)
getPartLandingGear thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingGear" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartLandingGearStream :: Part -> RPCContext (KRPCStream (LandingGear))
getPartLandingGearStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingGear" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartLandingLeg :: Part -> RPCContext (LandingLeg)
getPartLandingLeg thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingLeg" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartLandingLegStream :: Part -> RPCContext (KRPCStream (LandingLeg))
getPartLandingLegStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingLeg" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartLaunchClamp :: Part -> RPCContext (LaunchClamp)
getPartLaunchClamp thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartLaunchClampStream :: Part -> RPCContext (KRPCStream (LaunchClamp))
getPartLaunchClampStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartLight :: Part -> RPCContext (Light)
getPartLight thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Light" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartLightStream :: Part -> RPCContext (KRPCStream (Light))
getPartLightStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Light" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartParachute :: Part -> RPCContext (Parachute)
getPartParachute thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parachute" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartParachuteStream :: Part -> RPCContext (KRPCStream (Parachute))
getPartParachuteStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parachute" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartRadiator :: Part -> RPCContext (Radiator)
getPartRadiator thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Radiator" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartRadiatorStream :: Part -> RPCContext (KRPCStream (Radiator))
getPartRadiatorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Radiator" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartRCS :: Part -> RPCContext (RCS)
getPartRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RCS" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartRCSStream :: Part -> RPCContext (KRPCStream (RCS))
getPartRCSStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RCS" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartReactionWheel :: Part -> RPCContext (ReactionWheel)
getPartReactionWheel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartReactionWheelStream :: Part -> RPCContext (KRPCStream (ReactionWheel))
getPartReactionWheelStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartResourceConverter :: Part -> RPCContext (ResourceConverter)
getPartResourceConverter thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartResourceConverterStream :: Part -> RPCContext (KRPCStream (ResourceConverter))
getPartResourceConverterStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartResourceHarvester :: Part -> RPCContext (ResourceHarvester)
getPartResourceHarvester thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartResourceHarvesterStream :: Part -> RPCContext (KRPCStream (ResourceHarvester))
getPartResourceHarvesterStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartSensor :: Part -> RPCContext (Sensor)
getPartSensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Sensor" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartSensorStream :: Part -> RPCContext (KRPCStream (Sensor))
getPartSensorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Sensor" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartSolarPanel :: Part -> RPCContext (SolarPanel)
getPartSolarPanel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SolarPanel" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartSolarPanelStream :: Part -> RPCContext (KRPCStream (SolarPanel))
getPartSolarPanelStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SolarPanel" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartMomentOfInertia :: Part -> RPCContext ((Double, Double, Double))
getPartMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getPartMomentOfInertiaStream :: Part -> RPCContext (KRPCStream ((Double, Double, Double)))
getPartMomentOfInertiaStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getPartInertiaTensor :: Part -> RPCContext ([Double])
getPartInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartInertiaTensorStream :: Part -> RPCContext (KRPCStream ([Double]))
getPartInertiaTensorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartReferenceFrame :: Part -> RPCContext (ReferenceFrame)
getPartReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartReferenceFrameStream :: Part -> RPCContext (KRPCStream (ReferenceFrame))
getPartReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartCenterOfMassReferenceFrame :: Part -> RPCContext (ReferenceFrame)
getPartCenterOfMassReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [ makeArgument 0 (partId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartCenterOfMassReferenceFrameStream :: Part -> RPCContext (KRPCStream (ReferenceFrame))
getPartCenterOfMassReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [ makeArgument 0 (partId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

partsWithName :: Parts -> Text -> RPCContext ([Part])
partsWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithName" [ makeArgument 0 (partsId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractList res

partsWithNameStream :: Parts -> Text -> RPCContext (KRPCStream ([Part]))
partsWithNameStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithName" [ makeArgument 0 (partsId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

partsWithTitle :: Parts -> Text -> RPCContext ([Part])
partsWithTitle thisArg titleArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTitle" [ makeArgument 0 (partsId thisArg), makeArgument 1 titleArg ]
    res <- sendRequest r
    processResponse extractList res

partsWithTitleStream :: Parts -> Text -> RPCContext (KRPCStream ([Part]))
partsWithTitleStream thisArg titleArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTitle" [ makeArgument 0 (partsId thisArg), makeArgument 1 titleArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

partsWithModule :: Parts -> Text -> RPCContext ([Part])
partsWithModule thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithModule" [ makeArgument 0 (partsId thisArg), makeArgument 1 moduleNameArg ]
    res <- sendRequest r
    processResponse extractList res

partsWithModuleStream :: Parts -> Text -> RPCContext (KRPCStream ([Part]))
partsWithModuleStream thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithModule" [ makeArgument 0 (partsId thisArg), makeArgument 1 moduleNameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

partsInStage :: Parts -> Int32 -> RPCContext ([Part])
partsInStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InStage" [ makeArgument 0 (partsId thisArg), makeArgument 1 stageArg ]
    res <- sendRequest r
    processResponse extractList res

partsInStageStream :: Parts -> Int32 -> RPCContext (KRPCStream ([Part]))
partsInStageStream thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InStage" [ makeArgument 0 (partsId thisArg), makeArgument 1 stageArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

partsInDecoupleStage :: Parts -> Int32 -> RPCContext ([Part])
partsInDecoupleStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [ makeArgument 0 (partsId thisArg), makeArgument 1 stageArg ]
    res <- sendRequest r
    processResponse extractList res

partsInDecoupleStageStream :: Parts -> Int32 -> RPCContext (KRPCStream ([Part]))
partsInDecoupleStageStream thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [ makeArgument 0 (partsId thisArg), makeArgument 1 stageArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

partsModulesWithName :: Parts -> Text -> RPCContext ([Module])
partsModulesWithName thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_ModulesWithName" [ makeArgument 0 (partsId thisArg), makeArgument 1 moduleNameArg ]
    res <- sendRequest r
    processResponse extractList res

partsModulesWithNameStream :: Parts -> Text -> RPCContext (KRPCStream ([Module]))
partsModulesWithNameStream thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_ModulesWithName" [ makeArgument 0 (partsId thisArg), makeArgument 1 moduleNameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

partsDockingPortWithName :: Parts -> Text -> RPCContext (DockingPort)
partsDockingPortWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_DockingPortWithName" [ makeArgument 0 (partsId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

partsDockingPortWithNameStream :: Parts -> Text -> RPCContext (KRPCStream (DockingPort))
partsDockingPortWithNameStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_DockingPortWithName" [ makeArgument 0 (partsId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartsAll :: Parts -> RPCContext ([Part])
getPartsAll thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_All" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsAllStream :: Parts -> RPCContext (KRPCStream ([Part]))
getPartsAllStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_All" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsRoot :: Parts -> RPCContext (Part)
getPartsRoot thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Root" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartsRootStream :: Parts -> RPCContext (KRPCStream (Part))
getPartsRootStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Root" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getPartsControlling :: Parts -> RPCContext (Part)
getPartsControlling thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Controlling" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getPartsControllingStream :: Parts -> RPCContext (KRPCStream (Part))
getPartsControllingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Controlling" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setPartsControlling :: Parts -> Part -> RPCContext (Bool)
setPartsControlling thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parts_set_Controlling" [ makeArgument 0 (partsId thisArg), makeArgument 1 (partId valueArg) ]
    res <- sendRequest r
    processResponse extractNothing res

getPartsControlSurfaces :: Parts -> RPCContext ([ControlSurface])
getPartsControlSurfaces thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsControlSurfacesStream :: Parts -> RPCContext (KRPCStream ([ControlSurface]))
getPartsControlSurfacesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsCargoBays :: Parts -> RPCContext ([CargoBay])
getPartsCargoBays thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_CargoBays" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsCargoBaysStream :: Parts -> RPCContext (KRPCStream ([CargoBay]))
getPartsCargoBaysStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_CargoBays" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsDecouplers :: Parts -> RPCContext ([Decoupler])
getPartsDecouplers thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Decouplers" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsDecouplersStream :: Parts -> RPCContext (KRPCStream ([Decoupler]))
getPartsDecouplersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Decouplers" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsDockingPorts :: Parts -> RPCContext ([DockingPort])
getPartsDockingPorts thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsDockingPortsStream :: Parts -> RPCContext (KRPCStream ([DockingPort]))
getPartsDockingPortsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsEngines :: Parts -> RPCContext ([Engine])
getPartsEngines thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Engines" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsEnginesStream :: Parts -> RPCContext (KRPCStream ([Engine]))
getPartsEnginesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Engines" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsFairings :: Parts -> RPCContext ([Fairing])
getPartsFairings thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Fairings" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsFairingsStream :: Parts -> RPCContext (KRPCStream ([Fairing]))
getPartsFairingsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Fairings" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsIntakes :: Parts -> RPCContext ([Intake])
getPartsIntakes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Intakes" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsIntakesStream :: Parts -> RPCContext (KRPCStream ([Intake]))
getPartsIntakesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Intakes" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsLandingGear :: Parts -> RPCContext ([LandingGear])
getPartsLandingGear thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingGear" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsLandingGearStream :: Parts -> RPCContext (KRPCStream ([LandingGear]))
getPartsLandingGearStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingGear" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsLandingLegs :: Parts -> RPCContext ([LandingLeg])
getPartsLandingLegs thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingLegs" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsLandingLegsStream :: Parts -> RPCContext (KRPCStream ([LandingLeg]))
getPartsLandingLegsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingLegs" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsLaunchClamps :: Parts -> RPCContext ([LaunchClamp])
getPartsLaunchClamps thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsLaunchClampsStream :: Parts -> RPCContext (KRPCStream ([LaunchClamp]))
getPartsLaunchClampsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsLights :: Parts -> RPCContext ([Light])
getPartsLights thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Lights" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsLightsStream :: Parts -> RPCContext (KRPCStream ([Light]))
getPartsLightsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Lights" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsParachutes :: Parts -> RPCContext ([Parachute])
getPartsParachutes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Parachutes" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsParachutesStream :: Parts -> RPCContext (KRPCStream ([Parachute]))
getPartsParachutesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Parachutes" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsRadiators :: Parts -> RPCContext ([Radiator])
getPartsRadiators thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Radiators" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsRadiatorsStream :: Parts -> RPCContext (KRPCStream ([Radiator]))
getPartsRadiatorsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Radiators" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsRCS :: Parts -> RPCContext ([RCS])
getPartsRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_RCS" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsRCSStream :: Parts -> RPCContext (KRPCStream ([RCS]))
getPartsRCSStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_RCS" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsReactionWheels :: Parts -> RPCContext ([ReactionWheel])
getPartsReactionWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsReactionWheelsStream :: Parts -> RPCContext (KRPCStream ([ReactionWheel]))
getPartsReactionWheelsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsResourceConverters :: Parts -> RPCContext ([ResourceConverter])
getPartsResourceConverters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsResourceConvertersStream :: Parts -> RPCContext (KRPCStream ([ResourceConverter]))
getPartsResourceConvertersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsResourceHarvesters :: Parts -> RPCContext ([ResourceHarvester])
getPartsResourceHarvesters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsResourceHarvestersStream :: Parts -> RPCContext (KRPCStream ([ResourceHarvester]))
getPartsResourceHarvestersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsSensors :: Parts -> RPCContext ([Sensor])
getPartsSensors thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Sensors" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsSensorsStream :: Parts -> RPCContext (KRPCStream ([Sensor]))
getPartsSensorsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Sensors" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getPartsSolarPanels :: Parts -> RPCContext ([SolarPanel])
getPartsSolarPanels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [ makeArgument 0 (partsId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getPartsSolarPanelsStream :: Parts -> RPCContext (KRPCStream ([SolarPanel]))
getPartsSolarPanelsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [ makeArgument 0 (partsId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getRCSPart :: RCS -> RPCContext (Part)
getRCSPart thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Part" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSPartStream :: RCS -> RPCContext (KRPCStream (Part))
getRCSPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Part" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSActive :: RCS -> RPCContext (Bool)
getRCSActive thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Active" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSActiveStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Active" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSEnabled :: RCS -> RPCContext (Bool)
getRCSEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Enabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Enabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_Enabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSPitchEnabled :: RCS -> RPCContext (Bool)
getRCSPitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSPitchEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSPitchEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSPitchEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSPitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_PitchEnabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSYawEnabled :: RCS -> RPCContext (Bool)
getRCSYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSYawEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSYawEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSYawEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_YawEnabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSRollEnabled :: RCS -> RPCContext (Bool)
getRCSRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSRollEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSRollEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSRollEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RollEnabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSForwardEnabled :: RCS -> RPCContext (Bool)
getRCSForwardEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSForwardEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSForwardEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSForwardEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSForwardEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_ForwardEnabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSUpEnabled :: RCS -> RPCContext (Bool)
getRCSUpEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSUpEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSUpEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSUpEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSUpEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_UpEnabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSRightEnabled :: RCS -> RPCContext (Bool)
getRCSRightEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSRightEnabledStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSRightEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRCSRightEnabled :: RCS -> Bool -> RPCContext (Bool)
setRCSRightEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RightEnabled" [ makeArgument 0 (rCSId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRCSMaxThrust :: RCS -> RPCContext (Float)
getRCSMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSMaxThrustStream :: RCS -> RPCContext (KRPCStream (Float))
getRCSMaxThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSMaxVacuumThrust :: RCS -> RPCContext (Float)
getRCSMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSMaxVacuumThrustStream :: RCS -> RPCContext (KRPCStream (Float))
getRCSMaxVacuumThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSThrusters :: RCS -> RPCContext ([Thruster])
getRCSThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Thrusters" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getRCSThrustersStream :: RCS -> RPCContext (KRPCStream ([Thruster]))
getRCSThrustersStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Thrusters" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getRCSSpecificImpulse :: RCS -> RPCContext (Float)
getRCSSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSSpecificImpulseStream :: RCS -> RPCContext (KRPCStream (Float))
getRCSSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSVacuumSpecificImpulse :: RCS -> RPCContext (Float)
getRCSVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSVacuumSpecificImpulseStream :: RCS -> RPCContext (KRPCStream (Float))
getRCSVacuumSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSKerbinSeaLevelSpecificImpulse :: RCS -> RPCContext (Float)
getRCSKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSKerbinSeaLevelSpecificImpulseStream :: RCS -> RPCContext (KRPCStream (Float))
getRCSKerbinSeaLevelSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRCSPropellants :: RCS -> RPCContext ([Text])
getRCSPropellants thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Propellants" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getRCSPropellantsStream :: RCS -> RPCContext (KRPCStream ([Text]))
getRCSPropellantsStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Propellants" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getRCSPropellantRatios :: RCS -> RPCContext (Map Text Float)
getRCSPropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractMap res

getRCSPropellantRatiosStream :: RCS -> RPCContext (KRPCStream (Map Text Float))
getRCSPropellantRatiosStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractMap

getRCSHasFuel :: RCS -> RPCContext (Bool)
getRCSHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_HasFuel" [ makeArgument 0 (rCSId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRCSHasFuelStream :: RCS -> RPCContext (KRPCStream (Bool))
getRCSHasFuelStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_HasFuel" [ makeArgument 0 (rCSId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRadiatorPart :: Radiator -> RPCContext (Part)
getRadiatorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Part" [ makeArgument 0 (radiatorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRadiatorPartStream :: Radiator -> RPCContext (KRPCStream (Part))
getRadiatorPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Part" [ makeArgument 0 (radiatorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRadiatorDeployable :: Radiator -> RPCContext (Bool)
getRadiatorDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployable" [ makeArgument 0 (radiatorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRadiatorDeployableStream :: Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployableStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployable" [ makeArgument 0 (radiatorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getRadiatorDeployed :: Radiator -> RPCContext (Bool)
getRadiatorDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployed" [ makeArgument 0 (radiatorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRadiatorDeployedStream :: Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployed" [ makeArgument 0 (radiatorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setRadiatorDeployed :: Radiator -> Bool -> RPCContext (Bool)
setRadiatorDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Radiator_set_Deployed" [ makeArgument 0 (radiatorId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getRadiatorState :: Radiator -> RPCContext (RadiatorState)
getRadiatorState thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_State" [ makeArgument 0 (radiatorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getRadiatorStateStream :: Radiator -> RPCContext (KRPCStream (RadiatorState))
getRadiatorStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_State" [ makeArgument 0 (radiatorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getReactionWheelPart :: ReactionWheel -> RPCContext (Part)
getReactionWheelPart thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [ makeArgument 0 (reactionWheelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getReactionWheelPartStream :: ReactionWheel -> RPCContext (KRPCStream (Part))
getReactionWheelPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [ makeArgument 0 (reactionWheelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getReactionWheelActive :: ReactionWheel -> RPCContext (Bool)
getReactionWheelActive thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [ makeArgument 0 (reactionWheelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getReactionWheelActiveStream :: ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [ makeArgument 0 (reactionWheelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setReactionWheelActive :: ReactionWheel -> Bool -> RPCContext (Bool)
setReactionWheelActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_set_Active" [ makeArgument 0 (reactionWheelId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getReactionWheelBroken :: ReactionWheel -> RPCContext (Bool)
getReactionWheelBroken thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [ makeArgument 0 (reactionWheelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getReactionWheelBrokenStream :: ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelBrokenStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [ makeArgument 0 (reactionWheelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getReactionWheelTorque :: ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Torque" [ makeArgument 0 (reactionWheelId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getReactionWheelTorqueStream :: ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Torque" [ makeArgument 0 (reactionWheelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

resourceConverterActive :: ResourceConverter -> Int32 -> RPCContext (Bool)
resourceConverterActive thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Active" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractValue res

resourceConverterActiveStream :: ResourceConverter -> Int32 -> RPCContext (KRPCStream (Bool))
resourceConverterActiveStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Active" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourceConverterName :: ResourceConverter -> Int32 -> RPCContext (Text)
resourceConverterName thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Name" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractValue res

resourceConverterNameStream :: ResourceConverter -> Int32 -> RPCContext (KRPCStream (Text))
resourceConverterNameStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Name" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourceConverterStart :: ResourceConverter -> Int32 -> RPCContext (Bool)
resourceConverterStart thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Start" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractNothing res

resourceConverterStop :: ResourceConverter -> Int32 -> RPCContext (Bool)
resourceConverterStop thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Stop" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractNothing res

resourceConverterState :: ResourceConverter -> Int32 -> RPCContext (ResourceConverterState)
resourceConverterState thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_State" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractValue res

resourceConverterStateStream :: ResourceConverter -> Int32 -> RPCContext (KRPCStream (ResourceConverterState))
resourceConverterStateStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_State" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourceConverterStatusInfo :: ResourceConverter -> Int32 -> RPCContext (Text)
resourceConverterStatusInfo thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractValue res

resourceConverterStatusInfoStream :: ResourceConverter -> Int32 -> RPCContext (KRPCStream (Text))
resourceConverterStatusInfoStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourceConverterInputs :: ResourceConverter -> Int32 -> RPCContext ([Text])
resourceConverterInputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractList res

resourceConverterInputsStream :: ResourceConverter -> Int32 -> RPCContext (KRPCStream ([Text]))
resourceConverterInputsStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

resourceConverterOutputs :: ResourceConverter -> Int32 -> RPCContext ([Text])
resourceConverterOutputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
    res <- sendRequest r
    processResponse extractList res

resourceConverterOutputsStream :: ResourceConverter -> Int32 -> RPCContext (KRPCStream ([Text]))
resourceConverterOutputsStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [ makeArgument 0 (resourceConverterId thisArg), makeArgument 1 indexArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getResourceConverterPart :: ResourceConverter -> RPCContext (Part)
getResourceConverterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [ makeArgument 0 (resourceConverterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceConverterPartStream :: ResourceConverter -> RPCContext (KRPCStream (Part))
getResourceConverterPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [ makeArgument 0 (resourceConverterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceConverterCount :: ResourceConverter -> RPCContext (Int32)
getResourceConverterCount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [ makeArgument 0 (resourceConverterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceConverterCountStream :: ResourceConverter -> RPCContext (KRPCStream (Int32))
getResourceConverterCountStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [ makeArgument 0 (resourceConverterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceHarvesterPart :: ResourceHarvester -> RPCContext (Part)
getResourceHarvesterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterPartStream :: ResourceHarvester -> RPCContext (KRPCStream (Part))
getResourceHarvesterPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceHarvesterState :: ResourceHarvester -> RPCContext (ResourceHarvesterState)
getResourceHarvesterState thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterStateStream :: ResourceHarvester -> RPCContext (KRPCStream (ResourceHarvesterState))
getResourceHarvesterStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceHarvesterDeployed :: ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterDeployedStream :: ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setResourceHarvesterDeployed :: ResourceHarvester -> Bool -> RPCContext (Bool)
setResourceHarvesterDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Deployed" [ makeArgument 0 (resourceHarvesterId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getResourceHarvesterActive :: ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterActive thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterActiveStream :: ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setResourceHarvesterActive :: ResourceHarvester -> Bool -> RPCContext (Bool)
setResourceHarvesterActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Active" [ makeArgument 0 (resourceHarvesterId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getResourceHarvesterExtractionRate :: ResourceHarvester -> RPCContext (Float)
getResourceHarvesterExtractionRate thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterExtractionRateStream :: ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterExtractionRateStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceHarvesterThermalEfficiency :: ResourceHarvester -> RPCContext (Float)
getResourceHarvesterThermalEfficiency thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterThermalEfficiencyStream :: ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterThermalEfficiencyStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceHarvesterCoreTemperature :: ResourceHarvester -> RPCContext (Float)
getResourceHarvesterCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterCoreTemperatureStream :: ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterCoreTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceHarvesterOptimumCoreTemperature :: ResourceHarvester -> RPCContext (Float)
getResourceHarvesterOptimumCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [ makeArgument 0 (resourceHarvesterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceHarvesterOptimumCoreTemperatureStream :: ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterOptimumCoreTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [ makeArgument 0 (resourceHarvesterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSensorPart :: Sensor -> RPCContext (Part)
getSensorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Part" [ makeArgument 0 (sensorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSensorPartStream :: Sensor -> RPCContext (KRPCStream (Part))
getSensorPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Part" [ makeArgument 0 (sensorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSensorActive :: Sensor -> RPCContext (Bool)
getSensorActive thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Active" [ makeArgument 0 (sensorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSensorActiveStream :: Sensor -> RPCContext (KRPCStream (Bool))
getSensorActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Active" [ makeArgument 0 (sensorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setSensorActive :: Sensor -> Bool -> RPCContext (Bool)
setSensorActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Sensor_set_Active" [ makeArgument 0 (sensorId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getSensorValue :: Sensor -> RPCContext (Text)
getSensorValue thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Value" [ makeArgument 0 (sensorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSensorValueStream :: Sensor -> RPCContext (KRPCStream (Text))
getSensorValueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Value" [ makeArgument 0 (sensorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSensorPowerUsage :: Sensor -> RPCContext (Float)
getSensorPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_PowerUsage" [ makeArgument 0 (sensorId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSensorPowerUsageStream :: Sensor -> RPCContext (KRPCStream (Float))
getSensorPowerUsageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_PowerUsage" [ makeArgument 0 (sensorId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSolarPanelPart :: SolarPanel -> RPCContext (Part)
getSolarPanelPart thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Part" [ makeArgument 0 (solarPanelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSolarPanelPartStream :: SolarPanel -> RPCContext (KRPCStream (Part))
getSolarPanelPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Part" [ makeArgument 0 (solarPanelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSolarPanelDeployed :: SolarPanel -> RPCContext (Bool)
getSolarPanelDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [ makeArgument 0 (solarPanelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSolarPanelDeployedStream :: SolarPanel -> RPCContext (KRPCStream (Bool))
getSolarPanelDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [ makeArgument 0 (solarPanelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setSolarPanelDeployed :: SolarPanel -> Bool -> RPCContext (Bool)
setSolarPanelDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_set_Deployed" [ makeArgument 0 (solarPanelId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getSolarPanelState :: SolarPanel -> RPCContext (SolarPanelState)
getSolarPanelState thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_State" [ makeArgument 0 (solarPanelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSolarPanelStateStream :: SolarPanel -> RPCContext (KRPCStream (SolarPanelState))
getSolarPanelStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_State" [ makeArgument 0 (solarPanelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSolarPanelEnergyFlow :: SolarPanel -> RPCContext (Float)
getSolarPanelEnergyFlow thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [ makeArgument 0 (solarPanelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSolarPanelEnergyFlowStream :: SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelEnergyFlowStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [ makeArgument 0 (solarPanelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getSolarPanelSunExposure :: SolarPanel -> RPCContext (Float)
getSolarPanelSunExposure thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [ makeArgument 0 (solarPanelId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getSolarPanelSunExposureStream :: SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelSunExposureStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [ makeArgument 0 (solarPanelId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

thrusterThrustPosition :: Thruster -> ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

thrusterThrustPositionStream :: Thruster -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

thrusterThrustDirection :: Thruster -> ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

thrusterThrustDirectionStream :: Thruster -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

thrusterInitialThrustPosition :: Thruster -> ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

thrusterInitialThrustPositionStream :: Thruster -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

thrusterInitialThrustDirection :: Thruster -> ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

thrusterInitialThrustDirectionStream :: Thruster -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

thrusterGimbalPosition :: Thruster -> ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterGimbalPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

thrusterGimbalPositionStream :: Thruster -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterGimbalPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [ makeArgument 0 (thrusterId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getThrusterPart :: Thruster -> RPCContext (Part)
getThrusterPart thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Part" [ makeArgument 0 (thrusterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getThrusterPartStream :: Thruster -> RPCContext (KRPCStream (Part))
getThrusterPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Part" [ makeArgument 0 (thrusterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getThrusterThrustReferenceFrame :: Thruster -> RPCContext (ReferenceFrame)
getThrusterThrustReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [ makeArgument 0 (thrusterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getThrusterThrustReferenceFrameStream :: Thruster -> RPCContext (KRPCStream (ReferenceFrame))
getThrusterThrustReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [ makeArgument 0 (thrusterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getThrusterGimballed :: Thruster -> RPCContext (Bool)
getThrusterGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [ makeArgument 0 (thrusterId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getThrusterGimballedStream :: Thruster -> RPCContext (KRPCStream (Bool))
getThrusterGimballedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [ makeArgument 0 (thrusterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getThrusterGimbalAngle :: Thruster -> RPCContext ((Double, Double, Double))
getThrusterGimbalAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [ makeArgument 0 (thrusterId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getThrusterGimbalAngleStream :: Thruster -> RPCContext (KRPCStream ((Double, Double, Double)))
getThrusterGimbalAngleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [ makeArgument 0 (thrusterId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getResourceName :: Resource -> RPCContext (Text)
getResourceName thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Name" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceNameStream :: Resource -> RPCContext (KRPCStream (Text))
getResourceNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Name" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourcePart :: Resource -> RPCContext (Part)
getResourcePart thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Part" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourcePartStream :: Resource -> RPCContext (KRPCStream (Part))
getResourcePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Part" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceMax :: Resource -> RPCContext (Float)
getResourceMax thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Max" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceMaxStream :: Resource -> RPCContext (KRPCStream (Float))
getResourceMaxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Max" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceAmount :: Resource -> RPCContext (Float)
getResourceAmount thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Amount" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceAmountStream :: Resource -> RPCContext (KRPCStream (Float))
getResourceAmountStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Amount" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceDensity :: Resource -> RPCContext (Float)
getResourceDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Density" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceDensityStream :: Resource -> RPCContext (KRPCStream (Float))
getResourceDensityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Density" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceFlowMode :: Resource -> RPCContext (ResourceFlowMode)
getResourceFlowMode thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_FlowMode" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceFlowModeStream :: Resource -> RPCContext (KRPCStream (ResourceFlowMode))
getResourceFlowModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_FlowMode" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceEnabled :: Resource -> RPCContext (Bool)
getResourceEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Enabled" [ makeArgument 0 (resourceId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceEnabledStream :: Resource -> RPCContext (KRPCStream (Bool))
getResourceEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Enabled" [ makeArgument 0 (resourceId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setResourceEnabled :: Resource -> Bool -> RPCContext (Bool)
setResourceEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Resource_set_Enabled" [ makeArgument 0 (resourceId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

resourceTransferStart :: Part -> Part -> Text -> Float -> RPCContext (ResourceTransfer)
resourceTransferStart fromPartArg toPartArg resourceArg maxAmountArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_Start" [ makeArgument 0 (partId fromPartArg), makeArgument 1 (partId toPartArg), makeArgument 2 resourceArg, makeArgument 3 maxAmountArg ]
    res <- sendRequest r
    processResponse extractValue res

resourceTransferStartStream :: Part -> Part -> Text -> Float -> RPCContext (KRPCStream (ResourceTransfer))
resourceTransferStartStream fromPartArg toPartArg resourceArg maxAmountArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_Start" [ makeArgument 0 (partId fromPartArg), makeArgument 1 (partId toPartArg), makeArgument 2 resourceArg, makeArgument 3 maxAmountArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceTransferComplete :: ResourceTransfer -> RPCContext (Bool)
getResourceTransferComplete thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [ makeArgument 0 (resourceTransferId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceTransferCompleteStream :: ResourceTransfer -> RPCContext (KRPCStream (Bool))
getResourceTransferCompleteStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [ makeArgument 0 (resourceTransferId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourceTransferAmount :: ResourceTransfer -> RPCContext (Float)
getResourceTransferAmount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [ makeArgument 0 (resourceTransferId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getResourceTransferAmountStream :: ResourceTransfer -> RPCContext (KRPCStream (Float))
getResourceTransferAmountStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [ makeArgument 0 (resourceTransferId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourcesWithResource :: Resources -> Text -> RPCContext ([Resource])
resourcesWithResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_WithResource" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractList res

resourcesWithResourceStream :: Resources -> Text -> RPCContext (KRPCStream ([Resource]))
resourcesWithResourceStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_WithResource" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

resourcesHasResource :: Resources -> Text -> RPCContext (Bool)
resourcesHasResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_HasResource" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

resourcesHasResourceStream :: Resources -> Text -> RPCContext (KRPCStream (Bool))
resourcesHasResourceStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_HasResource" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourcesMax :: Resources -> Text -> RPCContext (Float)
resourcesMax thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Max" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

resourcesMaxStream :: Resources -> Text -> RPCContext (KRPCStream (Float))
resourcesMaxStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Max" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourcesAmount :: Resources -> Text -> RPCContext (Float)
resourcesAmount thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Amount" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

resourcesAmountStream :: Resources -> Text -> RPCContext (KRPCStream (Float))
resourcesAmountStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Amount" [ makeArgument 0 (resourcesId thisArg), makeArgument 1 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourcesDensity :: Text -> RPCContext (Float)
resourcesDensity nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Density" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

resourcesDensityStream :: Text -> RPCContext (KRPCStream (Float))
resourcesDensityStream nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Density" [ makeArgument 0 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

resourcesFlowMode :: Text -> RPCContext (ResourceFlowMode)
resourcesFlowMode nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_FlowMode" [ makeArgument 0 nameArg ]
    res <- sendRequest r
    processResponse extractValue res

resourcesFlowModeStream :: Text -> RPCContext (KRPCStream (ResourceFlowMode))
resourcesFlowModeStream nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_FlowMode" [ makeArgument 0 nameArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getResourcesAll :: Resources -> RPCContext ([Resource])
getResourcesAll thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_All" [ makeArgument 0 (resourcesId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getResourcesAllStream :: Resources -> RPCContext (KRPCStream ([Resource]))
getResourcesAllStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_All" [ makeArgument 0 (resourcesId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getResourcesNames :: Resources -> RPCContext ([Text])
getResourcesNames thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Names" [ makeArgument 0 (resourcesId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getResourcesNamesStream :: Resources -> RPCContext (KRPCStream ([Text]))
getResourcesNamesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Names" [ makeArgument 0 (resourcesId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

vesselFlight :: Vessel -> ReferenceFrame -> RPCContext (Flight)
vesselFlight thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Flight" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractValue res

vesselFlightStream :: Vessel -> ReferenceFrame -> RPCContext (KRPCStream (Flight))
vesselFlightStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Flight" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

vesselResourcesInDecoupleStage :: Vessel -> Int32 -> Bool -> RPCContext (Resources)
vesselResourcesInDecoupleStage thisArg stageArg cumulativeArg = do
    let r = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [ makeArgument 0 (vesselId thisArg), makeArgument 1 stageArg, makeArgument 2 cumulativeArg ]
    res <- sendRequest r
    processResponse extractValue res

vesselResourcesInDecoupleStageStream :: Vessel -> Int32 -> Bool -> RPCContext (KRPCStream (Resources))
vesselResourcesInDecoupleStageStream thisArg stageArg cumulativeArg = do
    let r = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [ makeArgument 0 (vesselId thisArg), makeArgument 1 stageArg, makeArgument 2 cumulativeArg ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

vesselPosition :: Vessel -> ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Position" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

vesselPositionStream :: Vessel -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Position" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

vesselVelocity :: Vessel -> ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Velocity" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

vesselVelocityStream :: Vessel -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Velocity" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

vesselRotation :: Vessel -> ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
vesselRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Rotation" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple4 res

vesselRotationStream :: Vessel -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
vesselRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Rotation" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple4

vesselDirection :: Vessel -> ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Direction" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

vesselDirectionStream :: Vessel -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Direction" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

vesselAngularVelocity :: Vessel -> ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

vesselAngularVelocityStream :: Vessel -> ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselAngularVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [ makeArgument 0 (vesselId thisArg), makeArgument 1 (referenceFrameId referenceFrameArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getVesselName :: Vessel -> RPCContext (Text)
getVesselName thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Name" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselNameStream :: Vessel -> RPCContext (KRPCStream (Text))
getVesselNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Name" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setVesselName :: Vessel -> Text -> RPCContext (Bool)
setVesselName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Name" [ makeArgument 0 (vesselId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getVesselType :: Vessel -> RPCContext (VesselType)
getVesselType thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Type" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselTypeStream :: Vessel -> RPCContext (KRPCStream (VesselType))
getVesselTypeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Type" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

setVesselType :: Vessel -> VesselType -> RPCContext (Bool)
setVesselType thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Type" [ makeArgument 0 (vesselId thisArg), makeArgument 1 valueArg ]
    res <- sendRequest r
    processResponse extractNothing res

getVesselSituation :: Vessel -> RPCContext (VesselSituation)
getVesselSituation thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Situation" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselSituationStream :: Vessel -> RPCContext (KRPCStream (VesselSituation))
getVesselSituationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Situation" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselMET :: Vessel -> RPCContext (Double)
getVesselMET thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MET" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselMETStream :: Vessel -> RPCContext (KRPCStream (Double))
getVesselMETStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MET" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselOrbit :: Vessel -> RPCContext (Orbit)
getVesselOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Orbit" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselOrbitStream :: Vessel -> RPCContext (KRPCStream (Orbit))
getVesselOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Orbit" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselControl :: Vessel -> RPCContext (Control)
getVesselControl thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Control" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselControlStream :: Vessel -> RPCContext (KRPCStream (Control))
getVesselControlStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Control" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselAutoPilot :: Vessel -> RPCContext (AutoPilot)
getVesselAutoPilot thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselAutoPilotStream :: Vessel -> RPCContext (KRPCStream (AutoPilot))
getVesselAutoPilotStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselResources :: Vessel -> RPCContext (Resources)
getVesselResources thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Resources" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselResourcesStream :: Vessel -> RPCContext (KRPCStream (Resources))
getVesselResourcesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Resources" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselParts :: Vessel -> RPCContext (Parts)
getVesselParts thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Parts" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselPartsStream :: Vessel -> RPCContext (KRPCStream (Parts))
getVesselPartsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Parts" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselComms :: Vessel -> RPCContext (Comms)
getVesselComms thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Comms" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselCommsStream :: Vessel -> RPCContext (KRPCStream (Comms))
getVesselCommsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Comms" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselMass :: Vessel -> RPCContext (Float)
getVesselMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Mass" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselMassStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Mass" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselDryMass :: Vessel -> RPCContext (Float)
getVesselDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_DryMass" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselDryMassStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselDryMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_DryMass" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselThrust :: Vessel -> RPCContext (Float)
getVesselThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Thrust" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselThrustStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Thrust" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselAvailableThrust :: Vessel -> RPCContext (Float)
getVesselAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselAvailableThrustStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselAvailableThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselMaxThrust :: Vessel -> RPCContext (Float)
getVesselMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselMaxThrustStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselMaxVacuumThrust :: Vessel -> RPCContext (Float)
getVesselMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselMaxVacuumThrustStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxVacuumThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselSpecificImpulse :: Vessel -> RPCContext (Float)
getVesselSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselSpecificImpulseStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselVacuumSpecificImpulse :: Vessel -> RPCContext (Float)
getVesselVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselVacuumSpecificImpulseStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselVacuumSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselKerbinSeaLevelSpecificImpulse :: Vessel -> RPCContext (Float)
getVesselKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselKerbinSeaLevelSpecificImpulseStream :: Vessel -> RPCContext (KRPCStream (Float))
getVesselKerbinSeaLevelSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselMomentOfInertia :: Vessel -> RPCContext ((Double, Double, Double))
getVesselMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getVesselMomentOfInertiaStream :: Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselMomentOfInertiaStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getVesselInertiaTensor :: Vessel -> RPCContext ([Double])
getVesselInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractList res

getVesselInertiaTensorStream :: Vessel -> RPCContext (KRPCStream ([Double]))
getVesselInertiaTensorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractList

getVesselReactionWheelTorque :: Vessel -> RPCContext ((Double, Double, Double))
getVesselReactionWheelTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReactionWheelTorque" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractTuple3 res

getVesselReactionWheelTorqueStream :: Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselReactionWheelTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReactionWheelTorque" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractTuple3

getVesselReferenceFrame :: Vessel -> RPCContext (ReferenceFrame)
getVesselReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselReferenceFrameStream :: Vessel -> RPCContext (KRPCStream (ReferenceFrame))
getVesselReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselOrbitalReferenceFrame :: Vessel -> RPCContext (ReferenceFrame)
getVesselOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselOrbitalReferenceFrameStream :: Vessel -> RPCContext (KRPCStream (ReferenceFrame))
getVesselOrbitalReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselSurfaceReferenceFrame :: Vessel -> RPCContext (ReferenceFrame)
getVesselSurfaceReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselSurfaceReferenceFrameStream :: Vessel -> RPCContext (KRPCStream (ReferenceFrame))
getVesselSurfaceReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

getVesselSurfaceVelocityReferenceFrame :: Vessel -> RPCContext (ReferenceFrame)
getVesselSurfaceVelocityReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
    res <- sendRequest r
    processResponse extractValue res

getVesselSurfaceVelocityReferenceFrameStream :: Vessel -> RPCContext (KRPCStream (ReferenceFrame))
getVesselSurfaceVelocityReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [ makeArgument 0 (vesselId thisArg) ]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extractValue res
    return $ KRPCStream sid extractValue

