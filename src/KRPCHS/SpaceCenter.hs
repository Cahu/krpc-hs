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
, RCS
, Radiator
, ReactionWheel
, ReferenceFrame
, Resource
, ResourceConverter
, ResourceHarvester
, ResourceTransfer
, Resources
, Sensor
, SolarPanel
, Thruster
, Vessel
, autoPilotDisengage
, autoPilotEngage
, autoPilotSetPIDParameters
, autoPilotTargetPitchAndHeading
, autoPilotWait
, getAutoPilotError
, getAutoPilotErrorStream
, getAutoPilotMaxRollSpeed
, getAutoPilotMaxRollSpeedStream
, getAutoPilotMaxRotationSpeed
, getAutoPilotMaxRotationSpeedStream
, getAutoPilotReferenceFrame
, getAutoPilotReferenceFrameStream
, getAutoPilotRollError
, getAutoPilotRollErrorStream
, getAutoPilotRollSpeedMultiplier
, getAutoPilotRollSpeedMultiplierStream
, getAutoPilotRotationSpeedMultiplier
, getAutoPilotRotationSpeedMultiplierStream
, getAutoPilotSAS
, getAutoPilotSASStream
, getAutoPilotSASMode
, getAutoPilotSASModeStream
, getAutoPilotTargetDirection
, getAutoPilotTargetDirectionStream
, getAutoPilotTargetRoll
, getAutoPilotTargetRollStream
, setAutoPilotMaxRollSpeed
, setAutoPilotMaxRotationSpeed
, setAutoPilotReferenceFrame
, setAutoPilotRollSpeedMultiplier
, setAutoPilotRotationSpeedMultiplier
, setAutoPilotSAS
, setAutoPilotSASMode
, setAutoPilotTargetDirection
, setAutoPilotTargetRoll
, getCameraDefaultDistance
, getCameraDefaultDistanceStream
, getCameraDistance
, getCameraDistanceStream
, getCameraFocussedBody
, getCameraFocussedBodyStream
, getCameraFocussedNode
, getCameraFocussedNodeStream
, getCameraFocussedVessel
, getCameraFocussedVesselStream
, getCameraHeading
, getCameraHeadingStream
, getCameraMaxDistance
, getCameraMaxDistanceStream
, getCameraMaxPitch
, getCameraMaxPitchStream
, getCameraMinDistance
, getCameraMinDistanceStream
, getCameraMinPitch
, getCameraMinPitchStream
, getCameraMode
, getCameraModeStream
, getCameraPitch
, getCameraPitchStream
, setCameraDistance
, setCameraFocussedBody
, setCameraFocussedNode
, setCameraFocussedVessel
, setCameraHeading
, setCameraMode
, setCameraPitch
, canRailsWarpAt
, canRailsWarpAtStream
, getCargoBayOpen
, getCargoBayOpenStream
, getCargoBayPart
, getCargoBayPartStream
, getCargoBayState
, getCargoBayStateStream
, setCargoBayOpen
, celestialBodyAngularVelocity
, celestialBodyAngularVelocityStream
, celestialBodyBedrockHeight
, celestialBodyBedrockHeightStream
, celestialBodyBedrockPosition
, celestialBodyBedrockPositionStream
, celestialBodyDirection
, celestialBodyDirectionStream
, celestialBodyMSLPosition
, celestialBodyMSLPositionStream
, celestialBodyPosition
, celestialBodyPositionStream
, celestialBodyRotation
, celestialBodyRotationStream
, celestialBodySurfaceHeight
, celestialBodySurfaceHeightStream
, celestialBodySurfacePosition
, celestialBodySurfacePositionStream
, celestialBodyVelocity
, celestialBodyVelocityStream
, getCelestialBodyAtmosphereDepth
, getCelestialBodyAtmosphereDepthStream
, getCelestialBodyEquatorialRadius
, getCelestialBodyEquatorialRadiusStream
, getCelestialBodyGravitationalParameter
, getCelestialBodyGravitationalParameterStream
, getCelestialBodyHasAtmosphere
, getCelestialBodyHasAtmosphereStream
, getCelestialBodyHasAtmosphericOxygen
, getCelestialBodyHasAtmosphericOxygenStream
, getCelestialBodyMass
, getCelestialBodyMassStream
, getCelestialBodyName
, getCelestialBodyNameStream
, getCelestialBodyNonRotatingReferenceFrame
, getCelestialBodyNonRotatingReferenceFrameStream
, getCelestialBodyOrbit
, getCelestialBodyOrbitStream
, getCelestialBodyOrbitalReferenceFrame
, getCelestialBodyOrbitalReferenceFrameStream
, getCelestialBodyReferenceFrame
, getCelestialBodyReferenceFrameStream
, getCelestialBodyRotationalPeriod
, getCelestialBodyRotationalPeriodStream
, getCelestialBodyRotationalSpeed
, getCelestialBodyRotationalSpeedStream
, getCelestialBodySatellites
, getCelestialBodySatellitesStream
, getCelestialBodySphereOfInfluence
, getCelestialBodySphereOfInfluenceStream
, getCelestialBodySurfaceGravity
, getCelestialBodySurfaceGravityStream
, clearTarget
, getControlSurfaceAvailableTorque
, getControlSurfaceAvailableTorqueStream
, getControlSurfaceDeployed
, getControlSurfaceDeployedStream
, getControlSurfaceInverted
, getControlSurfaceInvertedStream
, getControlSurfacePart
, getControlSurfacePartStream
, getControlSurfacePitchEnabled
, getControlSurfacePitchEnabledStream
, getControlSurfaceRollEnabled
, getControlSurfaceRollEnabledStream
, getControlSurfaceSurfaceArea
, getControlSurfaceSurfaceAreaStream
, getControlSurfaceYawEnabled
, getControlSurfaceYawEnabledStream
, setControlSurfaceDeployed
, setControlSurfaceInverted
, setControlSurfacePitchEnabled
, setControlSurfaceRollEnabled
, setControlSurfaceYawEnabled
, controlActivateNextStage
, controlActivateNextStageStream
, controlAddNode
, controlAddNodeStream
, controlGetActionGroup
, controlGetActionGroupStream
, controlRemoveNodes
, controlSetActionGroup
, controlToggleActionGroup
, getControlAbort
, getControlAbortStream
, getControlBrakes
, getControlBrakesStream
, getControlCurrentStage
, getControlCurrentStageStream
, getControlForward
, getControlForwardStream
, getControlGear
, getControlGearStream
, getControlLights
, getControlLightsStream
, getControlNodes
, getControlNodesStream
, getControlPitch
, getControlPitchStream
, getControlRCS
, getControlRCSStream
, getControlRight
, getControlRightStream
, getControlRoll
, getControlRollStream
, getControlSAS
, getControlSASStream
, getControlSASMode
, getControlSASModeStream
, getControlSpeedMode
, getControlSpeedModeStream
, getControlThrottle
, getControlThrottleStream
, getControlUp
, getControlUpStream
, getControlWheelSteering
, getControlWheelSteeringStream
, getControlWheelThrottle
, getControlWheelThrottleStream
, getControlYaw
, getControlYawStream
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
, getDecouplerDecoupled
, getDecouplerDecoupledStream
, getDecouplerImpulse
, getDecouplerImpulseStream
, getDecouplerPart
, getDecouplerPartStream
, dockingPortDirection
, dockingPortDirectionStream
, dockingPortPosition
, dockingPortPositionStream
, dockingPortRotation
, dockingPortRotationStream
, dockingPortUndock
, dockingPortUndockStream
, getDockingPortDockedPart
, getDockingPortDockedPartStream
, getDockingPortHasShield
, getDockingPortHasShieldStream
, getDockingPortName
, getDockingPortNameStream
, getDockingPortPart
, getDockingPortPartStream
, getDockingPortReengageDistance
, getDockingPortReengageDistanceStream
, getDockingPortReferenceFrame
, getDockingPortReferenceFrameStream
, getDockingPortShielded
, getDockingPortShieldedStream
, getDockingPortState
, getDockingPortStateStream
, setDockingPortName
, setDockingPortShielded
, engineToggleMode
, getEngineActive
, getEngineActiveStream
, getEngineAutoModeSwitch
, getEngineAutoModeSwitchStream
, getEngineAvailableThrust
, getEngineAvailableThrustStream
, getEngineAvailableTorque
, getEngineAvailableTorqueStream
, getEngineCanRestart
, getEngineCanRestartStream
, getEngineCanShutdown
, getEngineCanShutdownStream
, getEngineGimbalLimit
, getEngineGimbalLimitStream
, getEngineGimbalLocked
, getEngineGimbalLockedStream
, getEngineGimbalRange
, getEngineGimbalRangeStream
, getEngineGimballed
, getEngineGimballedStream
, getEngineHasFuel
, getEngineHasFuelStream
, getEngineHasModes
, getEngineHasModesStream
, getEngineKerbinSeaLevelSpecificImpulse
, getEngineKerbinSeaLevelSpecificImpulseStream
, getEngineMaxThrust
, getEngineMaxThrustStream
, getEngineMaxVacuumThrust
, getEngineMaxVacuumThrustStream
, getEngineMode
, getEngineModeStream
, getEngineModes
, getEngineModesStream
, getEnginePart
, getEnginePartStream
, getEnginePropellantRatios
, getEnginePropellantRatiosStream
, getEnginePropellants
, getEnginePropellantsStream
, getEngineSpecificImpulse
, getEngineSpecificImpulseStream
, getEngineThrottle
, getEngineThrottleStream
, getEngineThrottleLocked
, getEngineThrottleLockedStream
, getEngineThrust
, getEngineThrustStream
, getEngineThrustLimit
, getEngineThrustLimitStream
, getEngineThrusters
, getEngineThrustersStream
, getEngineVacuumSpecificImpulse
, getEngineVacuumSpecificImpulseStream
, setEngineActive
, setEngineAutoModeSwitch
, setEngineGimbalLimit
, setEngineGimbalLocked
, setEngineMode
, setEngineThrustLimit
, fairingJettison
, getFairingJettisoned
, getFairingJettisonedStream
, getFairingPart
, getFairingPartStream
, getFlightAerodynamicForce
, getFlightAerodynamicForceStream
, getFlightAngleOfAttack
, getFlightAngleOfAttackStream
, getFlightAntiNormal
, getFlightAntiNormalStream
, getFlightAntiRadial
, getFlightAntiRadialStream
, getFlightAtmosphereDensity
, getFlightAtmosphereDensityStream
, getFlightBallisticCoefficient
, getFlightBallisticCoefficientStream
, getFlightBedrockAltitude
, getFlightBedrockAltitudeStream
, getFlightCenterOfMass
, getFlightCenterOfMassStream
, getFlightDirection
, getFlightDirectionStream
, getFlightDrag
, getFlightDragStream
, getFlightDragCoefficient
, getFlightDragCoefficientStream
, getFlightDynamicPressure
, getFlightDynamicPressureStream
, getFlightElevation
, getFlightElevationStream
, getFlightEquivalentAirSpeed
, getFlightEquivalentAirSpeedStream
, getFlightGForce
, getFlightGForceStream
, getFlightHeading
, getFlightHeadingStream
, getFlightHorizontalSpeed
, getFlightHorizontalSpeedStream
, getFlightLatitude
, getFlightLatitudeStream
, getFlightLift
, getFlightLiftStream
, getFlightLiftCoefficient
, getFlightLiftCoefficientStream
, getFlightLongitude
, getFlightLongitudeStream
, getFlightMach
, getFlightMachStream
, getFlightMeanAltitude
, getFlightMeanAltitudeStream
, getFlightNormal
, getFlightNormalStream
, getFlightPitch
, getFlightPitchStream
, getFlightPrograde
, getFlightProgradeStream
, getFlightRadial
, getFlightRadialStream
, getFlightRetrograde
, getFlightRetrogradeStream
, getFlightRoll
, getFlightRollStream
, getFlightRotation
, getFlightRotationStream
, getFlightSideslipAngle
, getFlightSideslipAngleStream
, getFlightSpeed
, getFlightSpeedStream
, getFlightSpeedOfSound
, getFlightSpeedOfSoundStream
, getFlightStallFraction
, getFlightStallFractionStream
, getFlightStaticAirTemperature
, getFlightStaticAirTemperatureStream
, getFlightStaticPressure
, getFlightStaticPressureStream
, getFlightSurfaceAltitude
, getFlightSurfaceAltitudeStream
, getFlightTerminalVelocity
, getFlightTerminalVelocityStream
, getFlightThrustSpecificFuelConsumption
, getFlightThrustSpecificFuelConsumptionStream
, getFlightTotalAirTemperature
, getFlightTotalAirTemperatureStream
, getFlightVelocity
, getFlightVelocityStream
, getFlightVerticalSpeed
, getFlightVerticalSpeedStream
, getIntakeArea
, getIntakeAreaStream
, getIntakeFlow
, getIntakeFlowStream
, getIntakeOpen
, getIntakeOpenStream
, getIntakePart
, getIntakePartStream
, getIntakeSpeed
, getIntakeSpeedStream
, setIntakeOpen
, getLandingGearDeployable
, getLandingGearDeployableStream
, getLandingGearDeployed
, getLandingGearDeployedStream
, getLandingGearPart
, getLandingGearPartStream
, getLandingGearState
, getLandingGearStateStream
, setLandingGearDeployed
, getLandingLegDeployed
, getLandingLegDeployedStream
, getLandingLegPart
, getLandingLegPartStream
, getLandingLegState
, getLandingLegStateStream
, setLandingLegDeployed
, launchClampRelease
, getLaunchClampPart
, getLaunchClampPartStream
, launchVesselFromSPH
, launchVesselFromVAB
, getLightActive
, getLightActiveStream
, getLightColor
, getLightColorStream
, getLightPart
, getLightPartStream
, getLightPowerUsage
, getLightPowerUsageStream
, setLightActive
, setLightColor
, load
, moduleGetField
, moduleGetFieldStream
, moduleHasAction
, moduleHasActionStream
, moduleHasEvent
, moduleHasEventStream
, moduleHasField
, moduleHasFieldStream
, moduleResetField
, moduleSetAction
, moduleSetFieldFloat
, moduleSetFieldInt
, moduleSetFieldString
, moduleTriggerEvent
, getModuleActions
, getModuleActionsStream
, getModuleEvents
, getModuleEventsStream
, getModuleFields
, getModuleFieldsStream
, getModuleName
, getModuleNameStream
, getModulePart
, getModulePartStream
, nodeBurnVector
, nodeBurnVectorStream
, nodeDirection
, nodeDirectionStream
, nodePosition
, nodePositionStream
, nodeRemainingBurnVector
, nodeRemainingBurnVectorStream
, nodeRemove
, getNodeDeltaV
, getNodeDeltaVStream
, getNodeNormal
, getNodeNormalStream
, getNodeOrbit
, getNodeOrbitStream
, getNodeOrbitalReferenceFrame
, getNodeOrbitalReferenceFrameStream
, getNodePrograde
, getNodeProgradeStream
, getNodeRadial
, getNodeRadialStream
, getNodeReferenceFrame
, getNodeReferenceFrameStream
, getNodeRemainingDeltaV
, getNodeRemainingDeltaVStream
, getNodeTimeTo
, getNodeTimeToStream
, getNodeUT
, getNodeUTStream
, setNodeDeltaV
, setNodeNormal
, setNodePrograde
, setNodeRadial
, setNodeUT
, orbitReferencePlaneDirection
, orbitReferencePlaneDirectionStream
, orbitReferencePlaneNormal
, orbitReferencePlaneNormalStream
, getOrbitApoapsis
, getOrbitApoapsisStream
, getOrbitApoapsisAltitude
, getOrbitApoapsisAltitudeStream
, getOrbitArgumentOfPeriapsis
, getOrbitArgumentOfPeriapsisStream
, getOrbitBody
, getOrbitBodyStream
, getOrbitEccentricAnomaly
, getOrbitEccentricAnomalyStream
, getOrbitEccentricity
, getOrbitEccentricityStream
, getOrbitEpoch
, getOrbitEpochStream
, getOrbitInclination
, getOrbitInclinationStream
, getOrbitLongitudeOfAscendingNode
, getOrbitLongitudeOfAscendingNodeStream
, getOrbitMeanAnomaly
, getOrbitMeanAnomalyStream
, getOrbitMeanAnomalyAtEpoch
, getOrbitMeanAnomalyAtEpochStream
, getOrbitNextOrbit
, getOrbitNextOrbitStream
, getOrbitPeriapsis
, getOrbitPeriapsisStream
, getOrbitPeriapsisAltitude
, getOrbitPeriapsisAltitudeStream
, getOrbitPeriod
, getOrbitPeriodStream
, getOrbitRadius
, getOrbitRadiusStream
, getOrbitSemiMajorAxis
, getOrbitSemiMajorAxisStream
, getOrbitSemiMinorAxis
, getOrbitSemiMinorAxisStream
, getOrbitSpeed
, getOrbitSpeedStream
, getOrbitTimeToApoapsis
, getOrbitTimeToApoapsisStream
, getOrbitTimeToPeriapsis
, getOrbitTimeToPeriapsisStream
, getOrbitTimeToSOIChange
, getOrbitTimeToSOIChangeStream
, parachuteDeploy
, getParachuteDeployAltitude
, getParachuteDeployAltitudeStream
, getParachuteDeployMinPressure
, getParachuteDeployMinPressureStream
, getParachuteDeployed
, getParachuteDeployedStream
, getParachutePart
, getParachutePartStream
, getParachuteState
, getParachuteStateStream
, setParachuteDeployAltitude
, setParachuteDeployMinPressure
, partCenterOfMass
, partCenterOfMassStream
, partDirection
, partDirectionStream
, partPosition
, partPositionStream
, partRotation
, partRotationStream
, partVelocity
, partVelocityStream
, getPartAxiallyAttached
, getPartAxiallyAttachedStream
, getPartCargoBay
, getPartCargoBayStream
, getPartCenterOfMassReferenceFrame
, getPartCenterOfMassReferenceFrameStream
, getPartChildren
, getPartChildrenStream
, getPartControlSurface
, getPartControlSurfaceStream
, getPartCost
, getPartCostStream
, getPartCrossfeed
, getPartCrossfeedStream
, getPartDecoupleStage
, getPartDecoupleStageStream
, getPartDecoupler
, getPartDecouplerStream
, getPartDockingPort
, getPartDockingPortStream
, getPartDryMass
, getPartDryMassStream
, getPartDynamicPressure
, getPartDynamicPressureStream
, getPartEngine
, getPartEngineStream
, getPartFairing
, getPartFairingStream
, getPartFuelLinesFrom
, getPartFuelLinesFromStream
, getPartFuelLinesTo
, getPartFuelLinesToStream
, getPartImpactTolerance
, getPartImpactToleranceStream
, getPartInertiaTensor
, getPartInertiaTensorStream
, getPartIntake
, getPartIntakeStream
, getPartIsFuelLine
, getPartIsFuelLineStream
, getPartLandingGear
, getPartLandingGearStream
, getPartLandingLeg
, getPartLandingLegStream
, getPartLaunchClamp
, getPartLaunchClampStream
, getPartLight
, getPartLightStream
, getPartMass
, getPartMassStream
, getPartMassless
, getPartMasslessStream
, getPartMaxSkinTemperature
, getPartMaxSkinTemperatureStream
, getPartMaxTemperature
, getPartMaxTemperatureStream
, getPartModules
, getPartModulesStream
, getPartMomentOfInertia
, getPartMomentOfInertiaStream
, getPartName
, getPartNameStream
, getPartParachute
, getPartParachuteStream
, getPartParent
, getPartParentStream
, getPartRCS
, getPartRCSStream
, getPartRadiallyAttached
, getPartRadiallyAttachedStream
, getPartRadiator
, getPartRadiatorStream
, getPartReactionWheel
, getPartReactionWheelStream
, getPartReferenceFrame
, getPartReferenceFrameStream
, getPartResourceConverter
, getPartResourceConverterStream
, getPartResourceHarvester
, getPartResourceHarvesterStream
, getPartResources
, getPartResourcesStream
, getPartSensor
, getPartSensorStream
, getPartShielded
, getPartShieldedStream
, getPartSkinTemperature
, getPartSkinTemperatureStream
, getPartSolarPanel
, getPartSolarPanelStream
, getPartStage
, getPartStageStream
, getPartTemperature
, getPartTemperatureStream
, getPartThermalConductionFlux
, getPartThermalConductionFluxStream
, getPartThermalConvectionFlux
, getPartThermalConvectionFluxStream
, getPartThermalInternalFlux
, getPartThermalInternalFluxStream
, getPartThermalMass
, getPartThermalMassStream
, getPartThermalRadiationFlux
, getPartThermalRadiationFluxStream
, getPartThermalResourceMass
, getPartThermalResourceMassStream
, getPartThermalSkinMass
, getPartThermalSkinMassStream
, getPartThermalSkinToInternalFlux
, getPartThermalSkinToInternalFluxStream
, getPartTitle
, getPartTitleStream
, getPartVessel
, getPartVesselStream
, partsDockingPortWithName
, partsDockingPortWithNameStream
, partsInDecoupleStage
, partsInDecoupleStageStream
, partsInStage
, partsInStageStream
, partsModulesWithName
, partsModulesWithNameStream
, partsWithModule
, partsWithModuleStream
, partsWithName
, partsWithNameStream
, partsWithTitle
, partsWithTitleStream
, getPartsAll
, getPartsAllStream
, getPartsCargoBays
, getPartsCargoBaysStream
, getPartsControlSurfaces
, getPartsControlSurfacesStream
, getPartsControlling
, getPartsControllingStream
, getPartsDecouplers
, getPartsDecouplersStream
, getPartsDockingPorts
, getPartsDockingPortsStream
, getPartsEngines
, getPartsEnginesStream
, getPartsFairings
, getPartsFairingsStream
, getPartsIntakes
, getPartsIntakesStream
, getPartsLandingGear
, getPartsLandingGearStream
, getPartsLandingLegs
, getPartsLandingLegsStream
, getPartsLaunchClamps
, getPartsLaunchClampsStream
, getPartsLights
, getPartsLightsStream
, getPartsParachutes
, getPartsParachutesStream
, getPartsRCS
, getPartsRCSStream
, getPartsRadiators
, getPartsRadiatorsStream
, getPartsReactionWheels
, getPartsReactionWheelsStream
, getPartsResourceConverters
, getPartsResourceConvertersStream
, getPartsResourceHarvesters
, getPartsResourceHarvestersStream
, getPartsRoot
, getPartsRootStream
, getPartsSensors
, getPartsSensorsStream
, getPartsSolarPanels
, getPartsSolarPanelsStream
, setPartsControlling
, quickload
, quicksave
, getRCSActive
, getRCSActiveStream
, getRCSAvailableTorque
, getRCSAvailableTorqueStream
, getRCSEnabled
, getRCSEnabledStream
, getRCSForwardEnabled
, getRCSForwardEnabledStream
, getRCSHasFuel
, getRCSHasFuelStream
, getRCSKerbinSeaLevelSpecificImpulse
, getRCSKerbinSeaLevelSpecificImpulseStream
, getRCSMaxThrust
, getRCSMaxThrustStream
, getRCSMaxVacuumThrust
, getRCSMaxVacuumThrustStream
, getRCSPart
, getRCSPartStream
, getRCSPitchEnabled
, getRCSPitchEnabledStream
, getRCSPropellantRatios
, getRCSPropellantRatiosStream
, getRCSPropellants
, getRCSPropellantsStream
, getRCSRightEnabled
, getRCSRightEnabledStream
, getRCSRollEnabled
, getRCSRollEnabledStream
, getRCSSpecificImpulse
, getRCSSpecificImpulseStream
, getRCSThrusters
, getRCSThrustersStream
, getRCSUpEnabled
, getRCSUpEnabledStream
, getRCSVacuumSpecificImpulse
, getRCSVacuumSpecificImpulseStream
, getRCSYawEnabled
, getRCSYawEnabledStream
, setRCSEnabled
, setRCSForwardEnabled
, setRCSPitchEnabled
, setRCSRightEnabled
, setRCSRollEnabled
, setRCSUpEnabled
, setRCSYawEnabled
, getRadiatorDeployable
, getRadiatorDeployableStream
, getRadiatorDeployed
, getRadiatorDeployedStream
, getRadiatorPart
, getRadiatorPartStream
, getRadiatorState
, getRadiatorStateStream
, setRadiatorDeployed
, getReactionWheelActive
, getReactionWheelActiveStream
, getReactionWheelAvailableTorque
, getReactionWheelAvailableTorqueStream
, getReactionWheelBroken
, getReactionWheelBrokenStream
, getReactionWheelMaxTorque
, getReactionWheelMaxTorqueStream
, getReactionWheelPart
, getReactionWheelPartStream
, setReactionWheelActive
, resourceConverterActive
, resourceConverterActiveStream
, resourceConverterInputs
, resourceConverterInputsStream
, resourceConverterName
, resourceConverterNameStream
, resourceConverterOutputs
, resourceConverterOutputsStream
, resourceConverterStart
, resourceConverterState
, resourceConverterStateStream
, resourceConverterStatusInfo
, resourceConverterStatusInfoStream
, resourceConverterStop
, getResourceConverterCount
, getResourceConverterCountStream
, getResourceConverterPart
, getResourceConverterPartStream
, getResourceHarvesterActive
, getResourceHarvesterActiveStream
, getResourceHarvesterCoreTemperature
, getResourceHarvesterCoreTemperatureStream
, getResourceHarvesterDeployed
, getResourceHarvesterDeployedStream
, getResourceHarvesterExtractionRate
, getResourceHarvesterExtractionRateStream
, getResourceHarvesterOptimumCoreTemperature
, getResourceHarvesterOptimumCoreTemperatureStream
, getResourceHarvesterPart
, getResourceHarvesterPartStream
, getResourceHarvesterState
, getResourceHarvesterStateStream
, getResourceHarvesterThermalEfficiency
, getResourceHarvesterThermalEfficiencyStream
, setResourceHarvesterActive
, setResourceHarvesterDeployed
, resourceTransferStart
, resourceTransferStartStream
, getResourceTransferAmount
, getResourceTransferAmountStream
, getResourceTransferComplete
, getResourceTransferCompleteStream
, getResourceAmount
, getResourceAmountStream
, getResourceDensity
, getResourceDensityStream
, getResourceEnabled
, getResourceEnabledStream
, getResourceFlowMode
, getResourceFlowModeStream
, getResourceMax
, getResourceMaxStream
, getResourceName
, getResourceNameStream
, getResourcePart
, getResourcePartStream
, setResourceEnabled
, resourcesAmount
, resourcesAmountStream
, resourcesDensity
, resourcesDensityStream
, resourcesFlowMode
, resourcesFlowModeStream
, resourcesHasResource
, resourcesHasResourceStream
, resourcesMax
, resourcesMaxStream
, resourcesWithResource
, resourcesWithResourceStream
, getResourcesAll
, getResourcesAllStream
, getResourcesNames
, getResourcesNamesStream
, save
, getSensorActive
, getSensorActiveStream
, getSensorPart
, getSensorPartStream
, getSensorPowerUsage
, getSensorPowerUsageStream
, getSensorValue
, getSensorValueStream
, setSensorActive
, getSolarPanelDeployed
, getSolarPanelDeployedStream
, getSolarPanelEnergyFlow
, getSolarPanelEnergyFlowStream
, getSolarPanelPart
, getSolarPanelPartStream
, getSolarPanelState
, getSolarPanelStateStream
, getSolarPanelSunExposure
, getSolarPanelSunExposureStream
, setSolarPanelDeployed
, thrusterGimbalPosition
, thrusterGimbalPositionStream
, thrusterInitialThrustDirection
, thrusterInitialThrustDirectionStream
, thrusterInitialThrustPosition
, thrusterInitialThrustPositionStream
, thrusterThrustDirection
, thrusterThrustDirectionStream
, thrusterThrustPosition
, thrusterThrustPositionStream
, getThrusterGimbalAngle
, getThrusterGimbalAngleStream
, getThrusterGimballed
, getThrusterGimballedStream
, getThrusterPart
, getThrusterPartStream
, getThrusterThrustReferenceFrame
, getThrusterThrustReferenceFrameStream
, transformDirection
, transformDirectionStream
, transformPosition
, transformPositionStream
, transformRotation
, transformRotationStream
, transformVelocity
, transformVelocityStream
, vesselAngularVelocity
, vesselAngularVelocityStream
, vesselDirection
, vesselDirectionStream
, vesselFlight
, vesselFlightStream
, vesselPosition
, vesselPositionStream
, vesselResourcesInDecoupleStage
, vesselResourcesInDecoupleStageStream
, vesselRotation
, vesselRotationStream
, vesselVelocity
, vesselVelocityStream
, getVesselAutoPilot
, getVesselAutoPilotStream
, getVesselAvailableControlSurfaceTorque
, getVesselAvailableControlSurfaceTorqueStream
, getVesselAvailableEngineTorque
, getVesselAvailableEngineTorqueStream
, getVesselAvailableRCSTorque
, getVesselAvailableRCSTorqueStream
, getVesselAvailableReactionWheelTorque
, getVesselAvailableReactionWheelTorqueStream
, getVesselAvailableThrust
, getVesselAvailableThrustStream
, getVesselAvailableTorque
, getVesselAvailableTorqueStream
, getVesselControl
, getVesselControlStream
, getVesselDryMass
, getVesselDryMassStream
, getVesselInertiaTensor
, getVesselInertiaTensorStream
, getVesselKerbinSeaLevelSpecificImpulse
, getVesselKerbinSeaLevelSpecificImpulseStream
, getVesselMET
, getVesselMETStream
, getVesselMass
, getVesselMassStream
, getVesselMaxThrust
, getVesselMaxThrustStream
, getVesselMaxVacuumThrust
, getVesselMaxVacuumThrustStream
, getVesselMomentOfInertia
, getVesselMomentOfInertiaStream
, getVesselName
, getVesselNameStream
, getVesselOrbit
, getVesselOrbitStream
, getVesselOrbitalReferenceFrame
, getVesselOrbitalReferenceFrameStream
, getVesselParts
, getVesselPartsStream
, getVesselReferenceFrame
, getVesselReferenceFrameStream
, getVesselResources
, getVesselResourcesStream
, getVesselSituation
, getVesselSituationStream
, getVesselSpecificImpulse
, getVesselSpecificImpulseStream
, getVesselSurfaceReferenceFrame
, getVesselSurfaceReferenceFrameStream
, getVesselSurfaceVelocityReferenceFrame
, getVesselSurfaceVelocityReferenceFrameStream
, getVesselThrust
, getVesselThrustStream
, getVesselType
, getVesselTypeStream
, getVesselVacuumSpecificImpulse
, getVesselVacuumSpecificImpulseStream
, setVesselName
, setVesselType
, warpTo
, getActiveVessel
, getActiveVesselStream
, getBodies
, getBodiesStream
, getCamera
, getCameraStream
, getFARAvailable
, getFARAvailableStream
, getG
, getGStream
, getMaximumRailsWarpFactor
, getMaximumRailsWarpFactorStream
, getPhysicsWarpFactor
, getPhysicsWarpFactorStream
, getRailsWarpFactor
, getRailsWarpFactorStream
, getTargetBody
, getTargetBodyStream
, getTargetDockingPort
, getTargetDockingPortStream
, getTargetVessel
, getTargetVesselStream
, getUT
, getUTStream
, getVessels
, getVesselsStream
, getWarpFactor
, getWarpFactorStream
, getWarpMode
, getWarpModeStream
, getWarpRate
, getWarpRateStream
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
 -}
newtype Camera = Camera { cameraId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Camera where
    encodePb   = encodePb . cameraId
    decodePb b = Camera <$> decodePb b

instance KRPCResponseExtractable Camera

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.CargoBay" />.
 -}
newtype CargoBay = CargoBay { cargoBayId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable CargoBay where
    encodePb   = encodePb . cargoBayId
    decodePb b = CargoBay <$> decodePb b

instance KRPCResponseExtractable CargoBay

{-
 - Represents a celestial body (such as a planet or moon).
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
 - direction in which the vessel is pointing.Control inputs (such as pitch, yaw and roll) are zeroed when all clients
 - that have set one or more of these inputs are no longer connected.
 -}
newtype Control = Control { controlId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Control where
    encodePb   = encodePb . controlId
    decodePb b = Control <$> decodePb b

instance KRPCResponseExtractable Control

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.ControlSurface" />.
 - Provides functionality to interact with aerodynamic control surfaces.
 -}
newtype ControlSurface = ControlSurface { controlSurfaceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ControlSurface where
    encodePb   = encodePb . controlSurfaceId
    decodePb b = ControlSurface <$> decodePb b

instance KRPCResponseExtractable ControlSurface

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Decoupler" />
 -}
newtype Decoupler = Decoupler { decouplerId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Decoupler where
    encodePb   = encodePb . decouplerId
    decodePb b = Decoupler <$> decodePb b

instance KRPCResponseExtractable Decoupler

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.DockingPort" />
 -}
newtype DockingPort = DockingPort { dockingPortId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable DockingPort where
    encodePb   = encodePb . dockingPortId
    decodePb b = DockingPort <$> decodePb b

instance KRPCResponseExtractable DockingPort

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Engine" />.Provides functionality to interact with engines of various types,
 - for example liquid fuelled gimballed engines, solid rocket boosters and jet engines.
 - For RCS thrusters <see cref="M:SpaceCenter.Part.RCS" />.
 -}
newtype Engine = Engine { engineId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Engine where
    encodePb   = encodePb . engineId
    decodePb b = Engine <$> decodePb b

instance KRPCResponseExtractable Engine

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Fairing" />.
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
 - passed to that method.To get orbital information, such as the apoapsis or inclination, see <see cref="T:SpaceCenter.Orbit" />.
 -}
newtype Flight = Flight { flightId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Flight where
    encodePb   = encodePb . flightId
    decodePb b = Flight <$> decodePb b

instance KRPCResponseExtractable Flight

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Intake" />.
 -}
newtype Intake = Intake { intakeId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Intake where
    encodePb   = encodePb . intakeId
    decodePb b = Intake <$> decodePb b

instance KRPCResponseExtractable Intake

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.LandingGear" />.
 -}
newtype LandingGear = LandingGear { landingGearId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LandingGear where
    encodePb   = encodePb . landingGearId
    decodePb b = LandingGear <$> decodePb b

instance KRPCResponseExtractable LandingGear

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.LandingLeg" />.
 -}
newtype LandingLeg = LandingLeg { landingLegId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LandingLeg where
    encodePb   = encodePb . landingLegId
    decodePb b = LandingLeg <$> decodePb b

instance KRPCResponseExtractable LandingLeg

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.LaunchClamp" />.
 -}
newtype LaunchClamp = LaunchClamp { launchClampId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable LaunchClamp where
    encodePb   = encodePb . launchClampId
    decodePb b = LaunchClamp <$> decodePb b

instance KRPCResponseExtractable LaunchClamp

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Light" />.
 -}
newtype Light = Light { lightId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Light where
    encodePb   = encodePb . lightId
    decodePb b = Light <$> decodePb b

instance KRPCResponseExtractable Light

{-
 - In KSP, each part has zero or more
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation#MODULES">PartModulesassociated with it. Each one contains some of the functionality of the part.
 - For example, an engine has a "ModuleEngines" PartModule that contains all the
 - functionality of an engine.
 - This class allows you to interact with KSPs PartModules, and any PartModules
 - that have been added by other mods.
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
 - Obtained by calling <see cref="M:SpaceCenter.Part.Parachute" />.
 -}
newtype Parachute = Parachute { parachuteId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Parachute where
    encodePb   = encodePb . parachuteId
    decodePb b = Parachute <$> decodePb b

instance KRPCResponseExtractable Parachute

{-
 - Instances of this class represents a part. A vessel is made of multiple parts.
 - Instances can be obtained by various methods in <see cref="T:SpaceCenter.Parts" />.
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
 - Obtained by calling <see cref="M:SpaceCenter.Part.RCS" />.
 - Provides functionality to interact with RCS blocks and thrusters.
 -}
newtype RCS = RCS { rCSId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable RCS where
    encodePb   = encodePb . rCSId
    decodePb b = RCS <$> decodePb b

instance KRPCResponseExtractable RCS

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.Radiator" />.
 -}
newtype Radiator = Radiator { radiatorId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Radiator where
    encodePb   = encodePb . radiatorId
    decodePb b = Radiator <$> decodePb b

instance KRPCResponseExtractable Radiator

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.ReactionWheel" />.
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
 - A resource stored within a part.
 -}
newtype Resource = Resource { resourceId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Resource where
    encodePb   = encodePb . resourceId
    decodePb b = Resource <$> decodePb b

instance KRPCResponseExtractable Resource

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.ResourceConverter" />.
 -}
newtype ResourceConverter = ResourceConverter { resourceConverterId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable ResourceConverter where
    encodePb   = encodePb . resourceConverterId
    decodePb b = ResourceConverter <$> decodePb b

instance KRPCResponseExtractable ResourceConverter

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.ResourceHarvester" />.
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
 - Obtained by calling <see cref="M:SpaceCenter.Part.Sensor" />.
 -}
newtype Sensor = Sensor { sensorId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Sensor where
    encodePb   = encodePb . sensorId
    decodePb b = Sensor <$> decodePb b

instance KRPCResponseExtractable Sensor

{-
 - Obtained by calling <see cref="M:SpaceCenter.Part.SolarPanel" />.
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
 - See <see cref="M:SpaceCenter.CargoBay.State" />.
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
 - See <see cref="M:SpaceCenter.DockingPort.State" />.
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
 - See <see cref="M:SpaceCenter.LandingGear.State" />.
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
 - See <see cref="M:SpaceCenter.LandingLeg.State" />.
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
 - See <see cref="M:SpaceCenter.Parachute.State" />.
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
 - <see cref="T:SpaceCenter.RadiatorState" />
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
 - See <see cref="M:SpaceCenter.ResourceConverter.State" />.
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
 - See <see cref="M:SpaceCenter.Resources.FlowMode" />.
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
 - See <see cref="M:SpaceCenter.ResourceHarvester.State" />.
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
 - See <see cref="M:SpaceCenter.SolarPanel.State" />.
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
autoPilotDisengage :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
autoPilotDisengage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Disengage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Engage the auto-pilot.
 -}
autoPilotEngage :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
autoPilotEngage thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Engage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Sets the gains for the rotation rate PID controller.<param name="kp">Proportional gain.<param name="ki">Integral gain.<param name="kd">Derivative gain.
 -}
autoPilotSetPIDParameters :: KRPCHS.SpaceCenter.AutoPilot -> Float -> Float -> Float -> RPCContext (Bool)
autoPilotSetPIDParameters thisArg kpArg kiArg kdArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_SetPIDParameters" [makeArgument 0 thisArg, makeArgument 1 kpArg, makeArgument 2 kiArg, makeArgument 3 kdArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />) from a pitch and heading angle.<param name="pitch">Target pitch angle, in degrees between -90° and +90°.<param name="heading">Target heading angle, in degrees between 0° and 360°.
 -}
autoPilotTargetPitchAndHeading :: KRPCHS.SpaceCenter.AutoPilot -> Float -> Float -> RPCContext (Bool)
autoPilotTargetPitchAndHeading thisArg pitchArg headingArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_TargetPitchAndHeading" [makeArgument 0 thisArg, makeArgument 1 pitchArg, makeArgument 2 headingArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Blocks until the vessel is pointing in the target direction (if set) and has the target roll (if set).
 -}
autoPilotWait :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
autoPilotWait thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_Wait" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The error, in degrees, between the direction the ship has been asked
 - to point in and the direction it is pointing in. Returns zero if the auto-pilot
 - has not been engaged, SAS is not enabled, SAS is in stability assist mode,
 - or no target direction is set.
 -}
getAutoPilotError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotErrorStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_Error" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Maximum target roll speed. Defaults to 1.
 -}
getAutoPilotMaxRollSpeed :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotMaxRollSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRollSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotMaxRollSpeedStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotMaxRollSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRollSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Maximum target rotation speed. Defaults to 1.
 -}
getAutoPilotMaxRotationSpeed :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotMaxRotationSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRotationSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotMaxRotationSpeedStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotMaxRotationSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_MaxRotationSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).
 -}
getAutoPilotReferenceFrame :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getAutoPilotReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotReferenceFrameStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getAutoPilotReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The error, in degrees, between the roll the ship has been asked to be
 - in and the actual roll. Returns zero if the auto-pilot has not been engaged
 - or no target roll is set.
 -}
getAutoPilotRollError :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotRollError thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotRollErrorStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRollErrorStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollError" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Target roll speed multiplier. Defaults to 1.
 -}
getAutoPilotRollSpeedMultiplier :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotRollSpeedMultiplier thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollSpeedMultiplier" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotRollSpeedMultiplierStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRollSpeedMultiplierStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RollSpeedMultiplier" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Target rotation speed multiplier. Defaults to 1.
 -}
getAutoPilotRotationSpeedMultiplier :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotRotationSpeedMultiplier thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RotationSpeedMultiplier" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotRotationSpeedMultiplierStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotRotationSpeedMultiplierStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_RotationSpeedMultiplier" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
getAutoPilotSAS :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Bool)
getAutoPilotSAS thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotSASStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Bool))
getAutoPilotSASStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SAS" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
getAutoPilotSASMode :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCHS.SpaceCenter.SASMode)
getAutoPilotSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotSASModeStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getAutoPilotSASModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_SASMode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The target direction.nullif no target direction is set.
 -}
getAutoPilotTargetDirection :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext ((Double, Double, Double))
getAutoPilotTargetDirection thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotTargetDirectionStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream ((Double, Double, Double)))
getAutoPilotTargetDirectionStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetDirection" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The target roll, in degrees.NaNif no target roll is set.
 -}
getAutoPilotTargetRoll :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (Float)
getAutoPilotTargetRoll thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getAutoPilotTargetRollStream :: KRPCHS.SpaceCenter.AutoPilot -> RPCContext (KRPCStream (Float))
getAutoPilotTargetRollStream thisArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_get_TargetRoll" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Maximum target roll speed. Defaults to 1.
 -}
setAutoPilotMaxRollSpeed :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotMaxRollSpeed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_MaxRollSpeed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Maximum target rotation speed. Defaults to 1.
 -}
setAutoPilotMaxRotationSpeed :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotMaxRotationSpeed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_MaxRotationSpeed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The reference frame for the target direction (<see cref="M:SpaceCenter.AutoPilot.TargetDirection" />).
 -}
setAutoPilotReferenceFrame :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (Bool)
setAutoPilotReferenceFrame thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Target roll speed multiplier. Defaults to 1.
 -}
setAutoPilotRollSpeedMultiplier :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotRollSpeedMultiplier thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RollSpeedMultiplier" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Target rotation speed multiplier. Defaults to 1.
 -}
setAutoPilotRotationSpeedMultiplier :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotRotationSpeedMultiplier thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_RotationSpeedMultiplier" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.Control.SAS" />
 -}
setAutoPilotSAS :: KRPCHS.SpaceCenter.AutoPilot -> Bool -> RPCContext (Bool)
setAutoPilotSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.Control.SASMode" />
 -}
setAutoPilotSASMode :: KRPCHS.SpaceCenter.AutoPilot -> KRPCHS.SpaceCenter.SASMode -> RPCContext (Bool)
setAutoPilotSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The target direction.nullif no target direction is set.
 -}
setAutoPilotTargetDirection :: KRPCHS.SpaceCenter.AutoPilot -> (Double, Double, Double) -> RPCContext (Bool)
setAutoPilotTargetDirection thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetDirection" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The target roll, in degrees.NaNif no target roll is set.
 -}
setAutoPilotTargetRoll :: KRPCHS.SpaceCenter.AutoPilot -> Float -> RPCContext (Bool)
setAutoPilotTargetRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "AutoPilot_set_TargetRoll" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Default distance from the camera to the subject.
 -}
getCameraDefaultDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraDefaultDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraDefaultDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraDefaultDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_DefaultDistance" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The distance from the camera to the subject.
 - A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
getCameraDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Distance" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - In map mode, the celestial body that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a celestial body.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedBody :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getCameraFocussedBody thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraFocussedBodyStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getCameraFocussedBodyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedBody" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - In map mode, the maneuver node that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a maneuver node.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedNode :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.Node)
getCameraFocussedNode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraFocussedNodeStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Node))
getCameraFocussedNodeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedNode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - In map mode, the vessel that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a vessel.
 - Returns an error is the camera is not in map mode.
 -}
getCameraFocussedVessel :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getCameraFocussedVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraFocussedVesselStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getCameraFocussedVesselStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_FocussedVessel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The heading of the camera, in degrees.
 -}
getCameraHeading :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraHeadingStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraHeadingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Heading" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Maximum distance from the camera to the subject.
 -}
getCameraMaxDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMaxDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraMaxDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMaxDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxDistance" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum pitch of the camera.
 -}
getCameraMaxPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMaxPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraMaxPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMaxPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MaxPitch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Minimum distance from the camera to the subject.
 -}
getCameraMinDistance :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMinDistance thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraMinDistanceStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMinDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinDistance" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The minimum pitch of the camera.
 -}
getCameraMinPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraMinPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraMinPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraMinPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_MinPitch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current mode of the camera.
 -}
getCameraMode :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCHS.SpaceCenter.CameraMode)
getCameraMode thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraModeStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CameraMode))
getCameraModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Mode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The pitch of the camera, in degrees.
 - A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
getCameraPitch :: KRPCHS.SpaceCenter.Camera -> RPCContext (Float)
getCameraPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCameraPitchStream :: KRPCHS.SpaceCenter.Camera -> RPCContext (KRPCStream (Float))
getCameraPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Camera_get_Pitch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The distance from the camera to the subject.
 - A value between <see cref="M:SpaceCenter.Camera.MinDistance" /> and <see cref="M:SpaceCenter.Camera.MaxDistance" />.
 -}
setCameraDistance :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext (Bool)
setCameraDistance thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Distance" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - In map mode, the celestial body that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a celestial body.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedBody :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
setCameraFocussedBody thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedBody" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - In map mode, the maneuver node that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a maneuver node.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedNode :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Node -> RPCContext (Bool)
setCameraFocussedNode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedNode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - In map mode, the vessel that the camera is focussed on.
 - Returnsnullif the camera is not focussed on a vessel.
 - Returns an error is the camera is not in map mode.
 -}
setCameraFocussedVessel :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
setCameraFocussedVessel thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_FocussedVessel" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The heading of the camera, in degrees.
 -}
setCameraHeading :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext (Bool)
setCameraHeading thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Heading" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The current mode of the camera.
 -}
setCameraMode :: KRPCHS.SpaceCenter.Camera -> KRPCHS.SpaceCenter.CameraMode -> RPCContext (Bool)
setCameraMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The pitch of the camera, in degrees.
 - A value between <see cref="M:SpaceCenter.Camera.MinPitch" /> and <see cref="M:SpaceCenter.Camera.MaxPitch" />
 -}
setCameraPitch :: KRPCHS.SpaceCenter.Camera -> Float -> RPCContext (Bool)
setCameraPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Camera_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


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
    processResponse extract res 

canRailsWarpAtStream :: Data.Int.Int32 -> RPCContext (KRPCStream (Bool))
canRailsWarpAtStream factorArg = do
    let r = makeRequest "SpaceCenter" "CanRailsWarpAt" [makeArgument 0 factorArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the cargo bay is open.
 -}
getCargoBayOpen :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (Bool)
getCargoBayOpen thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCargoBayOpenStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (Bool))
getCargoBayOpenStream thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Open" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this cargo bay.
 -}
getCargoBayPart :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCHS.SpaceCenter.Part)
getCargoBayPart thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCargoBayPartStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getCargoBayPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the cargo bay.
 -}
getCargoBayState :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCHS.SpaceCenter.CargoBayState)
getCargoBayState thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCargoBayStateStream :: KRPCHS.SpaceCenter.CargoBay -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CargoBayState))
getCargoBayStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the cargo bay is open.
 -}
setCargoBayOpen :: KRPCHS.SpaceCenter.CargoBay -> Bool -> RPCContext (Bool)
setCargoBayOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "CargoBay_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


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
    processResponse extract res 

celestialBodyAngularVelocityStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyAngularVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The height of the surface relative to mean sea level at the given position,
 - in meters. When over water, this is the height of the sea-bed and is therefore a
 - negative value.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodyBedrockHeight :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodyBedrockHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyBedrockHeightStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodyBedrockHeightStream thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position of the surface at the given latitude and longitude, in the given
 - reference frame. When over water, this is the position at the bottom of the sea-bed.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyBedrockPosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyBedrockPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyBedrockPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyBedrockPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_BedrockPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the direction in which the north pole of the celestial body is
 - pointing, as a unit vector, in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyDirection :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyDirectionStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position at mean sea level at the given latitude and longitude, in the given reference frame.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodyMSLPosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyMSLPosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyMSLPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyMSLPositionStream thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_MSLPosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the position vector of the center of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyPosition :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyPositionStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the rotation of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyRotation :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
celestialBodyRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyRotationStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
celestialBodyRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The height of the surface relative to mean sea level at the given position,
 - in meters. When over water this is equal to 0.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees
 -}
celestialBodySurfaceHeight :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (Double)
celestialBodySurfaceHeight thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodySurfaceHeightStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> RPCContext (KRPCStream (Double))
celestialBodySurfaceHeightStream thisArg latitudeArg longitudeArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfaceHeight" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position of the surface at the given latitude and longitude, in the given
 - reference frame. When over water, this is the position of the surface of the water.<param name="latitude">Latitude in degrees<param name="longitude">Longitude in degrees<param name="referenceFrame">Reference frame for the returned position vector
 -}
celestialBodySurfacePosition :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodySurfacePosition thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodySurfacePositionStream :: KRPCHS.SpaceCenter.CelestialBody -> Double -> Double -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodySurfacePositionStream thisArg latitudeArg longitudeArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_SurfacePosition" [makeArgument 0 thisArg, makeArgument 1 latitudeArg, makeArgument 2 longitudeArg, makeArgument 3 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the velocity vector of the body in the specified reference frame.<param name="referenceFrame">
 -}
celestialBodyVelocity :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
celestialBodyVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

celestialBodyVelocityStream :: KRPCHS.SpaceCenter.CelestialBody -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
celestialBodyVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The depth of the atmosphere, in meters.
 -}
getCelestialBodyAtmosphereDepth :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyAtmosphereDepth thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyAtmosphereDepthStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyAtmosphereDepthStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_AtmosphereDepth" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The equatorial radius of the body, in meters.
 -}
getCelestialBodyEquatorialRadius :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyEquatorialRadius thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyEquatorialRadiusStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyEquatorialRadiusStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_EquatorialRadius" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Standard_gravitational_parameter">standard
 - gravitational parameterof the body inm^3s^{ -2}.
 -}
getCelestialBodyGravitationalParameter :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyGravitationalParameter thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyGravitationalParameterStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyGravitationalParameterStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_GravitationalParameter" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - trueif the body has an atmosphere.
 -}
getCelestialBodyHasAtmosphere :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphere thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyHasAtmosphereStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphereStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphere" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - trueif there is oxygen in the atmosphere, required for air-breathing engines.
 -}
getCelestialBodyHasAtmosphericOxygen :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
getCelestialBodyHasAtmosphericOxygen thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyHasAtmosphericOxygenStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Bool))
getCelestialBodyHasAtmosphericOxygenStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_HasAtmosphericOxygen" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The mass of the body, in kilograms.
 -}
getCelestialBodyMass :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyMass thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyMassStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Mass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the body.
 -}
getCelestialBodyName :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Data.Text.Text)
getCelestialBodyName thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyNameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Data.Text.Text))
getCelestialBodyNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getCelestialBodyNonRotatingReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyNonRotatingReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_NonRotatingReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The orbit of the body.
 -}
getCelestialBodyOrbit :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getCelestialBodyOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyOrbitStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getCelestialBodyOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Orbit" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getCelestialBodyOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyOrbitalReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getCelestialBodyReferenceFrameStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getCelestialBodyReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The sidereal rotational period of the body, in seconds.
 -}
getCelestialBodyRotationalPeriod :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyRotationalPeriodStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalPeriodStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalPeriod" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rotational speed of the body, in radians per second.
 -}
getCelestialBodyRotationalSpeed :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodyRotationalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodyRotationalSpeedStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodyRotationalSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_RotationalSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of celestial bodies that are in orbit around this celestial body.
 -}
getCelestialBodySatellites :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext ([KRPCHS.SpaceCenter.CelestialBody])
getCelestialBodySatellites thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodySatellitesStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.CelestialBody]))
getCelestialBodySatellitesStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_Satellites" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The radius of the sphere of influence of the body, in meters.
 -}
getCelestialBodySphereOfInfluence :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySphereOfInfluence thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodySphereOfInfluenceStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySphereOfInfluenceStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SphereOfInfluence" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The acceleration due to gravity at sea level (mean altitude) on the body, inm/s^2.
 -}
getCelestialBodySurfaceGravity :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Float)
getCelestialBodySurfaceGravity thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getCelestialBodySurfaceGravityStream :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (KRPCStream (Float))
getCelestialBodySurfaceGravityStream thisArg = do
    let r = makeRequest "SpaceCenter" "CelestialBody_get_SurfaceGravity" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Clears the current target.
 -}
clearTarget :: RPCContext (Bool)
clearTarget  = do
    let r = makeRequest "SpaceCenter" "ClearTarget" []
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getControlSurfaceAvailableTorque :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext ((Double, Double, Double))
getControlSurfaceAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfaceAvailableTorqueStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream ((Double, Double, Double)))
getControlSurfaceAvailableTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_AvailableTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the control surface has been fully deployed.
 -}
getControlSurfaceDeployed :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfaceDeployedStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the control surface movement is inverted.
 -}
getControlSurfaceInverted :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceInverted thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfaceInvertedStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceInvertedStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Inverted" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this control surface.
 -}
getControlSurfacePart :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCHS.SpaceCenter.Part)
getControlSurfacePart thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfacePartStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getControlSurfacePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the control surface has pitch control enabled.
 -}
getControlSurfacePitchEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfacePitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfacePitchEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfacePitchEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_PitchEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the control surface has roll control enabled.
 -}
getControlSurfaceRollEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfaceRollEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceRollEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_RollEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Surface area of the control surface inm^2.
 -}
getControlSurfaceSurfaceArea :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Float)
getControlSurfaceSurfaceArea thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfaceSurfaceAreaStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Float))
getControlSurfaceSurfaceAreaStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_SurfaceArea" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the control surface has yaw control enabled.
 -}
getControlSurfaceYawEnabled :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (Bool)
getControlSurfaceYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSurfaceYawEnabledStream :: KRPCHS.SpaceCenter.ControlSurface -> RPCContext (KRPCStream (Bool))
getControlSurfaceYawEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_get_YawEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the control surface has been fully deployed.
 -}
setControlSurfaceDeployed :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the control surface movement is inverted.
 -}
setControlSurfaceInverted :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceInverted thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_Inverted" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the control surface has pitch control enabled.
 -}
setControlSurfacePitchEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfacePitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the control surface has roll control enabled.
 -}
setControlSurfaceRollEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the control surface has yaw control enabled.
 -}
setControlSurfaceYawEnabled :: KRPCHS.SpaceCenter.ControlSurface -> Bool -> RPCContext (Bool)
setControlSurfaceYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ControlSurface_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Activates the next stage. Equivalent to pressing the space bar in-game.A list of vessel objects that are jettisoned from the active vessel.
 -}
controlActivateNextStage :: KRPCHS.SpaceCenter.Control -> RPCContext ([KRPCHS.SpaceCenter.Vessel])
controlActivateNextStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

controlActivateNextStageStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
controlActivateNextStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_ActivateNextStage" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

controlAddNodeStream :: KRPCHS.SpaceCenter.Control -> Double -> Float -> Float -> Float -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Node))
controlAddNodeStream thisArg utArg progradeArg normalArg radialArg = do
    let r = makeRequest "SpaceCenter" "Control_AddNode" [makeArgument 0 thisArg, makeArgument 1 utArg, makeArgument 2 progradeArg, makeArgument 3 normalArg, makeArgument 4 radialArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returnstrueif the given action group is enabled.<param name="group">A number between 0 and 9 inclusive.
 -}
controlGetActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (Bool)
controlGetActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    res <- sendRequest r
    processResponse extract res 

controlGetActionGroupStream :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (KRPCStream (Bool))
controlGetActionGroupStream thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_GetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Remove all maneuver nodes.
 -}
controlRemoveNodes :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
controlRemoveNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_RemoveNodes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Sets the state of the given action group (a value between 0 and 9
 - inclusive).<param name="group">A number between 0 and 9 inclusive.<param name="state">
 -}
controlSetActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> Bool -> RPCContext (Bool)
controlSetActionGroup thisArg groupArg stateArg = do
    let r = makeRequest "SpaceCenter" "Control_SetActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg, makeArgument 2 stateArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Toggles the state of the given action group.<param name="group">A number between 0 and 9 inclusive.
 -}
controlToggleActionGroup :: KRPCHS.SpaceCenter.Control -> Data.Word.Word32 -> RPCContext (Bool)
controlToggleActionGroup thisArg groupArg = do
    let r = makeRequest "SpaceCenter" "Control_ToggleActionGroup" [makeArgument 0 thisArg, makeArgument 1 groupArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the abort action group.
 -}
getControlAbort :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlAbort thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlAbortStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlAbortStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Abort" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the wheel brakes.
 -}
getControlBrakes :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlBrakes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlBrakesStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlBrakesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Brakes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current stage of the vessel. Corresponds to the stage number in
 - the in-game UI.
 -}
getControlCurrentStage :: KRPCHS.SpaceCenter.Control -> RPCContext (Data.Int.Int32)
getControlCurrentStage thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlCurrentStageStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Data.Int.Int32))
getControlCurrentStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_CurrentStage" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the forward translational control.
 - A value between -1 and 1.
 - Equivalent to the h and n keys.
 -}
getControlForward :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlForward thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlForwardStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlForwardStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Forward" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the landing gear/legs.
 -}
getControlGear :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlGear thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlGearStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlGearStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Gear" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the lights.
 -}
getControlLights :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlLights thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlLightsStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlLightsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Lights" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns a list of all existing maneuver nodes, ordered by time from first to last.
 -}
getControlNodes :: KRPCHS.SpaceCenter.Control -> RPCContext ([KRPCHS.SpaceCenter.Node])
getControlNodes thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlNodesStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Node]))
getControlNodesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Nodes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the pitch control.
 - A value between -1 and 1.
 - Equivalent to the w and s keys.
 -}
getControlPitch :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlPitchStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Pitch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of RCS.
 -}
getControlRCS :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlRCSStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlRCSStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_RCS" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the right translational control.
 - A value between -1 and 1.
 - Equivalent to the j and l keys.
 -}
getControlRight :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlRight thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlRightStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlRightStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Right" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the roll control.
 - A value between -1 and 1.
 - Equivalent to the q and e keys.
 -}
getControlRoll :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlRollStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlRollStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Roll" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
getControlSAS :: KRPCHS.SpaceCenter.Control -> RPCContext (Bool)
getControlSAS thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSASStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Bool))
getControlSASStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SAS" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
getControlSASMode :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCHS.SpaceCenter.SASMode)
getControlSASMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSASModeStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SASMode))
getControlSASModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SASMode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
 - This is the mode displayed next to the speed at the top of the navball.
 -}
getControlSpeedMode :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCHS.SpaceCenter.SpeedMode)
getControlSpeedMode thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlSpeedModeStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SpeedMode))
getControlSpeedModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_SpeedMode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the throttle. A value between 0 and 1.
 -}
getControlThrottle :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlThrottle thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlThrottleStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlThrottleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Throttle" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the up translational control.
 - A value between -1 and 1.
 - Equivalent to the i and k keys.
 -}
getControlUp :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlUp thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlUpStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlUpStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Up" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the wheel steering.
 - A value between -1 and 1.
 - A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
getControlWheelSteering :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlWheelSteering thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlWheelSteeringStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlWheelSteeringStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelSteering" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getControlWheelThrottleStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlWheelThrottleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_WheelThrottle" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the yaw control.
 - A value between -1 and 1.
 - Equivalent to the a and d keys.
 -}
getControlYaw :: KRPCHS.SpaceCenter.Control -> RPCContext (Float)
getControlYaw thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getControlYawStream :: KRPCHS.SpaceCenter.Control -> RPCContext (KRPCStream (Float))
getControlYawStream thisArg = do
    let r = makeRequest "SpaceCenter" "Control_get_Yaw" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the abort action group.
 -}
setControlAbort :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext (Bool)
setControlAbort thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Abort" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the wheel brakes.
 -}
setControlBrakes :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext (Bool)
setControlBrakes thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Brakes" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the forward translational control.
 - A value between -1 and 1.
 - Equivalent to the h and n keys.
 -}
setControlForward :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlForward thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Forward" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the landing gear/legs.
 -}
setControlGear :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext (Bool)
setControlGear thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Gear" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the lights.
 -}
setControlLights :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext (Bool)
setControlLights thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Lights" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the pitch control.
 - A value between -1 and 1.
 - Equivalent to the w and s keys.
 -}
setControlPitch :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlPitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Pitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of RCS.
 -}
setControlRCS :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext (Bool)
setControlRCS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_RCS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the right translational control.
 - A value between -1 and 1.
 - Equivalent to the j and l keys.
 -}
setControlRight :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlRight thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Right" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the roll control.
 - A value between -1 and 1.
 - Equivalent to the q and e keys.
 -}
setControlRoll :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlRoll thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Roll" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of SAS.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SAS" />
 -}
setControlSAS :: KRPCHS.SpaceCenter.Control -> Bool -> RPCContext (Bool)
setControlSAS thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SAS" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The current <see cref="T:SpaceCenter.SASMode" />.
 - These modes are equivalent to the mode buttons to
 - the left of the navball that appear when SAS is enabled.Equivalent to <see cref="M:SpaceCenter.AutoPilot.SASMode" />
 -}
setControlSASMode :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SASMode -> RPCContext (Bool)
setControlSASMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SASMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The current <see cref="T:SpaceCenter.SpeedMode" /> of the navball.
 - This is the mode displayed next to the speed at the top of the navball.
 -}
setControlSpeedMode :: KRPCHS.SpaceCenter.Control -> KRPCHS.SpaceCenter.SpeedMode -> RPCContext (Bool)
setControlSpeedMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_SpeedMode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the throttle. A value between 0 and 1.
 -}
setControlThrottle :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Throttle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the up translational control.
 - A value between -1 and 1.
 - Equivalent to the i and k keys.
 -}
setControlUp :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlUp thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Up" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the wheel steering.
 - A value between -1 and 1.
 - A value of 1 steers to the left, and a value of -1 steers to the right.
 -}
setControlWheelSteering :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlWheelSteering thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelSteering" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the wheel throttle.
 - A value between -1 and 1.
 - A value of 1 rotates the wheels forwards, a value of -1 rotates
 - the wheels backwards.
 -}
setControlWheelThrottle :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlWheelThrottle thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_WheelThrottle" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the yaw control.
 - A value between -1 and 1.
 - Equivalent to the a and d keys.
 -}
setControlYaw :: KRPCHS.SpaceCenter.Control -> Float -> RPCContext (Bool)
setControlYaw thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Control_set_Yaw" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Fires the decoupler. Returns the new vessel created when the decoupler fires.
 - Throws an exception if the decoupler has already fired.
 -}
decouplerDecouple :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCHS.SpaceCenter.Vessel)
decouplerDecouple thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

decouplerDecoupleStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
decouplerDecoupleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_Decouple" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the decoupler has fired.
 -}
getDecouplerDecoupled :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Bool)
getDecouplerDecoupled thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDecouplerDecoupledStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Bool))
getDecouplerDecoupledStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Decoupled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The impulse that the decoupler imparts when it is fired, in Newton seconds.
 -}
getDecouplerImpulse :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (Float)
getDecouplerImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDecouplerImpulseStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (Float))
getDecouplerImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Impulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this decoupler.
 -}
getDecouplerPart :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCHS.SpaceCenter.Part)
getDecouplerPart thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDecouplerPartStream :: KRPCHS.SpaceCenter.Decoupler -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDecouplerPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Decoupler_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The direction that docking port points in, in the given reference frame.
 -}
dockingPortDirection :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

dockingPortDirectionStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position of the docking port in the given reference frame.
 -}
dockingPortPosition :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
dockingPortPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

dockingPortPositionStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
dockingPortPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rotation of the docking port, in the given reference frame.
 -}
dockingPortRotation :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
dockingPortRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

dockingPortRotationStream :: KRPCHS.SpaceCenter.DockingPort -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
dockingPortRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Undocks the docking port and returns the new <see cref="T:SpaceCenter.Vessel" /> that is created.
 - This method can be called for either docking port in a docked pair.
 - Throws an exception if the docking port is not docked to anything.After undocking, the active vessel may change. See <see cref="M:SpaceCenter.ActiveVessel" />.
 -}
dockingPortUndock :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Vessel)
dockingPortUndock thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

dockingPortUndockStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
dockingPortUndockStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_Undock" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part that this docking port is docked to. Returnsnullif this
 - docking port is not docked to anything.
 -}
getDockingPortDockedPart :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Part)
getDockingPortDockedPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDockingPortDockedPartStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortDockedPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_DockedPart" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the docking port has a shield.
 -}
getDockingPortHasShield :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Bool)
getDockingPortHasShield thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDockingPortHasShieldStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortHasShieldStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_HasShield" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getDockingPortNameStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Data.Text.Text))
getDockingPortNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this docking port.
 -}
getDockingPortPart :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.Part)
getDockingPortPart thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDockingPortPartStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getDockingPortPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The distance a docking port must move away when it undocks before it
 - becomes ready to dock with another port, in meters.
 -}
getDockingPortReengageDistance :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Float)
getDockingPortReengageDistance thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDockingPortReengageDistanceStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Float))
getDockingPortReengageDistanceStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReengageDistance" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getDockingPortReferenceFrameStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getDockingPortReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getDockingPortShieldedStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (Bool))
getDockingPortShieldedStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_Shielded" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current state of the docking port.
 -}
getDockingPortState :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCHS.SpaceCenter.DockingPortState)
getDockingPortState thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getDockingPortStateStream :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPortState))
getDockingPortStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The port name of the docking port. This is the name of the port that can be set
 - in the right click menu, when the
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/40423-11-docking-port-alignment-indicator-version-621-beta-updated-04122016/">Docking Port Alignment Indicatormod is installed. If this mod is not installed, returns the title of the part
 - (<see cref="M:SpaceCenter.Part.Title" />).
 -}
setDockingPortName :: KRPCHS.SpaceCenter.DockingPort -> Data.Text.Text -> RPCContext (Bool)
setDockingPortName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the docking ports shield, if it has one.
 - Returnstrueif the docking port has a shield, and the shield is
 - closed. Otherwise returnsfalse. When set totrue, the shield is
 - closed, and when set tofalsethe shield is opened. If the docking
 - port does not have a shield, setting this attribute has no effect.
 -}
setDockingPortShielded :: KRPCHS.SpaceCenter.DockingPort -> Bool -> RPCContext (Bool)
setDockingPortShielded thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "DockingPort_set_Shielded" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Toggle the current engine mode.
 -}
engineToggleMode :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
engineToggleMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_ToggleMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the engine is active. Setting this attribute may have no effect,
 - depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
getEngineActive :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineActive thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineActiveStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Active" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine will automatically switch modes.
 -}
getEngineAutoModeSwitch :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineAutoModeSwitch thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineAutoModeSwitchStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineAutoModeSwitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AutoModeSwitch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum available amount of thrust that can be produced by the
 - engine, in Newtons. This takes <see cref="M:SpaceCenter.Engine.ThrustLimit" /> into account,
 - and is the amount of thrust produced by the engine when activated and the
 - main throttle is set to 100%. Returns zero if the engine does not have any fuel.
 -}
getEngineAvailableThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineAvailableThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineAvailableThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the engine is inactive, or not gimballed.
 -}
getEngineAvailableTorque :: KRPCHS.SpaceCenter.Engine -> RPCContext ((Double, Double, Double))
getEngineAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineAvailableTorqueStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ((Double, Double, Double)))
getEngineAvailableTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_AvailableTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine can be restarted once shutdown. If the engine cannot be shutdown,
 - returnsfalse. For example, this istruefor liquid fueled rockets
 - andfalsefor solid rocket boosters.
 -}
getEngineCanRestart :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineCanRestart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineCanRestartStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineCanRestartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanRestart" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine can be shutdown once activated. For example, this istruefor liquid fueled rockets andfalsefor solid rocket boosters.
 -}
getEngineCanShutdown :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineCanShutdown thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineCanShutdownStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineCanShutdownStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_CanShutdown" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The gimbal limiter of the engine. A value between 0 and 1.
 - Returns 0 if the gimbal is locked.
 -}
getEngineGimbalLimit :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineGimbalLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineGimbalLimitStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalLimitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLimit" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engines gimbal is locked in place. Setting this attribute has
 - no effect if the engine is not gimballed.
 -}
getEngineGimbalLocked :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineGimbalLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineGimbalLockedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineGimbalLockedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalLocked" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The range over which the gimbal can move, in degrees.
 - Returns 0 if the engine is not gimballed.
 -}
getEngineGimbalRange :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineGimbalRange thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineGimbalRangeStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineGimbalRangeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_GimbalRange" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine is gimballed.
 -}
getEngineGimballed :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineGimballedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineGimballedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Gimballed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine has any fuel available.The engine must be activated for this property to update correctly.
 -}
getEngineHasFuel :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineHasFuelStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineHasFuelStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasFuel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine has multiple modes of operation.
 -}
getEngineHasModes :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineHasModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineHasModesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineHasModesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_HasModes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The specific impulse of the engine at sea level on Kerbin, in seconds.
 -}
getEngineKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineKerbinSeaLevelSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum amount of thrust that can be produced by the engine, in
 - Newtons. This is the amount of thrust produced by the engine when
 - activated, <see cref="M:SpaceCenter.Engine.ThrustLimit" /> is set to 100% and the main vessel's
 - throttle is set to 100%.
 -}
getEngineMaxThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineMaxThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineMaxThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getEngineMaxVacuumThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineMaxVacuumThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_MaxVacuumThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the current engine mode.
 -}
getEngineMode :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Text.Text)
getEngineMode thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineModeStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Text.Text))
getEngineModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Mode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The available modes for the engine.
 - A dictionary mapping mode names to <see cref="T:SpaceCenter.Engine" /> objects.
 -}
getEngineModes :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine))
getEngineModes thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineModesStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.Engine)))
getEngineModesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Modes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this engine.
 -}
getEnginePart :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCHS.SpaceCenter.Part)
getEnginePart thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEnginePartStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getEnginePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The ratios of resources that the engine consumes. A dictionary mapping resource names
 - to the ratios at which they are consumed by the engine.
 -}
getEnginePropellantRatios :: KRPCHS.SpaceCenter.Engine -> RPCContext (Data.Map.Map (Data.Text.Text) (Float))
getEnginePropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEnginePropellantRatiosStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getEnginePropellantRatiosStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_PropellantRatios" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The names of resources that the engine consumes.
 -}
getEnginePropellants :: KRPCHS.SpaceCenter.Engine -> RPCContext ([Data.Text.Text])
getEnginePropellants thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEnginePropellantsStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([Data.Text.Text]))
getEnginePropellantsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Propellants" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current specific impulse of the engine, in seconds. Returns zero
 - if the engine is not active.
 -}
getEngineSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_SpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getEngineThrottleStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrottleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Throttle" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the <see cref="M:SpaceCenter.Control.Throttle" /> affects the engine. For example,
 - this istruefor liquid fueled rockets, andfalsefor solid rocket
 - boosters.
 -}
getEngineThrottleLocked :: KRPCHS.SpaceCenter.Engine -> RPCContext (Bool)
getEngineThrottleLocked thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineThrottleLockedStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Bool))
getEngineThrottleLockedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrottleLocked" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current amount of thrust being produced by the engine, in
 - Newtons. Returns zero if the engine is not active or if it has no fuel.
 -}
getEngineThrust :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineThrustStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The thrust limiter of the engine. A value between 0 and 1. Setting this
 - attribute may have no effect, for example the thrust limit for a solid
 - rocket booster cannot be changed in flight.
 -}
getEngineThrustLimit :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineThrustLimit thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineThrustLimitStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineThrustLimitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_ThrustLimit" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getEngineThrustersStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getEngineThrustersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_Thrusters" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vacuum specific impulse of the engine, in seconds.
 -}
getEngineVacuumSpecificImpulse :: KRPCHS.SpaceCenter.Engine -> RPCContext (Float)
getEngineVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getEngineVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.Engine -> RPCContext (KRPCStream (Float))
getEngineVacuumSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Engine_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the engine is active. Setting this attribute may have no effect,
 - depending on <see cref="M:SpaceCenter.Engine.CanShutdown" /> and <see cref="M:SpaceCenter.Engine.CanRestart" />.
 -}
setEngineActive :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext (Bool)
setEngineActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the engine will automatically switch modes.
 -}
setEngineAutoModeSwitch :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext (Bool)
setEngineAutoModeSwitch thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_AutoModeSwitch" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The gimbal limiter of the engine. A value between 0 and 1.
 - Returns 0 if the gimbal is locked.
 -}
setEngineGimbalLimit :: KRPCHS.SpaceCenter.Engine -> Float -> RPCContext (Bool)
setEngineGimbalLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the engines gimbal is locked in place. Setting this attribute has
 - no effect if the engine is not gimballed.
 -}
setEngineGimbalLocked :: KRPCHS.SpaceCenter.Engine -> Bool -> RPCContext (Bool)
setEngineGimbalLocked thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_GimbalLocked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The name of the current engine mode.
 -}
setEngineMode :: KRPCHS.SpaceCenter.Engine -> Data.Text.Text -> RPCContext (Bool)
setEngineMode thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_Mode" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The thrust limiter of the engine. A value between 0 and 1. Setting this
 - attribute may have no effect, for example the thrust limit for a solid
 - rocket booster cannot be changed in flight.
 -}
setEngineThrustLimit :: KRPCHS.SpaceCenter.Engine -> Float -> RPCContext (Bool)
setEngineThrustLimit thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Engine_set_ThrustLimit" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Jettison the fairing. Has no effect if it has already been jettisoned.
 -}
fairingJettison :: KRPCHS.SpaceCenter.Fairing -> RPCContext (Bool)
fairingJettison thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_Jettison" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the fairing has been jettisoned.
 -}
getFairingJettisoned :: KRPCHS.SpaceCenter.Fairing -> RPCContext (Bool)
getFairingJettisoned thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFairingJettisonedStream :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCStream (Bool))
getFairingJettisonedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Jettisoned" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this fairing.
 -}
getFairingPart :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCHS.SpaceCenter.Part)
getFairingPart thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFairingPartStream :: KRPCHS.SpaceCenter.Fairing -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getFairingPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Fairing_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total aerodynamic forces acting on the vessel, as a vector pointing in the direction of the force, with its
 - magnitude equal to the strength of the force in Newtons.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightAerodynamicForce :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAerodynamicForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightAerodynamicForceStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAerodynamicForceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AerodynamicForce" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the pitch angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightAngleOfAttack :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightAngleOfAttack thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightAngleOfAttackStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightAngleOfAttackStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AngleOfAttack" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector pointing in the anti-normal direction.
 -}
getFlightAntiNormal :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAntiNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightAntiNormalStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiNormalStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiNormal" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector pointing in the anti-radial direction.
 -}
getFlightAntiRadial :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightAntiRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightAntiRadialStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightAntiRadialStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AntiRadial" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current density of the atmosphere around the vessel, inkg/m^3.
 -}
getFlightAtmosphereDensity :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightAtmosphereDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightAtmosphereDensityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightAtmosphereDensityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_AtmosphereDensity" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the <a href="https://en.wikipedia.org/wiki/Ballistic_coefficient">ballistic coefficient.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightBallisticCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightBallisticCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightBallisticCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightBallisticCoefficientStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BallisticCoefficient" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The altitude above the surface of the body, in meters. When over water, this is the altitude above the sea floor.
 - Measured from the center of mass of the vessel.
 -}
getFlightBedrockAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightBedrockAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightBedrockAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightBedrockAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_BedrockAltitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position of the center of mass of the vessel.
 -}
getFlightCenterOfMass :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightCenterOfMass thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightCenterOfMassStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightCenterOfMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_CenterOfMass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The direction vector that the vessel is pointing in.
 -}
getFlightDirection :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightDirection thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightDirectionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDirectionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Direction" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic dragcurrently acting on the vessel,
 - as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightDrag :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightDrag thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightDragStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightDragStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Drag" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the coefficient of drag. This is the amount of drag produced by the vessel. It depends on air speed,
 - air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightDragCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightDragCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightDragCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightDragCoefficientStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DragCoefficient" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getFlightDynamicPressureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightDynamicPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_DynamicPressure" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The elevation of the terrain under the vessel, in meters. This is the height of the terrain above sea level,
 - and is negative when the vessel is over the sea.
 -}
getFlightElevation :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightElevation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightElevationStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightElevationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Elevation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Equivalent_airspeed">equivalent air speedof the vessel, inm/s.Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightEquivalentAirSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightEquivalentAirSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightEquivalentAirSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightEquivalentAirSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_EquivalentAirSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current G force acting on the vessel inm/s^2.
 -}
getFlightGForce :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightGForce thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightGForceStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightGForceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_GForce" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The heading angle of the vessel relative to north, in degrees. A value between 0° and 360°.
 -}
getFlightHeading :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightHeading thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightHeadingStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightHeadingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Heading" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The horizontal speed of the vessel in meters per second.
 -}
getFlightHorizontalSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightHorizontalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightHorizontalSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightHorizontalSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_HorizontalSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Latitude">latitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLatitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightLatitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightLatitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightLatitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Latitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Aerodynamic_force">aerodynamic liftcurrently acting on the vessel,
 - as a vector pointing in the direction of the force, with its magnitude equal to the strength of the force in Newtons.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightLift :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightLift thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightLiftStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightLiftStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Lift" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the coefficient of lift. This is the amount of lift produced by the vessel, and depends on air speed, air density and wing area.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightLiftCoefficient :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightLiftCoefficient thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightLiftCoefficientStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightLiftCoefficientStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_LiftCoefficient" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Longitude">longitudeof the vessel for the body being orbited, in degrees.
 -}
getFlightLongitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightLongitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightLongitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightLongitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Longitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The speed of the vessel, in multiples of the speed of sound.Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightMach :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightMach thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightMachStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightMachStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Mach" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The altitude above sea level, in meters.
 - Measured from the center of mass of the vessel.
 -}
getFlightMeanAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightMeanAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightMeanAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightMeanAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_MeanAltitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector pointing in the normal direction.
 -}
getFlightNormal :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightNormalStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightNormalStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Normal" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The pitch angle of the vessel relative to the horizon, in degrees. A value between -90° and +90°.
 -}
getFlightPitch :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightPitch thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightPitchStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightPitchStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Pitch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector pointing in the prograde direction.
 -}
getFlightPrograde :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightPrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightProgradeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightProgradeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Prograde" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector pointing in the radial direction.
 -}
getFlightRadial :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightRadialStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRadialStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Radial" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector pointing in the retrograde direction.
 -}
getFlightRetrograde :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightRetrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightRetrogradeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightRetrogradeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Retrograde" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The roll angle of the vessel relative to the horizon, in degrees. A value between -180° and +180°.
 -}
getFlightRoll :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightRoll thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightRollStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightRollStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Roll" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rotation of the vessel.
 -}
getFlightRotation :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double, Double))
getFlightRotation thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightRotationStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getFlightRotationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Rotation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the yaw angle between the orientation of the vessel and its velocity vector, in degrees.
 -}
getFlightSideslipAngle :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightSideslipAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightSideslipAngleStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightSideslipAngleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SideslipAngle" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The speed of the vessel in meters per second.
 -}
getFlightSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Speed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The speed of sound, in the atmosphere around the vessel, inm/s.Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightSpeedOfSound :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightSpeedOfSound thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightSpeedOfSoundStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightSpeedOfSoundStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SpeedOfSound" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the current amount of stall, between 0 and 1. A value greater than 0.005 indicates a minor stall
 - and a value greater than 0.5 indicates a large-scale stall.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightStallFraction :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStallFraction thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightStallFractionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStallFractionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StallFraction" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">static (ambient) temperatureof the
 - atmosphere around the vessel, in Kelvin.
 -}
getFlightStaticAirTemperature :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightStaticAirTemperatureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticAirTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticAirTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The static atmospheric pressure acting on the vessel, in Pascals.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model.
 - Not available when <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFlightStaticPressure :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightStaticPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightStaticPressureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightStaticPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_StaticPressure" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The altitude above the surface of the body or sea level, whichever is closer, in meters.
 - Measured from the center of mass of the vessel.
 -}
getFlightSurfaceAltitude :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightSurfaceAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightSurfaceAltitudeStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightSurfaceAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_SurfaceAltitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - An estimate of the current terminal velocity of the vessel, inm/s.
 - This is the speed at which the drag forces cancel out the force of gravity.Calculated using <a href="http://wiki.kerbalspaceprogram.com/wiki/Atmosphere">KSPs stock aerodynamic model, or
 - <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchif it is installed.
 -}
getFlightTerminalVelocity :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTerminalVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightTerminalVelocityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTerminalVelocityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TerminalVelocity" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the thrust specific fuel consumption for the jet engines on the vessel. This is a measure of the
 - efficiency of the engines, with a lower value indicating a more efficient vessel. This value is the
 - number of Newtons of fuel that are burned, per hour, to product one newton of thrust.Requires <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Research.
 -}
getFlightThrustSpecificFuelConsumption :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightThrustSpecificFuelConsumption thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightThrustSpecificFuelConsumptionStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightThrustSpecificFuelConsumptionStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_ThrustSpecificFuelConsumption" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Total_air_temperature">total air temperatureof the atmosphere
 - around the vessel, in Kelvin. This temperature includes the <see cref="M:SpaceCenter.Flight.StaticAirTemperature" /> and the vessel's kinetic energy.
 -}
getFlightTotalAirTemperature :: KRPCHS.SpaceCenter.Flight -> RPCContext (Float)
getFlightTotalAirTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightTotalAirTemperatureStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Float))
getFlightTotalAirTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_TotalAirTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The velocity vector of the vessel. The magnitude of the vector is the speed of the vessel in meters per second.
 - The direction of the vector is the direction of the vessels motion.
 -}
getFlightVelocity :: KRPCHS.SpaceCenter.Flight -> RPCContext ((Double, Double, Double))
getFlightVelocity thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightVelocityStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream ((Double, Double, Double)))
getFlightVelocityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_Velocity" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vertical speed of the vessel in meters per second.
 -}
getFlightVerticalSpeed :: KRPCHS.SpaceCenter.Flight -> RPCContext (Double)
getFlightVerticalSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getFlightVerticalSpeedStream :: KRPCHS.SpaceCenter.Flight -> RPCContext (KRPCStream (Double))
getFlightVerticalSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Flight_get_VerticalSpeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The area of the intake's opening, in square meters.
 -}
getIntakeArea :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeArea thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getIntakeAreaStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeAreaStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Area" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rate of flow into the intake, in units of resource per second.
 -}
getIntakeFlow :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeFlow thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getIntakeFlowStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeFlowStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Flow" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the intake is open.
 -}
getIntakeOpen :: KRPCHS.SpaceCenter.Intake -> RPCContext (Bool)
getIntakeOpen thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getIntakeOpenStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Bool))
getIntakeOpenStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Open" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this intake.
 -}
getIntakePart :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCHS.SpaceCenter.Part)
getIntakePart thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getIntakePartStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getIntakePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Speed of the flow into the intake, inm/s.
 -}
getIntakeSpeed :: KRPCHS.SpaceCenter.Intake -> RPCContext (Float)
getIntakeSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getIntakeSpeedStream :: KRPCHS.SpaceCenter.Intake -> RPCContext (KRPCStream (Float))
getIntakeSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Intake_get_Speed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the intake is open.
 -}
setIntakeOpen :: KRPCHS.SpaceCenter.Intake -> Bool -> RPCContext (Bool)
setIntakeOpen thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Intake_set_Open" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the landing gear is deployable.
 -}
getLandingGearDeployable :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (Bool)
getLandingGearDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingGearDeployableStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployableStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployable" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the landing gear is deployed.Fixed landing gear are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
getLandingGearDeployed :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (Bool)
getLandingGearDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingGearDeployedStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (Bool))
getLandingGearDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this landing gear.
 -}
getLandingGearPart :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCHS.SpaceCenter.Part)
getLandingGearPart thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingGearPartStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLandingGearPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the current state of the landing gear.Fixed landing gear are always deployed.
 -}
getLandingGearState :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCHS.SpaceCenter.LandingGearState)
getLandingGearState thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingGearStateStream :: KRPCHS.SpaceCenter.LandingGear -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingGearState))
getLandingGearStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the landing gear is deployed.Fixed landing gear are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
setLandingGearDeployed :: KRPCHS.SpaceCenter.LandingGear -> Bool -> RPCContext (Bool)
setLandingGearDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "LandingGear_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the landing leg is deployed.Fixed landing legs are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
getLandingLegDeployed :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (Bool)
getLandingLegDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingLegDeployedStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (Bool))
getLandingLegDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this landing leg.
 -}
getLandingLegPart :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCHS.SpaceCenter.Part)
getLandingLegPart thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingLegPartStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLandingLegPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current state of the landing leg.
 -}
getLandingLegState :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCHS.SpaceCenter.LandingLegState)
getLandingLegState thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLandingLegStateStream :: KRPCHS.SpaceCenter.LandingLeg -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingLegState))
getLandingLegStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the landing leg is deployed.Fixed landing legs are always deployed.
 - Returns an error if you try to deploy fixed landing gear.
 -}
setLandingLegDeployed :: KRPCHS.SpaceCenter.LandingLeg -> Bool -> RPCContext (Bool)
setLandingLegDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "LandingLeg_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Releases the docking clamp. Has no effect if the clamp has already been released.
 -}
launchClampRelease :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (Bool)
launchClampRelease thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_Release" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The part object for this launch clamp.
 -}
getLaunchClampPart :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (KRPCHS.SpaceCenter.Part)
getLaunchClampPart thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLaunchClampPartStream :: KRPCHS.SpaceCenter.LaunchClamp -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLaunchClampPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "LaunchClamp_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Launch a new vessel from the SPH onto the runway.<param name="name">Name of the vessel's craft file.
 -}
launchVesselFromSPH :: Data.Text.Text -> RPCContext (Bool)
launchVesselFromSPH nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromSPH" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Launch a new vessel from the VAB onto the launchpad.<param name="name">Name of the vessel's craft file.
 -}
launchVesselFromVAB :: Data.Text.Text -> RPCContext (Bool)
launchVesselFromVAB nameArg = do
    let r = makeRequest "SpaceCenter" "LaunchVesselFromVAB" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the light is switched on.
 -}
getLightActive :: KRPCHS.SpaceCenter.Light -> RPCContext (Bool)
getLightActive thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLightActiveStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (Bool))
getLightActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Active" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The color of the light, as an RGB triple.
 -}
getLightColor :: KRPCHS.SpaceCenter.Light -> RPCContext ((Float, Float, Float))
getLightColor thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLightColorStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream ((Float, Float, Float)))
getLightColorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Color" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this light.
 -}
getLightPart :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCHS.SpaceCenter.Part)
getLightPart thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLightPartStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getLightPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current power usage, in units of charge per second.
 -}
getLightPowerUsage :: KRPCHS.SpaceCenter.Light -> RPCContext (Float)
getLightPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLightPowerUsageStream :: KRPCHS.SpaceCenter.Light -> RPCContext (KRPCStream (Float))
getLightPowerUsageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Light_get_PowerUsage" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the light is switched on.
 -}
setLightActive :: KRPCHS.SpaceCenter.Light -> Bool -> RPCContext (Bool)
setLightActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The color of the light, as an RGB triple.
 -}
setLightColor :: KRPCHS.SpaceCenter.Light -> (Float, Float, Float) -> RPCContext (Bool)
setLightColor thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Light_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Load the game with the given name.
 - This will create a load a save file calledname.sfsfrom the folder of the current save game.
 -}
load :: Data.Text.Text -> RPCContext (Bool)
load nameArg = do
    let r = makeRequest "SpaceCenter" "Load" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Returns the value of a field.<param name="name">Name of the field.
 -}
moduleGetField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Data.Text.Text)
moduleGetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

moduleGetFieldStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Data.Text.Text))
moduleGetFieldStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_GetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - trueif the part has an action with the given name.<param name="name">
 -}
moduleHasAction :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasAction thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

moduleHasActionStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasActionStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasAction" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - trueif the module has an event with the given name.<param name="name">
 -}
moduleHasEvent :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

moduleHasEventStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasEventStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returnstrueif the module has a field with the given name.<param name="name">Name of the field.
 -}
moduleHasField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleHasField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

moduleHasFieldStream :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
moduleHasFieldStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_HasField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the value of a field to its original value.<param name="name">Name of the field.
 -}
moduleResetField :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleResetField thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_ResetField" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the value of an action with the given name.<param name="name"><param name="value">
 -}
moduleSetAction :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Bool -> RPCContext (Bool)
moduleSetAction thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetAction" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the value of a field to the given floating point number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldFloat :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Float -> RPCContext (Bool)
moduleSetFieldFloat thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldFloat" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the value of a field to the given integer number.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldInt :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Int.Int32 -> RPCContext (Bool)
moduleSetFieldInt thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldInt" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the value of a field to the given string.<param name="name">Name of the field.<param name="value">Value to set.
 -}
moduleSetFieldString :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> Data.Text.Text -> RPCContext (Bool)
moduleSetFieldString thisArg nameArg valueArg = do
    let r = makeRequest "SpaceCenter" "Module_SetFieldString" [makeArgument 0 thisArg, makeArgument 1 nameArg, makeArgument 2 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Trigger the named event. Equivalent to clicking the button in the right-click menu of the part.<param name="name">
 -}
moduleTriggerEvent :: KRPCHS.SpaceCenter.Module -> Data.Text.Text -> RPCContext (Bool)
moduleTriggerEvent thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Module_TriggerEvent" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - A list of all the names of the modules actions. These are the parts actions that can be assigned
 - to action groups in the in-game editor.
 -}
getModuleActions :: KRPCHS.SpaceCenter.Module -> RPCContext ([Data.Text.Text])
getModuleActions thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getModuleActionsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream ([Data.Text.Text]))
getModuleActionsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Actions" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of the names of all of the modules events. Events are the clickable buttons
 - visible in the right-click menu of the part.
 -}
getModuleEvents :: KRPCHS.SpaceCenter.Module -> RPCContext ([Data.Text.Text])
getModuleEvents thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getModuleEventsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream ([Data.Text.Text]))
getModuleEventsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Events" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The modules field names and their associated values, as a dictionary.
 - These are the values visible in the right-click menu of the part.
 -}
getModuleFields :: KRPCHS.SpaceCenter.Module -> RPCContext (Data.Map.Map (Data.Text.Text) (Data.Text.Text))
getModuleFields thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getModuleFieldsStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Data.Text.Text)))
getModuleFieldsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Fields" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Name of the PartModule. For example, "ModuleEngines".
 -}
getModuleName :: KRPCHS.SpaceCenter.Module -> RPCContext (Data.Text.Text)
getModuleName thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getModuleNameStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (Data.Text.Text))
getModuleNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part that contains this module.
 -}
getModulePart :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCHS.SpaceCenter.Part)
getModulePart thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getModulePartStream :: KRPCHS.SpaceCenter.Module -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getModulePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Module_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
 - is the delta-v of the burn in m/s.<param name="referenceFrame">Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingBurnVector" />.
 -}
nodeBurnVector :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

nodeBurnVectorStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeBurnVectorStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_BurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the unit direction vector of the maneuver nodes burn in the given reference frame.<param name="referenceFrame">
 -}
nodeDirection :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

nodeDirectionStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the position vector of the maneuver node in the given reference frame.<param name="referenceFrame">
 -}
nodePosition :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodePosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

nodePositionStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodePositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns a vector whose direction the direction of the maneuver node burn, and whose magnitude
 - is the delta-v of the burn in m/s. The direction and magnitude change as the burn is executed.<param name="referenceFrame">
 -}
nodeRemainingBurnVector :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
nodeRemainingBurnVector thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

nodeRemainingBurnVectorStream :: KRPCHS.SpaceCenter.Node -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
nodeRemainingBurnVectorStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Node_RemainingBurnVector" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Removes the maneuver node.
 -}
nodeRemove :: KRPCHS.SpaceCenter.Node -> RPCContext (Bool)
nodeRemove thisArg = do
    let r = makeRequest "SpaceCenter" "Node_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
getNodeDeltaV :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeDeltaVStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeDeltaVStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_DeltaV" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
getNodeNormal :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeNormal thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeNormalStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeNormalStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Normal" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The orbit that results from executing the maneuver node.
 -}
getNodeOrbit :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getNodeOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeOrbitStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getNodeOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Orbit" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getNodeOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeOrbitalReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
getNodePrograde :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodePrograde thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeProgradeStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeProgradeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Prograde" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
getNodeRadial :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeRadial thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeRadialStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeRadialStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_Radial" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the reference frame that is fixed relative to the maneuver node's burn.
 - <list type="bullet">The origin is at the position of the maneuver node.The y-axis points in the direction of the burn.The x-axis and z-axis point in arbitrary but fixed directions.
 -}
getNodeReferenceFrame :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getNodeReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeReferenceFrameStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getNodeReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the remaining delta-v of the maneuver node, in meters per second. Changes as the node
 - is executed. This is equivalent to the delta-v reported in-game.
 -}
getNodeRemainingDeltaV :: KRPCHS.SpaceCenter.Node -> RPCContext (Float)
getNodeRemainingDeltaV thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeRemainingDeltaVStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Float))
getNodeRemainingDeltaVStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_RemainingDeltaV" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time until the maneuver node will be encountered, in seconds.
 -}
getNodeTimeTo :: KRPCHS.SpaceCenter.Node -> RPCContext (Double)
getNodeTimeTo thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeTimeToStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Double))
getNodeTimeToStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_TimeTo" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The universal time at which the maneuver will occur, in seconds.
 -}
getNodeUT :: KRPCHS.SpaceCenter.Node -> RPCContext (Double)
getNodeUT thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getNodeUTStream :: KRPCHS.SpaceCenter.Node -> RPCContext (KRPCStream (Double))
getNodeUTStream thisArg = do
    let r = makeRequest "SpaceCenter" "Node_get_UT" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The delta-v of the maneuver node, in meters per second.Does not change when executing the maneuver node. See <see cref="M:SpaceCenter.Node.RemainingDeltaV" />.
 -}
setNodeDeltaV :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext (Bool)
setNodeDeltaV thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_DeltaV" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The magnitude of the maneuver nodes delta-v in the normal direction, in meters per second.
 -}
setNodeNormal :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext (Bool)
setNodeNormal thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Normal" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The magnitude of the maneuver nodes delta-v in the prograde direction, in meters per second.
 -}
setNodePrograde :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext (Bool)
setNodePrograde thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Prograde" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The magnitude of the maneuver nodes delta-v in the radial direction, in meters per second.
 -}
setNodeRadial :: KRPCHS.SpaceCenter.Node -> Float -> RPCContext (Bool)
setNodeRadial thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_Radial" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The universal time at which the maneuver will occur, in seconds.
 -}
setNodeUT :: KRPCHS.SpaceCenter.Node -> Double -> RPCContext (Bool)
setNodeUT thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Node_set_UT" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The unit direction vector from which the orbits longitude of ascending node is measured,
 - in the given reference frame.<param name="referenceFrame">
 -}
orbitReferencePlaneDirection :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitReferencePlaneDirection referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

orbitReferencePlaneDirectionStream :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneDirectionStream referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneDirection" [makeArgument 0 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The unit direction vector that is normal to the orbits reference plane, in the given
 - reference frame. The reference plane is the plane from which the orbits inclination is measured.<param name="referenceFrame">
 -}
orbitReferencePlaneNormal :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
orbitReferencePlaneNormal referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

orbitReferencePlaneNormalStream :: KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
orbitReferencePlaneNormalStream referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Orbit_ReferencePlaneNormal" [makeArgument 0 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the apoapsis of the orbit, in meters, from the center of mass of the body being orbited.For the apoapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.ApoapsisAltitude" />.
 -}
getOrbitApoapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitApoapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Apoapsis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The apoapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Apoapsis" /> minus the equatorial radius of the body.
 -}
getOrbitApoapsisAltitude :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitApoapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitApoapsisAltitudeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitApoapsisAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ApoapsisAltitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Argument_of_periapsis">argument of periapsis, in radians.
 -}
getOrbitArgumentOfPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitArgumentOfPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitArgumentOfPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitArgumentOfPeriapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_ArgumentOfPeriapsis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The celestial body (e.g. planet or moon) around which the object is orbiting.
 -}
getOrbitBody :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getOrbitBody thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitBodyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getOrbitBodyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Body" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Eccentric_anomaly">eccentric anomaly.
 -}
getOrbitEccentricAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEccentricAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitEccentricAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricAnomalyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_EccentricAnomaly" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Orbital_eccentricity">eccentricityof the orbit.
 -}
getOrbitEccentricity :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEccentricity thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitEccentricityStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEccentricityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Eccentricity" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time since the epoch (the point at which the
 - <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epochwas measured, in seconds.
 -}
getOrbitEpoch :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitEpochStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitEpochStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Epoch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Orbital_inclination">inclinationof the orbit,
 - in radians.
 -}
getOrbitInclination :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitInclination thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitInclinationStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitInclinationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Inclination" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Longitude_of_the_ascending_node">longitude of the
 - ascending node, in radians.
 -}
getOrbitLongitudeOfAscendingNode :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitLongitudeOfAscendingNode thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitLongitudeOfAscendingNodeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitLongitudeOfAscendingNodeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_LongitudeOfAscendingNode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly.
 -}
getOrbitMeanAnomaly :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitMeanAnomaly thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitMeanAnomalyStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomaly" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <a href="https://en.wikipedia.org/wiki/Mean_anomaly">mean anomaly at epoch.
 -}
getOrbitMeanAnomalyAtEpoch :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitMeanAnomalyAtEpoch thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitMeanAnomalyAtEpochStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitMeanAnomalyAtEpochStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_MeanAnomalyAtEpoch" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - If the object is going to change sphere of influence in the future, returns the new orbit
 - after the change. Otherwise returnsnull.
 -}
getOrbitNextOrbit :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getOrbitNextOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitNextOrbitStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getOrbitNextOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_NextOrbit" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The periapsis of the orbit, in meters, from the center of mass of the body being orbited.For the periapsis altitude reported on the in-game map view, use <see cref="M:SpaceCenter.Orbit.PeriapsisAltitude" />.
 -}
getOrbitPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Periapsis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The periapsis of the orbit, in meters, above the sea level of the body being orbited.This is equal to <see cref="M:SpaceCenter.Orbit.Periapsis" /> minus the equatorial radius of the body.
 -}
getOrbitPeriapsisAltitude :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriapsisAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitPeriapsisAltitudeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriapsisAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_PeriapsisAltitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The orbital period, in seconds.
 -}
getOrbitPeriod :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitPeriod thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitPeriodStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitPeriodStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Period" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current radius of the orbit, in meters. This is the distance between the center
 - of mass of the object in orbit, and the center of mass of the body around which it is orbiting.This value will change over time if the orbit is elliptical.
 -}
getOrbitRadius :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitRadius thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitRadiusStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitRadiusStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Radius" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The semi-major axis of the orbit, in meters.
 -}
getOrbitSemiMajorAxis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSemiMajorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitSemiMajorAxisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMajorAxisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMajorAxis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The semi-minor axis of the orbit, in meters.
 -}
getOrbitSemiMinorAxis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSemiMinorAxis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitSemiMinorAxisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSemiMinorAxisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_SemiMinorAxis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current orbital speed of the object in meters per second.This value will change over time if the orbit is elliptical.
 -}
getOrbitSpeed :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitSpeed thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitSpeedStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitSpeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_Speed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time until the object reaches apoapsis, in seconds.
 -}
getOrbitTimeToApoapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToApoapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitTimeToApoapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToApoapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToApoapsis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time until the object reaches periapsis, in seconds.
 -}
getOrbitTimeToPeriapsis :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToPeriapsis thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitTimeToPeriapsisStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToPeriapsisStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToPeriapsis" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The time until the object changes sphere of influence, in seconds. ReturnsNaNif the
 - object is not going to change sphere of influence.
 -}
getOrbitTimeToSOIChange :: KRPCHS.SpaceCenter.Orbit -> RPCContext (Double)
getOrbitTimeToSOIChange thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getOrbitTimeToSOIChangeStream :: KRPCHS.SpaceCenter.Orbit -> RPCContext (KRPCStream (Double))
getOrbitTimeToSOIChangeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Orbit_get_TimeToSOIChange" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Deploys the parachute. This has no effect if the parachute has already
 - been deployed.
 -}
parachuteDeploy :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Bool)
parachuteDeploy thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_Deploy" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The altitude at which the parachute will full deploy, in meters.
 -}
getParachuteDeployAltitude :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Float)
getParachuteDeployAltitude thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getParachuteDeployAltitudeStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployAltitudeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployAltitude" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 -}
getParachuteDeployMinPressure :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Float)
getParachuteDeployMinPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getParachuteDeployMinPressureStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Float))
getParachuteDeployMinPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_DeployMinPressure" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the parachute has been deployed.
 -}
getParachuteDeployed :: KRPCHS.SpaceCenter.Parachute -> RPCContext (Bool)
getParachuteDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getParachuteDeployedStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (Bool))
getParachuteDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this parachute.
 -}
getParachutePart :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCHS.SpaceCenter.Part)
getParachutePart thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getParachutePartStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getParachutePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current state of the parachute.
 -}
getParachuteState :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCHS.SpaceCenter.ParachuteState)
getParachuteState thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getParachuteStateStream :: KRPCHS.SpaceCenter.Parachute -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ParachuteState))
getParachuteStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parachute_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The altitude at which the parachute will full deploy, in meters.
 -}
setParachuteDeployAltitude :: KRPCHS.SpaceCenter.Parachute -> Float -> RPCContext (Bool)
setParachuteDeployAltitude thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployAltitude" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The minimum pressure at which the parachute will semi-deploy, in atmospheres.
 -}
setParachuteDeployMinPressure :: KRPCHS.SpaceCenter.Parachute -> Float -> RPCContext (Bool)
setParachuteDeployMinPressure thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parachute_set_DeployMinPressure" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The position of the parts center of mass in the given reference frame.
 - If the part is physicsless, this is equivalent to <see cref="M:SpaceCenter.Part.Position" />.<param name="referenceFrame">
 -}
partCenterOfMass :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partCenterOfMass thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

partCenterOfMassStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partCenterOfMassStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_CenterOfMass" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The direction of the part in the given reference frame.<param name="referenceFrame">
 -}
partDirection :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

partDirectionStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position of the part in the given reference frame.This is a fixed position in the part, defined by the parts model.
 - It s not necessarily the same as the parts center of mass.
 - Use <see cref="M:SpaceCenter.Part.CenterOfMass" /> to get the parts center of mass.<param name="referenceFrame">
 -}
partPosition :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

partPositionStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rotation of the part in the given reference frame.<param name="referenceFrame">
 -}
partRotation :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
partRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

partRotationStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
partRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The velocity of the part in the given reference frame.<param name="referenceFrame">
 -}
partVelocity :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
partVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

partVelocityStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
partVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Part_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the part is axially attached to its parent, i.e. on the top
 - or bottom of its parent. If the part has no parent, returnsfalse.
 -}
getPartAxiallyAttached :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartAxiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartAxiallyAttachedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartAxiallyAttachedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_AxiallyAttached" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.CargoBay" /> if the part is a cargo bay, otherwisenull.
 -}
getPartCargoBay :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.CargoBay)
getPartCargoBay thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartCargoBayStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.CargoBay))
getPartCargoBayStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CargoBay" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The reference frame that is fixed relative to this part, and centered on its center of mass.
 - <list type="bullet">The origin is at the center of mass of the part, as returned by <see cref="M:SpaceCenter.Part.CenterOfMass" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
 - for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartCenterOfMassReferenceFrame :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPartCenterOfMassReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartCenterOfMassReferenceFrameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartCenterOfMassReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_CenterOfMassReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The parts children. Returns an empty list if the part has no children.
 - This, in combination with <see cref="M:SpaceCenter.Part.Parent" />, can be used to traverse the vessels parts tree.
 -}
getPartChildren :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartChildren thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartChildrenStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartChildrenStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Children" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.ControlSurface" /> if the part is an aerodynamic control surface, otherwisenull.
 -}
getPartControlSurface :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ControlSurface)
getPartControlSurface thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartControlSurfaceStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ControlSurface))
getPartControlSurfaceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ControlSurface" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The cost of the part, in units of funds.
 -}
getPartCost :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartCost thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartCostStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartCostStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Cost" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether this part is crossfeed capable.
 -}
getPartCrossfeed :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartCrossfeed thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartCrossfeedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartCrossfeedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Crossfeed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The stage in which this part will be decoupled. Returns -1 if the part is never decoupled from the vessel.
 -}
getPartDecoupleStage :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Int.Int32)
getPartDecoupleStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartDecoupleStageStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Int.Int32))
getPartDecoupleStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DecoupleStage" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Decoupler" /> if the part is a decoupler, otherwisenull.
 -}
getPartDecoupler :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Decoupler)
getPartDecoupler thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartDecouplerStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Decoupler))
getPartDecouplerStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Decoupler" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.DockingPort" /> if the part is a docking port, otherwisenull.
 -}
getPartDockingPort :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.DockingPort)
getPartDockingPort thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartDockingPortStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getPartDockingPortStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DockingPort" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The mass of the part, not including any resources it contains, in kilograms. Returns zero if the part is massless.
 -}
getPartDryMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartDryMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartDryMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DryMass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The dynamic pressure acting on the part, in Pascals.
 -}
getPartDynamicPressure :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartDynamicPressure thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartDynamicPressureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartDynamicPressureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_DynamicPressure" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - An <see cref="T:SpaceCenter.Engine" /> if the part is an engine, otherwisenull.
 -}
getPartEngine :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Engine)
getPartEngine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartEngineStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Engine))
getPartEngineStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Engine" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Fairing" /> if the part is a fairing, otherwisenull.
 -}
getPartFairing :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Fairing)
getPartFairing thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartFairingStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Fairing))
getPartFairingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Fairing" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The parts that are connected to this part via fuel lines, where the direction of the fuel line is into this part.
 -}
getPartFuelLinesFrom :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesFrom thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartFuelLinesFromStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesFromStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesFrom" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The parts that are connected to this part via fuel lines, where the direction of the fuel line is out of this part.
 -}
getPartFuelLinesTo :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartFuelLinesTo thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartFuelLinesToStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartFuelLinesToStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_FuelLinesTo" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The impact tolerance of the part, in meters per second.
 -}
getPartImpactTolerance :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartImpactTolerance thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartImpactToleranceStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartImpactToleranceStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ImpactTolerance" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The inertia tensor of the part in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 - Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getPartInertiaTensor :: KRPCHS.SpaceCenter.Part -> RPCContext ([Double])
getPartInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartInertiaTensorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([Double]))
getPartInertiaTensorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_InertiaTensor" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - An <see cref="T:SpaceCenter.Intake" /> if the part is an intake, otherwisenull.This includes any part that generates thrust. This covers many different types of engine,
 - including liquid fuel rockets, solid rocket boosters and jet engines.
 - For RCS thrusters see <see cref="T:SpaceCenter.RCS" />.
 -}
getPartIntake :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Intake)
getPartIntake thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartIntakeStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Intake))
getPartIntakeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Intake" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether this part is a fuel line.
 -}
getPartIsFuelLine :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartIsFuelLine thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartIsFuelLineStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartIsFuelLineStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_IsFuelLine" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.LandingGear" /> if the part is a landing gear, otherwisenull.
 -}
getPartLandingGear :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LandingGear)
getPartLandingGear thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingGear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartLandingGearStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingGear))
getPartLandingGearStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingGear" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.LandingLeg" /> if the part is a landing leg, otherwisenull.
 -}
getPartLandingLeg :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LandingLeg)
getPartLandingLeg thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingLeg" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartLandingLegStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LandingLeg))
getPartLandingLegStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LandingLeg" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.LaunchClamp" /> if the part is a launch clamp, otherwisenull.
 -}
getPartLaunchClamp :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.LaunchClamp)
getPartLaunchClamp thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartLaunchClampStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.LaunchClamp))
getPartLaunchClampStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_LaunchClamp" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Light" /> if the part is a light, otherwisenull.
 -}
getPartLight :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Light)
getPartLight thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartLightStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Light))
getPartLightStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Light" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current mass of the part, including resources it contains, in kilograms.
 - Returns zero if the part is massless.
 -}
getPartMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Mass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the part is <a href="http://wiki.kerbalspaceprogram.com/wiki/Massless_part">massless.
 -}
getPartMassless :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartMassless thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartMasslessStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartMasslessStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Massless" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Maximum temperature that the skin of the part can survive, in Kelvin.
 -}
getPartMaxSkinTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMaxSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartMaxSkinTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMaxSkinTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxSkinTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Maximum temperature that the part can survive, in Kelvin.
 -}
getPartMaxTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartMaxTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartMaxTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartMaxTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MaxTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The modules for this part.
 -}
getPartModules :: KRPCHS.SpaceCenter.Part -> RPCContext ([KRPCHS.SpaceCenter.Module])
getPartModules thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartModulesStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Module]))
getPartModulesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Modules" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The moment of inertia of the part inkg.m^2around its center of mass
 - in the parts reference frame (<see cref="T:SpaceCenter.ReferenceFrame" />).
 -}
getPartMomentOfInertia :: KRPCHS.SpaceCenter.Part -> RPCContext ((Double, Double, Double))
getPartMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartMomentOfInertiaStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream ((Double, Double, Double)))
getPartMomentOfInertiaStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_MomentOfInertia" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Internal name of the part, as used in
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/CFG_File_Documentation">part cfg files.
 - For example "Mark1-2Pod".
 -}
getPartName :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartName thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartNameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Parachute" /> if the part is a parachute, otherwisenull.
 -}
getPartParachute :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Parachute)
getPartParachute thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartParachuteStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Parachute))
getPartParachuteStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parachute" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The parts parent. Returnsnullif the part does not have a parent.
 - This, in combination with <see cref="M:SpaceCenter.Part.Children" />, can be used to traverse the vessels parts tree.
 -}
getPartParent :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartParent thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartParentStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartParentStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Parent" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.RCS" /> if the part is an RCS block/thruster, otherwisenull.
 -}
getPartRCS :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.RCS)
getPartRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartRCSStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.RCS))
getPartRCSStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RCS" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the part is radially attached to its parent, i.e. on the side of its parent.
 - If the part has no parent, returnsfalse.
 -}
getPartRadiallyAttached :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartRadiallyAttached thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartRadiallyAttachedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartRadiallyAttachedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_RadiallyAttached" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Radiator" /> if the part is a radiator, otherwisenull.
 -}
getPartRadiator :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Radiator)
getPartRadiator thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartRadiatorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Radiator))
getPartRadiatorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Radiator" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.ReactionWheel" /> if the part is a reaction wheel, otherwisenull.
 -}
getPartReactionWheel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReactionWheel)
getPartReactionWheel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartReactionWheelStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReactionWheel))
getPartReactionWheelStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReactionWheel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The reference frame that is fixed relative to this part, and centered on a fixed position within the part, defined by the parts model.
 - <list type="bullet">The origin is at the position of the part, as returned by <see cref="M:SpaceCenter.Part.Position" />.The axes rotate with the part.The x, y and z axis directions depend on the design of the part.For docking port parts, this reference frame is not necessarily equivalent to the reference frame
 - for the docking port, returned by <see cref="M:SpaceCenter.DockingPort.ReferenceFrame" />.
 -}
getPartReferenceFrame :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPartReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartReferenceFrameStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPartReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.ResourceConverter" /> if the part is a resource converter, otherwisenull.
 -}
getPartResourceConverter :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ResourceConverter)
getPartResourceConverter thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartResourceConverterStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceConverter))
getPartResourceConverterStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceConverter" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.ResourceHarvester" /> if the part is a resource harvester, otherwisenull.
 -}
getPartResourceHarvester :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.ResourceHarvester)
getPartResourceHarvester thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartResourceHarvesterStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvester))
getPartResourceHarvesterStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ResourceHarvester" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Resources" /> object for the part.
 -}
getPartResources :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Resources)
getPartResources thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartResourcesStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
getPartResourcesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Resources" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Sensor" /> if the part is a sensor, otherwisenull.
 -}
getPartSensor :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Sensor)
getPartSensor thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartSensorStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Sensor))
getPartSensorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Sensor" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the part is shielded from the exterior of the vessel, for example by a fairing.
 -}
getPartShielded :: KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
getPartShielded thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartShieldedStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Bool))
getPartShieldedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Shielded" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Temperature of the skin of the part, in Kelvin.
 -}
getPartSkinTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartSkinTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartSkinTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartSkinTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SkinTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.SolarPanel" /> if the part is a solar panel, otherwisenull.
 -}
getPartSolarPanel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.SolarPanel)
getPartSolarPanel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartSolarPanelStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SolarPanel))
getPartSolarPanelStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_SolarPanel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The stage in which this part will be activated. Returns -1 if the part is not activated by staging.
 -}
getPartStage :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Int.Int32)
getPartStage thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartStageStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Int.Int32))
getPartStageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Stage" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Temperature of the part, in Kelvin.
 -}
getPartTemperature :: KRPCHS.SpaceCenter.Part -> RPCContext (Double)
getPartTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartTemperatureStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Double))
getPartTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Temperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rate at which heat energy is conducting into or out of the part via contact with other parts.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConductionFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalConductionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartThermalConductionFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalConductionFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConductionFlux" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rate at which heat energy is convecting into or out of the part from the surrounding atmosphere.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalConvectionFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalConvectionFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartThermalConvectionFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalConvectionFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalConvectionFlux" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getPartThermalInternalFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalInternalFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalInternalFlux" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A measure of how much energy it takes to increase the internal temperature of the part, in Joules per Kelvin.
 -}
getPartThermalMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartThermalMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalMass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rate at which heat energy is radiating into or out of the part from the surrounding environment.
 - Measured in energy per unit time, or power, in Watts.
 - A positive value means the part is gaining heat energy, and negative means it is losing heat energy.
 -}
getPartThermalRadiationFlux :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalRadiationFlux thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartThermalRadiationFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalRadiationFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalRadiationFlux" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A measure of how much energy it takes to increase the temperature of the resources contained in the part, in Joules per Kelvin.
 -}
getPartThermalResourceMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalResourceMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartThermalResourceMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalResourceMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalResourceMass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A measure of how much energy it takes to increase the skin temperature of the part, in Joules per Kelvin.
 -}
getPartThermalSkinMass :: KRPCHS.SpaceCenter.Part -> RPCContext (Float)
getPartThermalSkinMass thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartThermalSkinMassStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinMass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getPartThermalSkinToInternalFluxStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Float))
getPartThermalSkinToInternalFluxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_ThermalSkinToInternalFlux" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Title of the part, as shown when the part is right clicked in-game. For example "Mk1-2 Command Pod".
 -}
getPartTitle :: KRPCHS.SpaceCenter.Part -> RPCContext (Data.Text.Text)
getPartTitle thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartTitleStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (Data.Text.Text))
getPartTitleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Title" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vessel that contains this part.
 -}
getPartVessel :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCHS.SpaceCenter.Vessel)
getPartVessel thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartVesselStream :: KRPCHS.SpaceCenter.Part -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getPartVesselStream thisArg = do
    let r = makeRequest "SpaceCenter" "Part_get_Vessel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The first docking port in the vessel with the given port name, as returned by <see cref="M:SpaceCenter.DockingPort.Name" />.
 - Returnsnullif there are no such docking ports.<param name="name">
 -}
partsDockingPortWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCHS.SpaceCenter.DockingPort)
partsDockingPortWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_DockingPortWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

partsDockingPortWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
partsDockingPortWithNameStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_DockingPortWithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all parts that are decoupled in the given <paramref name="stage" />.<param name="stage">
 -}
partsInDecoupleStage :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsInDecoupleStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    res <- sendRequest r
    processResponse extract res 

partsInDecoupleStageStream :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInDecoupleStageStream thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all parts that are activated in the given <paramref name="stage" />.<param name="stage">
 -}
partsInStage :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsInStage thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
    res <- sendRequest r
    processResponse extract res 

partsInStageStream :: KRPCHS.SpaceCenter.Parts -> Data.Int.Int32 -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsInStageStream thisArg stageArg = do
    let r = makeRequest "SpaceCenter" "Parts_InStage" [makeArgument 0 thisArg, makeArgument 1 stageArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of modules (combined across all parts in the vessel) whose
 - <see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsModulesWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Module])
partsModulesWithName thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    res <- sendRequest r
    processResponse extract res 

partsModulesWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Module]))
partsModulesWithNameStream thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_ModulesWithName" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all parts that contain a <see cref="T:SpaceCenter.Module" /> whose
 - <see cref="M:SpaceCenter.Module.Name" /> is <paramref name="moduleName" />.<param name="moduleName">
 -}
partsWithModule :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithModule thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
    res <- sendRequest r
    processResponse extract res 

partsWithModuleStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithModuleStream thisArg moduleNameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithModule" [makeArgument 0 thisArg, makeArgument 1 moduleNameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of parts whose <see cref="M:SpaceCenter.Part.Name" /> is <paramref name="name" />.<param name="name">
 -}
partsWithName :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithName thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

partsWithNameStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithNameStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithName" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all parts whose <see cref="M:SpaceCenter.Part.Title" /> is <paramref name="title" />.<param name="title">
 -}
partsWithTitle :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Part])
partsWithTitle thisArg titleArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]
    res <- sendRequest r
    processResponse extract res 

partsWithTitleStream :: KRPCHS.SpaceCenter.Parts -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
partsWithTitleStream thisArg titleArg = do
    let r = makeRequest "SpaceCenter" "Parts_WithTitle" [makeArgument 0 thisArg, makeArgument 1 titleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all of the vessels parts.
 -}
getPartsAll :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Part])
getPartsAll thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsAllStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Part]))
getPartsAllStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_All" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all cargo bays in the vessel.
 -}
getPartsCargoBays :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.CargoBay])
getPartsCargoBays thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsCargoBaysStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.CargoBay]))
getPartsCargoBaysStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_CargoBays" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all control surfaces in the vessel.
 -}
getPartsControlSurfaces :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ControlSurface])
getPartsControlSurfaces thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsControlSurfacesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ControlSurface]))
getPartsControlSurfacesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ControlSurfaces" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part from which the vessel is controlled.
 -}
getPartsControlling :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartsControlling thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsControllingStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsControllingStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Controlling" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all decouplers in the vessel.
 -}
getPartsDecouplers :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Decoupler])
getPartsDecouplers thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsDecouplersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Decoupler]))
getPartsDecouplersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Decouplers" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all docking ports in the vessel.
 -}
getPartsDockingPorts :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.DockingPort])
getPartsDockingPorts thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsDockingPortsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.DockingPort]))
getPartsDockingPortsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_DockingPorts" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all engines in the vessel.This includes any part that generates thrust. This covers many different types of engine,
 - including liquid fuel rockets, solid rocket boosters, jet engines and RCS thrusters.
 -}
getPartsEngines :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Engine])
getPartsEngines thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsEnginesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Engine]))
getPartsEnginesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Engines" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all fairings in the vessel.
 -}
getPartsFairings :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Fairing])
getPartsFairings thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsFairingsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Fairing]))
getPartsFairingsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Fairings" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all intakes in the vessel.
 -}
getPartsIntakes :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Intake])
getPartsIntakes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsIntakesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Intake]))
getPartsIntakesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Intakes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all landing gear attached to the vessel.
 -}
getPartsLandingGear :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LandingGear])
getPartsLandingGear thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingGear" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsLandingGearStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LandingGear]))
getPartsLandingGearStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingGear" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all landing legs attached to the vessel.
 -}
getPartsLandingLegs :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LandingLeg])
getPartsLandingLegs thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingLegs" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsLandingLegsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LandingLeg]))
getPartsLandingLegsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LandingLegs" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all launch clamps attached to the vessel.
 -}
getPartsLaunchClamps :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.LaunchClamp])
getPartsLaunchClamps thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsLaunchClampsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.LaunchClamp]))
getPartsLaunchClampsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_LaunchClamps" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all lights in the vessel.
 -}
getPartsLights :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Light])
getPartsLights thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsLightsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Light]))
getPartsLightsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Lights" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all parachutes in the vessel.
 -}
getPartsParachutes :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Parachute])
getPartsParachutes thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsParachutesStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Parachute]))
getPartsParachutesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Parachutes" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all RCS blocks/thrusters in the vessel.
 -}
getPartsRCS :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.RCS])
getPartsRCS thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsRCSStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.RCS]))
getPartsRCSStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_RCS" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all radiators in the vessel.
 -}
getPartsRadiators :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Radiator])
getPartsRadiators thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsRadiatorsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Radiator]))
getPartsRadiatorsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Radiators" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all reaction wheels in the vessel.
 -}
getPartsReactionWheels :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ReactionWheel])
getPartsReactionWheels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsReactionWheelsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ReactionWheel]))
getPartsReactionWheelsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ReactionWheels" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all resource converters in the vessel.
 -}
getPartsResourceConverters :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ResourceConverter])
getPartsResourceConverters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsResourceConvertersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ResourceConverter]))
getPartsResourceConvertersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceConverters" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all resource harvesters in the vessel.
 -}
getPartsResourceHarvesters :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.ResourceHarvester])
getPartsResourceHarvesters thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsResourceHarvestersStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.ResourceHarvester]))
getPartsResourceHarvestersStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_ResourceHarvesters" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vessels root part.
 -}
getPartsRoot :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCHS.SpaceCenter.Part)
getPartsRoot thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsRootStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getPartsRootStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Root" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all sensors in the vessel.
 -}
getPartsSensors :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.Sensor])
getPartsSensors thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsSensorsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Sensor]))
getPartsSensorsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_Sensors" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all solar panels in the vessel.
 -}
getPartsSolarPanels :: KRPCHS.SpaceCenter.Parts -> RPCContext ([KRPCHS.SpaceCenter.SolarPanel])
getPartsSolarPanels thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPartsSolarPanelsStream :: KRPCHS.SpaceCenter.Parts -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.SolarPanel]))
getPartsSolarPanelsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Parts_get_SolarPanels" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part from which the vessel is controlled.
 -}
setPartsControlling :: KRPCHS.SpaceCenter.Parts -> KRPCHS.SpaceCenter.Part -> RPCContext (Bool)
setPartsControlling thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Parts_set_Controlling" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Load a quicksave.This is the same as calling <see cref="M:SpaceCenter.Load" /> with the name "quicksave".
 -}
quickload :: RPCContext (Bool)
quickload  = do
    let r = makeRequest "SpaceCenter" "Quickload" []
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Save a quicksave.This is the same as calling <see cref="M:SpaceCenter.Save" /> with the name "quicksave".
 -}
quicksave :: RPCContext (Bool)
quicksave  = do
    let r = makeRequest "SpaceCenter" "Quicksave" []
    res <- sendRequest r
    processResponse extractNothing res
      


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
    processResponse extract res 

getRCSActiveStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Active" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the RCS is inactive.
 -}
getRCSAvailableTorque :: KRPCHS.SpaceCenter.RCS -> RPCContext ((Double, Double, Double))
getRCSAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSAvailableTorqueStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ((Double, Double, Double)))
getRCSAvailableTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_AvailableTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thrusters are enabled.
 -}
getRCSEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Enabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSForwardEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSForwardEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSForwardEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSForwardEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_ForwardEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS has fuel available.The RCS thruster must be activated for this property to update correctly.
 -}
getRCSHasFuel :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSHasFuel thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSHasFuelStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSHasFuelStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_HasFuel" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The specific impulse of the RCS at sea level on Kerbin, in seconds.
 -}
getRCSKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSKerbinSeaLevelSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum amount of thrust that can be produced by the RCS thrusters when active, in Newtons.
 -}
getRCSMaxThrust :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSMaxThrustStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSMaxThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum amount of thrust that can be produced by the RCS thrusters when active in a vacuum, in Newtons.
 -}
getRCSMaxVacuumThrust :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSMaxVacuumThrustStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSMaxVacuumThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_MaxVacuumThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this RCS.
 -}
getRCSPart :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCHS.SpaceCenter.Part)
getRCSPart thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSPartStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getRCSPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
getRCSPitchEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSPitchEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSPitchEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSPitchEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PitchEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The ratios of resources that the RCS consumes. A dictionary mapping resource names
 - to the ratios at which they are consumed by the RCS.
 -}
getRCSPropellantRatios :: KRPCHS.SpaceCenter.RCS -> RPCContext (Data.Map.Map (Data.Text.Text) (Float))
getRCSPropellantRatios thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSPropellantRatiosStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (Float)))
getRCSPropellantRatiosStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_PropellantRatios" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The names of resources that the RCS consumes.
 -}
getRCSPropellants :: KRPCHS.SpaceCenter.RCS -> RPCContext ([Data.Text.Text])
getRCSPropellants thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSPropellantsStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ([Data.Text.Text]))
getRCSPropellantsStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Propellants" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRightEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSRightEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSRightEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSRightEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RightEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
getRCSRollEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSRollEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSRollEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSRollEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_RollEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current specific impulse of the RCS, in seconds. Returns zero
 - if the RCS is not active.
 -}
getRCSSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_SpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of thrusters, one of each nozzel in the RCS part.
 -}
getRCSThrusters :: KRPCHS.SpaceCenter.RCS -> RPCContext ([KRPCHS.SpaceCenter.Thruster])
getRCSThrusters thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSThrustersStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Thruster]))
getRCSThrustersStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_Thrusters" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSUpEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSUpEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSUpEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSUpEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_UpEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The vacuum specific impulse of the RCS, in seconds.
 -}
getRCSVacuumSpecificImpulse :: KRPCHS.SpaceCenter.RCS -> RPCContext (Float)
getRCSVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Float))
getRCSVacuumSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
getRCSYawEnabled :: KRPCHS.SpaceCenter.RCS -> RPCContext (Bool)
getRCSYawEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRCSYawEnabledStream :: KRPCHS.SpaceCenter.RCS -> RPCContext (KRPCStream (Bool))
getRCSYawEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "RCS_get_YawEnabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the RCS thrusters are enabled.
 -}
setRCSEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSForwardEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSForwardEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_ForwardEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the RCS thruster will fire when pitch control input is given.
 -}
setRCSPitchEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSPitchEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_PitchEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRightEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSRightEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RightEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the RCS thruster will fire when roll control input is given.
 -}
setRCSRollEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSRollEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_RollEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSUpEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSUpEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_UpEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the RCS thruster will fire when yaw control input is given.
 -}
setRCSYawEnabled :: KRPCHS.SpaceCenter.RCS -> Bool -> RPCContext (Bool)
setRCSYawEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "RCS_set_YawEnabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the radiator is deployable.
 -}
getRadiatorDeployable :: KRPCHS.SpaceCenter.Radiator -> RPCContext (Bool)
getRadiatorDeployable thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRadiatorDeployableStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployableStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployable" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - For a deployable radiator,trueif the radiator is extended.
 - If the radiator is not deployable, this is alwaystrue.
 -}
getRadiatorDeployed :: KRPCHS.SpaceCenter.Radiator -> RPCContext (Bool)
getRadiatorDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRadiatorDeployedStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (Bool))
getRadiatorDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this radiator.
 -}
getRadiatorPart :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCHS.SpaceCenter.Part)
getRadiatorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRadiatorPartStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getRadiatorPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current state of the radiator.A fixed radiator is always <see cref="M:SpaceCenter.RadiatorState.Extended" />.
 -}
getRadiatorState :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCHS.SpaceCenter.RadiatorState)
getRadiatorState thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRadiatorStateStream :: KRPCHS.SpaceCenter.Radiator -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.RadiatorState))
getRadiatorStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "Radiator_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - For a deployable radiator,trueif the radiator is extended.
 - If the radiator is not deployable, this is alwaystrue.
 -}
setRadiatorDeployed :: KRPCHS.SpaceCenter.Radiator -> Bool -> RPCContext (Bool)
setRadiatorDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Radiator_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the reaction wheel is active.
 -}
getReactionWheelActive :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (Bool)
getReactionWheelActive thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getReactionWheelActiveStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Active" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The available torque in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 - Returns zero if the reaction wheel is inactive or broken.
 -}
getReactionWheelAvailableTorque :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelAvailableTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getReactionWheelAvailableTorqueStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelAvailableTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_AvailableTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the reaction wheel is broken.
 -}
getReactionWheelBroken :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (Bool)
getReactionWheelBroken thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getReactionWheelBrokenStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (Bool))
getReactionWheelBrokenStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Broken" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The maximum torque the reaction wheel can provide, is it active,
 - in the pitch, roll and yaw axes of the vessel, in Newton meters.
 - These axes correspond to the coordinate axes of the <see cref="M:SpaceCenter.Vessel.ReferenceFrame" />.
 -}
getReactionWheelMaxTorque :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext ((Double, Double, Double))
getReactionWheelMaxTorque thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getReactionWheelMaxTorqueStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream ((Double, Double, Double)))
getReactionWheelMaxTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_MaxTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this reaction wheel.
 -}
getReactionWheelPart :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCHS.SpaceCenter.Part)
getReactionWheelPart thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getReactionWheelPartStream :: KRPCHS.SpaceCenter.ReactionWheel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getReactionWheelPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the reaction wheel is active.
 -}
setReactionWheelActive :: KRPCHS.SpaceCenter.ReactionWheel -> Bool -> RPCContext (Bool)
setReactionWheelActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ReactionWheel_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - True if the specified converter is active.<param name="index">Index of the converter.
 -}
resourceConverterActive :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Bool)
resourceConverterActive thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extract res 

resourceConverterActiveStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Bool))
resourceConverterActiveStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Active" [makeArgument 0 thisArg, makeArgument 1 indexArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - List of the names of resources consumed by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterInputs :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ([Data.Text.Text])
resourceConverterInputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extract res 

resourceConverterInputsStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream ([Data.Text.Text]))
resourceConverterInputsStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Inputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterName :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Data.Text.Text)
resourceConverterName thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extract res 

resourceConverterNameStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Data.Text.Text))
resourceConverterNameStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Name" [makeArgument 0 thisArg, makeArgument 1 indexArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - List of the names of resources produced by the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterOutputs :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext ([Data.Text.Text])
resourceConverterOutputs thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extract res 

resourceConverterOutputsStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream ([Data.Text.Text]))
resourceConverterOutputsStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Outputs" [makeArgument 0 thisArg, makeArgument 1 indexArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Start the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStart :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Bool)
resourceConverterStart thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Start" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The state of the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterState :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCHS.SpaceCenter.ResourceConverterState)
resourceConverterState thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extract res 

resourceConverterStateStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceConverterState))
resourceConverterStateStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_State" [makeArgument 0 thisArg, makeArgument 1 indexArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Status information for the specified converter.
 - This is the full status message shown in the in-game UI.<param name="index">Index of the converter.
 -}
resourceConverterStatusInfo :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Data.Text.Text)
resourceConverterStatusInfo thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extract res 

resourceConverterStatusInfoStream :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (KRPCStream (Data.Text.Text))
resourceConverterStatusInfoStream thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_StatusInfo" [makeArgument 0 thisArg, makeArgument 1 indexArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Stop the specified converter.<param name="index">Index of the converter.
 -}
resourceConverterStop :: KRPCHS.SpaceCenter.ResourceConverter -> Data.Int.Int32 -> RPCContext (Bool)
resourceConverterStop thisArg indexArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_Stop" [makeArgument 0 thisArg, makeArgument 1 indexArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The number of converters in the part.
 -}
getResourceConverterCount :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (Data.Int.Int32)
getResourceConverterCount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceConverterCountStream :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCStream (Data.Int.Int32))
getResourceConverterCountStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Count" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this converter.
 -}
getResourceConverterPart :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourceConverterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceConverterPartStream :: KRPCHS.SpaceCenter.ResourceConverter -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceConverterPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceConverter_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the harvester is actively drilling.
 -}
getResourceHarvesterActive :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterActive thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterActiveStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Active" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The core temperature of the drill, in Kelvin.
 -}
getResourceHarvesterCoreTemperature :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterCoreTemperatureStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterCoreTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_CoreTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the harvester is deployed.
 -}
getResourceHarvesterDeployed :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Bool)
getResourceHarvesterDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterDeployedStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Bool))
getResourceHarvesterDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rate at which the drill is extracting ore, in units per second.
 -}
getResourceHarvesterExtractionRate :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterExtractionRate thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterExtractionRateStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterExtractionRateStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ExtractionRate" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The core temperature at which the drill will operate with peak efficiency, in Kelvin.
 -}
getResourceHarvesterOptimumCoreTemperature :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterOptimumCoreTemperature thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterOptimumCoreTemperatureStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterOptimumCoreTemperatureStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_OptimumCoreTemperature" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this harvester.
 -}
getResourceHarvesterPart :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourceHarvesterPart thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterPartStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourceHarvesterPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The state of the harvester.
 -}
getResourceHarvesterState :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCHS.SpaceCenter.ResourceHarvesterState)
getResourceHarvesterState thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterStateStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceHarvesterState))
getResourceHarvesterStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The thermal efficiency of the drill, as a percentage of its maximum.
 -}
getResourceHarvesterThermalEfficiency :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (Float)
getResourceHarvesterThermalEfficiency thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceHarvesterThermalEfficiencyStream :: KRPCHS.SpaceCenter.ResourceHarvester -> RPCContext (KRPCStream (Float))
getResourceHarvesterThermalEfficiencyStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_get_ThermalEfficiency" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the harvester is actively drilling.
 -}
setResourceHarvesterActive :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> RPCContext (Bool)
setResourceHarvesterActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the harvester is deployed.
 -}
setResourceHarvesterDeployed :: KRPCHS.SpaceCenter.ResourceHarvester -> Bool -> RPCContext (Bool)
setResourceHarvesterDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "ResourceHarvester_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


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
    processResponse extract res 

resourceTransferStartStream :: KRPCHS.SpaceCenter.Part -> KRPCHS.SpaceCenter.Part -> Data.Text.Text -> Float -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceTransfer))
resourceTransferStartStream fromPartArg toPartArg resourceArg maxAmountArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_Start" [makeArgument 0 fromPartArg, makeArgument 1 toPartArg, makeArgument 2 resourceArg, makeArgument 3 maxAmountArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The amount of the resource that has been transferred.
 -}
getResourceTransferAmount :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (Float)
getResourceTransferAmount thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceTransferAmountStream :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (KRPCStream (Float))
getResourceTransferAmountStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Amount" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the transfer has completed.
 -}
getResourceTransferComplete :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (Bool)
getResourceTransferComplete thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceTransferCompleteStream :: KRPCHS.SpaceCenter.ResourceTransfer -> RPCContext (KRPCStream (Bool))
getResourceTransferCompleteStream thisArg = do
    let r = makeRequest "SpaceCenter" "ResourceTransfer_get_Complete" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The amount of the resource that is currently stored in the part.
 -}
getResourceAmount :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceAmount thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceAmountStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceAmountStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Amount" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The density of the resource, inkg/l.
 -}
getResourceDensity :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceDensity thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceDensityStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceDensityStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Density" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether use of this resource is enabled.
 -}
getResourceEnabled :: KRPCHS.SpaceCenter.Resource -> RPCContext (Bool)
getResourceEnabled thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceEnabledStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Bool))
getResourceEnabledStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Enabled" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The flow mode of the resource.
 -}
getResourceFlowMode :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCHS.SpaceCenter.ResourceFlowMode)
getResourceFlowMode thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceFlowModeStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
getResourceFlowModeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_FlowMode" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total amount of the resource that can be stored in the part.
 -}
getResourceMax :: KRPCHS.SpaceCenter.Resource -> RPCContext (Float)
getResourceMax thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceMaxStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Float))
getResourceMaxStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Max" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the resource.
 -}
getResourceName :: KRPCHS.SpaceCenter.Resource -> RPCContext (Data.Text.Text)
getResourceName thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourceNameStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (Data.Text.Text))
getResourceNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part containing the resource.
 -}
getResourcePart :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCHS.SpaceCenter.Part)
getResourcePart thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourcePartStream :: KRPCHS.SpaceCenter.Resource -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getResourcePartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resource_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether use of this resource is enabled.
 -}
setResourceEnabled :: KRPCHS.SpaceCenter.Resource -> Bool -> RPCContext (Bool)
setResourceEnabled thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Resource_set_Enabled" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Returns the amount of a resource that is currently stored.<param name="name">The name of the resource.
 -}
resourcesAmount :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Float)
resourcesAmount thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

resourcesAmountStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesAmountStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Amount" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the density of a resource, in kg/l.<param name="name">The name of the resource.
 -}
resourcesDensity :: Data.Text.Text -> RPCContext (Float)
resourcesDensity nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Density" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse extract res 

resourcesDensityStream :: Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesDensityStream nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Density" [makeArgument 0 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the flow mode of a resource.<param name="name">The name of the resource.
 -}
resourcesFlowMode :: Data.Text.Text -> RPCContext (KRPCHS.SpaceCenter.ResourceFlowMode)
resourcesFlowMode nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_FlowMode" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse extract res 

resourcesFlowModeStream :: Data.Text.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ResourceFlowMode))
resourcesFlowModeStream nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_FlowMode" [makeArgument 0 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Check whether the named resource can be stored.<param name="name">The name of the resource.
 -}
resourcesHasResource :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Bool)
resourcesHasResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

resourcesHasResourceStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Bool))
resourcesHasResourceStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_HasResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the amount of a resource that can be stored.<param name="name">The name of the resource.
 -}
resourcesMax :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (Float)
resourcesMax thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

resourcesMaxStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream (Float))
resourcesMaxStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_Max" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - All the individual resources with the given name that can be stored.
 -}
resourcesWithResource :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext ([KRPCHS.SpaceCenter.Resource])
resourcesWithResource thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
    res <- sendRequest r
    processResponse extract res 

resourcesWithResourceStream :: KRPCHS.SpaceCenter.Resources -> Data.Text.Text -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
resourcesWithResourceStream thisArg nameArg = do
    let r = makeRequest "SpaceCenter" "Resources_WithResource" [makeArgument 0 thisArg, makeArgument 1 nameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - All the individual resources that can be stored.
 -}
getResourcesAll :: KRPCHS.SpaceCenter.Resources -> RPCContext ([KRPCHS.SpaceCenter.Resource])
getResourcesAll thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourcesAllStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Resource]))
getResourcesAllStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_All" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of resource names that can be stored.
 -}
getResourcesNames :: KRPCHS.SpaceCenter.Resources -> RPCContext ([Data.Text.Text])
getResourcesNames thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getResourcesNamesStream :: KRPCHS.SpaceCenter.Resources -> RPCContext (KRPCStream ([Data.Text.Text]))
getResourcesNamesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Resources_get_Names" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Save the game with a given name.
 - This will create a save file calledname.sfsin the folder of the current save game.
 -}
save :: Data.Text.Text -> RPCContext (Bool)
save nameArg = do
    let r = makeRequest "SpaceCenter" "Save" [makeArgument 0 nameArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the sensor is active.
 -}
getSensorActive :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Bool)
getSensorActive thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSensorActiveStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Bool))
getSensorActiveStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Active" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this sensor.
 -}
getSensorPart :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCHS.SpaceCenter.Part)
getSensorPart thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSensorPartStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getSensorPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current power usage of the sensor, in units of charge per second.
 -}
getSensorPowerUsage :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Float)
getSensorPowerUsage thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_PowerUsage" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSensorPowerUsageStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Float))
getSensorPowerUsageStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_PowerUsage" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current value of the sensor.
 -}
getSensorValue :: KRPCHS.SpaceCenter.Sensor -> RPCContext (Data.Text.Text)
getSensorValue thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSensorValueStream :: KRPCHS.SpaceCenter.Sensor -> RPCContext (KRPCStream (Data.Text.Text))
getSensorValueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Sensor_get_Value" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the sensor is active.
 -}
setSensorActive :: KRPCHS.SpaceCenter.Sensor -> Bool -> RPCContext (Bool)
setSensorActive thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Sensor_set_Active" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the solar panel is extended.
 -}
getSolarPanelDeployed :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Bool)
getSolarPanelDeployed thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSolarPanelDeployedStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Bool))
getSolarPanelDeployedStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Deployed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current amount of energy being generated by the solar panel, in
 - units of charge per second.
 -}
getSolarPanelEnergyFlow :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Float)
getSolarPanelEnergyFlow thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSolarPanelEnergyFlowStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelEnergyFlowStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_EnergyFlow" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The part object for this solar panel.
 -}
getSolarPanelPart :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCHS.SpaceCenter.Part)
getSolarPanelPart thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSolarPanelPartStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getSolarPanelPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current state of the solar panel.
 -}
getSolarPanelState :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCHS.SpaceCenter.SolarPanelState)
getSolarPanelState thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSolarPanelStateStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.SolarPanelState))
getSolarPanelStateStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_State" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current amount of sunlight that is incident on the solar panel,
 - as a percentage. A value between 0 and 1.
 -}
getSolarPanelSunExposure :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (Float)
getSolarPanelSunExposure thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getSolarPanelSunExposureStream :: KRPCHS.SpaceCenter.SolarPanel -> RPCContext (KRPCStream (Float))
getSolarPanelSunExposureStream thisArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_get_SunExposure" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the solar panel is extended.
 -}
setSolarPanelDeployed :: KRPCHS.SpaceCenter.SolarPanel -> Bool -> RPCContext (Bool)
setSolarPanelDeployed thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "SolarPanel_set_Deployed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Position around which the gimbal pivots.
 -}
thrusterGimbalPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterGimbalPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

thrusterGimbalPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterGimbalPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_GimbalPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The direction of the force generated by the thruster, when the engine is in its
 - initial position (no gimballing), in the given reference frame.
 - This is opposite to the direction in which the thruster expels propellant.<param name="referenceFrame">
 -}
thrusterInitialThrustDirection :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

thrusterInitialThrustDirectionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position at which the thruster generates thrust, when the engine is in its
 - initial position (no gimballing), in the given reference frame.<param name="referenceFrame">This position can move when the gimbal rotates. This is because the thrust position and
 - gimbal position are not necessarily the same.
 -}
thrusterInitialThrustPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterInitialThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

thrusterInitialThrustPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterInitialThrustPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_InitialThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The direction of the force generated by the thruster, in the given reference frame.
 - This is opposite to the direction in which the thruster expels propellant.
 - For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustDirection :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

thrusterThrustDirectionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustDirection" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The position at which the thruster generates thrust, in the given reference frame.
 - For gimballed engines, this takes into account the current rotation of the gimbal.<param name="referenceFrame">
 -}
thrusterThrustPosition :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
thrusterThrustPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

thrusterThrustPositionStream :: KRPCHS.SpaceCenter.Thruster -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
thrusterThrustPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Thruster_ThrustPosition" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current gimbal angle in the pitch, roll and yaw axes.
 -}
getThrusterGimbalAngle :: KRPCHS.SpaceCenter.Thruster -> RPCContext ((Double, Double, Double))
getThrusterGimbalAngle thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getThrusterGimbalAngleStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream ((Double, Double, Double)))
getThrusterGimbalAngleStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_GimbalAngle" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the thruster is gimballed.
 -}
getThrusterGimballed :: KRPCHS.SpaceCenter.Thruster -> RPCContext (Bool)
getThrusterGimballed thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getThrusterGimballedStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (Bool))
getThrusterGimballedStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Gimballed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The <see cref="T:SpaceCenter.Part" /> that contains this thruster.
 -}
getThrusterPart :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCHS.SpaceCenter.Part)
getThrusterPart thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getThrusterPartStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Part))
getThrusterPartStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_Part" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getThrusterThrustReferenceFrameStream :: KRPCHS.SpaceCenter.Thruster -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getThrusterThrustReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Thruster_get_ThrustReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Converts a direction vector from one reference frame to another.<param name="direction">Direction vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the direction vector is in.<param name="to">The reference frame to covert the direction vector to.The corresponding direction vector in reference frame <paramref name="to" />.
 -}
transformDirection :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformDirection directionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse extract res 

transformDirectionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformDirectionStream directionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformDirection" [makeArgument 0 directionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Converts a position vector from one reference frame to another.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position vector is in.<param name="to">The reference frame to covert the position vector to.The corresponding position vector in reference frame <paramref name="to" />.
 -}
transformPosition :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformPosition positionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse extract res 

transformPositionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformPositionStream positionArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformPosition" [makeArgument 0 positionArg, makeArgument 1 fromArg, makeArgument 2 toArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Converts a rotation from one reference frame to another.<param name="rotation">Rotation in reference frame <paramref name="from" />.<param name="from">The reference frame that the rotation is in.<param name="to">The corresponding rotation in reference frame <paramref name="to" />.The corresponding rotation in reference frame <paramref name="to" />.
 -}
transformRotation :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
transformRotation rotationArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]
    res <- sendRequest r
    processResponse extract res 

transformRotationStream :: (Double, Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
transformRotationStream rotationArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformRotation" [makeArgument 0 rotationArg, makeArgument 1 fromArg, makeArgument 2 toArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Converts a velocity vector (acting at the specified position vector) from one
 - reference frame to another. The position vector is required to take the
 - relative angular velocity of the reference frames into account.<param name="position">Position vector in reference frame <paramref name="from" />.<param name="velocity">Velocity vector in reference frame <paramref name="from" />.<param name="from">The reference frame that the position and velocity vectors are in.<param name="to">The reference frame to covert the velocity vector to.The corresponding velocity in reference frame <paramref name="to" />.
 -}
transformVelocity :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
transformVelocity positionArg velocityArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]
    res <- sendRequest r
    processResponse extract res 

transformVelocityStream :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
transformVelocityStream positionArg velocityArg fromArg toArg = do
    let r = makeRequest "SpaceCenter" "TransformVelocity" [makeArgument 0 positionArg, makeArgument 1 velocityArg, makeArgument 2 fromArg, makeArgument 3 toArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the angular velocity of the vessel in the given reference frame. The magnitude of the returned
 - vector is the rotational speed in radians per second, and the direction of the vector indicates the
 - axis of rotation (using the right hand rule).<param name="referenceFrame">
 -}
vesselAngularVelocity :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselAngularVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

vesselAngularVelocityStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselAngularVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_AngularVelocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the direction in which the vessel is pointing, as a unit vector, in the given reference frame.<param name="referenceFrame">
 -}
vesselDirection :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselDirection thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

vesselDirectionStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselDirectionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Direction" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns a <see cref="T:SpaceCenter.Flight" /> object that can be used to get flight
 - telemetry for the vessel, in the specified reference frame.<param name="referenceFrame">
 - Reference frame. Defaults to the vessel's surface reference frame (<see cref="M:SpaceCenter.Vessel.SurfaceReferenceFrame" />).
 -}
vesselFlight :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCHS.SpaceCenter.Flight)
vesselFlight thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

vesselFlightStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Flight))
vesselFlightStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Flight" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the position vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselPosition :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselPosition thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

vesselPositionStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselPositionStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Position" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

vesselResourcesInDecoupleStageStream :: KRPCHS.SpaceCenter.Vessel -> Data.Int.Int32 -> Bool -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
vesselResourcesInDecoupleStageStream thisArg stageArg cumulativeArg = do
    let r = makeRequest "SpaceCenter" "Vessel_ResourcesInDecoupleStage" [makeArgument 0 thisArg, makeArgument 1 stageArg, makeArgument 2 cumulativeArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the rotation of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselRotation :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double, Double))
vesselRotation thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

vesselRotationStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
vesselRotationStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Rotation" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns the velocity vector of the center of mass of the vessel in the given reference frame.<param name="referenceFrame">
 -}
vesselVelocity :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ((Double, Double, Double))
vesselVelocity thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
    res <- sendRequest r
    processResponse extract res 

vesselVelocityStream :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (KRPCStream ((Double, Double, Double)))
vesselVelocityStream thisArg referenceFrameArg = do
    let r = makeRequest "SpaceCenter" "Vessel_Velocity" [makeArgument 0 thisArg, makeArgument 1 referenceFrameArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - An <see cref="T:SpaceCenter.AutoPilot" /> object, that can be used to perform
 - simple auto-piloting of the vessel.
 -}
getVesselAutoPilot :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.AutoPilot)
getVesselAutoPilot thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselAutoPilotStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.AutoPilot))
getVesselAutoPilotStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AutoPilot" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselAvailableControlSurfaceTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableControlSurfaceTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableControlSurfaceTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselAvailableEngineTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableEngineTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableEngineTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselAvailableRCSTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableRCSTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableRCSTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselAvailableReactionWheelTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableReactionWheelTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableReactionWheelTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Gets the total available thrust that can be produced by the vessel's
 - active engines, in Newtons. This is computed by summing
 - <see cref="M:SpaceCenter.Engine.AvailableThrust" /> for every active engine in the vessel.
 -}
getVesselAvailableThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselAvailableThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselAvailableThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselAvailableThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselAvailableTorqueStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselAvailableTorqueStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_AvailableTorque" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Returns a <see cref="T:SpaceCenter.Control" /> object that can be used to manipulate
 - the vessel's control inputs. For example, its pitch/yaw/roll controls,
 - RCS and thrust.
 -}
getVesselControl :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Control)
getVesselControl thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselControlStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Control))
getVesselControlStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Control" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total mass of the vessel, excluding resources, in kg.
 -}
getVesselDryMass :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselDryMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselDryMassStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselDryMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_DryMass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The inertia tensor of the vessel around its center of mass, in the vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 - Returns the 3x3 matrix as a list of elements, in row-major order.
 -}
getVesselInertiaTensor :: KRPCHS.SpaceCenter.Vessel -> RPCContext ([Double])
getVesselInertiaTensor thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselInertiaTensorStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ([Double]))
getVesselInertiaTensorStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_InertiaTensor" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The combined specific impulse of all active engines at sea level on Kerbin, in seconds.
 - This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselKerbinSeaLevelSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselKerbinSeaLevelSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselKerbinSeaLevelSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselKerbinSeaLevelSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_KerbinSeaLevelSpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The mission elapsed time in seconds.
 -}
getVesselMET :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Double)
getVesselMET thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselMETStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Double))
getVesselMETStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MET" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total mass of the vessel, including resources, in kg.
 -}
getVesselMass :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMass thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselMassStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMassStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Mass" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total maximum thrust that can be produced by the vessel's active
 - engines, in Newtons. This is computed by summing
 - <see cref="M:SpaceCenter.Engine.MaxThrust" /> for every active engine.
 -}
getVesselMaxThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMaxThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselMaxThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total maximum thrust that can be produced by the vessel's active
 - engines when the vessel is in a vacuum, in Newtons. This is computed by
 - summing <see cref="M:SpaceCenter.Engine.MaxVacuumThrust" /> for every active engine.
 -}
getVesselMaxVacuumThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselMaxVacuumThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselMaxVacuumThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselMaxVacuumThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MaxVacuumThrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The moment of inertia of the vessel around its center of mass inkg.m^2.
 - The inertia values are around the pitch, roll and yaw directions respectively.
 - This corresponds to the vessels reference frame (<see cref="M:SpaceCenter.Vessel.ReferenceFrame" />).
 -}
getVesselMomentOfInertia :: KRPCHS.SpaceCenter.Vessel -> RPCContext ((Double, Double, Double))
getVesselMomentOfInertia thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselMomentOfInertiaStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream ((Double, Double, Double)))
getVesselMomentOfInertiaStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_MomentOfInertia" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the vessel.
 -}
getVesselName :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Data.Text.Text)
getVesselName thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselNameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Data.Text.Text))
getVesselNameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Name" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current orbit of the vessel.
 -}
getVesselOrbit :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Orbit)
getVesselOrbit thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselOrbitStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Orbit))
getVesselOrbitStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Orbit" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The reference frame that is fixed relative to the vessel, and orientated with the vessels
 - orbital prograde/normal/radial directions.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the orbital prograde/normal/radial directions.The x-axis points in the orbital anti-radial direction.The y-axis points in the orbital prograde direction.The z-axis points in the orbital normal direction.Be careful not to confuse this with 'orbit' mode on the navball.
 -}
getVesselOrbitalReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselOrbitalReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselOrbitalReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselOrbitalReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_OrbitalReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Parts" /> object, that can used to interact with the parts that make up this vessel.
 -}
getVesselParts :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Parts)
getVesselParts thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselPartsStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Parts))
getVesselPartsStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Parts" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The reference frame that is fixed relative to the vessel, and orientated with the vessel.
 - <list type="bullet">The origin is at the center of mass of the vessel.The axes rotate with the vessel.The x-axis points out to the right of the vessel.The y-axis points in the forward direction of the vessel.The z-axis points out of the bottom off the vessel.
 -}
getVesselReferenceFrame :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getVesselReferenceFrame thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A <see cref="T:SpaceCenter.Resources" /> object, that can used to get information
 - about resources stored in the vessel.
 -}
getVesselResources :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.Resources)
getVesselResources thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselResourcesStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.Resources))
getVesselResourcesStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Resources" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The situation the vessel is in.
 -}
getVesselSituation :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.VesselSituation)
getVesselSituation thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselSituationStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.VesselSituation))
getVesselSituationStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Situation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The combined specific impulse of all active engines, in seconds. This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselSurfaceReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getVesselSurfaceVelocityReferenceFrameStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getVesselSurfaceVelocityReferenceFrameStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_SurfaceVelocityReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The total thrust currently being produced by the vessel's engines, in
 - Newtons. This is computed by summing <see cref="M:SpaceCenter.Engine.Thrust" /> for
 - every engine in the vessel.
 -}
getVesselThrust :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselThrust thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselThrustStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselThrustStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Thrust" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The type of the vessel.
 -}
getVesselType :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCHS.SpaceCenter.VesselType)
getVesselType thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselTypeStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.VesselType))
getVesselTypeStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_Type" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The combined vacuum specific impulse of all active engines, in seconds. This is computed using the formula
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Specific_impulse#Multiple_engines">described here.
 -}
getVesselVacuumSpecificImpulse :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Float)
getVesselVacuumSpecificImpulse thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getVesselVacuumSpecificImpulseStream :: KRPCHS.SpaceCenter.Vessel -> RPCContext (KRPCStream (Float))
getVesselVacuumSpecificImpulseStream thisArg = do
    let r = makeRequest "SpaceCenter" "Vessel_get_VacuumSpecificImpulse" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The name of the vessel.
 -}
setVesselName :: KRPCHS.SpaceCenter.Vessel -> Data.Text.Text -> RPCContext (Bool)
setVesselName thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Name" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The type of the vessel.
 -}
setVesselType :: KRPCHS.SpaceCenter.Vessel -> KRPCHS.SpaceCenter.VesselType -> RPCContext (Bool)
setVesselType thisArg valueArg = do
    let r = makeRequest "SpaceCenter" "Vessel_set_Type" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Uses time acceleration to warp forward to a time in the future, specified
 - by universal time <paramref name="ut" />. This call blocks until the desired
 - time is reached. Uses regular "on-rails" or physical time warp as appropriate.
 - For example, physical time warp is used when the active vessel is traveling
 - through an atmosphere. When using regular "on-rails" time warp, the warp
 - rate is limited by <paramref name="maxRailsRate" />, and when using physical
 - time warp, the warp rate is limited by <paramref name="maxPhysicsRate" />.<param name="ut">The universal time to warp to, in seconds.<param name="maxRailsRate">The maximum warp rate in regular "on-rails" time warp.<param name="maxPhysicsRate">The maximum warp rate in physical time warp.When the time warp is complete.
 -}
warpTo :: Double -> Float -> Float -> RPCContext (Bool)
warpTo utArg maxRailsRateArg maxPhysicsRateArg = do
    let r = makeRequest "SpaceCenter" "WarpTo" [makeArgument 0 utArg, makeArgument 1 maxRailsRateArg, makeArgument 2 maxPhysicsRateArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The currently active vessel.
 -}
getActiveVessel :: RPCContext (KRPCHS.SpaceCenter.Vessel)
getActiveVessel  = do
    let r = makeRequest "SpaceCenter" "get_ActiveVessel" []
    res <- sendRequest r
    processResponse extract res 

getActiveVesselStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getActiveVesselStream  = do
    let r = makeRequest "SpaceCenter" "get_ActiveVessel" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A dictionary of all celestial bodies (planets, moons, etc.) in the game,
 - keyed by the name of the body.
 -}
getBodies :: RPCContext (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody))
getBodies  = do
    let r = makeRequest "SpaceCenter" "get_Bodies" []
    res <- sendRequest r
    processResponse extract res 

getBodiesStream :: RPCContext (KRPCStream (Data.Map.Map (Data.Text.Text) (KRPCHS.SpaceCenter.CelestialBody)))
getBodiesStream  = do
    let r = makeRequest "SpaceCenter" "get_Bodies" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - An object that can be used to control the camera.
 -}
getCamera :: RPCContext (KRPCHS.SpaceCenter.Camera)
getCamera  = do
    let r = makeRequest "SpaceCenter" "get_Camera" []
    res <- sendRequest r
    processResponse extract res 

getCameraStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Camera))
getCameraStream  = do
    let r = makeRequest "SpaceCenter" "get_Camera" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether <a href="http://forum.kerbalspaceprogram.com/index.php?/topic/19321-105-ferram-aerospace-research-v01557-johnson-21816/">Ferram Aerospace Researchis installed.
 -}
getFARAvailable :: RPCContext (Bool)
getFARAvailable  = do
    let r = makeRequest "SpaceCenter" "get_FARAvailable" []
    res <- sendRequest r
    processResponse extract res 

getFARAvailableStream :: RPCContext (KRPCStream (Bool))
getFARAvailableStream  = do
    let r = makeRequest "SpaceCenter" "get_FARAvailable" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The value of the <a href="https://en.wikipedia.org/wiki/Gravitational_constant">gravitational constantG inN(m/kg)^2.
 -}
getG :: RPCContext (Float)
getG  = do
    let r = makeRequest "SpaceCenter" "get_G" []
    res <- sendRequest r
    processResponse extract res 

getGStream :: RPCContext (KRPCStream (Float))
getGStream  = do
    let r = makeRequest "SpaceCenter" "get_G" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current maximum regular "on-rails" warp factor that can be set.
 - A value between 0 and 7 inclusive.  See
 - <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">the KSP wikifor details.
 -}
getMaximumRailsWarpFactor :: RPCContext (Data.Int.Int32)
getMaximumRailsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" []
    res <- sendRequest r
    processResponse extract res 

getMaximumRailsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getMaximumRailsWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_MaximumRailsWarpFactor" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The physical time warp rate. A value between 0 and 3 inclusive. 0 means
 - no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
getPhysicsWarpFactor :: RPCContext (Data.Int.Int32)
getPhysicsWarpFactor  = do
    let r = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" []
    res <- sendRequest r
    processResponse extract res 

getPhysicsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getPhysicsWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_PhysicsWarpFactor" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getRailsWarpFactorStream :: RPCContext (KRPCStream (Data.Int.Int32))
getRailsWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_RailsWarpFactor" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The currently targeted celestial body.
 -}
getTargetBody :: RPCContext (KRPCHS.SpaceCenter.CelestialBody)
getTargetBody  = do
    let r = makeRequest "SpaceCenter" "get_TargetBody" []
    res <- sendRequest r
    processResponse extract res 

getTargetBodyStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.CelestialBody))
getTargetBodyStream  = do
    let r = makeRequest "SpaceCenter" "get_TargetBody" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The currently targeted docking port.
 -}
getTargetDockingPort :: RPCContext (KRPCHS.SpaceCenter.DockingPort)
getTargetDockingPort  = do
    let r = makeRequest "SpaceCenter" "get_TargetDockingPort" []
    res <- sendRequest r
    processResponse extract res 

getTargetDockingPortStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.DockingPort))
getTargetDockingPortStream  = do
    let r = makeRequest "SpaceCenter" "get_TargetDockingPort" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The currently targeted vessel.
 -}
getTargetVessel :: RPCContext (KRPCHS.SpaceCenter.Vessel)
getTargetVessel  = do
    let r = makeRequest "SpaceCenter" "get_TargetVessel" []
    res <- sendRequest r
    processResponse extract res 

getTargetVesselStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.Vessel))
getTargetVesselStream  = do
    let r = makeRequest "SpaceCenter" "get_TargetVessel" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current universal time in seconds.
 -}
getUT :: RPCContext (Double)
getUT  = do
    let r = makeRequest "SpaceCenter" "get_UT" []
    res <- sendRequest r
    processResponse extract res 

getUTStream :: RPCContext (KRPCStream (Double))
getUTStream  = do
    let r = makeRequest "SpaceCenter" "get_UT" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all the vessels in the game.
 -}
getVessels :: RPCContext ([KRPCHS.SpaceCenter.Vessel])
getVessels  = do
    let r = makeRequest "SpaceCenter" "get_Vessels" []
    res <- sendRequest r
    processResponse extract res 

getVesselsStream :: RPCContext (KRPCStream ([KRPCHS.SpaceCenter.Vessel]))
getVesselsStream  = do
    let r = makeRequest "SpaceCenter" "get_Vessels" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getWarpFactorStream :: RPCContext (KRPCStream (Float))
getWarpFactorStream  = do
    let r = makeRequest "SpaceCenter" "get_WarpFactor" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The current time warp mode. Returns <see cref="M:SpaceCenter.WarpMode.None" /> if time
 - warp is not active, <see cref="M:SpaceCenter.WarpMode.Rails" /> if regular "on-rails" time warp
 - is active, or <see cref="M:SpaceCenter.WarpMode.Physics" /> if physical time warp is active.
 -}
getWarpMode :: RPCContext (KRPCHS.SpaceCenter.WarpMode)
getWarpMode  = do
    let r = makeRequest "SpaceCenter" "get_WarpMode" []
    res <- sendRequest r
    processResponse extract res 

getWarpModeStream :: RPCContext (KRPCStream (KRPCHS.SpaceCenter.WarpMode))
getWarpModeStream  = do
    let r = makeRequest "SpaceCenter" "get_WarpMode" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


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
    processResponse extract res 

getWarpRateStream :: RPCContext (KRPCStream (Float))
getWarpRateStream  = do
    let r = makeRequest "SpaceCenter" "get_WarpRate" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The currently active vessel.
 -}
setActiveVessel :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
setActiveVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_ActiveVessel" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The physical time warp rate. A value between 0 and 3 inclusive. 0 means
 - no time warp. Returns 0 if regular "on-rails" time warp is active.
 -}
setPhysicsWarpFactor :: Data.Int.Int32 -> RPCContext (Bool)
setPhysicsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_PhysicsWarpFactor" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The time warp rate, using regular "on-rails" time warp. A value between
 - 0 and 7 inclusive. 0 means no time warp. Returns 0 if physical time warp
 - is active.
 - If requested time warp factor cannot be set, it will be set to the next
 - lowest possible value. For example, if the vessel is too close to a
 - planet. See <a href="http://wiki.kerbalspaceprogram.com/wiki/Time_warp">
 - the KSP wikifor details.
 -}
setRailsWarpFactor :: Data.Int.Int32 -> RPCContext (Bool)
setRailsWarpFactor valueArg = do
    let r = makeRequest "SpaceCenter" "set_RailsWarpFactor" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The currently targeted celestial body.
 -}
setTargetBody :: KRPCHS.SpaceCenter.CelestialBody -> RPCContext (Bool)
setTargetBody valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetBody" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The currently targeted docking port.
 -}
setTargetDockingPort :: KRPCHS.SpaceCenter.DockingPort -> RPCContext (Bool)
setTargetDockingPort valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetDockingPort" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The currently targeted vessel.
 -}
setTargetVessel :: KRPCHS.SpaceCenter.Vessel -> RPCContext (Bool)
setTargetVessel valueArg = do
    let r = makeRequest "SpaceCenter" "set_TargetVessel" [makeArgument 0 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


