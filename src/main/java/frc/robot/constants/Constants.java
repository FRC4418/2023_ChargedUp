package frc.robot.constants;


public class Constants {
	public static boolean kUsingTuningMode = false;
	
	public static void useV1Constants() {
		Drivetrain.kWheelDiameterMeters = Drivetrain.kWheelDiameterMetersV1;
		Drivetrain.kDrivetrainMPSReductionRatio = Drivetrain.kDrivetrainMPSReductionRatioV1;

		Drivetrain.kTicksToMeters = Drivetrain.kTicksToMetersV1;
		Drivetrain.kOutputMPSToInputTicksPer100ms = Drivetrain.kOutputMPSToInputTicksPer100msV1;

		Drivetrain.kTrackWidthMeters = Drivetrain.kTrackWidthMetersV1;
		Drivetrain.ksVolts = Drivetrain.ksVoltsV1;
		Drivetrain.kvVoltSecondsPerMeter = Drivetrain.kvVoltSecondsPerMeterV1;
		Drivetrain.kaVoltSecondsSquaredPerMeter = Drivetrain.kaVoltSecondsSquaredPerMeterV1;
	}

	public static void useV2Constants() {
		Drivetrain.kWheelDiameterMeters = Drivetrain.kWheelDiameterMetersV2;
		Drivetrain.kDrivetrainMPSReductionRatio = Drivetrain.kDrivetrainMPSReductionRatioV2;

		Drivetrain.kTicksToMeters = Drivetrain.kTicksToMetersV2;
		Drivetrain.kOutputMPSToInputTicksPer100ms = Drivetrain.kOutputMPSToInputTicksPer100msV2;

		Drivetrain.kTrackWidthMeters = Drivetrain.kTrackWidthMetersV2;
		Drivetrain.ksVolts = Drivetrain.ksVoltsV2;
		Drivetrain.kvVoltSecondsPerMeter = Drivetrain.kvVoltSecondsPerMeterV2;
		Drivetrain.kaVoltSecondsSquaredPerMeter = Drivetrain.kaVoltSecondsSquaredPerMeterV2;

	}

	public static int booleanToInt(boolean bool) {
		return bool ? 1 : 0;
	}

	private static final double kMetersToinches = 39.37007874;

	public static double inchesToMeters(double inches) {
		return inches / kMetersToinches;
	}
	public static double metersToInches(double meters) {
		return meters * kMetersToinches;
	}

	public static double feetToMeters(double feet) {
		return feet * 12. / kMetersToinches;
	}
	public static double metersToFeet(double meters) {
		return meters * kMetersToinches / 12.;
	}

	public static class Falcon500 {
		public static final int
			kTicksPerRevolution = 2048,
			// AKA the free RPM
			kMaxRPM = 6380;

		public static final double
			kRpmToTicksPer100ms = ((double) Falcon500.kTicksPerRevolution) / 600.,
			// 720 instead of 360 because our degree range is -180 to 180
			kDegreesToTicks = ((double) Falcon500.kTicksPerRevolution) / 720.;
	}

	public static class Drivetrain {
		// ----------------------------------------------------------
		// General

		public static double
			kWheelDiameterMeters,

			kDrivetrainMPSReductionRatio;
		private static final double
			kWheelDiameterMetersV1 = inchesToMeters(6.),
			kWheelDiameterMetersV2 = inchesToMeters(4.),

			kDrivetrainMPSReductionRatioV1 = 7.33,
			kDrivetrainMPSReductionRatioV2 = 7.75;

		public static enum MotorGroup {
			kLeft,
			kRight
		}

		public static class CAN_ID {
			public static final int
				kFrontLeft = 3,
				kBackLeft = 2,
				kFrontRight = 4,
				kBackRight = 5;
		}

		// ----------------------------------------------------------
		// Conversions

		public static double
			kTicksToMeters,

			kOutputMPSToInputTicksPer100ms;
		private static final double
			kTicksToMetersV1  = (kWheelDiameterMetersV1 * Math.PI) / ((double) Falcon500.kTicksPerRevolution),
			kTicksToMetersV2  = (kWheelDiameterMetersV2 * Math.PI) / ((double) Falcon500.kTicksPerRevolution),

			kOutputMPSToInputTicksPer100msV1 = Falcon500.kTicksPerRevolution / (10. * kWheelDiameterMetersV1 * Math.PI) * kDrivetrainMPSReductionRatioV1,
			kOutputMPSToInputTicksPer100msV2 = Falcon500.kTicksPerRevolution / (10. * kWheelDiameterMetersV2 * Math.PI) * kDrivetrainMPSReductionRatioV2;

		// ----------------------------------------------------------
		// Kinematics

		// horizontal distance between the left and right-side wheels
		public static double
			kTrackWidthMeters;
		private static final double
			kTrackWidthMetersV1 = inchesToMeters(24.6),
			kTrackWidthMetersV2 = inchesToMeters(24.);

		// ----------------------------------------------------------
		// Open-loop control

		public static double
			// units in seconds
			kTeleopOpenLoopRampTime = 0.7,
			kDriveStraightMaxPercent = 0.4;

		public static class CurvaturePolynomial {
			public static double
				kForwardMultiplier = 1.0,
				kForwardExponential = 1.0,

				kRotationMultiplier = 0.8,
				kRotationExponential = 1.0;
		}

		public static class ArcadePolynomial {
			public static double
				kForwardMultiplier = 1.0,
				kForwardExponential = 1.0,

				kTurnMultiplier = 0.8,
				kTurnExponential = 1.0;
		}

		public static class TankPolynomial {
			public static double
				kForwardMultiplier = 1.0,
				kForwardExponential = 1.0;
		}

		// ----------------------------------------------------------
		// Closed-loop control

		public static double
			ksVolts,
			kvVoltSecondsPerMeter,
			kaVoltSecondsSquaredPerMeter;
		private static final double
			ksVoltsV1 = 0.63599,
			ksVoltsV2 = 0.69552,
			kvVoltSecondsPerMeterV1 = 0.043021,
			kvVoltSecondsPerMeterV2 = 0.066546,
			kaVoltSecondsSquaredPerMeterV1 = 0.018985,
			kaVoltSecondsSquaredPerMeterV2 = 0.010455;

		public static final int
			kLeftPidIdx = 0,	// PID slots are basically different contorl-loop types (closed-loop, open-loop, etc)
			kLeftSlotIdx = 0,	// slots are basically different motor control types

			kRightPidIdx = 0,
			kRightSlotIdx = 0,
			// Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails.
			kTimeoutMs = 30;

		// ID Gains may have to be adjusted based on the responsiveness of control loop. kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
		// public static Gains
		// 	kLeftVelocityGains,
		// 	kRightVelocityGains;
		// private static final Gains
		// 	kLeftVelocityGainsV1 = new Gains(0.89077, 0., 0., 1023./20660., 300, 1.00),
		// 	kRightVelocityGainsV1 = new Gains(kLeftVelocityGainsV1),

		// 	kLeftVelocityGainsV2 = new Gains(0.45563, 0., 0., 1023./20660., 300, 1.00),
		// 	kRightVelocityGainsV2 = new Gains(kLeftVelocityGainsV2);

		// ----------------------------------------------------------
		// Max-output modes

		public static double
			kMaxOutput = 1.;

		public static boolean
			kUseSlewRateLimiters = false;

		public static class SlewRates {
			public static final double
				kCurvatureForward = 1.58,
				kCurvatureRotation = 1.08,

				kArcadeForward = 1.58,
				kArcadeTurn = 1.08,
	
				kTankForward = 1.0;
		}

		// ----------------------------------------------------------
		// Trajectory

		public static final double
			kMaxSpeedMetersPerSecond = 3.,
			kMaxAccelerationMetersPerSecondSquared = 3.;

		public static final double
			kRamseteB = 2,
			kRamseteZeta = 0.7;
	}

	public static class Intake {
		// ----------------------------------------------------------
		// General

		public static double
			kReverseFeederPercent = -0.5,
			kFeederPercent = 0.5,

			kRetractorUpDegree = 0.,
			kRetractorDownDegree = 100.;

		public static final double
			kRetractorDegreeTolerance = 4.,

			// means that for every 58.25 input ticks, the mechanism outputs 1 tick
			kRetractorTicksReductionRatio = 58.25,

			kRetractorMinDegree = -180.,
			kRetractorMaxDegree = 180.,

			kRetractorLockEndDelaySeconds = 0.75;

		public static class CAN_ID {
			public static final int
				kFeeder = 11,
				kRetractor = 12;
		}

		public static final int kWhiskerSensorDIOPort = 8;

		// ----------------------------------------------------------
		// Conversions

		public static final double
			kRetractorOutputDegreesToInputTicks = Falcon500.kDegreesToTicks * kRetractorTicksReductionRatio;

		// ----------------------------------------------------------
		// Open-loop controls

		public static final double kRetractorOpenLoopRampSeconds = 1.d;

		// ----------------------------------------------------------
		// Closed-loop control

		public static final double
			// in seconds
			kFeederRampTime = 0.25;
		
		public static final int
			kRetractorPidIdx = 0,
			kRetractorSlotIdx = 0,
			kTimeoutMs = 30;

		// public static Gains
		// 	kRetractorPositionGains;
		// private static final Gains
			// TODO: P3 tune V1 retractor gains
			// kRetractorPositionGainsV1 = new Gains(0.1, 0., 0., 1023./20660., 300, 1.00),
			// kRetractorPositionGainsV2 = new Gains(0.020, 0., 0., 1023./20660., 300, 1.00);
	}

	public static class Manipulator {
		// ----------------------------------------------------------
		// General

		public static final double
			kLauncherTicksReductionRatio = 3.,
			kIndexerTicksReductionRatio = 20.;

		public static final int
			kLauncherMinRPM = -Falcon500.kMaxRPM,
			kLauncherMaxRPM = Falcon500.kMaxRPM,

			kIndexerMinRPM = -Falcon500.kMaxRPM,
			kIndexerMaxRPM = Falcon500.kMaxRPM;

		public static double
			kIndexerPercent = -1.0,
			kReverseIndexerPercent = 1.0;

		public static int
			kLauncherFiringRPM = 1_600,
			kLauncherIdleRPM = -200;

		public static class CAN_ID {
			public static final int
				kIndexer = 21,
				kLauncher = 22;
		}

		// ----------------------------------------------------------
		// Conversions

		public static final double
			kIndexerOutputRPMToInputTicksPer100ms = Falcon500.kRpmToTicksPer100ms * kIndexerTicksReductionRatio,
			kLauncherOutputRPMToInputTicksPer100ms = Falcon500.kRpmToTicksPer100ms * kLauncherTicksReductionRatio;

		// ----------------------------------------------------------
		// Closed-loop control

		public static final int
			kIndexerPidIdx = 0,
			kLauncherPidIdx = 0,
			kTimeoutMs = 30;

		// public static Gains
		// 	kLauncherRPMGains;
		// private static final Gains
			// kLauncherRPMGainsV1 = new Gains(0.083708, 0., 0., 1023./20660., 300, 1.00),
			// kLauncherRPMGainsV2 = new Gains(0.07, 0., 0., 0.055, 300, 1.00);

		// public static Gains
		// 	kIndexerRPMGains;
		// private static final Gains
			// kIndexerRPMGainsV1 = new Gains(0.012, 0., 0., 1023./20660., 300, 1.00),
			// // TODO: P3 Tune V2 indexer RPM gains
			// kIndexerRPMGainsV2 = new Gains(0.012, 0., 0., 1023./20660., 300, 1.00);
	}

	public static class Climber {
		// ----------------------------------------------------------
		// General

		public static final int
			kRatchetPinServoPWMChannel = 4;
		
		public static final double
			kWinchTicksReductionRatio = 30.,

			kWinchSpoolDiameterInches = 0.56,

			kWinchMinPositionInches = 0.,
			// TODO: !!P1!! Figure out the max inches that the winch should allow the climber to raise
			kWinchMaxPositionInches = 70;

		public static double
			kWinchSpeedPercent = 0.75,

			kReleasePinAngle = 0.,
			kAttachPinAngle = 40.,

			kPinRollbackTimeSeconds = 0.1,

			kClimberExtendedHeightInches = 64.,
			kClimberLoweredHeightInches = 0.;
		
		public static class CAN_ID {
			public static final int
				kWinch = 31;
		}

		// ----------------------------------------------------------
		// Conversions

		public static final double
			kWinchOutputInchesToInputTicks = Falcon500.kTicksPerRevolution * kWinchTicksReductionRatio / (kWinchSpoolDiameterInches * Math.PI);

		// ----------------------------------------------------------
		// Closed-loop control

		public static final int
			kWinchPidIdx = 0,
			kTimeoutMs = 30;

		// public static Gains
		// 	kWinchPositionGains;
		//private static final Gains
			// kWinchPositionGainsV1 = new Gains(0.001, 0., 0., 1023./20660., 300, 1.00),
			// kWinchPositionGainsV2 = new Gains(0.001, 0., 0., 1023./20660., 300, 1.00);
	}

	public static class Autonomous {
		// ----------------------------------------------------------
		// General


	}

	public static class Vision {
		// ----------------------------------------------------------
		// General

		public static final int
			kFrontCameraUSBPort = 1,
			kBackCameraUSBPort = -1,
			kInnerCameraUSBPort = 0,

			// for 2022 Rapid React, only TCP/UDP ports 1180-1190 are allowed for camera data from the roboRIO to dashboard when camera is connected to the roboRIO via USB (section R704 of the game manual)
			kFrontCameraTCPPort = 1181,
			kBackCameraTCPPort = 1182,
			kInnerCameraTCPPort = 1183;

		public static boolean
			kUsingFrontCamera = true,
			kUsingBackCamera = false,
			kUsingInnerCamera = true;
		
		public static final int
			kNumCameras =
				booleanToInt(kUsingFrontCamera)
				+ booleanToInt(kUsingBackCamera)
				+ booleanToInt(kUsingInnerCamera);
	}

	public static class Lights {
		// ----------------------------------------------------------
		// General

		public static final byte
			I2C_DEVICE_ADDRESS = 0x44,
			VALUE_REGISTER = 0x0;
	}
}