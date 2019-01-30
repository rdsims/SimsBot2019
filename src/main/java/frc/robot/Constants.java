package frc.robot;

/**
 * Attribution: adapted from FRC Team 254
 */

import frc.robot.lib.util.ConstantsBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase
{
    public static double kLoopDt = 0.01;
    public static double kDriveWatchdogTimerThreshold = 0.500;    
    public static int kTalonTimeoutMs = 5;	// ms
    public static int kTalonPidIdx = 0;		// 0 for non-cascaded PIDs, 1 for cascaded PIDs
    
    // Front Bumper
    public static double kFrontBumperX = 18;	// position of front bumper with respect to robot center of rotation
    
	// Wheels
    public static double kDriveWheelCircumInches = 13.00;
    public static double kDriveWheelDiameterInches = kDriveWheelCircumInches / Math.PI;
    public static double kTrackLengthInches = 9.625;
    public static double kTrackWidthInches = 25.125;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;

    // Wheel Encoder
    public static int    kQuadEncoderCodesPerRev = 256;
    public static int    kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
    public static double kQuadEncoderStatusFramePeriod = 0.100;	// 100ms
    
    public static double kDriveSecondsFromNeutralToFull = 0.375;
    public static boolean kLeftMotorSensorPhase = true;
    public static boolean kRightMotorSensorPhase = true;
    
    // CONTROL LOOP GAINS
    public static double kFullThrottleRPM = 520;	// measured max RPM using NI web interface
    public static double kFullThrottleEncoderPulsePer100ms = kFullThrottleRPM / 60.0 * kQuadEncoderStatusFramePeriod * kQuadEncoderUnitsPerRev; 
    
    // PID gains for drive velocity loop (sent to Talon)
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 1.0;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 6.0;
    public static double kDriveVelocityKf = 1023.0 / kFullThrottleEncoderPulsePer100ms;
    public static int    kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int    kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int    kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int    kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;

    // Point Turn constants
    public static double kPointTurnMaxVel    = 80.0; // inches/sec  		
    public static double kPointTurnMaxAccel  = 200.0; // inches/sec^2	
    public static double kPointTurnMinSpeed  = 20.0; // inches/sec 
    public static double kPointTurnCompletionTolerance = 1.0 * (Math.PI/180.0); 
    
    // Path following constants
    public static double kPathFollowingMaxVel    = 60.0; // inches/sec  		
    public static double kPathFollowingMaxAccel  = 48.0; // inches/sec^2	
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingCompletionTolerance = 1.0; 

    // Vision Selection
    public enum VisionSelectionEnum { DROID, JEVOIS; }
    public static VisionSelectionEnum VisionSelection = VisionSelectionEnum.JEVOIS;

    
    // Droid Vision App Constants
    public static String kAppPackage  = "frc.robot.droidvision2017";
	public static String kAppActivity = "frc.robot.droidvision2017/.VisionTrackerActivity";
    public static int kAndroidAppTcpPort = 8686;

    // Vision constants
//    public static double kCameraFrameRate = 30.0;		// frames per second
    public static double kCameraFrameRate = 60.0;		// frames per second
    public static double kCameraPoseX        = +7.25;	// camera location with respect to robot center of rotation, X axis is in direction of travel
    public static double kCameraPoseY        =     0;	// camera location with respect to robot center of rotation, Y axis is positive to the left
    public static double kCameraPoseZ        =   9.0;	// camera location with respect to floor, Z axis is positive with increasing elevation
    public static double kCameraPoseThetaRad = Math.atan2(0.25, 9.0);	// camera angle with respect to robot heading, in radians
    public static double kCameraPitchRad     =     0;   // camera vertical angle with respect to level ground, in radians
    public static double kCameraDeadband = 0.0;

    public static double kVisionMaxVel    = 60.0; // inches/sec
    public static double kVisionMaxAccel  = 48.0; // inches/sec^2		
    public static double kTargetWidthInches = 10.25;
    public static double kTargetHeightInches = 5.00;
    public static double kCenterOfTargetHeightInches = 13.25;
    public static double kPegTargetDistanceThresholdFromBumperInches = 18;		// inches to stop from target, measured from front bumper
    public static double kPegTargetDistanceThresholdFromCameraInches = kFrontBumperX - kCameraPoseX + kPegTargetDistanceThresholdFromBumperInches;
    public static double kVisionCompletionTolerance = 1.0; 
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches

    // Goal Tracking constants
    public static double kMaxTrackerDistance = 18.0;	// inches
    public static double kGoalTrackAveragePeriod = 0.3;		// seconds (will average goal detections over this period)
    public static double kMaxTargetAge = 1.0; // 0.4;			// seconds (will not consider targets that haven't been updated in this time)
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;
    public static double kTrackReportComparatorSwitchingWeight = 3.0;
    public static double kTrackReportComparatorDistanceWeight = 2.0; // Unused

    // Shooter Constants
    public static double kShooterPoseX        =     0;	// shooter location with respect to robot center of rotation, X axis is in direction of travel
    public static double kShooterPoseY        =     0;	// shooter location with respect to robot center of rotation, Y axis is positive to the left
    public static double kShooterPoseZ        =     0;	// shooter location with respect to floor, Z axis is positive with increasing elevation
    public static double kShooterPoseThetaRad =     0;	// shooter angle with respect to robot heading, in radians
    public static double kAutoAimPredictionTime =   0;	// set to 0 since we don't have a turret and need to point the entire robot

    
    
    // Do not change anything after this line!
    
    // Motor Controllers
    // (Note that if multiple Talons are dedicated to a mechanism, any sensors are attached to the master)
    public static final int kLeftMotorMasterTalonId  = 1;
    public static final int kRightMotorMasterTalonId = 2;
    public static final int kLeftMotorSlave1TalonId   = 3;
    public static final int kRightMotorSlave1TalonId  = 4;

    // right motors are inverted
    public static boolean	kLeftMotorInverted = false;
    public static boolean	kRightMotorInverted = true;
    

    // Joystick Controls
    public static int kXboxButtonA  = 1;
    public static int kXboxButtonB  = 2;
    public static int kXboxButtonX  = 3;
    public static int kXboxButtonY  = 4;
    public static int kXboxButtonLB = 5;
    public static int kXboxButtonRB = 6;
    
    public static int kXboxLStickXAxis  = 0;
    public static int kXboxLStickYAxis  = 1;
    public static int kXboxLTriggerAxis = 2;
    public static int kXboxRTriggerAxis = 3;
    public static int kXboxRStickXAxis  = 4;
    public static int kXboxRStickYAxis  = 5;

    // Relay Ports
    public static int kLedRelayPort = 0;
    
    // Gyro
    public enum GyroSelectionEnum { BNO055, NAVX, PIGEON; }
    //public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.BNO055;
    // public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.NAVX;
    public static GyroSelectionEnum GyroSelection = GyroSelectionEnum.PIGEON;

	// The I2C port the BNO055 is connected to
    public static final I2C.Port BNO055_PORT = I2C.Port.kOnboard;
    
    // BNO055 accelerometer calibration constants
    // ( -7, -34,  33, -24) - taken 10/14/2016
    // (-13, -53,  18, -24) - taken 10/14/2016
    // (  0, -59,  25, -24) - taken 10/14/2016
    // using average of the above
    public static short kAccelOffsetX =  -7;
    public static short kAccelOffsetY = -53;
    public static short kAccelOffsetZ =  25;
    public static short kAccelRadius  = -24;
    
    // The SPI port the NavX is connected to
    // (see https://www.pdocs.kauailabs.com/navx-mxp/guidance/selecting-an-interface/)
    public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;						// the SPI port has low latency (<0.1 ms)
    public static byte NAVX_UPDATE_RATE = (byte) (1.0 / Constants.kLoopDt);		// the SPI port supports update rates from 4-200 Hz

    // to be deleted
    public static double kCameraHalfFOVRadians = Math.PI/2.0;
    public static double kCameraLatencySeconds = 250.0;
    public static double kTangentCameraHalfFOV = 0.5;
}
