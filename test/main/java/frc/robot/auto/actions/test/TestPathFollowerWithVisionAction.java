package frc.robot.auto.actions.test;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.Kinematics;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.PathFollowerWithVisionDriveController;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathFollowerWithVisionDriveController.PathVisionState;
import frc.robot.Constants;
import frc.robot.auto.actions.PathFollowerWithVisionAction;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.vision.VisionState;
import frc.robot.subsystems.Drive;



public class TestPathFollowerWithVisionAction
{
	Pose robotPose;
	Pose targetPose;

	Drive drive = Drive.getInstance();
	DriveState driveState = DriveState.getInstance();
	RobotState robotState = RobotState.getInstance();
	VisionState visionStatus = VisionState.getInstance();
	
	PathFollowerWithVisionAction pathVisionDriveAction;
	PathFollowerWithVisionDriveController driveCtrl;

	double currentTime;
	final double dt = 1.0/50.0;

	final double targetWidth = 10.25;

	ArrayList<Double> cameraTimestampQueue;
	ArrayList<Double> cameraTargetXQueue;
	ArrayList<Double> cameraTargetWidthQueue;
	
	DataLogController testLogger;
	
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {}

	@Before
	public void setUp() throws Exception
	{
		testLogger = DataLogController.getRobotLogController();
	}

	@After
	public void tearDown() throws Exception {} 

	
	
	@Test 
	public void testPathFollower()
	{
		robotPose  = new Pose( 0,  0,  0);
		targetPose = new Pose( 0,120,  0);

		robotState.reset(0, 0, 0, robotPose);
		drive.getCommand().setResetEncoders();
		
    	PathSegment.Options pathOptions   = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	
        Path path = new Path();
        path.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
        path.add(new Waypoint(new Vector2d(36, 0), pathOptions));
        path.add(new Waypoint(new Vector2d(36,18), pathOptions));
        path.add(new Waypoint(new Vector2d(60,48), pathOptions));
        path.add(new Waypoint(new Vector2d(80,48), pathOptions));
        path.add(new Waypoint(new Vector2d(80, 0), pathOptions));
        path.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
		
		cameraTimestampQueue = new ArrayList<Double>();
		cameraTargetXQueue = new ArrayList<Double>();
		cameraTargetWidthQueue = new ArrayList<Double>();
				
		// fill delay queue with initial values
		for (double t=0; t<Constants.kCameraLatencySeconds; t+=dt)
		{
			cameraTimestampQueue.add(-999.0);
			cameraTargetXQueue.add(-999.0);
			cameraTargetWidthQueue.add(-999.0);
		}
		
		pathVisionDriveAction = new PathFollowerWithVisionAction(path);
		driveCtrl = pathVisionDriveAction.getDriveController();		
		
		testLogger.deregister();
		testLogger.register(drive.getCommand().getLogger());
		testLogger.register(driveState.getLogger());
		testLogger.register(robotState.getLogger());
		testLogger.register(visionStatus.getLogger());
		testLogger.register(pathVisionDriveAction.getLogger());
		testLogger.setOutputMode(true, false);

		simulate();		
	}
	
	@Test 
	public void testPathFollowerReverse()
	{
		robotPose  = new Pose( 0,  0,180*Vector2d.degreesToRadians);
		targetPose = new Pose( 0,120,  0);

		robotState.reset(0, 0, 0, robotPose);
		drive.getCommand().setResetEncoders();
		
    	PathSegment.Options pathOptions   = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	
        Path path = new Path();
        path.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
        path.add(new Waypoint(new Vector2d(36, 0), pathOptions));
        path.add(new Waypoint(new Vector2d(36,18), pathOptions));
        path.add(new Waypoint(new Vector2d(60,48), pathOptions));
        path.add(new Waypoint(new Vector2d(80,48), pathOptions));
        path.add(new Waypoint(new Vector2d(80, 0), pathOptions));
        path.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
		
		cameraTimestampQueue = new ArrayList<Double>();
		cameraTargetXQueue = new ArrayList<Double>();
		cameraTargetWidthQueue = new ArrayList<Double>();
				
		// fill delay queue with initial values
		for (double t=0; t<Constants.kCameraLatencySeconds; t+=dt)
		{
			cameraTimestampQueue.add(-999.0);
			cameraTargetXQueue.add(-999.0);
			cameraTargetWidthQueue.add(-999.0);
		}
		
		path.setReverseOrder();
		path.setReverseDirection();
		pathVisionDriveAction = new PathFollowerWithVisionAction(path);
		driveCtrl = pathVisionDriveAction.getDriveController();		

		testLogger.deregister();
		testLogger.register(drive.getCommand().getLogger());
		testLogger.register(driveState.getLogger());
		testLogger.register(robotState.getLogger());
		testLogger.register(visionStatus.getLogger());
		testLogger.register(pathVisionDriveAction.getLogger());
		testLogger.setOutputMode(true, false);

		simulate();		
	}
	
	@Test 
	public void testVision()
	{
		robotPose  = new Pose( 0,  0,  0);
		targetPose = new Pose(96, 36,  0);

		robotState.reset(0, 0, 0, robotPose);
		drive.getCommand().setResetEncoders();
		
    	PathSegment.Options visionOptions = new PathSegment.Options(Constants.kVisionMaxVel,        Constants.kVisionMaxAccel,        Constants.kPathFollowingLookahead, true);
    	
        Path path = new Path();
        path.add(new Waypoint(robotPose.getPosition(), visionOptions));
        path.add(new Waypoint(robotPose.interpolate(targetPose, 0.3).getPosition(), visionOptions));
		
		cameraTimestampQueue = new ArrayList<Double>();
		cameraTargetXQueue = new ArrayList<Double>();
		cameraTargetWidthQueue = new ArrayList<Double>();
		
		
		// fill delay queue with initial values
		for (double t=0; t<Constants.kCameraLatencySeconds; t+=dt)
		{
			cameraTimestampQueue.add(-999.0);
			cameraTargetXQueue.add(-999.0);
			cameraTargetWidthQueue.add(-999.0);
		}
		
		pathVisionDriveAction = new PathFollowerWithVisionAction(path);
		driveCtrl = pathVisionDriveAction.getDriveController();		

		testLogger.deregister();
		testLogger.register(drive.getCommand().getLogger());
		testLogger.register(driveState.getLogger());
		testLogger.register(robotState.getLogger());
		testLogger.register(visionStatus.getLogger());
		testLogger.register(pathVisionDriveAction.getLogger());
		testLogger.setOutputMode(true, false);

		simulate();		
	}
	
	@Test 
	public void testPathVision()
	{
		robotPose  = new Pose( 0,  0,  0);
		targetPose = new Pose( 0,120,  0);

		robotState.reset(0, 0, 0, robotPose);
		drive.getCommand().setResetEncoders();
		
    	PathSegment.Options pathOptions   = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	PathSegment.Options visionOptions = new PathSegment.Options(Constants.kVisionMaxVel,        Constants.kVisionMaxAccel,        Constants.kPathFollowingLookahead, true);
    	
        Path path = new Path();
        path.add(new Waypoint(new Vector2d( 0, 0), pathOptions));
        path.add(new Waypoint(new Vector2d(96, 0), pathOptions));
        path.add(new Waypoint(new Vector2d(96,96), visionOptions));
        path.add(new Waypoint(new Vector2d( 0,96), visionOptions));
		
		cameraTimestampQueue = new ArrayList<Double>();
		cameraTargetXQueue = new ArrayList<Double>();
		cameraTargetWidthQueue = new ArrayList<Double>();	
		
		// fill delay queue with initial values
		for (double t=0; t<Constants.kCameraLatencySeconds; t+=dt)
		{
			cameraTimestampQueue.add(-999.0);
			cameraTargetXQueue.add(-999.0);
			cameraTargetWidthQueue.add(-999.0);
		}
		
		pathVisionDriveAction = new PathFollowerWithVisionAction(path);
		driveCtrl = pathVisionDriveAction.getDriveController();		

		testLogger.deregister();
		testLogger.register(drive.getCommand().getLogger());
		testLogger.register(driveState.getLogger());
		testLogger.register(robotState.getLogger());
		testLogger.register(visionStatus.getLogger());
		testLogger.register(pathVisionDriveAction.getLogger());
		testLogger.setOutputMode(true, false);

		simulate();		
	}
	
	public void simulate()
	{
		pathVisionDriveAction.start(); 		

		/*****************************************************
		 * Simulation
		 ****************************************************/
		for (currentTime = 0; currentTime < 10; currentTime += dt)
		{
			if (pathVisionDriveAction.isFinished())
				break;
			
			simulateTimestep();
			
			 // test that did not stray from path
			if (driveCtrl.getPathVisionState() == PathVisionState.PATH_FOLLOWING)
			{
				double distFromPath = driveCtrl.getDistanceFromPath(); 
				assertTrue(distFromPath < 12);
			}
		}
		pathVisionDriveAction.done();
		
		 // test that we finished in the time allotted
        assertTrue(pathVisionDriveAction.isFinished());


        // if final segment has vision enabled, check that we ended near the target
        if (driveCtrl.getPath().getSegmentVisionEnable())
        {
			// calculate relative position of target
			Vector2d robotToTarget = new Vector2d(targetPose.getPosition()).sub(robotPose.getPosition());
			double  distToTarget = robotToTarget.length(); 
			double angleToTarget = robotToTarget.angle() - robotPose.getHeading();
			angleToTarget = Vector2d.normalizeAngle(angleToTarget);	// modulo 2pi
	        
	        // test that visionDrive ended up close to target and pointed at target
			assertEquals(Constants.kPegTargetDistanceThresholdFromCameraInches, distToTarget, 1);
			assertEquals(angleToTarget, 0, 0.1);
        }
	}
	
	
	
	Pose currentPose;
	Pose previousPose;
	
	double imageTimestamp = currentTime;
	double normalizedTargetX = -999.0; 
	double normalizedTargetWidth = -999.0;
	
	public void simulateTimestep() 
	{
		// update RobotPose -- assume robot has moved with speed & curvature for time dt
		
		simulateDriveLoop();
		simulateRobotStateLoop();
		
		robotPose = robotState.getLatestFieldToVehicle();
		
		// calculate relative position of target
		Vector2d robotToTarget = targetPose.getPosition().sub(robotPose.getPosition());
		double  distToTarget = robotToTarget.length(); 
		double angleToTarget = robotToTarget.angle() - robotPose.getHeading();
		angleToTarget = Vector2d.normalizeAngle(angleToTarget);	// modulo 2pi
		
		// calculate Vision output
		imageTimestamp = currentTime;
		normalizedTargetX = -999.0; 
		normalizedTargetWidth = -999.0;
		
		if (Math.abs(angleToTarget) < Constants.kCameraHalfFOVRadians)
		{
			// target is within camera's field of view
			double fovWidth = 2*distToTarget*Constants.kTangentCameraHalfFOV;		// width of camera's field of view at distance D 
			normalizedTargetWidth = targetWidth / fovWidth;
			normalizedTargetX = -angleToTarget / Constants.kCameraHalfFOVRadians;
		}
		
		// delay Vision output
		cameraTimestampQueue.add(imageTimestamp);
		cameraTargetXQueue.add(normalizedTargetX);
		cameraTargetWidthQueue.add(normalizedTargetWidth);

		imageTimestamp = cameraTimestampQueue.remove(0);
		normalizedTargetX = cameraTargetXQueue.remove(0);
		normalizedTargetWidth = cameraTargetWidthQueue.remove(0);
		
		simulateVisionLoop();
		
		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		simulationUpdate();
		
		//---------------------------------------------------
		// Log
		//---------------------------------------------------
		testLogger.log();
		
		System.out.printf("RobotPose: %s\n", robotPose);
	}

	// equivalent to PathFollowerWithVisionDriveController.update(), but for off-robot testing
    public void simulationUpdate() 
    {
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		imageTimestamp    	  = visionStatus.getImageCaptureTimestamp();
		normalizedTargetX 	  = visionStatus.getNormalizedTargetX();
		normalizedTargetWidth = visionStatus.getNormalizedTargetWidth();

		currentPose  = robotState.getLatestFieldToVehicle();				
		previousPose = robotState.getFieldToVehicle(imageTimestamp);

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		WheelSpeed wheelSpeed = driveCtrl.pathVisionDrive(currentTime, currentPose, previousPose, imageTimestamp, normalizedTargetX, normalizedTargetWidth);	// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control 
		// (won't actually go to motors in simulation)
		//---------------------------------------------------
        drive.setVelocitySetpoint(wheelSpeed);
	}
	
    
	public void simulateDriveLoop()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// copy commands over to status, as if Talon's performed perfectly
		driveState.setTalonControlMode( newCmd.getTalonControlMode() );
		driveState.setNeutralMode(        newCmd.getNeutralMode() );
		
		// get encoder values from hardware, set in Drive
		double lSpeed = newCmd.getLeftMotor();
		double rSpeed = newCmd.getRightMotor();

		driveState.setLeftDistanceInches(  driveState.getLeftDistanceInches()  + lSpeed * dt );
		driveState.setRightDistanceInches( driveState.getRightDistanceInches() + rSpeed * dt );

		driveState.setLeftSpeedInchesPerSec(  lSpeed );
		driveState.setRightSpeedInchesPerSec( rSpeed );

		Kinematics.LinearAngularSpeed speed = Kinematics.forwardKinematics(lSpeed, rSpeed);

		driveState.setHeading( driveState.getHeading() + speed.angularSpeed*dt );

		if (newCmd.getResetEncoders())
		{
			driveState.setLeftDistanceInches(  0 );
			driveState.setRightDistanceInches( 0 );

			driveState.setLeftSpeedInchesPerSec(  0 );
			driveState.setRightSpeedInchesPerSec( 0 );
			
			// can't reset gyro, depend on RobotState.gyroCorrection
		}
	}

	
	public void simulateRobotStateLoop()
	{
		double time		 = currentTime;
        double lDistance = driveState.getLeftDistanceInches();
        double rDistance = driveState.getRightDistanceInches();
        double lSpeed    = driveState.getLeftSpeedInchesPerSec();
        double rSpeed    = driveState.getRightSpeedInchesPerSec(); 
        double gyroAngle = driveState.getHeading();

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngle);
	}
    
	
	public void simulateVisionLoop()
	{
		visionStatus.setImageTimestamp( 		imageTimestamp );
		visionStatus.setNormalizedTargetX( 		normalizedTargetX );
		visionStatus.setNormalizedTargetWidth( 	normalizedTargetWidth );
	}	
	
}
