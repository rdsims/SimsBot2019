package frc.robot.auto.actions.test;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import frc.robot.lib.sensors.FakeGyro;
import frc.robot.lib.sensors.GyroBase;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.Kinematics;
import frc.robot.lib.util.PathFollowerWithVisionDriveController;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.lib.util.MyTimer;
import frc.robot.lib.util.MyTimer.TimestampMode;
import frc.robot.lib.util.PathFollowerWithVisionDriveController.PathVisionState;
import frc.robot.Constants;
import frc.robot.auto.*;
import frc.robot.auto.actions.PathFollowerWithVisionAction;
import frc.robot.auto.modes.*;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.loops.GoalStateLoop;
import frc.robot.loops.RobotStateLoop;
import frc.robot.command_status.VisionStatus;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Drive;



public class TestAutonomousModes
{
	Pose robotPose;
	Pose targetPose;

	Drive drive = Drive.getInstance();
	GyroBase gyro = FakeGyro.getInstance();

	DriveState driveState = DriveState.getInstance();
	RobotState robotState = RobotState.getInstance();
	VisionStatus visionStatus = VisionStatus.getInstance();
	
	PathFollowerWithVisionAction pathVisionDriveAction;
	PathFollowerWithVisionDriveController driveCtrl;

	double currentTime;
	final double dt = Constants.kLoopDt;

	final double targetWidth = 10.25;

	ArrayList<Double> cameraTimestampQueue;
	ArrayList<Double> cameraTargetXQueue;
	ArrayList<Double> cameraTargetWidthQueue;
	
	DataLogController testLogger;
	
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {		
		MyTimer.setMode(TimestampMode.SIMULATED);
	}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {}

	@Before
	public void setUp() throws Exception
	{
		testLogger = DataLogController.getRobotLogController();
	}

	@After
	public void tearDown() throws Exception {} 

	
	// copied from Robot.java
	public void setInitialPose(Pose _initialPose)
	{
		robotState.reset(MyTimer.getTimestamp(), DriveState.getInstance().getLeftDistanceInches(),
				DriveState.getInstance().getRightDistanceInches(), _initialPose);

		System.out.println("InitialPose: " + _initialPose);
	}
	
	
	@Test 
	public void testAutonomous()
	{
		// robotInit()
		
		FakeLoopController loopController = new FakeLoopController();
		loopController.register(drive.getVelocityPIDLoop());
		loopController.register(FakeDriveLoop.getInstance());
		loopController.register(RobotStateLoop.getInstance());
		loopController.register(GoalStateLoop.getInstance());
		
		
		testLogger.deregister();
		testLogger.register(drive.getCommand().getLogger());
		testLogger.register(driveState.getLogger());
		testLogger.register(robotState.getLogger());
		testLogger.register(visionStatus.getLogger());
		testLogger.setOutputMode(true, false);


		loopController.start();
		
		
		
		// autonomousPeriodic()
		AutoModeExecuter autoModeExecuter = new AutoModeExecuter();
		autoModeExecuter.setAutoMode( new SquarePatternMode() );
		
		setInitialPose(autoModeExecuter.getAutoMode().getInitialPose());

		
				
		// autonomousPeriodic()
		currentTime = 0;
		while (currentTime <= 15)	// autonomous period lasts 15 seconds
		{
			// get robot update every kLoopDt
			autoModeExecuter.getAutoMode().run();
			loopController.run();

			// simulate robot physics at a higher timing resolution
			simulateRobotPhysics(Constants.kLoopDt);
			currentTime += Constants.kLoopDt;
			
			
			// test stuff executed every timestep
			errorCheckEachTimestep();
			// printout

			
			// get ready for next timestep
			MyTimer.update(Constants.kLoopDt);
		}
		
		errorCheckAtEnd();		
	}
	

	public void simulateRobotPhysics(double _dt)
	{
    	// simulate physics of mechanism over the robot's time step
    	double kSimTime = _dt/1.0;//100.0;	// simulate physics at a higher time resolution
    	double t = 0.0;
    	
    	while (t < _dt)
    	{
    		t += kSimTime;
    		
    		
    	}		
	}
	
	
	public void errorCheckEachTimestep()
	{
		 // test that did not stray from path
		if (driveCtrl.getPathVisionState() == PathVisionState.PATH_FOLLOWING)
		{
			double distFromPath = driveCtrl.getDistanceFromPath(); 
			assertTrue(distFromPath < 12);
		}
	}
	
	public void errorCheckAtEnd()
	{
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
		WheelSpeed wheelSpeed = driveCtrl.pathVisionDrive(currentTime, currentPose);	// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control 
		// (won't actually go to motors in simulation)
		//---------------------------------------------------
        drive.setVelocitySetpoint(wheelSpeed);
	}
	
    
	public void simulateDriveLoop()
	{
		DriveCommand newCmd = drive.getCommand();
		FakeDriveLoop fakeDriveLoop = FakeDriveLoop.getInstance();
		
		// get encoder values from hardware, set in Drive
		double lSpeed = newCmd.getLeftMotor();
		double rSpeed = newCmd.getRightMotor();

		fakeDriveLoop.setLeftDistanceInches( fakeDriveLoop.getLeftDistanceInches() + lSpeed * dt );
		fakeDriveLoop.setRightDistanceInches( fakeDriveLoop.getRightDistanceInches() + rSpeed * dt );

		fakeDriveLoop.setLeftSpeedInchesPerSecond( lSpeed );
		fakeDriveLoop.setRightSpeedInchesPerSecond( rSpeed );

		Kinematics.LinearAngularSpeed speed = Kinematics.forwardKinematics(lSpeed, rSpeed);

		((FakeGyro) gyro).setHeadingDeg( gyro.getHeadingDeg() + speed.angularSpeed*dt );

		if (newCmd.getResetEncoders())
		{
			fakeDriveLoop.setLeftDistanceInches(  0 );
			fakeDriveLoop.setRightDistanceInches( 0 );

			fakeDriveLoop.setLeftSpeedInchesPerSecond(  0 );
			fakeDriveLoop.setRightSpeedInchesPerSecond( 0 );
			
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
