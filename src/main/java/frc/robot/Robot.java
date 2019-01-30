
package frc.robot;

import java.util.List;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.GoalStates;
import frc.robot.command_status.RobotState;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.util.CrashTracker;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.LedRelay;
import frc.robot.lib.util.MyTimer;
import frc.robot.lib.util.Pose;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.GoalStateLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;
import frc.robot.subsystems.Drive;
import frc.robot.vision.VisionLoop;
import frc.robot.vision.VisionTargetState;

/**
 * The VM is configured tVisionLoopo automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot
{
	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
	RobotState robotState = RobotState.getInstance();
	Drive drive = Drive.getInstance();
	VisionLoop visionState = VisionLoop.getInstance();

	AutoModeExecuter autoModeExecuter = null;

	LoopController loopController;

	SmartDashboardInteractions smartDashboardInteractions;
	DataLogController robotLogger; 	// logger for Robot thread (autonomous thread has it's own logger)

	Relay ledRelay = LedRelay.getInstance();


	AnalogInput distanceSensor = new AnalogInput(0);

	enum OperationalMode
	{
		DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);

		private int val;

		private OperationalMode(int val)
		{
			this.val = val;
		}

		public int getVal()
		{
			return val;
		}
	}

	OperationalMode operationalMode = OperationalMode.DISABLED;

	public Robot()
	{
		CrashTracker.logRobotConstruction();
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		try
		{
			CrashTracker.logRobotInit();

			// TODO: make Enabled/Disabled LoopControllers

			// Configure LoopController
			loopController = new LoopController();
			loopController.register(drive.getVelocityPIDLoop());
			loopController.register(DriveLoop.getInstance());
			loopController.register(RobotStateLoop.getInstance());
			loopController.register(VisionLoop.getInstance());
			loopController.register(GoalStateLoop.getInstance());

			smartDashboardInteractions = new SmartDashboardInteractions();
			smartDashboardInteractions.initWithDefaults();

			// Set dataLogger and Time information
			TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));

			robotLogger = DataLogController.getRobotLogController();
			robotLogger.register(this.getLogger());
			robotLogger.register(Drive.getInstance().getLogger());
			robotLogger.register(drive.getCommand().getLogger());
			robotLogger.register(DriveState.getInstance().getLogger());
			robotLogger.register(RobotState.getInstance().getLogger());
			robotLogger.register(VisionLoop.getInstance().getLogger());
			robotLogger.register(GoalStateLoop.getInstance().getGoalTracker().getLogger());
			robotLogger.register(GoalStates.getInstance().getLogger());

			// set initial Pose (will be updated during autonomousInit())
			setInitialPose(new Pose());
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public void setInitialPose(Pose _initialPose)
	{
		robotState.reset(MyTimer.getTimestamp(), DriveState.getInstance().getLeftDistanceInches(),
				DriveState.getInstance().getRightDistanceInches(), _initialPose);

		System.out.println("InitialPose: " + _initialPose);
	}

	public void zeroAllSensors()
	{
		drive.zeroSensors();
		// mSuperstructure.zeroSensors();
	}

	public void stopAll()
	{
		drive.stop();
		// mSuperstructure.stop();
	}

	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	@Override
	public void disabledInit()
	{
		operationalMode = OperationalMode.DISABLED;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);
		ledRelay.set(Relay.Value.kOff); // turn off LEDs

		try
		{
			CrashTracker.logDisabledInit();
			if (autoModeExecuter != null)
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			stopAll(); // stop all actuators
			loopController.start();

		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic()
	{
		try
		{
			stopAll(); // stop all actuators
			
			
			System.out.printf("Distance: %d\n", distanceSensor.getValue());
			
			System.gc(); // runs garbage collector
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/****************************************************************
	 * AUTONOMOUS MODE
	 ****************************************************************/

	@Override
	public void autonomousInit()
	{
		operationalMode = OperationalMode.AUTONOMOUS;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		try
		{
			CrashTracker.logAutoInit();
			//visionServer.requestAppRestart();

			if (autoModeExecuter != null)
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getAutoModeSelection());

			setInitialPose(autoModeExecuter.getAutoMode().getInitialPose());

			autoModeExecuter.start();

		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic()
	{
		try
		{
			// outputAllToSmartDashboard();
			// updateDriverFeedback();
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/****************************************************************
	 * TELEOP MODE
	 ****************************************************************/

	@Override
	public void teleopInit()
	{
		operationalMode = OperationalMode.TELEOP;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);


		try
		{
			CrashTracker.logTeleopInit();

			// Select joystick control method
			controls = smartDashboardInteractions.getJoystickControlsMode();

			// Configure looper
			loopController.start();

			drive.setOpenLoop(DriveCommand.COAST());

		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		
	}

	@Override
	public void teleopPeriodic()
	{
		try
		{
			drive.setOpenLoop(controls.getDriveCommand());

			if (controls.getButton(Constants.kXboxButtonY))
			{
				ledRelay.set(Relay.Value.kOn);
			}
			else if (controls.getButton(Constants.kXboxButtonA))
			{
				ledRelay.set(Relay.Value.kOff);
			}
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		List<VisionTargetState> targets = visionState.getTargets();
		int k=0;
		for (VisionTargetState target : targets)
		{
			System.out.printf("Target %d -- %s\n", k, target.toString());
			k++;
		}
        System.out.println("===============================\n\n");
		
	}

	/****************************************************************
	 * TEST MODE
	 ****************************************************************/

	enum TestModeEnum
	{
		WHEEL_SPEED_TEST_1REV_PER_SEC
	};

	TestModeEnum testMode = TestModeEnum.WHEEL_SPEED_TEST_1REV_PER_SEC;

	@Override
	public void testInit()
	{
		switch (testMode)
		{
		case WHEEL_SPEED_TEST_1REV_PER_SEC:
			break;
		}

		loopController.start();
	}

	@Override
	public void testPeriodic()
	{
		switch (testMode)
		{
		case WHEEL_SPEED_TEST_1REV_PER_SEC:
			drive.testDriveSpeedControl();
			break;
		}

	}

	/****************************************************************
	 * ROBOT PERIODIC
	 ****************************************************************/

	// called after disabledPeriodic, autoPeriodic, and teleopPeriodic
	@Override
	public void robotPeriodic()
	{
		robotLogger.log();
	}

	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
			put("OperationalMode", operationalMode.getVal());
		}
	};

	public DataLogger getLogger()
	{
		return logger;
	}

}