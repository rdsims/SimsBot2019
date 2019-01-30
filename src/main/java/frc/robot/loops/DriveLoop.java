package frc.robot.loops;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.lib.sensors.*;
import frc.robot.lib.util.MyTimer;
import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.DriveState;
import frc.robot.subsystems.Drive;

/*
 * DriveLoop is the interface between Drive.java and the actual hardware.
 * It runs periodically, taking the commands sent by Drive and sending them to the hardware.
 * In this way, Drive.java does not access the hardware directly.  The benefits of this partition are: 
 * 1) Changes to drive hardware only requires changes to DriveLoop, not Drive
 * 2) DriveLoop can be easily replaced for simulation purposes.
 */

public class DriveLoop implements Loop 
{
	private static DriveLoop instance = new DriveLoop();
	public static DriveLoop getInstance() { return instance; }
	
    private static Drive drive;
	private static GyroBase gyro;
    private DriveState driveState;
    
	public final TalonSRX lMotorMaster, lMotorSlave;
	public final TalonSRX rMotorMaster, rMotorSlave;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	private static ControlMode talonControlMode = ControlMode.Disabled;
	
	private DriveLoop() 
	{
		drive = Drive.getInstance();

		// select which gyro is installed
		switch (Constants.GyroSelection)
		{
		case BNO055:
			System.out.println("Selected gyro = BNO055");
			gyro = BNO055.getInstance();
			break;
		case NAVX:
			System.out.println("Selected gyro = NavX");
			gyro = NavX.getInstance();
			break;
		case PIGEON:
		default:
			System.out.println("Selected gyro = Pigeon");
			gyro = Pigeon.getInstance();
			break;
		}

		driveState = DriveState.getInstance();
		
		lMotorMaster = new TalonSRX(Constants.kLeftMotorMasterTalonId);
        lMotorSlave  = new TalonSRX(Constants.kLeftMotorSlave1TalonId);

		rMotorMaster = new TalonSRX(Constants.kRightMotorMasterTalonId);
        rMotorSlave  = new TalonSRX(Constants.kRightMotorSlave1TalonId);

		lMotorSlave.set(ControlMode.Follower, Constants.kLeftMotorMasterTalonId);	// give slave the TalonID of it's master
		rMotorSlave.set(ControlMode.Follower, Constants.kRightMotorMasterTalonId);	// give slave the TalonID of it's master
        
		// Get status at 100Hz (faster than default 50 Hz)
		lMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTalonTimeoutMs);
		rMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, Constants.kTalonTimeoutMs);

		// Set initial settings
		DriveCommand neutralCmd = DriveCommand.COAST();
		setControlMode(neutralCmd);
		setMotors(neutralCmd);
		setNeutralMode(neutralCmd);
		resetEncoders(neutralCmd);
		
		lMotorMaster.set(ControlMode.PercentOutput, 0.0);
		rMotorMaster.set(ControlMode.PercentOutput, 0.0);
		lMotorMaster.setNeutralMode(NeutralMode.Coast);
		rMotorMaster.setNeutralMode(NeutralMode.Coast);
		lMotorSlave.setNeutralMode(NeutralMode.Coast);
		rMotorSlave.setNeutralMode(NeutralMode.Coast);

		// Set up the encoders
		lMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);	// configure for closed-loop PID
		rMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		lMotorMaster.setSensorPhase(false);
		rMotorMaster.setSensorPhase(false);
		lMotorMaster.setInverted(Constants.kLeftMotorInverted);
		rMotorMaster.setInverted(Constants.kRightMotorInverted);
		lMotorSlave.setInverted(Constants.kLeftMotorInverted);
		rMotorSlave.setInverted(Constants.kRightMotorInverted);

		// Load velocity control gains
		lMotorMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kTalonTimeoutMs);

		rMotorMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone, Constants.kTalonTimeoutMs);
		
		lMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, Constants.kDriveBaseLockAllowableError, Constants.kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kBaseLockControlSlot, Constants.kDriveBaseLockAllowableError, Constants.kTalonTimeoutMs);

		
		// Load base lock control gains
		lMotorMaster.config_kF(kBaseLockControlSlot, Constants.kDriveBaseLockKf, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kP(kBaseLockControlSlot, Constants.kDriveBaseLockKp, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kI(kBaseLockControlSlot, Constants.kDriveBaseLockKi, Constants.kTalonTimeoutMs);
		lMotorMaster.config_kD(kBaseLockControlSlot, Constants.kDriveBaseLockKd, Constants.kTalonTimeoutMs);
		lMotorMaster.config_IntegralZone(kBaseLockControlSlot, Constants.kDriveBaseLockIZone, Constants.kTalonTimeoutMs);

		rMotorMaster.config_kF(kBaseLockControlSlot, Constants.kDriveBaseLockKf, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kP(kBaseLockControlSlot, Constants.kDriveBaseLockKp, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kI(kBaseLockControlSlot, Constants.kDriveBaseLockKi, Constants.kTalonTimeoutMs);
		rMotorMaster.config_kD(kBaseLockControlSlot, Constants.kDriveBaseLockKd, Constants.kTalonTimeoutMs);
		rMotorMaster.config_IntegralZone(kBaseLockControlSlot, Constants.kDriveBaseLockIZone, Constants.kTalonTimeoutMs);

		lMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, Constants.kDriveVelocityAllowableError, Constants.kTalonTimeoutMs);
		rMotorMaster.configAllowableClosedloopError(kVelocityControlSlot, Constants.kDriveVelocityAllowableError, Constants.kTalonTimeoutMs);
	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		// get status from hardware
		getStatus();
		
		// send new commands to hardware
		sendCommands();
	}

	@Override public void onStop()
	{
		stopMotors();
	}

	private void stopMotors()
	{
		drive.setCommand(DriveCommand.COAST());		// override any incoming commands 
		sendCommands();
	}

	private void getStatus()
	{
		synchronized(driveState)	// lock DriveState until we update it, so that objects reading DriveState don't get partial updates	
		{
			// get Talon control & brake modes (assume right motor is configured identically)
			driveState.setTalonControlMode( lMotorMaster.getControlMode() );
			driveState.setNeutralMode( DriveCommand.getNeutralMode() );
			
			// get encoder values from hardware, set in Drive
			driveState.setLeftDistanceInches(  encoderEdgesToInches( lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) ));
			driveState.setRightDistanceInches( encoderEdgesToInches( rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) ));
	
			driveState.setLeftSpeedInchesPerSec(  encoderEdgesPerFrameToInchesPerSecond( lMotorMaster.getSelectedSensorVelocity(  Constants.kTalonPidIdx  ) ));
			driveState.setRightSpeedInchesPerSec( encoderEdgesPerFrameToInchesPerSecond( rMotorMaster.getSelectedSensorVelocity(  Constants.kTalonPidIdx  ) ));
				
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			driveState.setHeadingDeg( gyro.getHeadingDeg() );
	
			driveState.setMotorCurrent(lMotorMaster.getOutputCurrent(), rMotorMaster.getOutputCurrent() );
			driveState.setMotorPIDError(lMotorMaster.getClosedLoopError( Constants.kTalonPidIdx ), rMotorMaster.getClosedLoopError( Constants.kTalonPidIdx ) );
	
	        switch (driveState.getTalonControlMode())
	        {
	        	case PercentOutput: 
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	                break;
	
	        	case Position:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ), rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx ) );
	        		break;
	        		
	        	case Velocity:
	        		driveState.setMotorStatus(lMotorMaster.getSelectedSensorVelocity( Constants.kTalonPidIdx ), rMotorMaster.getSelectedSensorVelocity( Constants.kTalonPidIdx ) );
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		driveState.setMotorStatus(lMotorMaster.getMotorOutputPercent(), rMotorMaster.getMotorOutputPercent() );
	        		break;
			}
		}
	}
		
	private void sendCommands()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// Watchdog timer  
		double currentTime = MyTimer.getTimestamp();
		if (currentTime - newCmd.getCommandTime() > Constants.kDriveWatchdogTimerThreshold)
		{
			// Halt robot if new command hasn't been sent in a while
			stopMotors();
			return;
		}
				
		synchronized(newCmd)	// lock DriveCommand so no one changes it under us while we are sending the commands
		{
			setControlMode(newCmd);
			setMotors(newCmd);
			setNeutralMode(newCmd);
			resetEncoders(newCmd);
		}
	}
	
	
	private void setControlMode(DriveCommand newCmd)
    {
		ControlMode newMode = newCmd.getTalonControlMode();
		
		if (newMode != driveState.getTalonControlMode())
		{
	        switch (newMode)
	        {
	        	case PercentOutput: 
	                break;
	
	        	case Position:
	    			lMotorMaster.selectProfileSlot(kBaseLockControlSlot, Constants.kTalonPidIdx);
	    			rMotorMaster.selectProfileSlot(kBaseLockControlSlot, Constants.kTalonPidIdx);

	        		lMotorMaster.set(ControlMode.Position, (double)(lMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx )) );
	        		rMotorMaster.set(ControlMode.Position, (double)(rMotorMaster.getSelectedSensorPosition( Constants.kTalonPidIdx )) );
	        		break;
	        		
	        	case Velocity:
	        		lMotorMaster.selectProfileSlot(kVelocityControlSlot, Constants.kTalonPidIdx);
	        		rMotorMaster.selectProfileSlot(kVelocityControlSlot, Constants.kTalonPidIdx);
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		break;
	        }
		}
	}
	
	
	
	private void setNeutralMode(DriveCommand newCmd)
	{
		NeutralMode newNeutral = DriveCommand.getNeutralMode();
		setNeutralMode(newNeutral);
	}
	
	
	private void setNeutralMode(NeutralMode newNeutral) 
	{
		if (newNeutral != driveState.getNeutralMode()) 
		{
			lMotorMaster.setNeutralMode(newNeutral);
			rMotorMaster.setNeutralMode(newNeutral);
			lMotorSlave.setNeutralMode(newNeutral);
			rMotorSlave.setNeutralMode(newNeutral);
		}
	}
	
	
		
	private void setMotors(DriveCommand newCmd)
    {
		double lMotorCtrl = newCmd.getLeftMotor();
		double rMotorCtrl = newCmd.getRightMotor();
		
        switch (newCmd.getTalonControlMode())	// assuming new mode is already configured
        {
        	case PercentOutput:
        		// DriveCommand given in range +/-1, with 1 representing full throttle
        		lMotorMaster.set(ControlMode.PercentOutput, lMotorCtrl);
        		rMotorMaster.set(ControlMode.PercentOutput, rMotorCtrl);
        		break;

        	case Position:
        		// no changes to position set in setControlMode()
        		break;
        		
        	case Velocity:
        		// DriveCommand given in inches/sec
        		// Talon SRX needs RPM in closed-loop mode.
        		// convert inches/sec to RPM
           		lMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderEdgesPerFrame(lMotorCtrl)); 
        		rMotorMaster.set(ControlMode.Velocity, inchesPerSecondToEncoderEdgesPerFrame(rMotorCtrl));
        		break;
        		
        	case Disabled:
        	default:
        		lMotorMaster.set(ControlMode.Disabled, 0);
        		rMotorMaster.set(ControlMode.Disabled, 0);
        		break;
        }
	}

	// Talon SRX reports position in rotations while in closed-loop Position mode
	private static double encoderEdgesToInches(int _encoderPosition) {	return (double)_encoderPosition / (double)Constants.kQuadEncoderUnitsPerRev  * Constants.kDriveWheelCircumInches; }
	private static int inchesToEncoderEdges(double _inches) { return (int)(_inches / Constants.kDriveWheelCircumInches * Constants.kQuadEncoderUnitsPerRev); }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	private static double encoderEdgesPerFrameToInchesPerSecond(int _encoderEdgesPerFrame) { return encoderEdgesToInches(_encoderEdgesPerFrame) / Constants.kQuadEncoderStatusFramePeriod; }
	private static int inchesPerSecondToEncoderEdgesPerFrame(double _inchesPerSecond) { return (int)(inchesToEncoderEdges(_inchesPerSecond) * Constants.kQuadEncoderStatusFramePeriod); }

	
	

	private void resetEncoders(DriveCommand newCmd)
	{
		if (newCmd.getResetEncoders())
		{
			SensorCollection collection = lMotorMaster.getSensorCollection();
			collection.setQuadraturePosition(0, Constants.kTalonTimeoutMs);
			
			collection = rMotorMaster.getSensorCollection();
			collection.setQuadraturePosition(0, Constants.kTalonTimeoutMs);
			
			// cannot reset gyro heading in hardware.  
			// calibration to desired initial pose is done in RobotState.reset() called from Robot.autonomousInit()  
		}
	}	


};
