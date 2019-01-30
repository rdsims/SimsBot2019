package frc.robot.auto.actions.test;

import frc.robot.auto.actions.test.FakeDriveLoop;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class FakeMotorController
{
	ControlMode controlMode = ControlMode.PercentOutput;
	double motorVal = 0.0;
	
	double distanceInches = 0.0;
	double velocityInchesPerSec = 0.0;

	public FakeMotorController(int id)
	{
		// TODO Auto-generated constructor stub
	}

	public void setStatusFramePeriod(StatusFrameEnhanced status2Feedback0, int i, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void set(ControlMode _controlMode, double _motorVal)
	{
		controlMode = _controlMode;
		motorVal = _motorVal;
		if (controlMode == ControlMode.Velocity)
			velocityInchesPerSec = motorVal;
	}

	public void setNeutralMode(NeutralMode coast)
	{
		// TODO Auto-generated method stub
		
	}

	public void configSelectedFeedbackSensor(FeedbackDevice quadencoder, int kTalonPidIdx, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void setInverted(boolean _inverted)
	{
		// TODO Auto-generated method stub
		
	}

	public void setSensorPhase(boolean _sensorPhase)
	{
		// TODO Auto-generated method stub
		
	}
	
	public void config_kF(int kvelocitycontrolslot, double kDriveVelocityKf, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void config_kP(int kvelocitycontrolslot, double kDriveVelocityKp, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void config_kI(int kvelocitycontrolslot, double kDriveVelocityKi, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void config_kD(int kvelocitycontrolslot, double kDriveVelocityKd, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void config_IntegralZone(int kvelocitycontrolslot, int kDriveVelocityIZone, int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void configAllowableClosedloopError(int kvelocitycontrolslot, int kDriveVelocityAllowableError,
			int kTalonTimeoutMs)
	{
		// TODO Auto-generated method stub
		
	}

	public void configOpenloopRamp(double kDriveVelocityRampRate, int i)
	{
		// TODO Auto-generated method stub
		
	}

	public ControlMode getControlMode()
	{
		return controlMode;
	}

	public int getSelectedSensorPosition(int kTalonPidIdx)
	{
		return FakeDriveLoop.inchesToEncoderUnits( distanceInches );
	}

	public int getSelectedSensorVelocity(int kTalonPidIdx)
	{
		return FakeDriveLoop.inchesPerSecondToEncoderUnitsPerFrame( velocityInchesPerSec );
	}

	public double getOutputCurrent()
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public int getClosedLoopError(int kTalonPidIdx)
	{
		// TODO Auto-generated method stub
		return 0;
	}

	public double getMotorOutputPercent()
	{
		return motorVal;
	}

	public void selectProfileSlot(int kbaselockcontrolslot, int kTalonPidIdx)
	{
		// TODO Auto-generated method stub
	}

	public void follow(FakeMotorController lMotorMaster)
	{
		// TODO Auto-generated method stub	
	}

	public void setQuadraturePosition(int i, int kTalonTimeoutMs)
	{
		distanceInches = 0.0;
	}

	public void setDistanceInches(double _distanceInches)
	{
		distanceInches = _distanceInches;
	}

	public void setSpeedInchesPerSecond(double _velocityInchesPerSec)
	{
		velocityInchesPerSec = _velocityInchesPerSec;
	}

	public double getDistanceInches()
	{
		return distanceInches;
	}

	public double getSpeedInchesPerSecond()
	{
		return velocityInchesPerSec;
	}
	
}
