package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.MyTimer;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.Loop;

/**
 * VisionLoop contains the various attributes calculated by the vision system,
 * namely a list of targets and the timestamp at which it was captured.
 */
public class VisionLoop implements Loop
{
    private static VisionLoop instance = new VisionLoop();
    public static VisionLoop getInstance() { return instance; }
	
	public Limelight frontCamera = Limelight.getFrontInstance();

	protected List<VisionTargetState> targets = new ArrayList<>();
	protected double imageCaptureTimestamp = 0;			// note: assumes transport time from phone to this code is instantaneous	

	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
    	double currentTime = MyTimer.getTimestamp();

		// get target info from Limelight
		getTargets(currentTime);
	}

	@Override public void onStop()
	{
		// nothing
	}

	public void getTargets(double currentTime)
	{
		double cameraLatency = frontCamera.getTotalLatency();
		imageCaptureTimestamp = currentTime - cameraLatency;		// assumes transport time from phone to this code is instantaneous

		int numTargets = 1;	// for Limelight
		ArrayList<VisionTargetState> targetStates = new ArrayList<>(numTargets);

		if (frontCamera.getIsTargetFound())
		{
			double hAngle = frontCamera.getHorizontalAngleDegToTarget() * Vector2d.degreesToRadians;
			double vAngle = frontCamera.getVerticalAngleDegToTarget() * Vector2d.degreesToRadians;
			double area = frontCamera.getTargetAreaPercentage();
			double hWidth = 0;	// TODO: used advanced stuff
			double vWidth = 0;	// TODO: used advanced stuff
			
			targetStates.add(new VisionTargetState(hAngle, vAngle, hWidth, vWidth));
		}
		setTargets( targetStates );
	}
	
	
	
	
	// Synchronized get/set functions for access from other threads

	synchronized public void setTargets(List<VisionTargetState> _targets) 	{ targets = _targets; }
	
	synchronized public List<VisionTargetState> getTargets() 	{ return targets; }

	synchronized public double getImageCaptureTimestamp()	{ return imageCaptureTimestamp; }



	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
			int k=0;
			for (VisionTargetState target : targets)
			{
                put(String.format("VisionLoop/Target%d/hCenter", k), target.hCenter);
                put(String.format("VisionLoop/Target%d/hWidth",  k), target.hWidth);
                put(String.format("VisionLoop/Target%d/vCenter", k), target.vCenter);
                put(String.format("VisionLoop/Target%d/vWidth",  k), target.vWidth);
				k++;
			}
		}
	};

	public DataLogger getLogger()
	{
		return logger;
	}


	public double getNormalizedTargetX()
	{
		// TODO Auto-generated method stub
		return 0;
	}

}
