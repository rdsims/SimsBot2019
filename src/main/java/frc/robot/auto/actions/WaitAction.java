package frc.robot.auto.actions;

import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.MyTimer;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call
 * runAction(new WaitAction(your_time))
 */
public class WaitAction implements Action {

    private double mTimeToWait;
    private double mStartTime;

    public WaitAction(double timeToWait) {
        mTimeToWait = timeToWait;
    }

    @Override
    public boolean isFinished() {
        return MyTimer.getTimestamp() - mStartTime >= mTimeToWait;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mStartTime = MyTimer.getTimestamp();
    }

    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
