package frc.robot.auto.actions.test;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.util.MyTimer;
import frc.robot.Constants;
import frc.robot.loops.Loop;


// a version of LoopController without the threading
// run() is called every timestep

public class FakeLoopController 
{
    public final double kPeriod = Constants.kLoopDt;

    private final List<Loop> loops_;
    private double prev_time_ = 0;
	protected double dt_;
   
    
	public void run() 
	{
        double curr_time = MyTimer.getTimestamp();
        for (Loop loop : loops_) 
        {
            loop.onLoop();
        }
        dt_ = curr_time - prev_time_;
        prev_time_ = curr_time;
	};

    
    public FakeLoopController() 
    {
        loops_ = new ArrayList<>();
    }

    public synchronized void register(Loop loop) 
    {
        loops_.add(loop);
    }

    public synchronized void start() 
    {
       	System.out.println("Starting loops");
    	// lock during access to loop_ to avoid corruption from multiple threads

       	prev_time_ = MyTimer.getTimestamp();
		for (Loop loop : loops_) 
		{
		//  System.out.println("Starting " + loop);
		    loop.onStart();
		}
    }

    public synchronized void stop() 
    {
        System.out.println("Stopping loops");
     	// lock during access to loop_ to avoid corruption from multiple threads

       for (Loop loop : loops_) 
        {
//          System.out.println("Stopping " + loop);
            loop.onStop();
        }
    }

}
