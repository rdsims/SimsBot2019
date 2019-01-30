package frc.robot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.util.Path;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.Vector2d;
import frc.robot.Constants;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;



/**
 * Drive straight at target, calibrate vision delay such that prevPose is correct
 */
public class CalibrateVisionDelayMode extends AutoModeBase 
{
	// initialPose in inherited from AutoModeBase
	List<Path> pathList;
	
	Pose     start  = new Pose(150,  0, 180*Pose.degreesToRadians);
	Vector2d target = new Vector2d(  0,  0);
	
	
    public CalibrateVisionDelayMode() 
    {
    	initialPose = start;
    }
    
    private void init()
    {
    	PathSegment.Options visionOptions = new PathSegment.Options(Constants.kVisionMaxVel,        Constants.kVisionMaxAccel,        Constants.kPathFollowingLookahead, true);
    	
    	pathList = new ArrayList<Path>();
		
		Path path = new Path();
		path.add(new Waypoint( start.getPosition(), visionOptions));
		path.add(new Waypoint(target, visionOptions));
		pathList.add(path);
    }

    // called by AutoModeExecuter.start() --> AutoModeBase.run()
    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	// generate pathList 
    	init();

    	// execute each path, in sequence
    	for (Path path : pathList)
            runAction( new PathFollowerWithVisionAction( path ) );   
    		
    }
    
}
