package frc.robot.auto.modes;

import frc.robot.lib.util.Path;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.Vector2d;
import frc.robot.Constants;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;


/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class SimplePathMode extends AutoModeBase {

    public SimplePathMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");
  	
    	double XX = 48.0;
    	double X = 96.0;
    	double Y = 18.0;
    	
    	PathSegment.Options options = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);

    	
        Path path = new Path();
        path.add(new Waypoint(new Vector2d(  0, 0), options));
        path.add(new Waypoint(new Vector2d( XX, 0), options));
        path.add(new Waypoint(new Vector2d( XX, Y), options));
        path.add(new Waypoint(new Vector2d(  X, Y), options));

        Path revPath = new Path(path);
        revPath.setReverseOrder();
        revPath.setReverseDirection();
        
        runAction(new PathFollowerWithVisionAction(path));			// drive forward
        runAction(new PathFollowerWithVisionAction(revPath));    	// drive reversed 
    }
}
