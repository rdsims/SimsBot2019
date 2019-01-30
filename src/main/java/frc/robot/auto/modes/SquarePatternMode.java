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
public class SquarePatternMode extends AutoModeBase {

    public SquarePatternMode() 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");

    	float D = 72.0f;
    	
    	PathSegment.Options options = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	
        Path path = new Path();
        path.add(new Waypoint(new Vector2d( 0, 0), options));
        path.add(new Waypoint(new Vector2d( D, 0), options));
        path.add(new Waypoint(new Vector2d( D, D), options));
        path.add(new Waypoint(new Vector2d( 0, D), options));
        path.add(new Waypoint(new Vector2d( 0, 0), options));
  
        Path revPath = new Path(path);
        revPath.setReverseOrder();
        revPath.setReverseDirection();
        
        runAction(new PathFollowerWithVisionAction(path));			// drive forward
        runAction(new PathFollowerWithVisionAction(revPath));    	// drive reversed 
    }
}
