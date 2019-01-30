package frc.robot.vision;

/**
 * A container class for Targets detected by the vision system, containing the
 * horizontal and vertical angles from the optical axis.
 */
public class VisionTargetState
{
	protected double hCenter; 	// horizontal angle to center of target from optical axis, in radians
	protected double vCenter; 	//   vertical angle to center of target from optical axis, in radians

	protected double hWidth; 	// horizontal angular width of target, in radians
	protected double vWidth; 	//   vertical angular width of target, in radians

	public VisionTargetState(double _hCenter, double _vCenter, double _hWidth, double _vWidth)
	{
		hCenter = _hCenter;
		vCenter = _vCenter;
		hWidth = _hWidth;
		vWidth = _vWidth;
	}

	public double getHorizontalAngle()	{ return hCenter; }
	public double getVerticalAngle()	{ return vCenter; }
	public double getHorizontalWidth()	{ return hWidth; }
	public double getVerticalWidth()	{ return vWidth; }
	
	public String toString()
	{
		return "hCenter:" + hCenter + ", hWidth:" + hWidth + ", vCenter:" + vCenter + ", vWidth:" + vWidth + ".";
	}
}