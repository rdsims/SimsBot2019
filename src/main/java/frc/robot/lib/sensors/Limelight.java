package frc.robot.lib.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.HashMap;
import java.util.Map;

/**
 * Limelight Class was started by Corey Applegate of Team 3244 Granite City
 * Gearheads. We Hope you Enjoy the Limelight Camera.
 */
public class Limelight
{
    private static Limelight frontInstance = new Limelight("FrontLimelight");
    private static Limelight  rearInstance = new Limelight( "RearLimelight");
    public static Limelight getFrontInstance() { return frontInstance; }
    public static Limelight getRearInstance()  { return  rearInstance; }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private NetworkTable m_table;
    private String m_tableName;

    /**
     * Using the Default Limelight NT table
     */
    public Limelight()
    {
        m_tableName = "limelight";
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    }

    /**
     * If you changed the name of your Limelight tell me the new name
     */
    public Limelight(String tableName)
    {
        m_tableName = tableName;
        m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    }

    /**
     * Send an instance of the NetworkTabe
     */
    public Limelight(NetworkTable table)
    {
        m_table = table;
        // ToDo
        // m_tableName = get the name of the NT key.
    }

    /**
     * Helper funciton to configure limelight after booting. 
     * 
     * Call this function from disabledPeriodic() in Robot.java
     * (Limelight boots to slowly to be configured from robotInit() or disabledInit() )
     * Modify configuration as needed
     */
    public void disabledPeriodic()
    {
        setPipeline(0);
        setLEDMode(LedMode.kOff);
        setCamMode(CamMode.kDriver);
        setSnapshot(Snapshot.kOff);
        setStream(StreamType.kPiPMain);
    }

    /**
     * Helper funciton to configure limelight after booting. 
     * 
     * Call this function from autoInit() in Robot.java
     * Modify configuration as needed
     */
    public void autoInit()
    {
        setPipeline(0);
        setLEDMode(LedMode.kOn);
        setCamMode(CamMode.kVision);
        setSnapshot(Snapshot.kOff);
        setStream(StreamType.kPiPMain);
    }

    /**
     * Helper funciton to configure limelight after booting. 
     * 
     * Call this function from teleopInit() in Robot.java
     * Modify configuration as needed
     */
    public void teleopInit()
    {
        setPipeline(0);
        setLEDMode(LedMode.kOff);
        setCamMode(CamMode.kDriver);
        setSnapshot(Snapshot.kOff);
        setStream(StreamType.kPiPMain);
    }

    public void LimelightInit()
    {
        // testAllTab();
    }
    // private void testAllTab(){
    // ShuffleboardTab LimeLightTab = Shuffleboard.getTab(m_tableName);
    // // To Do
    // // populate tab with all the data

    // }

    public boolean isConnected()
    {
        resetPipelineLatency();
        Timer.delay(.05); // How to make this not hold the thread?
        if (getPipelineLatency() == 0.0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * tv Whether the limelight has any valid targets (0 or 1)
     * 
     * @return
     */
    public boolean getIsTargetFound()
    {
        NetworkTableEntry tv = m_table.getEntry("tv");
        double v = tv.getDouble(0);
        if (v == 0.0f)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     * 
     * @return
     */
    public double getHorizontalAngleDegToTarget()
    {
        NetworkTableEntry tx = m_table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }

    /**
     * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     * 
     * @return
     */
    public double getVerticalAngleDegToTarget()
    {
        NetworkTableEntry ty = m_table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }

    /**
     * ta Target Area (0% of image to 100% of image)
     * 
     * @return
     */
    public double getTargetAreaPercentage()
    {
        NetworkTableEntry ta = m_table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    /**
     * ts Skew or rotation (-90 degrees to 0 degrees)
     * 
     * @return
     */
    public double getSkewRotation()
    {
        NetworkTableEntry ts = m_table.getEntry("ts");
        double s = ts.getDouble(0.0);
        return s;
    }

    /**
     * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
     * capture latency.
     * 
     * @return
     */
    public double getPipelineLatency()
    {
        NetworkTableEntry tl = m_table.getEntry("tl");
        double l = tl.getDouble(0.0);
        return l;
    }

    private void resetPipelineLatency()
    {
        m_table.getEntry("tl").setValue(0.0);
    }

    public double getTotalLatency()
    {
        // Limelight documentation says: 
        //      tl: The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
        return getPipelineLatency() + 0.011;
    }

    // Setters

    /**
     * LedMode Sets limelight’s LED state
     * 
     * kon koff kblink
     * 
     * @param ledMode
     */
    public void setLEDMode(LedMode ledMode)
    {
        m_table.getEntry("ledMode").setValue(ledMode.getValue());
    }

    /**
     * Returns current LED mode of the Limelight
     * 
     * @return LedMode
     */
    public LedMode getLEDMode()
    {
        NetworkTableEntry ledMode = m_table.getEntry("ledMode");
        double led = ledMode.getDouble(0.0);
        LedMode mode = LedMode.getByValue(led);
        return mode;
    }

    /**
     * camMode Sets limelight’s operation mode
     * 
     * kvision kdriver (Increases exposure, disables vision processing)
     * 
     * @param camMode
     */

    public void setCamMode(CamMode camMode)
    {
        m_table.getEntry("camMode").setValue(camMode.getValue());
    }

    /**
     * Returns current Cam mode of the Limelight
     * 
     * @return CamMode
     */
    public CamMode getCamMode()
    {
        NetworkTableEntry camMode = m_table.getEntry("camMode");
        double cam = camMode.getDouble(0.0);
        CamMode mode = CamMode.getByValue(cam);
        return mode;
    }

    /**
     * pipeline Sets limelight’s current pipeline
     * 
     * 0 . 9 Select pipeline 0.9
     * 
     * @param pipeline
     */
    /*
     * public void setPipeline(Double pipeline) { if(pipeline<0){ pipeline = 0.0;
     * throw new IllegalArgumentException("Pipeline can not be less than zero");
     * }else if(pipeline>9){ pipeline = 9.0; throw new
     * IllegalArgumentException("Pipeline can not be greater than nine"); }
     * m_table.getEntry("pipeline").setValue(pipeline); }
     */

    /**
     * pipeline Sets limelight’s current pipeline
     * 
     * 0 . 9 Select pipeline 0.9
     * 
     * @param pipeline
     */
    public void setPipeline(Integer pipeline)
    {
        if (pipeline < 0)
        {
            pipeline = 0;
            throw new IllegalArgumentException("Pipeline can not be less than zero");
        }
        else if (pipeline > 9)
        {
            pipeline = 9;
            throw new IllegalArgumentException("Pipeline can not be greater than nine");
        }
        m_table.getEntry("pipeline").setValue(pipeline);
    }

    /**
     * Returns current Pipeling of the Limelight
     * 
     * @return Pipelinge
     */
    public double getPipeline()
    {
        NetworkTableEntry pipeline = m_table.getEntry("pipeline");
        double pipe = pipeline.getDouble(0.0);
        return pipe;
    }

    /**
     * Returns current Pipeling of the Limelight
     * 
     * @return Pipelinge
     */
    public Integer getPipelineInt()
    {
        NetworkTableEntry pipeline = m_table.getEntry("pipeline");
        Integer pipe = (int) pipeline.getDouble(0.0);
        return pipe;
    }

    /**
     * stream Sets limelight’s streaming mode
     * 
     * kStandard - Side-by-side streams if a webcam is attached to Limelight
     * kPiPMain - The secondary camera stream is placed in the lower-right corner of
     * the primary camera stream kPiPSecondary - The primary camera stream is placed
     * in the lower-right corner of the secondary camera stream
     * 
     * @param stream
     */
    public void setStream(StreamType stream)
    {
        m_table.getEntry("stream").setValue(stream.getValue());
    }

    public StreamType getStream()
    {
        NetworkTableEntry stream = m_table.getEntry("stream");
        double st = stream.getDouble(0.0);
        StreamType mode = StreamType.getByValue(st);
        return mode;
    }

    /**
     * snapshot Allows users to take snapshots during a match
     * 
     * kon - Stop taking snapshots koff - Take two snapshots per second
     * 
     * @param snapshot
     */
    public void setSnapshot(Snapshot snapshot)
    {
        m_table.getEntry("snapshot").setValue(snapshot.getValue());
    }

    public Snapshot getSnapshot()
    {
        NetworkTableEntry snapshot = m_table.getEntry("snapshot");
        double snshot = snapshot.getDouble(0.0);
        Snapshot mode = Snapshot.getByValue(snshot);
        return mode;
    }

    // *************** Advanced Usage with Raw Contours *********************

    /**
     * Limelight posts three raw contours to NetworkTables that are not influenced
     * by your grouping mode. That is, they are filtered with your pipeline
     * parameters, but never grouped. X and Y are returned in normalized screen
     * space (-1 to 1) rather than degrees. *
     */

    public double getAdvancedRotationToTarget(AdvancedTarget raw)
    {
        NetworkTableEntry txRaw = m_table.getEntry("tx" + Integer.toString(raw.getValue()));
        double x = txRaw.getDouble(0.0);
        return x;
    }

    public double getAdvancedDegVerticalToTarget(AdvancedTarget raw)
    {
        NetworkTableEntry tyRaw = m_table.getEntry("ty" + Integer.toString(raw.getValue()));
        double y = tyRaw.getDouble(0.0);
        return y;
    }

    public double getAdvancedTargetArea(AdvancedTarget raw)
    {
        NetworkTableEntry taRaw = m_table.getEntry("ta" + Integer.toString(raw.getValue()));
        double a = taRaw.getDouble(0.0);
        return a;
    }

    public double getAdvancedSkewRotation(AdvancedTarget raw)
    {
        NetworkTableEntry tsRaw = m_table.getEntry("ts" + Integer.toString(raw.getValue()));
        double s = tsRaw.getDouble(0.0);
        return s;
    }

    // Raw Crosshairs:
    // If you are using raw targeting data, you can still utilize your calibrated
    // crosshairs:

    public double[] getAdvancedRawCrosshair(AdvancedCrosshair raw)
    {
        double[] crosshars = new double[2];
        crosshars[0] = getAdvancedRawCrosshair_X(raw);
        crosshars[1] = getAdvancedRawCrosshair_Y(raw);
        return crosshars;
    }

    public double getAdvancedRawCrosshair_X(AdvancedCrosshair raw)
    {
        NetworkTableEntry cxRaw = m_table.getEntry("cx" + Integer.toString(raw.getValue()));
        double x = cxRaw.getDouble(0.0);
        return x;
    }

    public double getAdvancedRawCrosshair_Y(AdvancedCrosshair raw)
    {
        NetworkTableEntry cyRaw = m_table.getEntry("cy" + Integer.toString(raw.getValue()));
        double y = cyRaw.getDouble(0.0);
        return y;
    }

    /**
     * Enumerations
     */
    public enum LedMode
    {
        kOn(0), kOff(1), kBlink(2);

        private static final Map<Double, LedMode> MY_MAP = new HashMap<Double, LedMode>();

        static
        {
            for (LedMode LedMode : values())
            {
                MY_MAP.put(LedMode.getValue(), LedMode);
            }
        }

        private double value;

        private LedMode(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static LedMode getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum CamMode
    {
        kVision(0), kDriver(1);

        private static final Map<Double, CamMode> MY_MAP = new HashMap<Double, CamMode>();

        static
        {
            for (CamMode CamMode : values())
            {
                MY_MAP.put(CamMode.getValue(), CamMode);
            }
        }

        private double value;

        private CamMode(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static CamMode getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }
    }

    public enum StreamType
    {
        kStandard(0), kPiPMain(1), kPiPSecondary(2);

        private static final Map<Double, StreamType> MY_MAP = new HashMap<Double, StreamType>();

        static
        {
            for (StreamType StreamType : values())
            {
                MY_MAP.put(StreamType.getValue(), StreamType);
            }
        }

        private double value;

        private StreamType(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static StreamType getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum Snapshot
    {

        kOn(1), kOff(0);

        private static final Map<Double, Snapshot> MY_MAP = new HashMap<Double, Snapshot>();

        static
        {
            for (Snapshot Snapshot : values())
            {
                MY_MAP.put(Snapshot.getValue(), Snapshot);
            }
        }

        private double value;

        private Snapshot(double value)
        {
            this.value = value;
        }

        public double getValue()
        {
            return value;
        }

        public static Snapshot getByValue(double value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum AdvancedTarget
    {

        kOne(0), kTwo(1), kThree(2);

        private static final Map<Integer, AdvancedTarget> MY_MAP = new HashMap<Integer, AdvancedTarget>();

        static
        {
            for (AdvancedTarget AdvancedTarget : values())
            {
                MY_MAP.put(AdvancedTarget.getValue(), AdvancedTarget);
            }
        }

        private Integer value;

        private AdvancedTarget(Integer value)
        {
            this.value = value;
        }

        public Integer getValue()
        {
            return value;
        }

        public static AdvancedTarget getByValue(Integer value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

    public enum AdvancedCrosshair
    {

        kOne(0), kTwo(1);

        private static final Map<Integer, AdvancedCrosshair> MY_MAP = new HashMap<Integer, AdvancedCrosshair>();

        static
        {
            for (AdvancedCrosshair AdvancedCrosshair : values())
            {
                MY_MAP.put(AdvancedCrosshair.getValue(), AdvancedCrosshair);
            }
        }

        private Integer value;

        private AdvancedCrosshair(Integer value)
        {
            this.value = value;
        }

        public Integer getValue()
        {
            return value;
        }

        public static AdvancedCrosshair getByValue(Integer value)
        {
            return MY_MAP.get(value);
        }

        public String toString()
        {
            return name();
        }

    }

}
