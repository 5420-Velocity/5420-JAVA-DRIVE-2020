package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

	private double knownArea = 0;
	private double knownDistance = 0;
	private final NetworkTableEntry limelightDistance;
	private final NetworkTableEntry limelightDistanceNew;

	private final NetworkTable table;

	public Limelight() {
		this(null, 0.0, 0.0);
	}

	public Limelight(String limelightSuffix) {
		this(limelightSuffix, 0.0, 0.0);
	}

	/**
	 * Creates a new Limelight Subsystem
	 *
	 * @param limelightSuffix When you setup the limelight you can give it a custom name.
	 *                        All setup with limelight-{name}
	 * @param knownArea       Used in combination with knownDistance to calculate the distance
	 * @param knownDistance   See knownArea
	 */
	public Limelight(String limelightSuffix, double knownArea, double knownDistance) {

		String tableName = limelightSuffix.isEmpty() ? "" : ("-" + limelightSuffix.toLowerCase());

		this.table = NetworkTableInstance.getDefault().getTable("limelight" + tableName);
		this.limelightDistance = SmartDashboard.getEntry("limelight" + tableName + "-dx");
		this.limelightDistanceNew = SmartDashboard.getEntry("limelight" + tableName + "-dx-new");

		this.knownArea = knownArea;
		this.knownDistance = knownDistance;

		this.limelightDistance.setDefaultDouble(0.0);
	}

	/**
	 * 
	 * @return Aprox Distance away from the Target in View with only 2 decimal places
	 */
	public double getDistance() {
		return this.getDistance(2);
	}

	/**
	 * Return the Distance from the Target
	 * Both Params should be constant, Both Known Values
	 * The KnownArea is based off of the KnownDistance
	 * <p>
	 * Place the Robot down on the ground and measure from the limelight
	 * then look at the limelight's web interface for the `ta` value.
	 * Pass the 2 values in as a constant and it will return the % of the
	 * selected KnownDistance.
	 * </p>
	 *
	 * @param precision Total Decimal Place precision.
	 * @return Approx Distance away from the Target in View with only n decimal places
	 * @link http://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area-to-estimate-distance
	 */
	public double getDistance(int precision) {
		double scale = Math.pow(10, precision);
		double area = this.getArea();

		// 0.01 is the small threshold of 0
		if (Math.abs(area) < 0.01) {
			return 0.0;
		}

		double k = this.knownDistance * Math.sqrt(this.knownArea);
		double v = k / Math.sqrt(area);
		System.out.println((double) Math.round(v * scale) / scale);
		return (double) Math.round(v * scale) / scale;
	}

	public double getDistanceNew() {
		return getDistanceNew(2);
	}

	/**
	 * Return the Distance from the Target
	 * Both Params should be constant, Both Known Values
	 * The KnownArea is based off of the KnownDistance
	 *
	 * @param precision Total Decimal Place precision.
	 * @return Approx Distance away from the Target in View with only n decimal places
	 * @link https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
	 */
	public double getDistanceNew(int precision) {
		double scale = Math.pow(10, precision);
		double area = this.getArea();

		// 0.01 is the small threshold of 0
		if (Math.abs(area) < 0.01) {
			return 0.0;
		}

		// d = (h2-h1) / tan(a1+a2)
		double d = (Constants.ShooterConstants.h2 - Constants.ShooterConstants.h1)
				/ Math.tan(Constants.ShooterConstants.a1 + this.getTY());

		return (double) Math.round(d * scale) / scale;
	}

	public double getTX() {
		return this.table.getEntry("tx").getDouble(0.0);
	}

	public double getTY() {
		return this.table.getEntry("ty").getDouble(0.0);
	}

	public double getArea() {
		return this.table.getEntry("ta").getDouble(0.0);
	}

	public boolean hasTarget() {
		double check = this.table.getEntry("tv").getDouble(0.0);

		return check == 1.0;
	}

	/**
	 * Set the Camera Mode
	 *
	 * @param value 0, 1
	 *    0 Vision processor
	 *     Driver Camera (Increases exposure, disables vision processing)
	 */
	public void setCamMode(int value) {
		this.table.getEntry("camMode").setDouble(value);
	}

	/**
	 * Set Stream Mode
	 *
	 * @param value 0, 1, 2
	 *    0 Standard - Side-by-side streams if a webcam is attached to Limelight
	 *    1 PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
	 *    2 PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
	 */
	public void setStreamMode(int value) {
		this.table.getEntry("stream").setDouble(value);
	}

	/**
	 * Set Pipeline
	 *
	 * @param value Pipeline Number 0-9
	 */
	public void setPipeline(int value) {
		this.table.getEntry("pipeline").setDouble(value);
	}

	/**
	 * Get the Pipeline
	 * This is the true value that the limelight is using.
	 *
	 * This has an additional delay because the limelight has to read the network table
	 *  value "pipeline" then set "getpipe" to the value.
	 *
	 * @return Pipeline Number 0-9
	 */
	public int getPipeline() {
		return (int) this.table.getEntry("getpipe").getDouble(0.0);
	}

	/**
	 * Set the LED Mode
	 *
	 * @param value Mode 0, 1, 2, 3
	 *    0 Use the LED Mode set in the current pipeline
	 *    1 Force off
	 *    2 Force blink
	 *    3 Force on
	 * @link https://docs.limelightvision.io/en/latest/networktables_api.html
	 */
	public void setLedMode(int value) {
		this.table.getEntry("ledMode").setDouble(value);
	}

	@Override
	public void periodic() {
		// Set the Limelight Distance Value from the calculated Value
		this.limelightDistance.setDouble(this.getDistance());
		this.limelightDistanceNew.setDouble(this.getDistanceNew());
	}

}
