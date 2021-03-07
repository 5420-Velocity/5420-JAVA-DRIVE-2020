package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

	private double knownArea = 0;
	private double knownDistance = 0;
	private final NetworkTableEntry limelightDistance;

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
	 * @param limelightSuffix When you setup the limelight you can given it a custom name.
	 *                        All setup with limelight-{name}
	 * @param knownArea       Used in combination with knownDistance to calculate the distance
	 * @param knownDistance   See knownArea
	 */
	public Limelight(String limelightSuffix, double knownArea, double knownDistance) {

		String tableName = limelightSuffix.isEmpty() ? "" : ("-" + limelightSuffix.toLowerCase());

		this.table = NetworkTableInstance.getDefault().getTable("limelight" + tableName);
		this.limelightDistance = SmartDashboard.getEntry("limelight" + tableName + "-dx");

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
	 *
	 * @param precision Total Decimal Place precision.
	 * @return Aprox Distance away from the Target in View with only n decimal places
	 * @link http://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area-to-estimate-distance
	 */
	public double getDistance(int precision) {
		double scale = Math.pow(10, precision);

		// 0.01 is the small threshold of 0
		if (Math.abs(this.getArea()) < 0.01) {
			return 0.0;
		}

		double k = this.knownDistance * Math.sqrt(this.knownArea);
		double v = k / Math.sqrt(this.getArea());
		return (double) Math.round(v * scale) / scale;
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
	 * @param value 0,1
	 *              0 Vision processor
	 *              1 Driver Camera (Increases exposure, disables vision processing)
	 */
	public void setCamMode(double value) {
		this.table.getEntry("camMode").setDouble(value);
	}

	/**
	 * Set Stream Mode
	 *
	 * @param value 0,1,2
	 *              0 Standard - Side-by-side streams if a webcam is attached to Limelight
	 *              1 PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
	 *              2 PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
	 */
	public void setStreamMode(double value) {
		this.table.getEntry("stream").setDouble(value);
	}

	/**
	 * Set Pipeline
	 *
	 * @param value 0-9
	 */
	public void setPipeline(double value) {
		this.table.getEntry("pipeline").setDouble(value);
	}

	/**
	 * Set the LED Mode
	 *
	 * @param value 0,1,2,3
	 *              0 Use the LED Mode set in the current pipeline
	 *              1 Force off
	 *              2 Force blink
	 *              3 Force on
	 * @link https://docs.limelightvision.io/en/latest/networktables_api.html
	 */
	public void setLedMode(double value) {
		this.table.getEntry("ledMode").setDouble(value);
	}

	@Override
	public void periodic() {
		// Set the Limelight Distance Value from the calculated Value
		this.limelightDistance.setDouble(this.getDistance());
	}

}
