package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Converts a Color Sensor into a simple Encoder Interface
 *  that keep tracking using a counter.
 * 
 * @author Noah Halatead <nhalstead00@gmail.com>
 */
public class ColorMatchCounter implements Sendable, AutoCloseable {

	private ColorSensorV3 colorSensor;
	private ColorMatch colorMatch;
	private ColorMatchResult lastResult;
	private int count = 0;
	private String tagId;

	public ColorMatchCounter(ColorSensorV3 colorSensor, ColorMatch colorMatch) {
		this.colorSensor = colorSensor;
		this.colorMatch = colorMatch;

		this.tagId = this.getClass().getName() + colorSensor.hashCode() + colorMatch.hashCode();

		SendableRegistry.addLW(this, "Color Match Counter", this.tagId);
	}

	public int get() {

		// Check the value of the given color,
		// if it has changed then add one for the counter.
		ColorMatchResult result = this.colorMatch.matchClosestColor(colorSensor.getColor());

		if (lastResult == null || !lastResult.equals(result)) {
			// Objects are not hte same, This a new Object
			this.lastResult = result;
			this.count++;
		}

		return this.count;
	}

	public void reset() {
		this.count = 0;
	}

	@Override
	public void close() throws Exception {
		// Close all elements.
		// I have nothing sooooo, bye.
	}

	/**
	 * Get if the sensor is connected
	 *
	 * @return true if the sensor is connected
	 */
	public boolean isConnected() {
		return true;
	}

	/**
	 * Pushed to the Dashboard
	 * 
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("ColorMatchCounter");
		builder.addDoubleProperty("Count", this::get, null);
		builder.addBooleanProperty("Is Connected", this::isConnected, null);
	}


}
