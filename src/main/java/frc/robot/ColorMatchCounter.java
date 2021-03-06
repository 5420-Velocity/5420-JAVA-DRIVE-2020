package frc.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Converts a Color Sensor into a simple Encoder Interface
 * that keep tracking using a counter.
 *
 * @author Noah Halatead <nhalstead00@gmail.com>
 */
public class ColorMatchCounter implements Sendable, AutoCloseable {

	private final ColorSensorV3 colorSensor;
	private final ColorMatch colorMatch;
	private Color lastColorResult;
	private int count = 0;
	private String tagId;

	public ColorMatchCounter(ColorSensorV3 colorSensor, ColorMatch colorMatch) {
		this.colorSensor = colorSensor;
		this.colorMatch = colorMatch;

		this.tagId = this.getClass().getName() + colorSensor.hashCode() + colorMatch.hashCode();

		SendableRegistry.addLW(this, "Color Match Counter", this.tagId);
	}

	/**
	 * Should only be called by the SendableBase
	 * This is a private function to update the sensor values.
	 *
	 * @return The Current Count of the Color Sensor
	 */
	private int refreshData() {

		// Check the value of the given color,
		// if it has changed then add one for the counter.
		ColorMatchResult result = this.colorMatch.matchClosestColor(colorSensor.getColor());

		if (lastColorResult == null || !lastColorResult.equals(result.color)) {
			// Objects are not hte same, This a new Object
			this.lastColorResult = result.color;
			this.count++;
		}

		return this.count;
	}

	public int get() {
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
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("ColorMatchCounter");
		builder.addDoubleProperty("Count", this::refreshData, null);
		builder.addBooleanProperty("Is Connected", this::isConnected, null);
	}


}
