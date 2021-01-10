/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.charlesConstants;
import frc.robot.Slot;
import frc.robot.exceptions.UnindexPositionException;

import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.function.Predicate;

public class CharlesSubsystem extends SubsystemBase {

	private final DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.charlesConstants.DIO.encoderPort);
	private final WPI_TalonSRX charlesMotor = new WPI_TalonSRX(charlesConstants.CAN.charlesMotor);
	private final ColorSensorV3 charlesColorSensor = new ColorSensorV3(Constants.ColorTargets.I2C.ColorSensor);
	private final ColorMatch colorMatch = new ColorMatch();

	private final HashMap<Integer, Boolean> storedBalls = new HashMap<Integer, Boolean>();

	public int index;

	public CharlesSubsystem() {

	}

	public double getEncoder() {
		return this.encoder.get();
	}

	public int getIndex() throws UnindexPositionException {
		double currentEncoderPosition = this.encoder.getDistance();

		for (Slot slot : Constants.charlesConstants.slots) {
			if (slot.inBetween(currentEncoderPosition)) {
				return slot.position;
			}
		}

		throw new UnindexPositionException(currentEncoderPosition);
	}

	/**
	 *
	 */
	public void inTook() {
		if (this.index > 4) {
			this.index = 0;
		}

		this.storedBalls.put(this.index, true);
		this.index++;
	}

	/**
	 * Rest the encoder
	 */
	public void reset() {
		this.encoder.reset();
	}

	public Color getColor() {
		return this.colorMatch.matchClosestColor(charlesColorSensor.getColor()).color;
	}

	public void encoderRun(double target) {
		if (encoder.get() < target) {
			charlesMotor.set(0.3);
		}
		else {
			charlesMotor.set(0);
		}
	}

	public void gotoSlot(int location) {
		// TODO: Read the encoder values, keep turning until it true.
		// Read the current encoder value, find what slot we are in.
		// Calculate the Delta, and then turn to the location.

	}

	public boolean hasBallAtIndex(int x) {
		return this.storedBalls.get(x);
	}

	public int filterFirstPosition(Predicate<Map.Entry<Integer, Boolean>> predicate) throws NoSuchElementException  {
		return this.storedBalls.entrySet()
				.stream()
				.filter(predicate)
				.findFirst()
				.orElseThrow()
				.getKey();
	}

	public void nextOpenLocation() {
		try {
			int firstOpen = this.filterFirstPosition(e -> !e.getValue());
			this.gotoSlot(firstOpen);
		}
		catch (NoSuchElementException ignored) {

		}
	}

	/**
	 * Turn the the next slot that has a ball near the current position to the shooter input.
	 * Following a counter-clockwise rotation.
	 */
	public void nextFilledLocation() {
		try {
			int firstFilled = this.filterFirstPosition(Map.Entry::getValue);
			this.gotoSlot(firstFilled);
		}
		catch (NoSuchElementException ignored) {

		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
