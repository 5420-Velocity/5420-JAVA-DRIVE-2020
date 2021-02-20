/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ColorMatchCounter;
import frc.robot.Constants;
import frc.robot.Constants.ColorTargets;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.commands.AutoPanelColorTickTurn;

import java.util.concurrent.atomic.AtomicReference;

public class ControlPanelController extends SubsystemBase {

	private final WPI_TalonSRX panelDriver = new WPI_TalonSRX(ControlPanelConstants.CAN.ControlPanelDriver);
	private final WPI_TalonSRX panelLift = new WPI_TalonSRX(ControlPanelConstants.CAN.ControlPanelLift);
	private final ColorSensorV3 colorSensor = new ColorSensorV3(ControlPanelConstants.ColorSensorPort);
	private final ColorMatch colorMatch = new ColorMatch();
	private final ColorMatchCounter colorMatchCounter = new ColorMatchCounter(colorSensor, colorMatch);
	private final NetworkTableEntry colorEncoderEntry = NetworkTableInstance.getDefault().getEntry(Constants.NetworkTableEntries.COLOR_ENCODER_VALUE);
	private final NetworkTableEntry colorSensorEntry = NetworkTableInstance.getDefault().getEntry(Constants.NetworkTableEntries.COLOR_VALUE);
	private final NetworkTableEntry FMSColorEntry = NetworkTableInstance.getDefault().getEntry(Constants.NetworkTableEntries.FMSCOLOR_VALUE);
	private final DigitalInput upperLimit = new DigitalInput(Constants.ControlPanelConstants.DIO.upperLimit);
	private final DigitalInput lowerLimit = new DigitalInput(Constants.ControlPanelConstants.DIO.lowerLimit);
	private String gameData = "";
	private final AutoPanelColorTickTurn colorCommand;
	private final AtomicReference<Boolean> colorCommandComplete = new AtomicReference<Boolean>();

	public ControlPanelController() {
		this.colorSensorEntry.setDefaultString("");
		this.FMSColorEntry.setDefaultString("");
		this.colorEncoderEntry.setDefaultDouble(0.0);

		this.colorMatch.addColorMatch(ColorTargets.COLOR_BLUE);
		this.colorMatch.addColorMatch(ColorTargets.COLOR_GREEN);
		this.colorMatch.addColorMatch(ColorTargets.COLOR_RED);
		this.colorMatch.addColorMatch(ColorTargets.COLOR_YELLOW);

		this.colorCommandComplete.set(false);

		this.colorCommand = new AutoPanelColorTickTurn(this, colorCommandComplete);
	}

	/**
	 * @return boolean True means that its is at the limit
	 */
	public boolean getUpper() {
		// The sensor value is true when there is no magnet detected
		return !this.upperLimit.get();
	}

	/**
	 * @return boolean True means that its is at the limit
	 */
	public boolean getLower() {
		// The sensor value is true when there is no magnet detected
		return !this.lowerLimit.get();
	}

	public void liftSpeed(double power) {
		this.panelLift.set(power);
	}

	public void turnSpeed(double power) {
		this.panelDriver.set(power * 0.75);
	}

	/**
	 * Get the Color from the ColorMatchResult Class
	 *
	 * @return Closest Color
	 */
	public Color getColor() {
		ColorMatchResult returnColor = this.colorMatch.matchClosestColor(colorSensor.getColor());

		return returnColor.color;
	}

	public Color GameColor() {
		Color value = null;

		if (gameData.length() > 0) {
			switch (gameData.charAt(0)) {
				case 'B':
					value = ColorTargets.COLOR_BLUE;
					break;
				case 'G':
					value = ColorTargets.COLOR_GREEN;
					break;
				case 'R':
					value = ColorTargets.COLOR_RED;
					break;
				case 'Y':
					value = ColorTargets.COLOR_YELLOW;
					break;
			}
		}

		return value;
	}

	@Override
	public void periodic() {
		this.gameData = DriverStation.getInstance().getGameSpecificMessage();

		// Save the Color to the Dashboard
		String colorTag = ColorTargets.resolveColor(this.getColor());
		this.colorSensorEntry.setString(colorTag);
		this.FMSColorEntry.setString(this.gameData);

		// Save the Encoder Value to the Dashboard
		this.colorEncoderEntry.setDouble(this.colorMatchCounter.get());

		if (this.getUpper() && !this.colorCommand.isScheduled() && !this.colorCommandComplete.get()) {
			this.colorCommand.schedule();
		}
		else if (!this.getUpper() && this.colorCommand.isScheduled()) {
			this.colorCommand.cancel();
		}

		if (this.getLower() && this.colorCommandComplete.get()) {
			// Reset the Is Completed Command
			this.colorCommandComplete.set(false);
		}

	}

}
