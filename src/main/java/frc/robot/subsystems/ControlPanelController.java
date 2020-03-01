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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ColorMatchCounter;
import frc.robot.Constants;
import frc.robot.Constants.ColorTargets;
import frc.robot.Constants.ControlPanelConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class ControlPanelController extends SubsystemBase {

	private WPI_TalonSRX panelDriver = new WPI_TalonSRX(ControlPanelConstants.ControlPanelDriver);
	private WPI_TalonSRX panelLift = new WPI_TalonSRX(ControlPanelConstants.ControlPanelLift);
	private I2C.Port ColorSensor = I2C.Port.kOnboard;
	private ColorSensorV3 colorSensor = new ColorSensorV3(ColorSensor);
	private ColorMatch colorMatch = new ColorMatch();
	private ColorMatchCounter colorMatchCounter = new ColorMatchCounter(colorSensor, colorMatch);
	private NetworkTableEntry colorEncoderEntry = NetworkTableInstance.getDefault().getEntry(Constants.NetworkTableEntries.COLOR_ENCODER_VALUE);
	private NetworkTableEntry colorSensorEntry = NetworkTableInstance.getDefault().getEntry(Constants.NetworkTableEntries.COLOR_VALUE);
	private boolean rotationsCompleted;
	private boolean panelCompleted;
	private DigitalInput upperLimit = new DigitalInput(Constants.ControlPanelConstants.upperLimit);
	private DigitalInput lowerLimit = new DigitalInput(Constants.ControlPanelConstants.lowerLimit);

	public ControlPanelController() {
		this.colorSensorEntry.setDefaultString("");
		this.colorEncoderEntry.setDefaultDouble(0.0);

		this.colorMatch.addColorMatch(ColorTargets.COLOR_BLUE);
		this.colorMatch.addColorMatch(ColorTargets.COLOR_GREEN);
		this.colorMatch.addColorMatch(ColorTargets.COLOR_RED);
		this.colorMatch.addColorMatch(ColorTargets.COLOR_YELLOW);
	}

	public boolean getRotCompletion(){
		return rotationsCompleted;
	}

	public void rotationsCompleted(boolean val){
		rotationsCompleted = val;
	}

	public boolean getUpper(){
		return upperLimit.get();
	}

	public boolean getlower(){
		return lowerLimit.get();
	}

	public boolean isPannelComplete(){
		return panelCompleted;
	}

	public void panelCompleted(boolean val){
		panelCompleted = val;
	}

	public void liftSpeed(double power){
		panelLift.set(power);
	}

	public void turnSpeed(double power){
		panelDriver.set(power * 0.75);
	}

	public Color getColor(){
		ColorMatchResult returnColor = this.colorMatch.matchClosestColor(colorSensor.getColor());

		return returnColor.color;
	}

	public Color GameColor(){
		Color value = null;
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		if(gameData.length() > 0){
			switch (gameData.charAt(0)) {
				case 'B' :
					value = ColorTargets.COLOR_BLUE;
					break;
				case 'G' :
					value = ColorTargets.COLOR_GREEN;
					break;
				case 'R' :
					value = ColorTargets.COLOR_RED;
					break;
				case 'Y' :
					value = ColorTargets.COLOR_YELLOW;
					break;
				default :
					value = null;
					break;
			}
		}
		else {
			value = null;
		}

		return value;
	}

	@Override
	public void periodic() {
		
		// Save the Color to the Dashboard
		String colorTag = ColorTargets.resolveColor(this.getColor());
		this.colorSensorEntry.setString(colorTag);

		// Save the Encoder Value to the Dashboard
		this.colorEncoderEntry.setDouble(this.colorMatchCounter.get());

	}
}
