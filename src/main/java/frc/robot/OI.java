/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.DS_Map;
import frc.robot.Constants.ButtonMapConstants;

public class OI{

	Joystick driverJoystick = new Joystick(DS_Map.JOYSTICK_USB_DRIVER);

	public double getSpeed(){
		return driverJoystick.getRawAxis(ButtonMapConstants.JOYSTICK_LEFT_Y_AXIS) * 0.5;
	}

	public double getTurn(){
		return driverJoystick.getRawAxis(ButtonMapConstants.JOYSTICK_RIGHT_X_AXIS);
	}
	
}
