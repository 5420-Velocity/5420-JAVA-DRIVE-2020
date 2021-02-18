/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class JoystickDriveTankdriveLocked extends JoystickDrive {

	public JoystickDriveTankdriveLocked(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation, Joystick DPADController) {
		super(driveTrain, speed, rotation, DPADController);
	}

	@Override
	public void executeDrive() {
		// Use the Locked Tank Drive
		double controllerY = (speed.getAsDouble() * 0.95);

		// driveTrain.arcadeDrive(-controllerY, controllerX);
		driveTrain.arcadeDrive(0, controllerY);
	}

}
