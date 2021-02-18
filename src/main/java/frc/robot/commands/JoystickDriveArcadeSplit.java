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

public class JoystickDriveArcadeSplit extends JoystickDrive {

	public JoystickDriveArcadeSplit(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation, Joystick DPADController) {
		super(driveTrain, speed, rotation, DPADController);
	}

	@Override
	public void executeDrive() {
		double controllerY = (-super.speed.getAsDouble() * 0.95);
		double controllerX = -rotation.getAsDouble() * 0.7;

		// Flip the controls of the drive forward and reverse code
		if (this.isControlFlipped == true) controllerY = controllerY * -1;

		// Apply a curve to the given input controls.
		controllerY = JoystickDrive.getCurve(controllerY);

		// driveTrain.arcadeDrive(-controllerY, controllerX);
		driveTrain.tankDrive(controllerX, controllerY);
	}

}
