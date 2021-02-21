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

	DoubleSupplier leanLeft;
	DoubleSupplier leanRight;

	public JoystickDriveArcadeSplit(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation, Joystick DPADController) {
		super(driveTrain, speed, rotation, DPADController);
		this.leanLeft = () -> 0;
		this.leanRight = () -> 0;
	}

	public JoystickDriveArcadeSplit(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation, Joystick DPADController, DoubleSupplier leanLeft, DoubleSupplier leanRight) {
		super(driveTrain, speed, rotation, DPADController);
		this.leanLeft = leanLeft;
		this.leanRight = leanRight;
	}

	@Override
	public boolean shouldDrive() {
		return (
			Math.abs(speed.getAsDouble()) > 0.03
			|| Math.abs(rotation.getAsDouble()) > 0.03
			|| this.leanLeft.getAsDouble() > 0.1
			|| this.leanRight.getAsDouble() > 0.1
		);
	}

	@Override
	public void executeDrive() {
		// Intake forward
		double fast = 0.7;
		double slow = 0.5;

		// Flip the controls of the drive forward and reverse code
		if (this.isControlFlipped == true) {
			// Shooter forward
			slow = -0.85;
			fast = -0.35;
		}

		if (this.leanLeft.getAsDouble() > 0.5) {
			driveTrain.tankDrive(fast, slow);
		}
		else if (this.leanRight.getAsDouble() > 0.5) {
			driveTrain.tankDrive(slow, fast);
		}
		else {
			double controllerY = (-super.speed.getAsDouble() * 0.85);
			double controllerX = -rotation.getAsDouble() * 0.6;

			// Flip the controls of the drive forward and reverse code
			if (this.isControlFlipped == true) controllerY = controllerY * -1;

			// Apply a curve to the given input controls.
			controllerY = JoystickDrive.getCurve(controllerY);

			// driveTrain.arcadeDrive(-controllerY, controllerX);
			driveTrain.arcadeDrive(controllerY, controllerX);
		}
	}

}
