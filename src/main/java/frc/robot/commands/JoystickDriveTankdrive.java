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
import java.util.function.BooleanSupplier;
import frc.robot.DPad;

public class JoystickDriveTankdrive extends JoystickDrive {

	private final BooleanSupplier lockTogerthSupplier;
	private final Joystick DPADLean;

	public JoystickDriveTankdrive(DriveTrain driveTrain, BooleanSupplier lockTogerthSupplier, DoubleSupplier speed, DoubleSupplier rotation, Joystick DPADLean, Joystick DPADController) {
		super(driveTrain, speed, rotation, DPADController);
		this.lockTogerthSupplier = lockTogerthSupplier;
		this.DPADLean = DPADLean;
	}

	@Override
	public void executeDrive() {

		// Use the Tank Drive
		double leftDrive = (rotation.getAsDouble() * 0.95);
		double rightDrive = (speed.getAsDouble() * 0.95);

		// Flip the controls of the drive forward and reverse code
		if (this.isControlFlipped == true) leftDrive = leftDrive * -1;
		if (this.isControlFlipped == true) rightDrive = rightDrive * -1;

		if (this.lockTogerthSupplier.getAsBoolean() == true) {
			// Lock Together. The Right Controller's value is used as the control
			leftDrive = rightDrive;

			if (this.isControlFlipped == true) {
				// Lean Control, This will give the emulation of one side dragging
				if (DPad.left(this.DPADLean)) {
					rightDrive = rightDrive * 0.6;
				}
				else if (DPad.right(this.DPADLean)) {
					leftDrive = leftDrive * 0.6;
				}
			}
			else {
				// Lean Control, This will give the emulation of one side dragging
				if (DPad.left(this.DPADLean)) {
					leftDrive = leftDrive * 0.6;
				}
				else if (DPad.right(this.DPADLean)) {
					rightDrive = rightDrive * 0.6;
				}
			}

		}

		driveTrain.tankDrive(leftDrive, rightDrive);
	}
	
}
