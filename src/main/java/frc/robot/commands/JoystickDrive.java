/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.utils.DPad;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class JoystickDrive extends CommandBase {

	protected final DriveTrain driveTrain;
	protected final DoubleSupplier speed;
	protected final DoubleSupplier rotation;
	protected final Joystick DPADController;
	protected boolean isControlFlipped = true;
	private boolean isFinished = false;

	/**
	 * This function will get the input value on a scale of
	 * a curve using the Square Root Function
	 *
	 * @param input
	 * @return Curved Value
	 */
	public static double getCurve(double input) {
		double sign = Math.signum(input);

		double value = Math.abs(input);
		value = Math.pow(value, 2);
		value += 0.02;

		return sign * value;
	}

	public JoystickDrive(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation, Joystick DPADController) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.driveTrain = driveTrain;
		this.DPADController = DPADController;
		this.speed = speed;
		this.rotation = rotation;
		addRequirements(driveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		this.isFinished = false;
	}

	public void setFlipped(boolean isFlipped) {
		this.isControlFlipped = isFlipped;
	}

	public void toggleFlipped() {
		this.setFlipped(!this.isControlFlipped);
	}

	public boolean shouldDrive() {
		return Math.abs(speed.getAsDouble()) > 0.03 || Math.abs(rotation.getAsDouble()) > 0.03;
	}

	public void executeDrive() {
		double controllerY = (-speed.getAsDouble() * 0.95);
		double controllerX = -rotation.getAsDouble() * 0.7;

		// Flip the controls of the drive forward and reverse code
		if (this.isControlFlipped) controllerY = controllerY * -1;

		// Apply a curve to the given input controls.
		controllerY = JoystickDrive.getCurve(controllerY);

		// driveTrain.arcadeDrive(-controllerY, controllerX);
		driveTrain.tankDrive(controllerX, controllerY);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		if (this.isFinished) return;

		// Apply a Deadband to the Input mapped at 0.03
		if (this.shouldDrive()) {
			this.executeDrive();
		}
		else if (DPad.up(DPADController)) {
			driveTrain.arcadeDrive(0.4, 0);
		}
		else if (DPad.down(DPADController)) {
			driveTrain.arcadeDrive(-0.4, 0);
		}
		else if (DPad.right(DPADController)) {
			driveTrain.arcadeDrive(0, -0.4);
		}
		else if (DPad.left(DPADController)) {
			driveTrain.arcadeDrive(0, 0.4);
		}
		else {
			// Set the motor to zero, Its not out of the deadband
			 driveTrain.arcadeDrive(0, 0);
		}

		SmartDashboard.putBoolean("Inverted", this.isControlFlipped);
		SmartDashboard.putString("Front", this.isControlFlipped ? "Shooter" : "Intake");

	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		this.isFinished = true;
		driveTrain.arcadeDrive(0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.isFinished;
	}

}
