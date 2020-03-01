/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.DPad;
import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase  {

	private final DriveTrain driveTrain;
	private final DoubleSupplier speed;
	private final DoubleSupplier rotation;
	private final Joystick DPADController;

	/**
	 * This function will get the input value on a scale of
	 *  a curve using the Square Root Function
	 * 
	 * @param double
	 * @return Curved Value
	 */
	public static double getCurve(double input)
	{
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
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// Apply a Deadband to the Input mapped at 0.05
		if (Math.abs(speed.getAsDouble()) > 0.05 || Math.abs(rotation.getAsDouble()) > 0.05) {

			double controllerY = (-speed.getAsDouble()*0.9);
			double controllerX = -rotation.getAsDouble()*0.6;

			// Apply a curve to the given input controls.
			controllerY = JoystickDrive.getCurve(controllerY);
			controllerX = JoystickDrive.getCurve(controllerX);

			driveTrain.arcadeDrive(controllerY, controllerX);
		} 
		else if(DPad.up(DPADController)){
			driveTrain.arcadeDrive(0.1, 0);
		}
		else if(DPad.down(DPADController)){
			driveTrain.arcadeDrive(-0.1, 0);
		}
		else if(DPad.right(DPADController)){
			driveTrain.arcadeDrive(0, 0.1);
		}
		else if(DPad.left(DPADController)){
			driveTrain.arcadeDrive(0, -0.1);
		}
		else {
			// Set the motor to zero, Its not out of the deadband
			driveTrain.arcadeDrive(0, 0);
		}

	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
	
}
