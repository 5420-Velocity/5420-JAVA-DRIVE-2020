/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDrive extends CommandBase  {

	private final DriveTrain driveTrain;
	private final DoubleSupplier speed;
	private final DoubleSupplier rotation;

	public JoystickDrive(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.driveTrain = driveTrain;
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
		if (Math.abs(speed.getAsDouble()) > 0.15 || Math.abs(rotation.getAsDouble()) > 0.15) {
			driveTrain.arcadeDrive((speed.getAsDouble()), rotation.getAsDouble());
		} else {
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
