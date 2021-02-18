/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveWithEncoder extends CommandBase {

	private final DriveTrain driveTrain;
	private final double encoderTarget;
	private final PIDController drivePidController;
	private final PIDCommand pidCommand;
	private double currentLocation;

	public DriveWithEncoder(DriveTrain subsystem, double targetDistance) {
		this.driveTrain = subsystem;
		this.encoderTarget = targetDistance;

		this.drivePidController = new PIDController(
			DriveTrainConstants.EncoderP,
			DriveTrainConstants.EncoderI,
			DriveTrainConstants.EncoderD);

		this.pidCommand = new PIDCommand(
			drivePidController,
			() -> (this.encoderTarget - this.driveTrain.getLeftEncoderPosition()),
			0.0,
			output -> this.driveTrain.arcadeDrive(output, 0),
			this.driveTrain
		);

		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.driveTrain.zero();

		// Start the PID Command to run the task.
		this.pidCommand.schedule();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (this.currentLocation == 4) {
			// TODO
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

		// Stop the PID Command to run the task.
		this.pidCommand.cancel();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
