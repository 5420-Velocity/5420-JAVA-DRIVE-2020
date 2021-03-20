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
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.StateList;

public class DriveWithEncoder extends CommandBase {

	private final DriveTrain driveTrain;
	private final double encoderTarget;
	private final PIDController drivePidController;
	private PIDCommand pidCommand;
	private final boolean reversed;
	private StateList<Boolean> state;

	public DriveWithEncoder(DriveTrain subsystem, double targetDistance, boolean reversed) {
		this.driveTrain = subsystem;
		this.encoderTarget = targetDistance;
		this.reversed = reversed;

		this.drivePidController = new PIDController(
			DriveTrainConstants.EncoderP,
			DriveTrainConstants.EncoderI,
			DriveTrainConstants.EncoderD);

		this.state = StateList.bool();
		
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.driveTrain.resetEncoders();

		this.pidCommand = new PIDCommand(drivePidController,
		() -> {
			double pos = (this.driveTrain.getRightEncoderPosition());
			return Math.abs(pos);
		},
		encoderTarget,
		output -> {
			if(this.reversed == true){
				output = -output;
			}
			output = MathUtil.clamp(output, -0.8, 0.8);
			this.driveTrain.tankDrive(-output, -output);
		});


		// Start the PID Command to run the task.
		this.pidCommand.schedule();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(Math.abs(this.driveTrain.getRightEncoderPosition()  - encoderTarget) < 2) {
			this.state.add(true);	
		}
		else {
			this.state.add(false);
		}
		
		this.pidCommand.execute();
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
		return this.state.get();
	}
}
