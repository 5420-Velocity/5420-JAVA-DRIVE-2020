/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private final double clampSpeed;
	private final double tolerance;

	public DriveWithEncoder(DriveTrain subsystem, double targetDistance, boolean reversed) {
		this(subsystem, targetDistance, reversed, 0.8);
	}

	public DriveWithEncoder(DriveTrain subsystem, double targetDistance, boolean reversed, double clampSpeed) {
		this(subsystem, targetDistance, reversed, clampSpeed, 8);
	}

	public DriveWithEncoder(DriveTrain subsystem, double targetDistance, boolean reversed, double clampSpeed, double tolerance) {
		this.driveTrain = subsystem;
		this.encoderTarget = targetDistance;
		this.reversed = reversed;
		this.clampSpeed = clampSpeed;
		this.tolerance = tolerance;

		if(targetDistance > 40) {
			this.drivePidController = new PIDController(
			DriveTrainConstants.LongEncoderP,
			DriveTrainConstants.LongEncoderI,
			DriveTrainConstants.LongEncoderD);
		}
		else{
			this.drivePidController = new PIDController(
			DriveTrainConstants.ShortEncoderP,
			DriveTrainConstants.ShortEncoderI,
			DriveTrainConstants.ShortEncoderD);
		}

		System.out.println("Command::DriveWithEncoder:" + this.hashCode() + ": CONSTRUCT");

		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		SmartDashboard.putString("Command", "Command::DriveWithEncoder:" + this.hashCode() + ": INIT");

		this.driveTrain.resetEncoders();

		this.state = StateList.bool(6);

		this.pidCommand = new PIDCommand(drivePidController,
			() -> Math.abs(this.driveTrain.getRightEncoderPosition()),
			encoderTarget,
			output -> {
				if(this.reversed == true){
					output = -output;
				}
				output = MathUtil.clamp(output, -clampSpeed, clampSpeed);
				this.driveTrain.tankDrive(-output, -output);
			}
		);

		// Start the PID Command to run the task.
		this.pidCommand.schedule();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (this.reversed == true) {
			if (this.driveTrain.getRightEncoderPosition() < 0) {
				if(Math.abs(Math.abs(this.driveTrain.getRightEncoderPosition()) - encoderTarget) < this.tolerance) {
					System.out.println(":: At Target (from LSS 0)");
					this.state.add(true);
				}
			}
			else {
				this.state.add(false);
			}
		}
		else {
			if(Math.abs(this.driveTrain.getRightEncoderPosition() - encoderTarget) < this.tolerance) {
				System.out.println(this.driveTrain.getRightEncoderPosition());
				System.out.println(":: At Target (from GTR 0)");
				this.state.add(true);
			}
			else {
				this.state.add(false);
			}
		}

		this.pidCommand.execute();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

		// Stop the PID Command to run the task.
		this.pidCommand.cancel();
		
		SmartDashboard.putString("Command", "Command::DriveWithEncoder:" + this.hashCode() + ": END");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.state.get();
	}
}
