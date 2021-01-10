/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeUp extends CommandBase {

	private Intake intake;

	public IntakeUp(Intake intakeRun) {
		this.intake = intakeRun;
		addRequirements(intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (intake.getEncoderValue() > 0) {
			intake.armRun(-0.8);
		}
		else {
			intake.armRun(0);
		}

		/**
		 * new PIDCommand(
		 pidController,
		 () -> intake.getEncoderFromHighValue(),
		 0.0,
		 output -> intake.armRun(-output),
		 intake
		 )
		 */

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
