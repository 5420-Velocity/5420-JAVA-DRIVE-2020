/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class LiftControl extends CommandBase {
	/**
	 * Creates a new liftControl.
	 */
	private final LiftSubsystem liftSubsystem;
	private final Intake intakeSubsystem;
	private final DoubleSupplier inputUp;
	private final DoubleSupplier inputDown;

	public LiftControl(LiftSubsystem subsystem, Intake intakeSubsystem, DoubleSupplier inputUp, DoubleSupplier inputDown) {
		this.liftSubsystem = subsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.inputDown = inputDown;
		this.inputUp = inputUp;

		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (inputUp.getAsDouble() >= 0.1) {
			liftSubsystem.liftSpeed(-inputUp.getAsDouble());
			this.intakeSubsystem.forceArmDown(true);
		}
		else if (inputDown.getAsDouble() >= 0.1) {
			liftSubsystem.liftSpeed(inputDown.getAsDouble());
		}
		else {
			liftSubsystem.liftSpeed(0);
		}

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
