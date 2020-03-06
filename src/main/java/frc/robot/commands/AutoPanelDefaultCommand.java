/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ControlPanelController;

public class AutoPanelDefaultCommand extends CommandBase {

	private ControlPanelController controlPanelController;
	private DoubleSupplier liftInput;


	public AutoPanelDefaultCommand(ControlPanelController controlPanelController, DoubleSupplier liftInput) {
		this.controlPanelController = controlPanelController;
		this.liftInput = liftInput;

		addRequirements(controlPanelController);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Set Inital Color
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Lift Control

		if(controlPanelController.getUpper() == false && liftInput.getAsDouble() > 0) {
			controlPanelController.liftSpeed(0);
		}
		else if(controlPanelController.getlower() == false && liftInput.getAsDouble() < 0) {
			controlPanelController.liftSpeed(0);
		}
		else {
			controlPanelController.liftSpeed(liftInput.getAsDouble());
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
