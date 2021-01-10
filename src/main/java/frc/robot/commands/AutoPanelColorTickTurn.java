/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.subsystems.ControlPanelController;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.concurrent.atomic.AtomicReference;

public class AutoPanelColorTickTurn extends CommandBase {

	private final ControlPanelController controlPanelController;
	private Date EStopCheckTime;
	private Color previous;
	private int index = 0;
	private boolean isFinished = false;
	private final AtomicReference<Boolean> commandCompleted;

	public AutoPanelColorTickTurn(ControlPanelController controlPanelController, AtomicReference<Boolean> commandCompleted) {
		this.controlPanelController = controlPanelController;

		addRequirements(controlPanelController);
		this.commandCompleted = commandCompleted;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, Constants.ControlPanelConstants.timeOutTime);
		this.EStopCheckTime = calculateDate.getTime();

		this.previous = controlPanelController.getColor();
		this.index = 0;
		this.isFinished = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (new Date().after(EStopCheckTime) && this.index == 0) {
			// Quit Early, Must not be connected.
			System.err.println("E-Stop >> Code Detected the Color wheel has not chagned color. Check Sensor!");

			this.isFinished = true;
		}

		// Update Changed Color
		if (this.controlPanelController.getColor() != this.previous) {
			this.index += 1;
			this.previous = this.controlPanelController.getColor();
		}

		// Completed rotations
		if (this.index >= ControlPanelConstants.targetRotations) {
			this.isFinished = true;
			this.commandCompleted.set(true);
		}
		else {
			// Start turning the Motor to turn the control pannel
			this.controlPanelController.turnSpeed(1);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.controlPanelController.turnSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.isFinished;
	}

}
