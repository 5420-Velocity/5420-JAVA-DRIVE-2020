/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelController;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

public class PanelLiftUp extends CommandBase {

	private final ControlPanelController panelControllerSubsystem;
	private boolean isFinished = false;
	private Date EStopCheckTime;

	public PanelLiftUp(ControlPanelController subsystem) {
		this.panelControllerSubsystem = subsystem;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, Constants.ControlPanelConstants.timeOutTime);
		this.EStopCheckTime = calculateDate.getTime();

		this.isFinished = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Safety time out
		if (new Date().after(EStopCheckTime) && !this.panelControllerSubsystem.getUpper()) {
			// Quit Early, Must not be connected.
			System.err.println("E-Stop >> Code Detected the upper limit on the control pannel isn't detected.");

			this.isFinished = true;
		}

		// Run till upper limit
		if (!this.panelControllerSubsystem.getUpper()) {
			this.panelControllerSubsystem.liftSpeed(1);
		}
		else {
			this.isFinished = true;
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.panelControllerSubsystem.liftSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return this.isFinished;
	}
}
