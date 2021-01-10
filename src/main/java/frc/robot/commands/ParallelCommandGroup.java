/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * CommandGroup
 * <p>
 * This allows you to run commands in a command group
 * in parallel calling all of the common functions.
 */
public class ParallelCommandGroup extends CommandBase {

	private Command[] commands;

	/**
	 * Creates a new CommandGroup.
	 */
	public ParallelCommandGroup(Command... commands) {
		this.commands = commands;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// Initialize Command Group Items
		for (Command command : this.commands) {
			command.initialize();
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Execute Commands in Command Group Item
		for (Command command : this.commands) {
			command.execute();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// Call End on the Command Group Items
		for (Command command : this.commands) {
			command.end(interrupted);
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// Check to see if the commands are Finished
		for (Command command : this.commands) {
			if (!command.isFinished()) return false;
		}
		return true;
	}
}
