package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothingAutoCommand extends CommandBase {
	/**
	 * Creates a new DoNothingAutoCommand.
	 */
	public DoNothingAutoCommand() {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
