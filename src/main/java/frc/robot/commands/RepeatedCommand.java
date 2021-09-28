package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * RepeatedCommand
 * This will run a set command a set amount of times then exit.
 */
public class RepeatedCommand extends CommandBase {

	private final Command command;
	private final int loopCountLimit;
	private int loopCount;
	private boolean isFinished = false;

	public RepeatedCommand(Command command) {
		this(2, command);
	}

	public RepeatedCommand(int loopCountLimit, Command command) {
		this.command = command;
		this.loopCountLimit = loopCountLimit;
		this.m_requirements = this.command.getRequirements();
	}

	@Override
	public void initialize() {
		this.isFinished = false;
		this.loopCount = 0;
	}

	@Override
	public void execute() {
		if (this.loopCount == this.loopCountLimit) {
			this.isFinished = true;
			return;
		}

		if (!this.command.isScheduled()) {
			// Not Scheduled
			this.loopCount++;
			this.command.schedule();
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (this.command.isScheduled()) {
			this.command.cancel();
		}
		this.isFinished = true;
	}

	@Override
	public boolean isFinished() {
		// Hold the command open if the command is running.
		return this.isFinished && this.command.isFinished();
	}

	@Override
	public boolean runsWhenDisabled() {
		return this.command.runsWhenDisabled();
	}
}
