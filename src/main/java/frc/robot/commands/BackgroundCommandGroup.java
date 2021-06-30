package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * BackgroundCommandGroup
 * This is the same as the ParallelCommandGroup but waits for the "lead" command to finish and holds
 */
public class BackgroundCommandGroup extends CommandBase {

	private final Command m_leadCommand;
	private boolean leadCommandFinished = false;
	private boolean leadCommandEnded = false;
	private final Map<Command, Boolean> m_commands = new HashMap<>();
	private boolean m_runWhenDisabled = true;

	/**
	 * Creates a new ParallelCommandGroup. The given commands will be executed simultaneously. The
	 * command group will finish when the last command finishes. If the CommandGroup is interrupted,
	 * only the commands that are still running will be interrupted.
	 *
	 * @param commands the commands to include in this group.
	 */
	public BackgroundCommandGroup(Command leadCommand, Command ...commands) {
		this.m_leadCommand = leadCommand;
		m_requirements.addAll(leadCommand.getRequirements());
		m_runWhenDisabled &= leadCommand.runsWhenDisabled();
		addCommands(commands);
	}

	public final void addCommands(Command ...commands) {
		if (m_commands.containsValue(true)) {
			throw new IllegalStateException("Commands cannot be added to a CommandGroup while the group is running");
		}

		for (Command command : commands) {
			if (!Collections.disjoint(command.getRequirements(), m_requirements)) {
				throw new IllegalArgumentException("Multiple commands in a parallel group cannot require the same subsystems");
			}
			m_commands.put(command, false);
			m_requirements.addAll(command.getRequirements());
			m_runWhenDisabled &= command.runsWhenDisabled();
		}
	}

	@Override
	public void initialize() {
		for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
			commandRunning.getKey().initialize();
			commandRunning.setValue(true);
		}
	}

	@Override
	public void execute() {
		this.m_leadCommand.execute();
		if (this.m_leadCommand.isFinished()) {
			this.leadCommandFinished = true;
			return;
		}

		for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
			if (!commandRunning.getValue()) {
				continue;
			}
			commandRunning.getKey().execute();
			if (commandRunning.getKey().isFinished()) {
				commandRunning.getKey().end(false);
				commandRunning.setValue(false);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			if (!this.leadCommandEnded) {
				this.m_leadCommand.end(true);
				this.leadCommandEnded = true;
			}

			for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
				if (commandRunning.getValue()) {
					commandRunning.getKey().end(true);
				}
			}
		}
	}

	@Override
	public boolean isFinished() {
		// Hold the command open if the lead command is running.
		return this.m_leadCommand.isFinished();
	}

	@Override
	public boolean runsWhenDisabled() {
		return m_runWhenDisabled;
	}
}
