package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

/**
 * Command to set the pipeline on the limelight.
 *
 * This command is used because there is a delay between the SET and
 * the target pipeline kicking in.
 */
public class LimelightSetPipeline extends CommandBase {

	private final Limelight ll;
	private final int targetPipeline;

	/**
	 * Creates a new DoNothingAutoCommand.
	 */
	public LimelightSetPipeline(Limelight ll, int targetPipeline) {
		this.ll = ll;
		this.targetPipeline = targetPipeline;
	}

	public void initialize() {
		this.ll.setPipeline(targetPipeline);
	}

	@Override
	public boolean isFinished() {
		return this.ll.getPipeline() == this.targetPipeline;
	}

}
