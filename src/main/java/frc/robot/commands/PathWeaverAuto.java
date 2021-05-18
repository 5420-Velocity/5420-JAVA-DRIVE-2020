/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PathWeaverAuto extends CommandBase {
  private DriveTrain driveTrain;
  private CommandBase finalCommand;
  private Trajectory trajectory = new Trajectory();

  public PathWeaverAuto(DriveTrain driveTrain, String trajectoryJSON) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		}

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	
		// Reset odometry to the starting pose of the trajectory.
		this.driveTrain.resetOdometry(trajectory.getInitialPose());
	
		// Run path following command, then stop at the end.

    this.finalCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.finalCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.finalCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finalCommand.isFinished();
  }
}
