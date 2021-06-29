/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakePIDCommand extends PIDCommand {
  /**
   * Creates a new IntakePIDCommand.
   */
  private Intake intake;
  public IntakePIDCommand(PIDController controller, Intake subsystem) {
    super(
        // The controller that the command will use
        controller,
        // This should return the measurement
        subsystem::getEncoderFromLowValue,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        subsystem::armSpeed
    );
        this.intake = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    System.out.println("start");
    intake.intakeMove(-1);;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("end");
    intake.intakeMove(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
