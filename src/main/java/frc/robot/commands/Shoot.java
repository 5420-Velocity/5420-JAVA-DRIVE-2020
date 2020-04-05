/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final ShooterSubsystem shooter;
  private final BooleanSupplier startShoot;
  private final BooleanSupplier aim1;
  private final BooleanSupplier aim2;

  public Shoot(ShooterSubsystem Shooter, BooleanSupplier startshoot, BooleanSupplier aim1, BooleanSupplier aim2) {
    this.aim1 = aim1;
    this.aim2 = aim2;
    this.shooter = Shooter;
    this.startShoot = startshoot;
    addRequirements(Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooting
    if(startShoot.getAsBoolean()){
      shooter.setSpeed(0.7, 0.8);
    }
    else{
      shooter.setSpeed(0, 0);
    }

    //aiming
    if(aim1.getAsBoolean()){
      shooter.turnSpeed(0.5);
    }
    else if(aim2.getAsBoolean()){
      shooter.turnSpeed(-0.5);
    }
    else{
      shooter.turnSpeed(0);
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
