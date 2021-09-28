/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Side;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class PixySearch extends CommandBase {
  /**
   * Creates a new PixySearch.
   */
  private Intake Intake;
  private DriveTrain DriveTrain;
  private boolean isFinished;

  private int duration = 1300;
  private Date finishingTime;
  private int numOscililations = 3;
  private int currentOscililations;

  private RobotContainer.Side primarySide;


  public PixySearch(Intake intake, DriveTrain driveTrain) {
    this.Intake = intake;
    this.DriveTrain = driveTrain;
    primarySide = Side.Left;
  }

  public PixySearch(Intake intake, DriveTrain driveTrain, RobotContainer.Side side) {
    this.Intake = intake;
    this.DriveTrain = driveTrain;
    primarySide = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
    currentOscililations = 0;

    if(primarySide == Side.Right){
      currentOscililations += 1;
      numOscililations += 1;
    }

    Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, this.duration);

		this.finishingTime = calculateDate.getTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Intake.pixyAlgo.getPixyBest() != null) {
      this.isFinished = true;
      System.out.println("Search Done");
    }

    if(new Date().after(this.finishingTime)) {
      currentOscililations += 1;
      Calendar calculateDate = GregorianCalendar.getInstance();

      if(currentOscililations % 2 == 0) {
        calculateDate.add(GregorianCalendar.MILLISECOND, this.duration + (currentOscililations * 200));
		    this.finishingTime = calculateDate.getTime();
      }
      else{
       calculateDate.add(GregorianCalendar.MILLISECOND, (int) 2.2 * this.duration + (currentOscililations * 200));
		   this.finishingTime = calculateDate.getTime();
      }
    }
    
    if(currentOscililations <= numOscililations) {
      if(currentOscililations % 2 == 0) {
        DriveTrain.arcadeDrive(0, 0.4);
      }
      else {
        DriveTrain.arcadeDrive(0, -0.4);
      }
    }
    else{
      this.isFinished = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
