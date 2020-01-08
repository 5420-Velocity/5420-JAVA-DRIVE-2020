/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.JoystickDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax Left1 = new CANSparkMax(RobotMap.Left1, MotorType.kBrushless );
  private CANSparkMax Left2 = new CANSparkMax(RobotMap.Left2, MotorType.kBrushless );

  private CANSparkMax Right1 = new CANSparkMax(RobotMap.Right1, MotorType.kBrushless );
  private CANSparkMax Right2 = new CANSparkMax(RobotMap.Right2, MotorType.kBrushless );

  private DifferentialDrive drive;


  public DriveTrain(){
    Left2.follow(Left1);
    Right2.follow(Right1);

    drive = new DifferentialDrive(Left1, Right1);
  }

  public void arcadeDrive(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }
}
