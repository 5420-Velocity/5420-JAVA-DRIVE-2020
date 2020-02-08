/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private Encoder encoder = new Encoder(Constants.EncoderArm.encoderPort1, Constants.EncoderArm.encoderPort2, false, Encoder.EncodingType.k2X);
  private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.EncoderArm.armMotor);
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.EncoderArm.intakeMotor);

  public Intake() {
    //config
    encoder.setDistancePerPulse(4./256.);
    encoder.setMaxPeriod(.1);
    encoder.setMinRate(10);
    encoder.setReverseDirection(true);
    encoder.setSamplesToAverage(5);
  }

  public double getEncoderValue(){
    return encoder.getDistance();
  }

  public void ArmRun(double power){
    armMotor.set(power);
  }

  public void IntakeMove(double power){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
