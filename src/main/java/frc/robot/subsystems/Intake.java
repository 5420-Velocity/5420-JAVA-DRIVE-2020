/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import  frc.robot.commands.IntakeUp;

public class Intake extends SubsystemBase {
  
  private Encoder encoder = new Encoder(Constants.EncoderArm.encoderPort1, Constants.EncoderArm.encoderPort2, false, Encoder.EncodingType.k2X);
  private WPI_TalonSRX armMotor = new WPI_TalonSRX(Constants.EncoderArm.armMotor);
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.EncoderArm.intakeMotor);
  private NetworkTableEntry ntEncoderValue = NetworkTableInstance.getDefault().getEntry("Ball Pickup Value");
  private NetworkTableEntry ntEncoderReset = NetworkTableInstance.getDefault().getEntry("Ball Pickup Reset");

  public Intake() {
    // Configure the Default Command for the Intake to go up
    IntakeUp intakeCommand = new IntakeUp(this);
    setDefaultCommand(intakeCommand);

    this.ntEncoderReset.setDefaultBoolean(false);

    /**
     * Setup the encoder setting
     */
    encoder.reset();
    encoder.setDistancePerPulse(4./256.);
    encoder.setMaxPeriod(.1);
    encoder.setMinRate(10);
    encoder.setReverseDirection(true);
    encoder.setSamplesToAverage(5);
  }

  public double getEncoderValue(){
    return encoder.getDistance();
  }

  // Get the offset of the low target and  current position
  public double getEncoderFromLowValue(){
    return this.getEncoderValue() - Constants.EncoderArm.highTarget;
  }

  // Get the offset of the high target and  current position
  public double getEncoderFromHighValue(){
    return this.getEncoderValue() - Constants.EncoderArm.highTarget;
  }

  public void encoderReset(){
    encoder.reset();
  }

  public void armRun(double power){
    armMotor.set(power);
  }

  public void intakeMove(double power){
    intakeMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * Push Encoder Value to the Dashbaord via NetworkTables.
     */
    this.ntEncoderValue.setDouble(this.getEncoderValue());

    /**
     * Reset the Encoder from the Dashboard
     */
    if(this.ntEncoderReset.getBoolean(false) == true) {
      this.encoderReset();
      this.ntEncoderReset.setBoolean(false);
    }

  }
}
