/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;


public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight() {
    CameraServer.getInstance().startAutomaticCapture();
  }

  public double getTX(){
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTY(){
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getArea(){
    return table.getEntry("ta").getDouble(0.0);
  }

  public boolean isValidTarget(){
    double check = table.getEntry("tv").getDouble(0.0);

    if(check == 1.0){
      return true;
    } else if(check == 0.0){
      return false;
    } else{
      return false;
    }
  }

  public void setCamMode(double value){
    table.getEntry("camMode").setDouble(value);
  }

  public void setStreamMode(double value){
    table.getEntry("stream").setDouble(value);
  }

  public void setPipeline(double value){
    table.getEntry("pipeline").setDouble(value);
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
