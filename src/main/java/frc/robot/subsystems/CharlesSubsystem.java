/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.charlesConstants;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;
import java.util.HashMap;
import java.util.Map;
import frc.robot.Slot;
import frc.robot.exceptions.*;

public class CharlesSubsystem extends SubsystemBase {

  private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.charlesConstants.encoderPort);
  private WPI_TalonSRX charlesMotor = new WPI_TalonSRX(charlesConstants.charlesMotor);
  private I2C.Port ColorSensor = I2C.Port.kMXP;
  private ColorSensorV3 charlesColorSensor = new ColorSensorV3(ColorSensor);
  private ColorMatch colorMatch = new ColorMatch();

  private HashMap<Integer, Boolean> storedBalls = new HashMap<Integer, Boolean>();

  public int index;

  public CharlesSubsystem() {
   
  }

  public double getEncoder() {
    return this.encoder.get();
  }

  public int getIndex() throws UnindexPositionException
  {
    double currentEncoderPosition = this.encoder.getDistance();

    for(Slot slot : Constants.charlesConstants.slots) {
      if (slot.inBetween(currentEncoderPosition)) {
        return slot.position;
      }
    }

    throw new UnindexPositionException(currentEncoderPosition);
  }

  public void ballReset() {
    this.storedBalls.clear();
  }

  public void inTook() {
    if(index > 4) {
      index = 0;
    }

    this.storedBalls.put(index, true);
    index++;
  }

  public void reset() {
    this.encoder.reset();
  }

  public Color getColor() {
    return this.colorMatch.matchClosestColor(charlesColorSensor.getColor()).color;
  }
  
  public void encoderRun(double target) {
    if(encoder.get() < target) {
      charlesMotor.set(0.3);
    }
    else {
      charlesMotor.set(0);
    }
  }

  public void gotoSlot(int location) {
    // TODO: Read the encoder values, keep tunring until it true.
  }

  public boolean hasBallAtIndex(int x) {
    return this.storedBalls.get(x);
  }

  public void nextOpenLocation() {
    int firstOpen = this.storedBalls.entrySet()
        .stream()
        .filter(x -> x.getValue() == true)
        .findFirst()
        .get()
        .getKey();

    this.gotoSlot(firstOpen);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
