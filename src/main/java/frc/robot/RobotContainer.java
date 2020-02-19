/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ButtonMapConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.EncoderArm;
import frc.robot.commands.AutoPanel;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ChuteSubsystem;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  private final Joystick driverJoystick = new Joystick(ControllerConstants.JOYSTICK_USB_DRIVER);
  private final Joystick operatorJoystick = new Joystick(ControllerConstants.JOYSTICK_USB_OPERATOR);


  private final NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  private final NetworkTableEntry entryColorSensor = tableInstance.getEntry(Constants.NetworkTableEntries.COLOR_VALUE);
  private final NetworkTableEntry encoder = tableInstance.getEntry(Constants.NetworkTableEntries.ENCODER_VALUE);

  private final DriveTrain driveTrain = new DriveTrain();
  private final ControlPanelController controlPanelController = new ControlPanelController(entryColorSensor);

  private static int index;

  private final AutoPanel AutoPanel = new AutoPanel(controlPanelController, 
    () -> {return driverJoystick.getRawButton(ButtonMapConstants.Yellow_Button_ID);},
    index);

  private final JoystickDrive joystickDrive = new JoystickDrive(driveTrain, 
    () -> {return driverJoystick.getRawAxis(1);}, 
    () -> { return driverJoystick.getRawAxis(4);});

  /**
   * Setup Intake Subsystm and the Extra Commands to Contorl It
   */
  private final Intake intake = new Intake();
  private final IntakeDown intakeRun = new IntakeDown(intake);

  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final ChuteSubsystem chute = new ChuteSubsystem();

  private final Shoot shoot = new Shoot(shooter,
    () -> {return operatorJoystick.getRawButton(Constants.ButtonMapConstants.Yellow_Button_ID); },
    () -> {return operatorJoystick.getRawButton(5);},
    () -> {return operatorJoystick.getRawButton(4);});

  // Encoder PID
  private PIDController pidController = new PIDController(
    EncoderArm.Proportional, 
    EncoderArm.Integral, 
    EncoderArm.Derivative);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Call of the configuration helper functions.
    configurePIDControllers();
    configureNetworkTableBindings();
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Configure PID Controlelrs that are stored in the current
   *  RobotContainer instance.
   * 
   */
  private void configurePIDControllers() {
    
    this.pidController.setTolerance(0.05);
    this.pidController.setIntegratorRange(-0.7, 0.7);

  }

  /**
   * Configure network table entries with the default values.
   * 
   */
  private void configureNetworkTableBindings() {

  }
  
  /**
   * Define button to command mappings.
   * 
   */
  private void configureButtonBindings() {

    /**
     * Setup Button Events for the Shooter on the Driver Controller
     */
    new JoystickButton(this.driverJoystick, Constants.ButtonMapConstants.Blue_Button_ID)
      .whenPressed(() -> this.shooter.setSpeed(-0.8, -0.7))
      .whenPressed(() -> this.chute.setSpeed(0.5))
      .whenReleased(() -> this.chute.setSpeed(0))
      .whenReleased(() -> this.shooter.setSpeed(0,0));

    /**
     * Setup the button event for the Intake on the Operator Controller
     */
    new JoystickButton(this.operatorJoystick, ButtonMapConstants.Green_Button_ID)
      // Go Down on Button Press
      .whenHeld(
        new PIDCommand(
          this.pidController,
          () -> this.intake.getEncoderFromLowValue(),
          0.0,
          output -> this.intake.armRun(output),
          this.intake
        )
      );

  }

  /**
   * Configure Default Commands for the Subsystems
   * 
   */
  private void configureDefaultCommands() {
    CommandScheduler scheduler = CommandScheduler.getInstance();

    scheduler.setDefaultCommand(this.driveTrain, this.joystickDrive);

    // Go Up By Default
    scheduler.setDefaultCommand(this.intake, new PIDCommand(
      this.pidController,
      () -> this.intake.getEncoderFromHighValue(),
      0.0,
      output -> intake.armRun(output),
      this.intake
    ));
    
  }
 
}
