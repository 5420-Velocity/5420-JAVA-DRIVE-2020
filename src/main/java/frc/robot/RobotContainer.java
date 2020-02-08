/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ButtonMapConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoPanel;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.ControlPanelController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

  private final DriveTrain driveTrain = new DriveTrain();
  private final ControlPanelController controlPanelController = new ControlPanelController(entryColorSensor);

  private static int index;

  private final AutoPanel AutoPanel = new AutoPanel(controlPanelController, 
  () -> {return driverJoystick.getRawButton(ButtonMapConstants.Yellow_Button_ID);}
  , index);

  private final JoystickDrive joystickDrive = 
    new JoystickDrive(driveTrain, 
      () -> {return driverJoystick.getRawAxis(1);}, 
      () -> { return driverJoystick.getRawAxis(4);});

  private final Intake intake = new Intake();

  private final Shooter shooter = new Shooter();

  private final IntakeRun encoderArmMove = new IntakeRun(intake,
  () -> {return operatorJoystick.getRawButton(Constants.ButtonMapConstants.Green_Button_ID);});

  private final Shoot shoot = new Shoot(shooter, 
  () -> {return operatorJoystick.getRawButton(Constants.ButtonMapConstants.Yellow_Button_ID); });

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  private void configureDefaultCommands() {
    CommandScheduler scheduler = CommandScheduler.getInstance();

    scheduler.setDefaultCommand(driveTrain, joystickDrive);
    //scheduler.registerSubsystem(driveTrain);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
}
