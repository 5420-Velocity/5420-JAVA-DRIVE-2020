package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //Drive motors
  public static int Left1 = 1;
  public static int Left2 = 2;
  public static int Right1 = 3;
  public static int Right2 = 4;

  public static int Left1T = 1;
  public static int Left2T = 2;


  //Joystick
  public static int JOYSTICK_USB_DRIVER = 0;
  public static int JOYSTICK_USB_OPERATOR = 1;

  public static int JOYSTICK_RIGHT_X_AXIS = 4;
  public static int JOYSTICK_RIGHT_Y_AXIS = 5;
  public static int JOYSTICK_LEFT_X_AXIS = 0;
  public static int JOYSTICK_LEFT_Y_AXIS = 1;

  public static int Red_Button_ID = 1;
  public static int Green_Button_ID = 0;
  public static int Yellow_Button_ID = 3;
  public static int Blue_Button_ID = 2;


}
