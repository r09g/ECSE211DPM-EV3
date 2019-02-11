package ca.mcgill.ecse211.lab4;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

/**
 * This class contains Lab4 Localization Lab implemented on the EV3 platform. This class specifies
 * constants that control the behaviour of the robot. This class also creates instances of the left
 * and right motors (two Large EV3 motors), the LCD display, and the sensors. Users are given the
 * option of choosing a mode of ultrasonic sensor localization, whether to detect falling edges or
 * rising edges. The program is started upon choosing the mode. After the ultrasonic sensor
 * localization completes, the user has to click any button other than the esc button to proceed to
 * light localization. The robot then corrects its position to the origin.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */
public class Lab4 {

  // -----------------------------------------------------------------------------
  // Constants
  // -----------------------------------------------------------------------------

  /**
   * The radius (in cm) of the left/right wheels of the EV3 robot.
   */
  public static final double WHEEL_RAD = 2.15;

  /**
   * The width (in cm) of the robot measured from the center of the left wheel to the center of the
   * right wheel
   */
  public static final double TRACK = 13.3;

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  public static final int SMOOTH_ACCELERATION = 500;
  
  /**
   * Specifies the speed of the left and right EV3 Large motors
   */
  public static final int SPEED = 100;

  /**
   * The heading/Theta value of the robot initially
   */
  public static final int INITIAL_ANGLE = 0;
  
  /**
   * A revolution of half of a circle in degrees
   */
  public static final int HALF_CIRCLE = 180;
  
  /**
   * A full revolution of a circle in degrees
   */
  public static final int FULL_CIRCLE = 360;

  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The port on the EV3 brick that the ultrasonic sensor is connected to.
   */
  private static final Port US_PORT = LocalEV3.get().getPort("S1");

  /**
   * The port on the EV3 brick that the color sensor is connected to.
   */
  private static final Port CS_PORT = LocalEV3.get().getPort("S2");

  /**
   * The LCD display instance, provides access and control of the material displayed on the screen.
   */
  private static final TextLCD LCD = LocalEV3.get().getTextLCD();

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * In this class, sensors are initiated and display is setup. User is given the option to choose a
   * mode of ultrasonic sensor localization. Program responses to user choices are specified here.
   * 
   * @param args - None
   * @throws OdometerExceptions - Thrown by odometer; only one instance of Odometer is allowed to
   *         exist
   */
  public static void main(String[] args) throws OdometerExceptions {

    // Variable to record button clicked by user
    int buttonChoice;

    // Initiates an odometer instance, used for recording and updating vehicle position data
    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    // The LCD display of the robot, allows control of information to be displayed
    Display odometryDisplay = new Display(LCD);

    // Ultrasonic sensor initialization
    // 1. get sensor instance to set operating mode
    // 2. set operating mode and setup uniform data reader
    // 3. buffer to store data read from sensor
    // The 3 steps correspond to the three lines of code setting up the sensor
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    // Color sensor initialization
    // Similar process to Ultrasonic sensor initialization above
    SensorModes csSensor = new EV3ColorSensor(CS_PORT);
    SampleProvider cs = csSensor.getMode("Red");
    float[] csData = new float[cs.sampleSize()];

    // Set up LCD display on EV3 brick
    // Option to choose mode for ultrasonic sensor localization
    // This prompt is repeated until user presses right, left, or esc button
    do {
      // Clear LCD display
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("Falling |  Rising ", 0, 2);
      LCD.drawString(" Edge   |   Edge  ", 0, 3);
      // Record button choice
      buttonChoice = Button.waitForAnyPress();
      // Condition to repeat prompt
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);

    // Left or right button: falling edge or rising edge
    // esc button: exits program
    if (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {

      // Set up ultrasonic sensor localization
      UltrasonicLocalizer usloc =
          new UltrasonicLocalizer(buttonChoice, usDistance, usData, odometer);

      // Create threads of odometer, display, and ultrasonic sensor localizer program
      Thread odoThread = new Thread(odometer);
      Thread odoDisplayThread = new Thread(odometryDisplay);
      Thread uslocThread = new Thread(usloc);

      // Start threads
      odoThread.start();
      odoDisplayThread.start();
      uslocThread.start();

      // Allows verification of ultrasonic localization before starting light localization
      if (Button.waitForAnyPress() == Button.ID_ESCAPE) {
        // Exits system if esc button pressed
        System.exit(0);
      } else {
        // Light localizer started
        LightLocalizer lightloc = new LightLocalizer(cs, csData, odometer);
        Thread lightlocThread = new Thread(lightloc);
        lightlocThread.start();
      }

    } else {
      // exits program
      System.exit(0);
    }

    // keep the program from ending unless esc button is pressed
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {

    }
    System.exit(0); // exit program after esc pressed
  }
}
