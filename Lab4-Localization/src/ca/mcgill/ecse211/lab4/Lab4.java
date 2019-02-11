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

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice; // variable to record button clicked by user

    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    Display odometryDisplay = new Display(LCD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    // CS sensor initialization
    SensorModes csSensor = new EV3ColorSensor(CS_PORT);
    SampleProvider cs = csSensor.getMode("Red");
    float[] csData = new float[cs.sampleSize()];

    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("Falling |  Rising ", 0, 2);
      LCD.drawString(" Edge   |   Edge  ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);

    if (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {
      // Falling Edge or Rising Edge

      UltrasonicLocalizer usloc =
          new UltrasonicLocalizer(buttonChoice, usDistance, usData, odometer);

      Thread odoThread = new Thread(odometer);
      Thread odoDisplayThread = new Thread(odometryDisplay);
      Thread uslocThread = new Thread(usloc);

      odoThread.start();
      odoDisplayThread.start();
      uslocThread.start();

      if (Button.waitForAnyPress() == Button.ID_ESCAPE) { // wait for user confirmation
        System.exit(0);
      } else {
        LightLocalizer lightloc = new LightLocalizer(cs, csData, odometer);
        Thread lightlocThread = new Thread(lightloc);
        lightlocThread.start();
      }

    } else {
      System.exit(0);
    }

    // keep the program from ending unless esc button is pressed
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {

    }
    System.exit(0); // exit program after esc pressed
  }
}
