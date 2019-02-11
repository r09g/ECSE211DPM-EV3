package ca.mcgill.ecse211.lab4;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import java.util.Arrays;
// static imports
import static ca.mcgill.ecse211.lab4.Lab4.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.TRACK;
import static ca.mcgill.ecse211.lab4.Lab4.WHEEL_RAD;
import static ca.mcgill.ecse211.lab4.Lab4.SPEED;
import static ca.mcgill.ecse211.lab4.Lab4.INITIAL_ANGLE;
import static ca.mcgill.ecse211.lab4.Lab4.HALF_CIRCLE;
import static ca.mcgill.ecse211.lab4.Lab4.FULL_CIRCLE;
import static ca.mcgill.ecse211.lab4.Lab4.SMOOTH_ACCELERATION;

/**
 * <p>
 * This class implements localization with the ultrasonic sensor. This class extends the Thread
 * class to allow simultaneous execution, so that other classes can work alongside this class.
 * Helper methods are added at the end to make conversions easier.
 * 
 * <p>
 * The robot is to be placed at the lower left tile of the demo platform, on the 45 degree line
 * (from the top-right corner to the bottom-left corner). The robot localizes to a heading of 0
 * degrees. The robot uses the convention that North = 0 degrees and clockwise turning = increase in
 * heading degrees. There are two algorithms available, the falling edge and the rising edge, each
 * works best in certain scenarios. From testing, we find that falling edge works best if robot is
 * not facing the wall initially; and rising edge works best if robot is facing the wall initially.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */
public class UltrasonicLocalizer extends Thread {

  // -----------------------------------------------------------------------------
  // Constants
  // -----------------------------------------------------------------------------

  /**
   * The threshold distance (in cm) for detecting a falling edge
   */
  private static final double FE_THRESHOLD = 25;

  /**
   * This value (in cm) is used to ensure the robot continues to turn clockwise after first
   * detection of falling edge, from left wall to bottom wall. In other words, this constant is used
   * to force the robot out of the noise margin.
   */
  private static final double FE_CONTINUATION = 40;

  /**
   * The threshold distance (in cm) for detecting a rising edge
   */
  private static final double RE_THRESHOLD = 40;

  /**
   * This value (in cm) is used to ensure the robot continues to turn counter-clockwise after first
   * detection of rising edge, from left wall to bottom wall. In other words, this constant is used
   * to force the robot out of the noise margin.
   */
  private static final double RE_CONTINUATION = 25;

  /**
   * If button selected by user at start of program is LEFT, the corresponding localization mode is
   * falling edge. Improves readability
   */
  private static final int FALLING_EDGE = Button.ID_LEFT;

  /**
   * If button selected by user at start of program is RIGHT, the corresponding localization mode is
   * rising edge. Improves readability
   */
  private static final int RISING_EDGE = Button.ID_RIGHT;

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * User's choice of localization mode. Records the button selected by the user, either LEFT or
   * RIGHT. Note that the user is allowed to select other buttons, but LEFT and RIGHT are the only
   * values that can reach this class.
   */
  private int type;

  /**
   * The instance of the odometer. Provides access to data from odometer and access to setting
   * odometer data values
   */
  private Odometer odometer;

  /**
   * Array to store data retrieved from the odometer.
   */
  private double[] odoData;

  /**
   * An object to read sensor data in a uniform way (Meter-Kilogram-Second units)
   */
  private SampleProvider usDistance;

  /**
   * Buffer to store data read from the ultrasonic sensor
   */
  private float[] usData;

  // -----------------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------------

  /**
   * Constructor for this class. Initializes class variables with existing instances and sets the
   * motor speed and acceleration.
   * 
   * @param buttonChoice - The button selected by the user to choose mode of localization
   * @param usDistance - Data reader to read ultrasonic sensor data in uniform way
   * @param usData - Buffer to store data from ultrasonic sensor
   * @param odometer - An existing instance of the odometer
   */
  public UltrasonicLocalizer(int buttonChoice, SampleProvider usDistance, float[] usData,
      Odometer odometer) {
    // initializes variables with existing instances
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.type = buttonChoice;
    // specifies left and right motor behaviours
    setAcceleration(SMOOTH_ACCELERATION);
    setSpeed(SPEED);
  }

  // -----------------------------------------------------------------------------
  // Run Method
  // -----------------------------------------------------------------------------

  /**
   * The {@code run()} method required in a Thread class. In this class, the corresponding method of
   * the mode of localization, selected by the user, is called. After localization finishes, the
   * robot turns to face 0 degrees. Refer to conventions specified in the documentation for the
   * UltrasonicLocalizer class.
   */
  public void run() {

    // Stores the change in theta value required for localization
    double dTheta = 0;

    // Determines which mode of localization is to be executed
    if (type == FALLING_EDGE) {
      // Falling edge
      dTheta = fallingEdge();
    } else if (type == RISING_EDGE) {
      // Rising edge
      dTheta = risingEdge();
    }

    // Angle collection is complete, no need to rotate on-the-spot anymore
    stopMotors();

    // Update odometer theta values to values that are with respect to the convention (North = 0
    // degrees)
    odometer.setTheta(odometer.getXYT()[2] + dTheta);

    // Signal completion of heading angle update
    Sound.beep();

    // Turn to a heading of 0 degrees
    turnTo(0);

  }

  // -----------------------------------------------------------------------------
  // Private Methods
  // -----------------------------------------------------------------------------

  /**
   * Method for falling edge implementation. The method works best when robot is initially placed to
   * not face the wall. The robot first turns left to record a falling edge for the left wall, then
   * switches direction and turns to record another falling edge for the bottom wall. The
   * computation for the correction of the current heading angle is computed in another method but
   * called in this method.
   * 
   * @return the correction needed to be applied to the current heading for localization
   */
  private double fallingEdge() {

    // Angle corresponding to first falling edge
    double alpha;

    // Angle corresponding to second falling edge
    double beta;

    // Turn left until first falling edge detected
    while (medianFilter() > FE_THRESHOLD) {
      turnLeft();
    }

    // Signal detection of falling edge for left wall
    Sound.beep();

    // Stops the robot to change turning direction
    stopMotors();

    // Record angle for left wall. The angle is stored in index = 2 slot.
    alpha = odometer.getXYT()[2];

    // Turns the robot right and forces it out of the noise margin to avoid incorrect detection of
    // second falling edge
    while (medianFilter() < FE_CONTINUATION) {
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }
    Sound.beep();

    while (medianFilter() > 25) {
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }

    Sound.beep();

    beta = odometer.getXYT()[2];

    return correctAngle(alpha, beta);
  }

  /**
   * 
   */
  private double risingEdge() {

    double alpha, beta;

    try {
      Thread.sleep(2000);
    } catch (Exception e) {

    }

    while (medianFilter() < 40) {
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }

    stopMotors();

    Sound.beep();

    alpha = odometer.getXYT()[2];

    while (medianFilter() > 25) {
      turnLeft();
    }
    Sound.beep();

    while (medianFilter() < 40) {
      turnLeft();
    }
    Sound.beep();

    beta = odometer.getXYT()[2];

    return correctAngle(alpha, beta);

  }

  private double correctAngle(double alpha, double beta) {
    if (alpha > beta) {
      return 225.0 - (alpha + beta) / 2.0;
    } else {
      return 45.0 - (alpha + beta) / 2.0;
    }

  }

  /**
   * Turns the robot at a fixed position to face the next destination, changes the robot heading.
   * The minimum angle is calculated by determining whether the clockwise angle Theta is greater
   * than half of a full circles
   * 
   * @param Theta - the clockwise angle to turn from Theta = 0
   */
  public void turnTo(double Theta) {

    // ensure angle is positive and within 360
    double minTheta = ((Theta - odometer.getXYT()[2]) + FULL_CIRCLE) % FULL_CIRCLE;

    if (minTheta > INITIAL_ANGLE && minTheta <= HALF_CIRCLE) {
      // angle is already minimum angle, robot should turn clockwise
      RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    } else if (minTheta > HALF_CIRCLE && minTheta < FULL_CIRCLE) {
      // angle is not minimum angle, robot should turn counter-clockwise to the
      // complementary angle of a full circle 360 degrees
      minTheta = FULL_CIRCLE - minTheta;
      RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    }

  }


  /**
   * This is a median filter. The filter takes 5 consecutive readings from the ultrasonic sensor,
   * amplifies them to increase sensor sensitivity, sorts them, and picks the median to minimize the
   * influence of false negatives and false positives in sensor readings, if any. The sensor is very
   * likely to report false negatives.
   * 
   * @return the median of the five readings, sorted from small to large
   */
  private double medianFilter() {
    double[] arr = new double[3];
    for (int i = 0; i < 3; i++) {
      this.usDistance.fetchSample(usData, 0);
      arr[i] = usData[0] * 100.0;
    }
    Arrays.sort(arr);

    if (arr[1] > 255) {
      return 255;
    }

    return arr[1];
  }

  /**
   * Sets the speed of both the left and right EV3 large motors of the robot.
   * 
   * @param speed - The speed to be set
   */
  private void setSpeed(int speed) {
    LEFT_MOTOR.setSpeed(speed);
    RIGHT_MOTOR.setSpeed(speed);
  }

  /**
   * Sets both the left and right EV3 large motor accelerations. Accelerations are set to smoothen
   * the start and stop motions of the robot, which reduces the wheel slippage that commonly leads
   * to inaccuracies in the odometer readings
   * 
   * @param acc - The motor acceleration to be set
   */
  private void setAcceleration(int acc) {
    LEFT_MOTOR.setAcceleration(acc);
    RIGHT_MOTOR.setAcceleration(acc);
  }

  /**
   * Stops the left and right motors simultaneously. The first call to stop the left motor is
   * non-blocking and the second call to stop the right motor is blocking. This is to ensure that
   * both motors stop as synchronized as possible.
   */
  private void stopMotors() {
    // call will return immediately
    LEFT_MOTOR.stop(true);
    // will only return control after stopping is completed
    RIGHT_MOTOR.stop(false);
  }

  /**
   * Turns robot left. Turns left EV3 large motor backwards while turning the right EV3 large motor
   * forwards.
   */
  private void turnLeft() {
    LEFT_MOTOR.backward();
    RIGHT_MOTOR.forward();
  }

  /**
   * This is a static method allows the conversion of a distance to the total rotation of each wheel
   * need to cover that distance.
   * 
   * (Distance / Wheel Circumference) = Number of wheel rotations. Number of rotations * 360.0
   * degrees = Total number of degrees needed to turn.
   * 
   * @param radius - Radius of the wheel
   * @param distance - Distance of path
   * @return an integer indicating the total rotation angle for wheel to cover the distance
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This is a static method that converts the angle needed to turn at a corner to the equivalent
   * total rotation. This method first converts the degrees of rotation, radius of wheels, and width
   * of robot to distance needed to cover by the wheel, then the method calls another static method
   * in process to convert distance to the number of degrees of rotation.
   * 
   * @param radius - the radius of the wheels
   * @param width - the track of the robot
   * @param angle - the angle for the turn
   * @return an int indicating the total rotation sufficient for wheel to cover turn angle
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
