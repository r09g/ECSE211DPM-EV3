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
   * detection of falling edge, from left wall to back wall. In other words, this constant is used
   * to force the robot out of the noise margin.
   */
  private static final double FE_CONTINUATION = 40;

  /**
   * The threshold distance (in cm) for detecting a rising edge
   */
  private static final double RE_THRESHOLD = 40;

  /**
   * This value (in cm) is used to ensure the robot continues to turn counter-clockwise after first
   * detection of rising edge, from left wall to back wall. In other words, this constant is used to
   * force the robot out of the noise margin.
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

  /**
   * The target angle for rising edge using our convention of North = 0 degrees and clockwise
   * turning = increase in Theta. This is used to correct the Odometer Theta value in order to
   * localize the robot.
   */
  private static final double RE_ANGLE = 225.0;

  /**
   * The target angle for falling edge using our convention of North = 0 degrees and clockwise
   * turning = increase in Theta. This is used to correct the Odometer Theta value in order to
   * localize the robot.
   */
  private static final double FE_ANGLE = 45.0;

  /**
   * Factor to amplify ultrasonic sensor readings, increases sensitivity of the ultrasonic sensor
   */
  private static final double AMP_FACTOR = 100.0;

  /**
   * Number of readings to be taken for median filter of the ultrasonic sensor. Used to filter out
   * extreme values and unwanted spikes
   */
  private static final int NUM_READINGS = 3;

  /**
   * The max value of the ultrasonic sensor. Used to filter out unrealistic values returned by the
   * ultrasonic sensor
   */
  private static final int MAX_READING = 255;

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
   * <p>
   * Method for falling edge implementation. The robot first turns left to record a falling edge for
   * the left wall, then switches direction and turns to record another falling edge for the back
   * wall. The computation for the correction of the current heading angle is computed in another
   * method but called in this method. The robot will beep when executing this method to signal
   * stages.
   * 
   * <p>
   * The method works best when robot is initially placed to not face the wall. The 45 degree line
   * can be used to decide whether falling edge or rising edge is best. For angles in the range
   * 315~360, and 0~135, falling edge works best.
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

    // Beep once to signal detection of falling edge for left wall
    Sound.beep();

    // Stops the robot to change turning direction
    stopMotors();

    // Record angle for left wall. The angle is stored in index = 2 slot.
    alpha = odometer.getXYT()[2];

    // Turns the robot right and forces it out of the noise margin to avoid incorrect detection of
    // second falling edge
    while (medianFilter() < FE_CONTINUATION) {
      turnRight();
    }

    // Beep once to signal exiting of noise margin
    Sound.beep();

    // Keeps the robot turning right until second falling edge is detected.
    // This is the falling edge for the back wall
    while (medianFilter() > FE_THRESHOLD) {
      turnRight();
    }

    // Beep once to signal detection
    Sound.beep();

    // Record angle for back wall. The angle is stored in index = 2 slot.
    beta = odometer.getXYT()[2];

    // compute correction angle and return
    return correctAngle(alpha, beta);
  }

  /**
   * <p>
   * Method for rising edge implementation. The robot first turns right to record a rising edge for
   * the left wall, then switches direction and turns to record another falling edge for the back
   * wall. The computation for the correction of the current heading angle is computed in another
   * method but called in this method. The robot will beep when executing this method to signal
   * stages.
   * 
   * <p>
   * The method works best when robot is initially placed to face the wall. The 45 degree line can
   * be used to decide whether falling edge or rising edge is best. For angles in the range 135~315,
   * rising edge works best.
   * 
   * @return the correction needed to be applied to the current heading for localization
   */
  private double risingEdge() {

    // Angle corresponding to first falling edge
    double alpha;

    // Angle corresponding to second falling edge
    double beta;

    // Turn right until first rising edge detected. Rising edge for left wall.
    while (medianFilter() < RE_THRESHOLD) {
      turnRight();
    }

    // Beep once to signal detection
    Sound.beep();

    // Stops the robot to change turning direction
    stopMotors();

    // Record angle for left wall
    alpha = odometer.getXYT()[2];

    // Turns the robot left and forces it out of the noise margin to avoid incorrect detection of
    // second rising edge
    while (medianFilter() > RE_CONTINUATION) {
      turnLeft();
    }

    // Beep once to signal exiting of noise margin
    Sound.beep();

    // Turn left until second rising edge detected. Rising edge for back wall.
    while (medianFilter() < RE_THRESHOLD) {
      turnLeft();
    }

    // Beep once to signal detection of second rising edge
    Sound.beep();

    // Record angle for back wall
    beta = odometer.getXYT()[2];

    // compute correction angle and return
    return correctAngle(alpha, beta);

  }

  /**
   * <p>
   * A helper method to compute the correction angle needed for the Odometer's Theta value. Corrects
   * the robot's heading to a value that references North as 0 degrees.
   * 
   * <p>
   * If alpha, the first angle detected, is greater than beta, the second angle detected, then the
   * robot is executing rising edge. For the opposite case, the robot is executing falling edge. For
   * falling edge, the average of the two angles is suppose to be 45 degrees with respect to our
   * convention (North = 0 degrees). For rising edge, the average of the two angles is suppose to be
   * 225 degrees with respect to our convention.
   * 
   * @param alpha - The first angle recorded
   * @param beta - The second angle recorded
   * @return The correction to Odometer's Theta value needed to localize the robot's heading
   */
  private double correctAngle(double alpha, double beta) {
    // The divisor 2.0 is used to take the average
    if (alpha > beta) {
      return RE_ANGLE - (alpha + beta) / 2.0;
    } else {
      return FE_ANGLE - (alpha + beta) / 2.0;
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
      // angle is already minimum angle, robot should turn clockwise (right)
      turnRight(minTheta);
    } else if (minTheta > HALF_CIRCLE && minTheta < FULL_CIRCLE) {
      // angle is not minimum angle, robot should turn counter-clockwise (left) to the
      // complementary angle of a full circle 360 degrees
      minTheta = FULL_CIRCLE - minTheta;
      turnLeft(minTheta);
    }
  }


  /**
   * This is a median filter. The filter takes 3 consecutive readings from the ultrasonic sensor,
   * amplifies them to increase sensor sensitivity, sorts them, and picks the median to minimize the
   * influence of false negatives and false positives in sensor readings, if any. The sensor is very
   * likely to report false negatives.
   * 
   * @return the median of the five readings, sorted from small to large
   */
  private double medianFilter() {

    // Buffer for median filter
    double[] arr = new double[NUM_READINGS];

    // Take 3 readings, each amplified by a factor of 100.0
    for (int i = 0; i < NUM_READINGS; i++) {
      // place reading into buffer
      this.usDistance.fetchSample(usData, 0);
      // Amplify reading and store in buffer for median filter
      arr[i] = usData[0] * AMP_FACTOR;
    }

    // Sort array in increasing size to find median
    // The item in the center is the median
    Arrays.sort(arr);

    // If median is greater than maximum distance value
    if (arr[NUM_READINGS / 2] > MAX_READING) {
      return MAX_READING;
    }

    // return median
    return arr[NUM_READINGS / 2];
  }

  // -----------------------------------------------------------------------------
  // Helper Methods
  // -----------------------------------------------------------------------------

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
   * Turns the robot left for a fixed angle. Turns left EV3 large motor backwards while turning the
   * right EV3 large motor forwards. First call to right motor is non-blocking and second call to
   * left motor is blocking to ensure synchronized movement of the two motors.
   * 
   * @param angle - The angle in degrees to be turned
   */
  private void turnLeft(double angle) {
    RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
    LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
  }

  /**
   * Turns robot right. Turns left EV3 large motor forwards while turning the right EV3 large motor
   * backwards.
   */
  private void turnRight() {
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.backward();
  }

  /**
   * Turns the robot right for a fixed angle. Turns left EV3 large motor forwards while turning the
   * right EV3 large motor backwards. First call to right motor is non-blocking and second call to
   * left motor is blocking to ensure synchronized movement of the two motors.
   * 
   * @param angle - The angle in degrees to be turned
   */
  private void turnRight(double angle) {
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), true);
    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angle), false);
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
