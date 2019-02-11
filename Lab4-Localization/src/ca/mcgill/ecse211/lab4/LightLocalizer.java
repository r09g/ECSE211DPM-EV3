package ca.mcgill.ecse211.lab4;

// non-static imports
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
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
import static ca.mcgill.ecse211.lab4.Lab4.TO_RAD;
import static ca.mcgill.ecse211.lab4.Lab4.TO_DEG;

/**
 * <p>
 * This class implements localization with the light sensor. This class extends the Thread class to
 * allow simultaneous execution, so that other classes can work alongside this class. Helper methods
 * are added at the end to make conversions easier.
 * 
 * <p>
 * The robot is assumed to have its heading (Theta value) localized. The X and Y will be localized
 * using the heading. The robot is to proceed the lower-left spot of the target origin (0,0). Then
 * the robot turns a full circle, detecting a total of four lines to compute its current X and Y
 * with respect to the origin. Finally, the robot moves to the origin and adjusts its heading to 0
 * degrees.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */
public class LightLocalizer extends Thread {

  // -----------------------------------------------------------------------------
  // Constants
  // -----------------------------------------------------------------------------

  /**
   * The value for deciding whether a grid line is passed. This is the ratio of the measured value
   * from the light sensor to the value of the background. If ratio is smaller than threshold, line
   * is passed.
   */
  private static final double THRESHOLD = 0.75;

  /**
   * Since robot is placed on 45 degree line, this is the angle robot needs to turn right to face
   * the origin after ultrasonic localization.
   */
  private static final double ANGLE_TO_ORIGIN = 45.0;

  /**
   * This is the distance (in cm) from the center of rotation of the robot to the center of the
   * light sensor.
   */
  private static final double SENSOR_DIST = 13;

  /**
   * Angle correction for Quadrant 1 and 4. Arctan returns the correct angle and the only adjustment
   * needed is to turn it into an angle that starts from Theta = 0 and increases clockwise
   */
  private static final int Q1Q4COR = 90;

  /**
   * Angle correction for Quadrant 2 and 3. Arctan returns the incorrect angle and the adjustment
   * needed is to add pi to the angle and turn it into an angle that starts from Theta = 0 and
   * increases clockwise
   */
  private static final int Q2Q3COR = 270;

  /**
   * The center of the board platform that the EV3 runs on. This is used in determining which
   * quadrant the destination coordinate is in relative to the robot's current location and whether
   * arctan needs correction
   */
  private static final int CENTER = 0;

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * The odometer instance
   */
  private Odometer odometer;

  /**
   * Buffer to store readings from the light sensor.
   */
  private float[] lightData;

  /**
   * An object to read sensor data in a uniform way (Meter-Kilogram-Second units)
   */
  private SampleProvider lightColor;

  /**
   * Stores the initial reading taken at the start of trial. The intensity for the background of the
   * demo floor. Used to compare with readings to decide whether a grid line is passed.
   */
  private double std;

  /**
   * The value of Odometer's Theta when the -x axis is detected by the robot
   */
  private double left_x;

  /**
   * The value of Odometer's Theta when the +x axis is detected by the robot
   */
  private double right_x;

  /**
   * The value of Odometer's Theta when the +y axis is detected by the robot
   */
  private double up_y;

  /**
   * The value of Odometer's Theta when the -y axis is detected by the robot
   */
  private double down_y;

  /**
   * The current value returned by the light sensor
   */
  private double intensity;

  // -----------------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------------

  /**
   * Constructor for this class. Initializes class variables with existing instances and sets the
   * motor speed and acceleration.
   * 
   * @param lightColor - Data reader to read light sensor data in uniform way
   * @param lightData - Buffer to store data from light sensor
   * @param odometer - The existing odometer instance
   */
  public LightLocalizer(SampleProvider lightColor, float[] lightData, Odometer odometer) {
    // Initialize variables with existing instances
    this.odometer = odometer;
    this.lightData = lightData;
    this.lightColor = lightColor;
    // Initialize variables used to store measured values
    this.std = 0;
    this.left_x = 0;
    this.right_x = 0;
    this.up_y = 0;
    this.down_y = 0;
    // Set motor behaviour
    setAcceleration(SMOOTH_ACCELERATION);

    LEFT_MOTOR.resetTachoCount();
    RIGHT_MOTOR.resetTachoCount();
  }
  // assuming zero orientation is robot facing north

  public void run() {

    std = initialReading();

    LEFT_MOTOR.setSpeed(SPEED);
    RIGHT_MOTOR.setSpeed(SPEED);

    int startTacho = (LEFT_MOTOR.getTachoCount() + RIGHT_MOTOR.getTachoCount()) / 2;

    while (true) {
      intensity = meanFilter();
      if ((intensity / std) < THRESHOLD) {
        Sound.beep();
        break;
      }
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }

    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop(false);

    int endTacho = (LEFT_MOTOR.getTachoCount() + RIGHT_MOTOR.getTachoCount()) / 2;

    int distanceToGrid = endTacho - startTacho;

    // reverse back to starting point
    LEFT_MOTOR.rotate((int) (-distanceToGrid), true);
    RIGHT_MOTOR.rotate((int) (-distanceToGrid), false);

    // face origin
    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, ANGLE_TO_ORIGIN), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, ANGLE_TO_ORIGIN), false);

    LEFT_MOTOR.rotate((int) ((distanceToGrid - SENSOR_DIST) / 3), true);
    RIGHT_MOTOR.rotate((int) ((distanceToGrid - SENSOR_DIST) / 3), false);

    // turn to face straight
    LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, ANGLE_TO_ORIGIN), true);
    RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, ANGLE_TO_ORIGIN), false);

    odometer.setTheta(0);

    find4Points();

    double xTheta = Math.abs(left_x - right_x) / 2.0;
    double dy = Math.cos(xTheta * TO_RAD) * SENSOR_DIST;

    odometer.setY(-dy);

    double yTheta = Math.abs(down_y - up_y) / 2.0;

    if (yTheta > 90) {
      yTheta = 180 - yTheta;
    }

    double dx = Math.cos(yTheta * TO_RAD) * SENSOR_DIST;

    odometer.setX(-dx);

    travelTo(0, 0);
    turnTo(6);

    // LEFT_MOTOR.rotate(-convertDistance(WHEEL_RAD, SENSOR_DIST), true);
    // RIGHT_MOTOR.rotate(-convertDistance(WHEEL_RAD, SENSOR_DIST), false);

  }

  private void find4Points() {

    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, FULL_CIRCLE + 7), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, FULL_CIRCLE + 7), true);
    // robot does a 360
    left_x = recordAngle();
    up_y = recordAngle();
    right_x = recordAngle();
    down_y = recordAngle();

    while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {
      // wait
      try {
        Thread.sleep(50);
      } catch (Exception e) {
        // nothing
      }
    }
  }

  private double recordAngle() {
    while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {
      intensity = meanFilter();
      if ((intensity / std) < THRESHOLD) { // detects another gridline
        Sound.beep();
        return odometer.getXYT()[2];
      }
    }
    return -1;
  }

  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
  private double initialReading() {
    for (int i = 0; i < 100; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, 0);
      // amplifies and sums the sample data
      std += lightData[0] * 100.0;
    }
    // take average of the standard
    return std /= 100.0;
  }

  /**
   * This is a private method which functions as a mean or average filter. The filter ensures the
   * correctness of the readings, filtering out the noise in the signal from the color sensor. The
   * filter takes 5 readings and sums the amplified value of each reading.
   * 
   * @return returns the average of the amplified reading
   */
  private double meanFilter() {
    double filterSum = 0;
    // take 5 readings
    for (int i = 0; i < 5; i++) {

      // acquire sample data and read into array with no offset
      lightColor.fetchSample(lightData, 0);

      // amplify signal for increased sensitivity
      filterSum += lightData[0] * 100;

    }

    // return an amplified average
    return filterSum / 5.0;

  }

  private void travelTo(double x, double y) {

    double[] position = odometer.getXYT(); // get current position data from odometer

    // position[0] = x, position[1] = y, position[2] = theta
    double dx = x - position[0]; // displacement in x
    double dy = y - position[1]; // displacment in y
    double ds = Math.hypot(dx, dy);

    double dTheta = Math.atan(dy / dx) * TO_DEG;

    if (dTheta >= CENTER && dx >= CENTER) {
      // 1st quadrant
      dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
    } else if (dTheta >= CENTER && dx < CENTER) {
      // 3rd quadrant, need to correct arctan value
      dTheta = Q2Q3COR - dTheta; // clockwise angle robot needs to turn
    } else if (dTheta < CENTER && dx >= CENTER) {
      // 4th quadrant
      dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
    } else if (dTheta < CENTER && dx < CENTER) {
      // 2nd quadrant, need to correct arctan value
      dTheta = Q2Q3COR - dTheta; // absolute angle
    }

    turnTo(dTheta); // robot turns at minimum angle

    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true);
    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), false);

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
