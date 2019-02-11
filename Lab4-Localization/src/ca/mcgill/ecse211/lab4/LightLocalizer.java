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

  /**
   * The angle (in degrees) corresponding to the North using our convention
   */
  private static final int NORTH = 0;

  /**
   * Adjustment needed to turn robot to face North. This constant exists because EV3 turns a smaller
   * angle than expected in reality.
   */
  private static final int TURN_ADJUSTMENT = 6;

  /**
   * This adjustment is very similar to the TURN_ADJUSTMENT but this is specially for turning a full
   * revolution of 360 degrees.
   */
  private static final int CIRCLE_ADJUSTMENT = 7;

  /**
   * Factor to amplify light sensor readings, increases sensitivity of the light sensor
   */
  private static final double AMP_FACTOR = 100.0;

  /**
   * Number of readings to be taken for mean filter (MF) of the light sensor.
   */
  private static final int MF_READINGS = 5;

  /**
   * Number of readings to be taken when computing the initial reading (IR) of the light sensor
   */
  private static final int IR_READINGS = 100;

  /**
   * The time for thread to sleep. To wait for turning motion to complete after detecting all 4 grid
   * lines, when detecting grid lines.
   */
  private static final int IDLE_TIME = 50;

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
    setSpeed(SPEED);
    // Reset motor TachoCount
    resetMotorCount();
  }

  // -----------------------------------------------------------------------------
  // Run Method
  // -----------------------------------------------------------------------------

  /**
   * The {@code run()} method required in a Thread class. In this class, a set of movements is
   * specified to localize the robot's position. The robot determines its current position, then
   * moves closer to the origin and turns a circle to help correct its odometer. Finally, robot
   * travels to origin.
   */
  public void run() {

    // Record reading for background color
    std = initialReading();

    // Record the initial tacho count
    int startTacho = recordTachoCount();

    // Keep sampling until a grid line is detected
    while (true) {
      // a mean filter is used to smooth out readings
      intensity = meanFilter();
      // detection of grid line
      if ((intensity / std) < THRESHOLD) {
        // Beep once for detection
        Sound.beep();
        break;
      }
      // Continue to move robot forwards if no grid line detected
      moveForward();
    }

    // Stop if a grid line is detected
    stopMotors();

    // Record the tacho count
    int endTacho = recordTachoCount();

    // Compute the distance (in degrees of wheel rotation) from the starting point to the grid line
    // detection
    int distanceToGrid = endTacho - startTacho;

    // convert distance from degrees of wheel rotation to cm
    distanceToGrid = (int) (deg2Distance(WHEEL_RAD, distanceToGrid));

    // Reverse back to starting point
    moveBackward(distanceToGrid);

    // Face origin
    turnRight(ANGLE_TO_ORIGIN);

    // Move closer to origin but stop before reaching the origin
    moveForward(distanceToGrid - SENSOR_DIST);

    // Turn to face straight
    turnLeft(ANGLE_TO_ORIGIN);

    // Robot should be facing north. Overwrite Theta to ensure accuracy.
    odometer.setTheta(NORTH);

    // Find four angles when robot intersects four grid lines
    find4Points();

    // Compute the current y position using trignometry
    double xTheta = Math.abs(left_x - right_x) / 2.0;
    double dy = Math.cos(xTheta * TO_RAD) * SENSOR_DIST;

    // Update the Y value to reference the origin
    odometer.setY(-dy);

    // Compute angle used to determine the x position
    double yTheta = Math.abs(down_y - up_y) / 2.0;

    // Hack to ensure correct computation
    // Needed when robot is really close to y axis
    // Although angle is less than 180 (average less than 90) but Odometer may be inaccurate and
    // measure angle greater than 180.
    if (yTheta > 90) {
      // Take the complementary angle
      yTheta = 180 - yTheta;
    }

    // Compute the current x position
    double dx = Math.cos(yTheta * TO_RAD) * SENSOR_DIST;

    // Update the X value to reference the origin
    odometer.setX(-dx);

    // Travel to the origin
    travelTo(0, 0);

    // Turn to face north
    // Adjustment needed since robot turns less in reality
    turnTo(TURN_ADJUSTMENT);

  }

  // -----------------------------------------------------------------------------
  // Private Methods
  // -----------------------------------------------------------------------------
  
  /**
   * Turns the robot 360 degrees to find a total of 4 grid lines, recording the angle at which each
   * grid line is detected. The angles will be used to compute the distances from the origin.
   */
  private void find4Points() {

    // Rotate 360 degrees clockwise, both non-blocking to allow detection of grid line while turning
    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, FULL_CIRCLE + CIRCLE_ADJUSTMENT), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, FULL_CIRCLE + CIRCLE_ADJUSTMENT), true);

    // Record four grid lines in turn
    left_x = recordAngle();
    up_y = recordAngle();
    right_x = recordAngle();
    down_y = recordAngle();

    // Wait for robot to complete turn before proceeding
    while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {
      // wait
      try {
        Thread.sleep(IDLE_TIME);
      } catch (Exception e) {
        // nothing
      }
    }
  }

  /**
   * Records the current angle (in degrees) when a grid line is detected
   * 
   * @return The angle if detection is successful and -1 if unsuccessful
   */
  private double recordAngle() {
    while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {
      // Take data from light sensor
      intensity = meanFilter();
      // Determine if grid line is passed
      if ((intensity / std) < THRESHOLD) {
        // Beep once to signal detection
        Sound.beep();
        // Return the current angle
        return odometer.getXYT()[2];
      }
    }
    // Case where robot finishes the turn but no grid line is detected
    return -1;
  }

  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
  private double initialReading() {
    for (int i = 0; i < IR_READINGS; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, 0);
      // amplifies and sums the sample data
      std += lightData[0] * AMP_FACTOR;
    }
    // take average of the standard
    return std /= IR_READINGS;
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
    for (int i = 0; i < MF_READINGS; i++) {

      // acquire sample data and read into array with no offset
      lightColor.fetchSample(lightData, 0);

      // amplify signal for increased sensitivity
      filterSum += lightData[0] * AMP_FACTOR;

    }

    // return an amplified average
    return filterSum / MF_READINGS;

  }

  /**
   * <p>
   * Controls the robot to travel to the coordinate (x,y) with the robot's initial starting location
   * as the origin. This is done by retrieving current position data of the robot and calculating
   * the new heading the robot needs to have, as well as the distance the robot needs to travel to
   * reach its next destination. A minimum angle approach is taken, meaning that the robot will turn
   * the smallest angle possible to adjust its heading.
   * 
   * <p>
   * There is a logic implemented in this method to determine the angle the robot needs to turn,
   * clockwise, to reach its new heading. This logic is necessary largely due to the values returned
   * by arctan function. The arctan function only returns values ranging from -pi/2 to pi/2, and
   * real values can be + or - pi from the returned value.
   * 
   * @param x - the x value of the destination with respect to the origin (in cm)
   * @param y - the y value of the destination with respect to the origin (in cm)
   */
  private void travelTo(double x, double y) {

    // get current position data from odometer
    double[] position = odometer.getXYT();

    // position[0] = x, position[1] = y, position[2] = theta
    double dx = x - position[0]; // displacement in x
    double dy = y - position[1]; // displacment in y
    double ds = Math.hypot(dx, dy);

    // compute heading (theta) using arctan of y and x positions
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

    // robot turns at minimum angle
    turnTo(dTheta);

    // move robot to destination
    moveForward(ds);

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
   * Starts the robot to move forward
   */
  private void moveForward() {
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  /**
   * An overloaded method. Moves the robot forward for a specified distance (in cm). First call is
   * non-blocking and second call is blocking to ensure synchronized motor movements.
   * 
   * @param distance - The distance to travel in cm
   */
  private void moveForward(double distance) {
    // calls convertDistance to convert distance from cm to degrees of rotation
    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance), true);
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance), false);
  }

  /**
   * Moves the robot backward for a specified distance (in cm). First call is non-blocking and
   * second call is blocking to ensure synchronized motor movements.
   * 
   * @param distance - The distance to travel in cm
   */
  private void moveBackward(double distance) {
    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, -distance), true);
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, -distance), false);
  }

  /**
   * Records the robot's current motor tacho count. Takes the average tacho count of the two motors.
   * Tacho counts are in degrees.
   * 
   * @return The average tacho count of the left and right motors
   */
  private int recordTachoCount() {
    return (LEFT_MOTOR.getTachoCount() + RIGHT_MOTOR.getTachoCount()) / 2;
  }

  /**
   * Resets the tacho count of the motors
   */
  private void resetMotorCount() {
    LEFT_MOTOR.resetTachoCount();
    RIGHT_MOTOR.resetTachoCount();
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
    return (int) ((HALF_CIRCLE * distance) / (Math.PI * radius));
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
    return convertDistance(radius, Math.PI * width * angle / FULL_CIRCLE);
  }

  /**
   * Helper method. Converts degrees of wheel rotation to distance in cm travelled
   * 
   * @param radius - radius of robot wheel
   * @param turncount - total number of degrees turned
   * @return an double indicating the total distance in cm travelled equivalent to the degrees
   *         turned
   */
  private static double deg2Distance(double radius, int turncount) {
    return turncount / 360.0 * 2 * Math.PI * radius;
  }
}
