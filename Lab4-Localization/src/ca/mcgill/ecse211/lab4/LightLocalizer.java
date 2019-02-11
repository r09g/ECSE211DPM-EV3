package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Lab4.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.TRACK;
import static ca.mcgill.ecse211.lab4.Lab4.WHEEL_RAD;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread {

  private Odometer odometer;
  private float[] lightData;
  private SampleProvider lightColor;
  private float filterSum;

  private static final int SPEED = 150; // might need to change
  private static final double THRESHOLD = 0.75; // threshold for finding gridlines
  private static final double TURN_CIRCLE = 360.0;
  private static final double ANGLE_TO_ORIGIN = 45.0;
  private static final int SMOOTH_ACCELERATION = 500;
  private static final double SENSOR_DIST = 13; // distance from sensor to center of mass of robot
  private static final int INITIAL_ANGLE = 0;
  private static final int HALF_CIRCLE = 180;
  private static final int FULL_CIRCLE = 360;
  private static final double TO_DEG = 180.0 / Math.PI;
  private static final double TO_RAD = Math.PI / 180.0;
  private static final int Q1Q4COR = 90;
  private static final int Q2Q3COR = 270;
  private static final int CENTER = 0;
  private double std;
  private double left_x;
  private double right_x;
  private double up_y;
  private double down_y;
  double intensity;

  public LightLocalizer(SampleProvider lightColor, float[] lightData, Odometer odometer) {
    this.odometer = odometer;
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.std = 0;
    this.left_x = 0;
    this.right_x = 0;
    this.up_y = 0;
    this.down_y = 0;
    LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
    RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

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
    
    if(yTheta > 90) {
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

    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, TURN_CIRCLE + 7), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, TURN_CIRCLE + 7), true);
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
    filterSum = 0;
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
