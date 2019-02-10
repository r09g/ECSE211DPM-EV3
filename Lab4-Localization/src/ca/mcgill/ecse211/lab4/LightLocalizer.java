package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Lab4.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.TRACK;
import static ca.mcgill.ecse211.lab4.Lab4.WHEEL_RAD;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.lab4.Navigation;

public class LightLocalizer extends Thread {

  private Odometer odometer;
  private float[] lightData;
  private SampleProvider lightColor;
  private float filterSum;
  
  private static final int SPEED = 150; // might need to change
  private static final double THRESHOLD = 0.75; // threshold for finding gridlines
  private static final double TURN_CIRCLE = 360.0;
  private static final double SENSOR_DIST = 10; // distance from sensor to center of mass of robot
  private double std = 0;
  double thetaY, thetaX, x, y, dTheta, ThetaNegY, tx1, tx2, ty1, ty2; // TODO: may want to add
                                                                      // ThetaNegX
  double intensity;

  public LightLocalizer(SampleProvider lightColor, float[] lightData) throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.std = 0;
    LEFT_MOTOR.setAcceleration(500);
    RIGHT_MOTOR.setAcceleration(500);
  }
  // assuming zero orientation is robot facing north

  public void run() {
    boolean worked = false;
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
    LEFT_MOTOR.setSpeed(SPEED);
    RIGHT_MOTOR.setSpeed(SPEED); // set the robot moving forward

    // since the robot should be facing the positive y axis by now (through ultrasonic localizer),
    // we will move forward until we catch a horizontal gridline with the light sensor

    // initial readings taken (100 times) and the average is used to distinguish
    // between the wooden board and the black line. Each reading is amplified to
    // enhance the sensitivity of the sensor
    for (int i = 0; i < 100; i++) {

      // acquires sample data
      lightColor.fetchSample(lightData, 0);

      // sums the sample data
      // signal amplification
      std += lightData[0] * 100;
    }

    // take average of the standard
    std /= 100.0;

    while (true) {
      // activates filter and takes the average
      intensity = meanFilter();

      if ((intensity / std) < THRESHOLD) { // detected a horizontal gridline
        worked = find4Points();
      }
      if (worked) {
        break; // leave while loop
      } else {
        LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, 45), true); // turns right
        RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, 45), true);
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
        LEFT_MOTOR.setSpeed(SPEED);
        RIGHT_MOTOR.setSpeed(SPEED);
        while (true) {
          intensity = meanFilter();
          if ((intensity / std) < THRESHOLD) { // detected a horizontal gridline
            LEFT_MOTOR.stop();
            RIGHT_MOTOR.stop();
            LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, 45), true); // turns left, so facing
                                                                          // north again
            RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, 45), true);
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.forward();
            LEFT_MOTOR.setSpeed(SPEED);
            RIGHT_MOTOR.setSpeed(SPEED);
            break;
          }
        }
      }
    }
    thetaY = ty2 - ty1;
    thetaX = tx2 - tx1;
    ThetaNegY = odometer.getXYT()[2];
    x = -SENSOR_DIST * Math.cos(thetaY / 2);
    y = -SENSOR_DIST * Math.cos(thetaX) / 2;
    dTheta = -90 - (ThetaNegY - 180) + thetaY / 2;
    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true); // turns right
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), true);
    Navigation.travelTo(x, y);

  }

  private boolean find4Points() {
    boolean worked = false;
    LEFT_MOTOR.stop();
    RIGHT_MOTOR.stop();

    tx1 = odometer.getXYT()[2];

    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, TURN_CIRCLE), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, TURN_CIRCLE), true);
    // robot does a 360
    while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {
      intensity = meanFilter();
      if (intensity / std < THRESHOLD) { // detects another gridline
        ty1 = odometer.getXYT()[2];
        intensity = meanFilter();
        if (intensity / std < THRESHOLD) { // detects another gridline
          tx2 = odometer.getXYT()[2];
          intensity = meanFilter();
          if (intensity / std < THRESHOLD) { // detects another gridline
            ty2 = odometer.getXYT()[2];
            LEFT_MOTOR.stop();
            RIGHT_MOTOR.stop();
          }
        }
      }
    }
    if (!((tx1 == 0) && (tx2 == 0) && (ty1 == 0) && (ty2 == 0))) { // if they all have values

      worked = true;
    }
    return worked;
  }

  /**
   * This is a private method which functions as a mean or average filter. The filter ensures the
   * correctness of the readings, filtering out the noise in the signal from the color sensor. The
   * filter takes 5 readings and sums the amplified value of each reading.
   * 
   * @return returns the average of the amplified reading
   */
  private double meanFilter() {

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
