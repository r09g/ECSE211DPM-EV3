package ca.mcgill.ecse211.lab4;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
// static imports
import static ca.mcgill.ecse211.lab4.Lab4.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab4.Lab4.TILE;
import static ca.mcgill.ecse211.lab4.Lab4.TRACK;
import static ca.mcgill.ecse211.lab4.Lab4.WHEEL_RAD;
import java.util.Arrays;

public class UltrasonicLocalizer extends Thread {

  private static final double THRESHOLD = 40;
  private static final double MARGIN = 5;
  private static final double TURN_CIRCLE = 360.0;
  private static final double TURN_BUFFER = 90.0;


  private static final int FALLING_EDGE = Button.ID_LEFT;
  private static final int RISING_EDGE = Button.ID_RIGHT;

  private int type;

  private Odometer odometer;
  private double[] odoData;

  private SampleProvider usDistance;
  private float[] usData;

  public UltrasonicLocalizer(int buttonChoice, SampleProvider usDistance, float[] usData)
      throws OdometerExceptions {
    this.odometer = Odometer.getOdometer();
    this.usData = usData;
    this.usDistance = usDistance;
    this.type = buttonChoice;
    LEFT_MOTOR.setAcceleration(500);
    RIGHT_MOTOR.setAcceleration(500);
  }

  public void run() {

    LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, TURN_CIRCLE), true);
    RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, TURN_CIRCLE), true);

    double dTheta = 0;
    
    if (type == FALLING_EDGE) {
      // falling edge
      dTheta = fallingEdge();
    } else if (type == RISING_EDGE) {
      // rising edge
      dTheta = risingEdge();
    }

    odometer.setTheta(odometer.getXYT()[2] + dTheta);
    
   // LEFT_MOTOR
    
  }

  /**
   * 
   */
  private double fallingEdge() {

    double T1, T2, alpha, T3, T4, beta;

    while (true) {
      double distance = medianFilter();

      // falling edge
      if (distance < THRESHOLD + MARGIN) {
        T1 = odometer.getXYT()[2];
        while (true) {
          distance = medianFilter();
          if (distance < THRESHOLD - MARGIN) {
            T2 = odometer.getXYT()[2];
            break;
          }
        }
        break;
      }
    }

    alpha = (T1 + T2) / 2;

    // LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, TURN_BUFFER), true);
    // RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, TURN_BUFFER), false);

    while (true) {
      double distance = medianFilter();

      // rising edge
      if (distance > THRESHOLD - MARGIN) {
        T3 = odometer.getXYT()[2];
        while (true) {
          distance = medianFilter();
          if (distance > THRESHOLD + MARGIN) {
            T4 = odometer.getXYT()[2];
            break;
          }
        }
        break;
      }
    }

    beta = (T3 + T4) / 2;

    return FEcorrect(alpha, beta);
  }

  /**
   * 
   */
  private double risingEdge() {

    double T1, T2, alpha, T3, T4, beta;

    while (true) {
      double distance = medianFilter();

      // falling edge
      if (distance > THRESHOLD - MARGIN) {
        T1 = odometer.getXYT()[2];
        while (true) {
          distance = medianFilter();
          if (distance > THRESHOLD + MARGIN) {
            T2 = odometer.getXYT()[2];
            break;
          }
        }
        break;
      }
    }

    alpha = (T1 + T2) / 2;

    // LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, TURN_BUFFER), true);
    // RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, TURN_BUFFER), false);

    while (true) {
      double distance = medianFilter();

      // rising edge
      if (distance < THRESHOLD + MARGIN) {
        T3 = odometer.getXYT()[2];
        while (true) {
          distance = medianFilter();
          if (distance < THRESHOLD - MARGIN) {
            T4 = odometer.getXYT()[2];
            break;
          }
        }
        break;
      }
    }

    beta = (T3 + T4) / 2;

    return REcorrect(alpha, beta);

  }

  private double FEcorrect(double alpha, double beta) {
    return 255.0 - (alpha + beta) / 2.0;

//    if (Math.abs(alpha - beta) <= 180) {
//      return 255.0 - (alpha + beta) / 2.0;
//    } else {
//      return 45.0 - (alpha + beta) / 2.0;
//    }

  }

  private double REcorrect(double alpha, double beta) {
    return 45.0 - (alpha + beta) / 2.0;

//    if (Math.abs(alpha - beta) <= 180) {
//      return 255.0 - (alpha + beta) / 2.0;
//    } else {
//      return 45.0 - (alpha + beta) / 2.0;
//    }

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
    double[] arr = new double[5];
    for (int i = 0; i < 5; i++) {
      this.usDistance.fetchSample(usData, 0);
      arr[i] = usData[0] * 100.0;
    }
    Arrays.sort(arr);
    return arr[2];
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
