/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position
      
      /* y position can be corrected any time the light sensor passes through a horizontal gridline
       * x position can be corrected any time the light sensor passes through a vertical gridline
       * these correction are made by taking the difference between the light sensor's calculated position (with odometer)
       * and the gridline and either adding this difference or subtracting it
       * this cannot be done near gridline intersections
       */

      // TODO Update odometer with new calculated (and more accurate) vales

      odometer.setXYT(0.3, 19.23, 5.0);

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
