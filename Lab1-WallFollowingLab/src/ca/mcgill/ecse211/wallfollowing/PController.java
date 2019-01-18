package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    // TODO: process a movement based on the us distance passed in (P style)
    /*A more elaborate approach would scale the correction, i.e., DELTASPD according to Error 
     * (this is called proportional control)
     */
    float error = bandCenter - distance;//creates error
    float deltaspeed = error*MOTOR_SPEED/500; // 500 may need to be tweaked to a different value 
    if (Math.abs(error) < bandWidth) { //if the error is less than the threshold, we do nothing
    	return;
    }
    else if (error < 0) { //decrease rotation of inside wheel to get cart closer to wall
    	WallFollowingLab.leftMotor.setSpeed(deltaspeed); 
    }
    else { //(if error > 0 aka too close to wall, decrease rotation of outside wheel to move cart away from wall
    	WallFollowingLab.rightMotor.setSpeed(deltaspeed); 
    }
    //might need to add something to make sure robot does not get confused by small gaps
    //idk if we need to add anything here to have it exit the method correctly
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
