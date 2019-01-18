package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    /* Consider a digital control loop that periodically samples the output at a rate S. 
     * •At each sample interval, compute error, e, and apply a correction in the appropriate direction (+/-) 
     * using a fixed step. 
     * •If the correction is applied often enough, the output will eventually catch up and follow the input as desired.
     * •At convergence, the controller will “hunt” about the zero error point applying successive +/- corrections. 
     * •We eliminate hunting by suppressing any correction if | r – y | is less than threshold*/
    /*Steering the cart:
     *- Error = reference control value
     *- measured distance from the wall. 
     *- If abs(Error) < Threshold,  we consider the cart to be on a correct heading. 
     *- if Error < 0, the cart is too far from the wall.  Increase rotation of outside wheel; 
     *		(decrease rotation of inside wheel). 
     *- If Error > 0, the cart is too close to the wall.  Decrease rotation of outside wheel; 
     *		(increase rotation of inside wheel). -Magnitude of change in rotation is proportional to the magnitude
     *		 of the error. 
     *- In our example, we follow the BANG BANG approach and define 2 speeds: 
     *	FWDSPEED – speed at which left and right wheel rotate to go straight. 
     *	DELTASPD – amount by which speed increased/decreased to effect motion towards/away from wall*/
    float error = bandCenter - distance;//creates error
    if (Math.abs(error) < bandwidth) { //if the error is less than the threshold, we do nothing
    	return;
    }
    else if (error < 0) { //decrease rotation of inside wheel to get cart closer to wall
    	WallFollowingLab.leftMotor.setSpeed(motorLow); 
    }
    else { //(if error > 0 aka too close to wall, decrease rotation of outside wheel to move cart away from wall
    	WallFollowingLab.rightMotor.setSpeed(motorLow); 
    }
    //might need to add something to make sure robot does not get confused by small gaps
    //idk if we need to add anything here to have it exit the method correctly
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
