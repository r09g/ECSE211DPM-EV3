package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;
import java.math.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int tally;		// decide corner

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.tally = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	}
 	
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

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// count to decide if at corner
		// check if distance stays max
		if(this.distance > 160) {
			this.tally++;
			
		}

		// check valid distance value
		if(this.distance < 160) {
			this.tally = 0;		// clear tally value

			if(this.distance > (this.bandCenter + this.bandwidth)) {
				// this is for far from wall
				// turn left
				WallFollowingLab.leftMotor.setSpeed(motorHigh - 110);
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 70);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();

			} else if(this.distance < (this.bandCenter - this.bandwidth)) {
				// this is for close to wall
				// turn right
				WallFollowingLab.leftMotor.setSpeed(motorHigh + 180);
				WallFollowingLab.rightMotor.setSpeed(10);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			
			} else {
				// within band
				WallFollowingLab.leftMotor.setSpeed(motorHigh);
				WallFollowingLab.rightMotor.setSpeed(motorHigh);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
		} else {
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 45) {
				// corner left turn
				WallFollowingLab.leftMotor.setSpeed(30);
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 160);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else if (tally > 15 && tally < 45) {
				// counting tally
				WallFollowingLab.leftMotor.setSpeed(motorLow);
				WallFollowingLab.rightMotor.setSpeed(motorLow);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
		}
		
		return;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
