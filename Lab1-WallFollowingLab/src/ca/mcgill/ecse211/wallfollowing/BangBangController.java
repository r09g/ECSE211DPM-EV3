package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

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
		this.motorLow = 50;
		this.motorHigh = 150;
		this.tally = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// count to decide if at corner
		// check if distance stays max
		if(this.distance > 200) {
			this.tally++;
			WallFollowingLab.leftMotor.setSpeed(100);
			WallFollowingLab.rightMotor.setSpeed(100);
		}

		// check valid distance value
		if(this.distance < 200) {
			this.tally = 0;		// clear tally value

			if(this.distance > (this.bandCenter + 20 + this.bandwidth)) {
				// this is for far from wall
				// turn left, sharply if far from wall
				WallFollowingLab.leftMotor.setSpeed(motorHigh - 70);
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 70);

			} else if(this.distance < (this.bandCenter + 20 - this.bandwidth)) {
				// this is for close to wall
				// turn right
				if(this.distance < 10) {
					// turn faster if too close
					WallFollowingLab.leftMotor.setSpeed(motorHigh + 200);
					WallFollowingLab.rightMotor.setSpeed(motorHigh - 120);
				} else {
					WallFollowingLab.leftMotor.setSpeed(motorHigh + 150);
					WallFollowingLab.rightMotor.setSpeed(motorHigh - 100);
				}

			} else {
				// within band
				WallFollowingLab.leftMotor.setSpeed(motorHigh);
				WallFollowingLab.rightMotor.setSpeed(motorHigh);
			}
		} else {
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 20) {
				WallFollowingLab.leftMotor.setSpeed(motorHigh - 70);
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 100);
			}
		}

		return;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
