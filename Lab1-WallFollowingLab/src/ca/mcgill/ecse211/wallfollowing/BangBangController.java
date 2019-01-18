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
		this.motorLow = 50;
		this.motorHigh = 150;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// adjustment factor 40 to distance; speed increment = 50

		if(distance < 200) {

			if(distance > (this.bandCenter + 40 + this.bandwidth)) {
				// far
				// turn left
				WallFollowingLab.leftMotor.setSpeed(motorHigh - 50);
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 50);
				System.out.println("far");

			} else if(distance < (this.bandCenter + 40 - this.bandwidth)) {
				// close
				// turn right
				WallFollowingLab.leftMotor.setSpeed(motorHigh + 50);
				WallFollowingLab.rightMotor.setSpeed(motorHigh - 50);
				System.out.println("close");
			} else {
				// keep going
			}
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
