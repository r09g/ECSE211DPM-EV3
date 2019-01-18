package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int tally;		// decide corner
	private boolean initial;	// first reading of US

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = 50;
		this.motorHigh = 150;
		this.tally = 0;
		this.initial = true;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		
		// initial start when robot is far from block
		if(this.initial) {
			if(this.distance == Integer.MAX_VALUE) {
				// robot has not reached valid starting point
				// wait until valid sensor reading
				return;
			} else {
				// robot reaches valid starting point
				this.initial = false;
			}
		}
		
		// count to decide if at corner
		// check if distance stays max
		if(this.distance > 250) {
			this.tally++;
		}
		
		// check valid distance value
		if(this.distance < 250) {
			this.tally = 0;		// clear tally value
			
			if(this.distance > (this.bandCenter + this.bandwidth)) {
				// this is for far from wall
				// turn left, sharply if far from wall
				if(this.distance > 80) {
					WallFollowingLab.leftMotor.setSpeed(motorHigh - 70);
					WallFollowingLab.rightMotor.setSpeed(motorHigh + 70);
				} else {
					WallFollowingLab.leftMotor.setSpeed(motorHigh - 50);
					WallFollowingLab.rightMotor.setSpeed(motorHigh + 50);
				}
				
				System.out.println("left");
			} else if(this.distance < (this.bandCenter - this.bandwidth)) {
				// this is for close to wall
				// turn right
				if(this.distance < 10) {
					// turn faster if too close
					WallFollowingLab.leftMotor.setSpeed(motorHigh + 100);
					WallFollowingLab.rightMotor.setSpeed(motorHigh - 50);
				} else {
					WallFollowingLab.leftMotor.setSpeed(motorHigh + 50);
					WallFollowingLab.rightMotor.setSpeed(motorHigh - 50);
				}
				System.out.println("right");
			} else {
				// keep going
			}
		} else {
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 100) {
				WallFollowingLab.leftMotor.setSpeed(motorHigh - 90);
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 50);
			}
		}
		
		return;
	}
	
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
