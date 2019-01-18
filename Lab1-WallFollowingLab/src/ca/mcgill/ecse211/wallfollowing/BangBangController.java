package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int tally;		// decide corner
	private int backuptally;		// back up decision
	private boolean initial;	// first reading of US
	private boolean backingup;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = 50;
		this.motorHigh = 150;
		this.tally = 0;
		this.backuptally = 0;
		this.initial = true;
		this.backingup = false;
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
		if(this.distance > 150) {
			this.tally++;
		}
		
		// check valid distance value
		if(this.distance < 150 && this.distance > 15) {
			this.tally = 0;		// clear tally value
			
			// check to change to forward drive when distance > 15
			if(this.backingup = true) {
				this.backingup = false;
				this.goforward(100);
			}
			
			if(this.distance > (this.bandCenter + 10 + this.bandwidth)) {
				// this is for far from wall
				// turn left
				this.turnleft(50, 50);
				System.out.println("left");
			} else if(this.distance < (this.bandCenter + 10 - this.bandwidth)) {
				// this is for close to wall
				// turn right
				if(this.distance < 15) {
					// turn faster if too close
					this.turnright(80, 50);
				} else {
					this.turnright(50, 50);
				}
				System.out.println("right");
			} else {
				// keep going
			}
		} else if(this.distance < 10) { 
			// this is for back up
			this.backuptally++;
			if(this.backuptally > 20) {
				this.backup(50,130);
			}
		} else {
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 100) {
				this.turnleft(70, 50);
			}
		}
		
		return;
	}
	
	// helper method: turn left
	private void turnleft(int leftmotor, int rightmotor) {
		WallFollowingLab.leftMotor.setSpeed(motorHigh - leftmotor);
		WallFollowingLab.rightMotor.setSpeed(motorHigh + rightmotor);
		return;
	}
	
	// turn right
	private void turnright(int leftmotor, int rightmotor) {
		WallFollowingLab.leftMotor.setSpeed(motorHigh + leftmotor);
		WallFollowingLab.rightMotor.setSpeed(motorHigh - rightmotor);
		return;
	}
	
	// go straight
	private void goforward(int speed) {
		WallFollowingLab.leftMotor.setSpeed(speed + 30);
		WallFollowingLab.rightMotor.setSpeed(speed);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	// back up straight
	private void backup(int leftspeed, int rightspeed) {
		this.backingup = true;
		WallFollowingLab.leftMotor.setSpeed(leftspeed);
		WallFollowingLab.rightMotor.setSpeed(rightspeed);
		WallFollowingLab.leftMotor.backward();
		WallFollowingLab.rightMotor.backward();
		return;
	}

	
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
