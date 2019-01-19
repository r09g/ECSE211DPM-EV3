package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;

	private final int bandCenter; //offset from the wall
	private final int bandWidth; //width of dead band
	private int distance; //distance recorded by sensor
	private int tally; //count the number of consecutive times out of bounds distances were recorded
	private int Delta; //absolute difference between required distance and actual distance from wall
	private final int bound = 160; //any distance above 160 will be considered out of bounds

	public PController(int bandCenter, int bandWidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		this.tally = 0;
		this.Delta = 0;
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		if(this.distance > bound) { //if sensor records out of bounds distance
			this.tally++;
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 50); //slows down left motor
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - 50); //slows down right motor
			WallFollowingLab.rightMotor.forward();
			WallFollowingLab.leftMotor.forward();
		}

		if (this.distance < bound) { //if sensor records valid distance
			this.tally = 0; // clear tally value
			Delta = Math.abs(this.distance - this.bandCenter);

			// regulate Delta
			if (Delta < 5) {
				Delta = 5;
			} 
			else if (Delta > 20) {
				Delta = 20;
			}

			if (this.distance > this.bandCenter - 5) {
				// this is for far from wall
				// turn left, sharply if far from wall
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 6 * Delta);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 8 * Delta);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} 
			else {
				// this is for close to wall
				// turn right
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 12 * Delta);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - 12 * Delta);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();	
				try {
					Thread.sleep(500);
				} catch(Exception e) {

				}
			}
		} else {
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 40) {
				WallFollowingLab.leftMotor.setSpeed(50);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 170);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else if(tally > 15 && tally < 40) {
				// counting tally
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 2);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 2);
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
