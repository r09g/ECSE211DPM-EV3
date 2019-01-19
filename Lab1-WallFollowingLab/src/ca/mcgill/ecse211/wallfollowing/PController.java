package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int tally;
	private int Delta;

	public PController(int bandCenter, int bandWidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		this.tally = 0;
		this.Delta = 0;
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// count to decide if at corner
		// check if distance stays max
		if(this.distance > 160) {
			this.tally++;
			WallFollowingLab.leftMotor.setSpeed(150);
			WallFollowingLab.rightMotor.setSpeed(150);
			WallFollowingLab.rightMotor.forward();
			WallFollowingLab.leftMotor.forward();
		}

		// check valid distance value
		if(this.distance < 180) {
			this.tally = 0;		// clear tally value
			Delta = Math.abs(this.distance - this.bandCenter);

			// regulate Delta
			if(Delta < 5) {
				Delta = 5;
			} else if(Delta > 20) {
				Delta = 20;
			}

			if(this.distance > this.bandCenter - 5) {
				// this is for far from wall
				// turn left, sharply if far from wall
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 6 * Delta);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 8 * Delta);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else {
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
