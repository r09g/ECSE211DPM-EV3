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
	//DO NOT FORGET TO JAVADOC ALL PUBLIC CLASSES AND METHODS EVEN THOSE FROM THE STARTER CODE
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		if(this.distance > bound) { //if sensor records out of bounds distance
			this.tally++;
		}

		if (this.distance < bound) { //if sensor records valid distance
			this.tally = 0; // clear tally value
			// Delta is the error between the current distance and the expected distance
			// Delta is used as a factor which controls the magnitude of change in the speed of motors
			Delta = Math.abs(this.distance - this.bandCenter);

			// regulate Delta
			// is this necessary?? need to test
			if (Delta < 5) {
				Delta = 5;
			} else if (Delta > 20) {
				Delta = 20;
			}

			if (this.distance > this.bandCenter - 5) { //if robot too far from wall
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 6 * Delta); //slow down left motor proportionally to delta
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 8 * Delta); //speed up right motor proportionally to delta
				//proceed to turn left, getting closer to the wall, proportionally to how far off the robot was from the required distance
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else { //if robot too close to wall
				
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 12 * Delta); //speed up left motor proportionally to delta
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - 12 * Delta); //slow down right motor proportionally to delta
				//proceed to turn right, moving away from wall, proportionally to how far off the robot was from the required distance
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();	
				
				try {
					Thread.sleep(500);
				} catch(Exception e) {
					
				}
			}
		} else {	//if sensor records out of bounds distance
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 40) {	
				// out of bounds distance recorded more than 40 times, so there really is nothing there
				// Make a sharper and 
				Delta = Math.abs(this.distance - this.bandCenter);	// calculate 
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 10 * Delta);		// slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 10 * Delta);		// speed up right motor
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else if(tally > 15 && tally < 40) { //if sensor needs more time to complete tally
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 2); //slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 2); //slow down right motor
				//proceeds to slow robot down in straight line, giving it more time to complete tally without it moving too far
				//away from wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
			//if tally < 15 then either it was a mistake and there is indeed a wall, either it was a small gap
			//do nothing, keep going straight
		}
		return;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
