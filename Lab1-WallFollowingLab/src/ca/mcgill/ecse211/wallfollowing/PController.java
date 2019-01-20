package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is the proportional-type controller 
 * 
 * @author1 Erica De Petrillo
 * McGill ID: 
 * @author2 Raymond Yang
 * McGill ID:
 * 
 * 
 * 
 */

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;		// normal speed of motor
	private static final int BOUND = 160; //any distance above 160 will be considered out of BOUNDs

	private final int bandCenter; //offset from the wall
	private final int bandWidth; //width of dead band
	private int distance;	//distance recorded by sensor
	
	// tally is the count of the number of consecutive times out of bounds distances were recorded
	// tally acts as a filter that filters out the false negative distance readings from the ultrasonic sensor
	// prevent robot from turning left sharply when it is not supposed to (i.e. a wall is there)
	private int tally;
	
	// absolute difference between required distance and actual distance from wall
	private int Delta;

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

		if(this.distance > BOUND) { //if sensor records out of bounds distance
			this.tally++;
		}

		// the distance reading is within a valid range
		if (this.distance < BOUND) { 
			// clear tally value since an object is detected
			this.tally = 0;
			// Delta is the absolute difference between required distance and actual distance from wall
			// Delta is used as a factor which controls the magnitude of change in the speed of motors
			Delta = Math.abs(this.distance - this.bandCenter);

			// regulate Delta
			// is this necessary?? need to test
			if (Delta < 5) {
				Delta = 5;
			} else if (Delta > 20) {
				Delta = 20;
			}

			// checks if the robot is too close or too far from the wall
			if (this.distance > this.bandCenter + this.bandWidth) { 
				// The distance is larger than the bandcenter by more than the size of the bandwidth
				// Robot is too far away from the wall
				// Robot needs to slow down left motor and speed up right motor proportional to Delta times a constant
				// The constants 6 and 8 for the left and right motor, respectively, are obtained through various tests
				// of the robot in different wall placements, and are the result of tuning
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 6 * Delta); 	// slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 8 * Delta); 	// speed up right motor
				//proceed to turn left, getting closer to the wall, proportionally to how far off the robot was from the required distance
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else if(this.distance < this.bandCenter - this.bandWidth) { //if robot too close to wall
				// The robot is too close to the wall
				// Robot should turn right by speeding up left motor and slowing down left motor proportionally to delta
				// The constant 12 for the left and right motor is obtained through various tests
				// of the robot in different wall placements, and is the result of tuning
				// The constants are larger compared to left turns because wall is on the left and we want the robot to get
				// out of the situation as fast as possible
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 12 * Delta); //speed up left motor 
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - 12 * Delta); //slow down right motor 
				//proceed to turn right, moving away from wall, proportionally to how far off the robot was from the required distance
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();	
				
				try {
					Thread.sleep(500);
				} catch(Exception e) {
					
				}
			} else {
				// the robot is within the bandwidth
				// do nothing, no adjustment needed
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
