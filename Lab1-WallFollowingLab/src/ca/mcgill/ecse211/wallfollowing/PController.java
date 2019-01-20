package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * <p>
 * This class specifies the proportional-type controller for the EV3 robot. This
 * type of controller adjusts the direction of the robot through making
 * adjustments that are proportional to the error. The error is the absolute
 * value of the difference between the current distance from the wall and the
 * optimal distance from the wall.
 * 
 * <p>
 * This class consists of constants and class variables, a constructor for
 * creating a new instance of this class, and non-static methods: a "wrapper"
 * method that returns the distance reading, and a primary method which process
 * distance readings and make adjustments to the left and right motors of the
 * robot.
 * 
 * 
 * <p>
 * Lab Group: 43
 * 
 * @author1 Erica De Petrillo <br>
 *          McGill ID:
 * @author2 Raymond Yang <br>
 *          McGill ID: 260777792
 */

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200; // normal speed of motor
	private static final int BOUND = 160; // any distance above 160 will be considered out of BOUNDs
	private static final int CONSTANT = 9;
	
	private final int bandCenter; // offset from the wall
	private final int bandWidth; // width of dead band
	private int distance; // distance recorded by sensor

	// tally is the count of the number of consecutive times out of bounds distances
	// were recorded
	// tally acts as a filter that filters out the false negative distance readings
	// from the ultrasonic sensor
	// prevent robot from turning left sharply when it is not supposed to (i.e. a
	// wall is there)
	private int tally;
	private int error;

	public PController(int bandCenter, int bandWidth) {
		this.bandCenter = bandCenter - 3;
		this.bandWidth = bandWidth - 1;
		this.tally = 0;
		this.error = 0;
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
	}

	// DO NOT FORGET TO JAVADOC ALL PUBLIC CLASSES AND METHODS EVEN THOSE FROM THE
	// STARTER CODE
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// if sensor records out of bounds distance
		if (this.distance > BOUND) {
			tally++;
		}

		// the distance reading is within a valid range
		if (this.distance < BOUND) {

			tally = 0; // clear tally value since an object is detected

			// error is the absolute difference between required distance and actual
			// distance from wall
			// error is used as a factor which controls the magnitude of change in the speed
			// of motors

			error = Math.abs(this.distance - this.bandCenter);

			// Control upper bound of error to protect motor
			// When there is no object the sensor returns 2147483647
			// which may destroy the motor if rpm is too high
			if (error > 20) {
				error = 20;
			}

			// checks if the robot is too close or too far from the wall
			if (this.distance > this.bandCenter + this.bandWidth) {

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - CONSTANT * error); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up right motor

				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();

			} else if (this.distance < this.bandCenter) {

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - CONSTANT * error); // slow down right motor

				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
				
				if(this.distance < 12) {
					try {
						Thread.sleep(200);
					} catch(Exception e) {

					}
				}

			} else {
				// the robot is within the bandwidth
				// do nothing, no adjustment needed
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
		} else { // if sensor records out of bounds distance
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if (tally > 45) {
				// out of bounds distance recorded more than 40 times, so there really is
				// nothing there
				// Make a sharper and
				error = Math.abs(this.distance - this.bandCenter);

				// Control upper bound of error to protect motor
				// When there is no object the sensor returns 2147483647
				// which may destroy the motor if rpm is too high
				if (error > 20) {
					error = 20;
				}

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 50 - CONSTANT * error); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up right motor
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else { // if sensor needs more time to complete tally
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); // slow down right motor
				// proceeds to slow robot down in straight line, giving it more time to complete
				// tally without it moving too far
				// away from wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
			// if tally < 15 then either it was a mistake and there is indeed a wall, either
			// it was a small gap
			// do nothing, keep going straight
		}
		return;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
