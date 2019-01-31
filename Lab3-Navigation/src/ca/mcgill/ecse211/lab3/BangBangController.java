package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.*;
import java.math.*;

/**
 * This class specifies the bangbang controller for the EV3 robot. The robot
 * moves in a counter-clockwise direction and sensor is on the left side of the
 * robot. This type of controller adjusts the direction of the robot through
 * making fixed adjustments to the motor speeds regardless of the error, which
 * is the absolute value of the difference between the current distance from the
 * wall and the optimal distance from the wall. This type of controller aims to
 * adjust as fast as possible until the error is within an acceptable range, the
 * band width, where the robot stops making adjustments.
 * 
 * <p>
 * This class implements the UltrasonicController interface and implements the
 * methods {@link #processUSData(int)} and {@link #readUSDistance()}.
 * 
 * <p>
 * This class consists of constants and class variables, a constructor for
 * creating a new instance of this class, and non-static methods: a "wrapper"
 * method that returns the distance reading, and a public method which process
 * distance readings and make adjustments to the left and right motors of the
 * robot. The constants are the parameters that are set specifically for this
 * type of controller. The private class variables are the parameters that are
 * common to both controllers.
 * 
 * <p>
 * As for the values of the constants and variables in this class, they are
 * there to account for various uncontrollable physical, environmental, and
 * device component limitations, such as the measurement range of the ultrasonic
 * sensor (only one sensor is used for this implementation). The values are
 * obtained through many experimental trials and testing with varying setup and
 * initial conditions. In other words, the values of the constants and variables
 * in this class is tuned to the specific environment at which the robot and
 * this piece of code is tested.
 * 
 * <p>
 * Lab Group: 43
 * 
 * @see WallFollowingLab
 * @see UltrasonicController
 * 
 * @author1 Erica De Petrillo <br>
 * @author2 Raymond Yang <br>
 */

public class BangBangController implements UltrasonicController {

	// ----------------------------------------------------------------------------
	// Constants
	// ----------------------------------------------------------------------------

	/**
	 * The upper limit for the distance readings from the ultrasonic sensor, any
	 * value above this limit would be identified as a potential false negative,
	 * {@code tally} would be activated.
	 */
	private static final int BOUND = 160;

	// ----------------------------------------------------------------------------
	// Class Variables
	// ----------------------------------------------------------------------------

	/**
	 * The required distance for the robot to keep from the wall
	 */
	private final int bandCenter;

	/**
	 * A range of distance where the robot does not make adjustments to prevent
	 * "hunting" behaviour
	 */
	private final int bandwidth;

	/**
	 * The slow speed value for the EV3 motor
	 */
	private final int motorLow;

	/**
	 * The fast speed value for the EV3 motor
	 */
	private final int motorHigh;

	/**
	 * The distance between the robot and the measured by the ultrasonic sensor at
	 * an angle of 45 degrees to the wall
	 */
	private int distance;

	/**
	 * Stores the number of consecutive occurrences values above the {@code BOUND}
	 * is measured by the ultrasonic sensor
	 */
	private int tally;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * Constructor that creates an instance of the BangBangController class and
	 * initializes the class variables. The {@code bandwidth} is adjusted by -1 with
	 * respect to the initial input to tune the robot to its best performance. This
	 * makes the robot adjust at smaller deviations from the band center. Note this
	 * adjustment should vary with different robot and physical conditions (i.e.
	 * tires, dirt on ground, ground material, and other factors). The initial speed
	 * of the robot's left and right motors are set, but forward movement is only
	 * started after sensor thread is started in the WallFollowingLab class.
	 * 
	 * @param bandCenter an int type of the distance for the robot to keep from the
	 *                   wall in centimeters
	 * @param bandWidth  an int type of the range where the robot does not make
	 *                   adjustments to the motor speed in centimeters
	 * @param motorLow   an int type of the slow rpm speed of the motor
	 * @param motorHigh  an int type of the fast rpm speed of the motor
	 * 
	 * @see WallFollowingLab
	 */
	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth - 1; // adjustment
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.tally = 0;
		Lab3.leftMotor.setSpeed(motorHigh); // set initial speed for robot
		Lab3.rightMotor.setSpeed(motorHigh);
	}

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * Core implementation of the bangbang controller. Processes the
	 * {@link distance} and make fixed adjustments on the motor speed regardless of
	 * the {@link error}.
	 * 
	 * <p>
	 * If the difference in magnitude between the current distance reading and the
	 * band center is greater than the band width and the error is positive, the
	 * robot makes a left turn by adjusting the right motor speed to 345 rpm and
	 * left motor to 165 rpm. If the distance is smaller than the band center by
	 * more than the band width, the robot turns right by adjusting the left motor
	 * speed to 455 rpm and right motor to 10 rpm. This asymmetry of speed
	 * adjustments between left and right turns is to account for 90-degree right
	 * turns. If the distance is within the band width, the robot is set to go
	 * straight.
	 * 
	 * <p>
	 * A tally is implemented in this class to 1) tally out false negatives and
	 * prevent unnecessary left/right oscillations of the robot, 2) distinguish
	 * between wall gap and edge of wall. The tally functions when the input is
	 * above the {@link BOUND}. At a particular angle to the wall, the sensor gives
	 * a false negative with value 2147483647.
	 * 
	 * <p>
	 * When distance readings are over {@code BOUND}, tally starts counting. As
	 * tally increases from 0 to 15, the robot performs no action and continue to
	 * move with previous parameter values. As tally increases from 15 - 45, both
	 * EV3 motors slow down to {@code motorLow} speed, the robot moves in a straight
	 * path. This is to prevent robot from turning away to unwanted directions while
	 * counting tally. As tally passes 45, the robot speeds up the right motor to
	 * 435 rpm and slows down the left motor to 30 rpm to turn. The adjustments are
	 * constant and are independent of {@code distance}. These adjustments are tuned
	 * to these values through numerous trials, they allow the robot to pass through
	 * wider 180-degree convex corners by making a U-turn.
	 * 
	 * <p>
	 * Following every adjustment in speed of the motor, the {@code forward()}
	 * method of the motor is called. The purpose is to fix a suspected EV3 motor
	 * bug where the wheel lock in their previous positions and their speeds cannot
	 * be adjusted.
	 * 
	 * @param distance an int representing the distance between the robot and the
	 *                 wall measured by the ultrasonic sensor, measured by the
	 *                 ultrasonic sensor at 45 degres to the wall.
	 * 
	 * @see java.lang.Thread#sleep(long)
	 */
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		int error = this.distance - this.bandCenter; // deviation from expected band center

		if (this.distance < BOUND) { // distance < 160
			this.tally = 0; // reset tally value

			if (Math.abs(error) > (this.bandwidth) && error > 0) { // robot is too far

				// turn left
				Lab3.leftMotor.setSpeed(motorHigh - 110); // slow down left motor
				Lab3.rightMotor.setSpeed(motorHigh + 70); // speed up right motor

				Lab3.rightMotor.forward(); // EV3 motor hack
				Lab3.leftMotor.forward();

			} else if (Math.abs(error) > (this.bandwidth) && error < 0) { // robot is too close

				// turn right
				Lab3.leftMotor.setSpeed(motorHigh + 180); // speed up left motor
				Lab3.rightMotor.setSpeed(10); // slow down right motor

				Lab3.rightMotor.forward();// EV3 motor hack
				Lab3.leftMotor.forward();

			} else { // error within dead band

				// robot to go straight
				Lab3.leftMotor.setSpeed(motorHigh);
				Lab3.rightMotor.setSpeed(motorHigh);

				Lab3.rightMotor.forward(); // EV3 motor hack
				Lab3.leftMotor.forward();

			}

		} else { // distance > 160
			this.tally++; // increment tally count

			if (tally > 45) { // out of BOUNDs distance recorded more than 45 times, so there really is

				// at edge of wall, make sharp left turn
				Lab3.leftMotor.setSpeed(30); // slow down left motor
				Lab3.rightMotor.setSpeed(motorHigh + 160); // speed up right motor

				Lab3.rightMotor.forward();// EV3 motor hack
				Lab3.leftMotor.forward();
			} else if (tally > 15 && tally < 45) { // if sensor needs more time to count tally

				// go straight, continue tally count
				Lab3.leftMotor.setSpeed(motorLow); // slow down left motor
				Lab3.rightMotor.setSpeed(motorLow); // slow down right motor

				Lab3.rightMotor.forward();// EV3 motor hack
				Lab3.leftMotor.forward();
			}

		}
		return;
	}

	/**
	 * A wrapper method which returns the value of the local private class variable
	 * {@code distance}
	 * 
	 * @return this.distance an int of the distance measured by the ultrasonic
	 *         sensor at ~45 degrees from the robot to the wall
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
