package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class specifies the proportional-type controller for the EV3 robot. This
 * type of controller adjusts the direction of the robot through making
 * adjustments that are proportional to the error, which is the absolute value
 * of the difference between the current distance from the wall and the optimal
 * distance from the wall. Ideally, the PController should achieve the purpose
 * of reducing oscillations until they are at insignificant magnitudes. However,
 * the effectiveness of this implementation depends heavily on the accuracy of
 * the ultrasonic sensor itself.
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

public class PController implements UltrasonicController {

	// ----------------------------------------------------------------------------
	// Constants
	// ----------------------------------------------------------------------------

	/**
	 * The normal speed of the left and right motor when following the wall.
	 */
	private static final int MOTOR_SPEED = 250; // normal speed

	/**
	 * The upper limit for the distance readings from the ultrasonic sensor, any
	 * value above this limit would be identified as a potential false negative,
	 * filter would be activated.
	 */
	private static final int BOUND = 160;

	/**
	 * The constant is an integer amplification applied to the error (difference
	 * between target band center and current distance value) for adjusting the left
	 * and right motor speed.
	 */
	private static final int CONSTANT = 9;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	/**
	 * The target distance (offset) the robot aims to keep from the wall. Note this
	 * distance is the distance in terms of the ultrasonic sensor's line of sight
	 * rather than the distance in terms of the robot itself.
	 * 
	 * @see WallFollowingLab
	 */
	private final int bandCenter;

	/**
	 * This final variable specifies the range of deviation from the
	 * {@link #bandCenter} that the robot does not act upon. Within this range of
	 * distance, the robot will not make changes to the speed of its motors
	 */
	private final int bandWidth;

	/**
	 * The distance read by the ultrasonic sensor
	 */
	private int distance;

	/**
	 * The number of times the robot's ultrasonic sensor gives a reading larger than
	 * the {@link #BOUND}, consecutively. This variable helps the robot determine
	 * whether it is at a corner (and turn left), or simply passing a wall gap (and
	 * go straight). This variable plays an important role in implementing a filter
	 * mechanism.
	 */
	private int filter;

	/**
	 * The absolute difference between the current distance measured by the
	 * ultrasonic sensor and the target distance for the robot to maintain.
	 */
	private int error;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * Constructor that creates an instance of the PController class and initializes
	 * the class variables. The {@code bandCenter} is adjusted by -3 with respect to
	 * the initial input to tune the robot to its best performance. This makes the
	 * robot stay closer to the wall. Note this adjustment should vary with
	 * different robot and physical conditions (i.e. tires, dirt on ground, ground
	 * material, and other factors).
	 * 
	 * @param bandCenter distance for the robot to keep from the wall in centimeters
	 * @param bandWidth  range where the robot does not make adjustments to the
	 *                   motor speed in centimeters
	 * 
	 * @see WallFollowingLab
	 */
	public PController(int bandCenter, int bandWidth) {
		this.bandCenter = bandCenter - 3; // band center adjustment
		this.bandWidth = bandWidth;
		this.filter = 0;
		this.error = 0;
		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // initial left motor speed
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); // initial right motor speed
	}

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * Core implementation of the proportional-type controller. Processes the
	 * {@link distance} and make adjustments on the motor speed depending on the
	 * {@link error}. The adjustments are proportional to {@code error}.
	 * 
	 * <p>
	 * When the distance is greater than the {@code bandCenter} by only more than
	 * the {@code bandWidth}, the robot makes a left turn. If the distance is ever
	 * smaller than the {@code bandCenter}, the robot turns right immediately. This
	 * allows the robot to maintain a greater safety clearance from the wall. When
	 * the distance is within the band width, the robot is set to go straight at
	 * {@link #MOTOR_SPEED}
	 * 
	 * <p>
	 * A filter is implemented in this class to 1) Filter out false negatives and
	 * prevent unnecessary left/right oscillations of the robot, 2) distinguish
	 * between wall gap and edge of wall. The filter functions when the input is
	 * above the {@link BOUND}. At a particular angle to the wall, the sensor gives
	 * a false negative with value 2147483647. Specifically, the robot suddenly
	 * slows down to 1/3 of its original speed and moves in a straight path while
	 * the filter counts the number of consecutive occurrences of distance inputs
	 * over {@code BOUND}, resulting in a sudden decrease in speed. If the filter
	 * count is interrupted by an input value lower than {@code BOUND}, the filter
	 * resets to 0 and the robot adjusts the motor speeds depending on the input; if
	 * the filter count (30) is satisfied, there is probably nothing there and the
	 * robot turns left, with speed adjustments that are in accordance with the
	 * p-type controller logic. When performing the left turn, the left motor speed
	 * is increased by 60 rpm in addition to the proportional adjustment to allow
	 * the robot to pass through wider 180-degree convex corners by making a U-turn.
	 * 
	 * <p>
	 * The {@code error} is capped in this method to protect the motor from spinning
	 * at a dangerously high speed. The cap is set at 20 for distance below the
	 * {@code BOUND} and 15 for distance over {@code BOUND}.
	 * 
	 * <p>
	 * A call to {@code Thread.sleep()} is also when the robot is too close to the
	 * wall, at a distance less than 12. The robot then sets the motor speeds to
	 * turns right and this thread is suspended for 175 ms, thus no new actions are
	 * performed within the time interval. However, the {@code Thread.sleep()}
	 * method must be wrapped in a try-catch block as the method throws
	 * {@code InterruptedException}, a checked exception that must be caught.
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
	 */
	@Override
	public void processUSData(int distance) {
		this.distance = distance;

		if (this.distance < BOUND) { // distance within 160 cm
			filter = 0; // reset filter
			error = Math.abs(this.distance - this.bandCenter); // absolute difference between robot and wall

			if (error > 20) { // error cap to protect motor
				error = 20;
			}

			if (this.distance > this.bandCenter + this.bandWidth) { // robot is too far

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - CONSTANT * error); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up right motor

				WallFollowingLab.rightMotor.forward(); // EV3 motor hack
				WallFollowingLab.leftMotor.forward();

			} else if (this.distance < this.bandCenter) { // robot is close to the wall

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - CONSTANT * error); // slow down right motor

				WallFollowingLab.rightMotor.forward(); // EV3 motor hack
				WallFollowingLab.leftMotor.forward();

				if (this.distance < 12) { // robot is very close to the wall
					try { // try-catch block, sleep method throws an exception
						Thread.sleep(175); // suspend thread to allow motor to turn right longer
					} catch (Exception e) {

					}
				}

			} else { // the robot is within the bandwidth
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // set left motor at 250 rpm
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); // set right motor at 250 rpm

				WallFollowingLab.rightMotor.forward(); // EV3 motor hack
				WallFollowingLab.leftMotor.forward();
			}
		} else { // distance over 160
			filter++; // increment filter count

			if (filter > 30) { // >30 consecutive large distance readings, probably really nothing there
				error = Math.abs(this.distance - this.bandCenter); // absolute value of distance between wall and robot

				if (error > 15) { // cap the error to protect motor
					error = 15;
				}

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 60 - CONSTANT * error); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up right motor

				WallFollowingLab.rightMotor.forward(); // EV3 motor hack
				WallFollowingLab.leftMotor.forward();
			} else { // filter count less than 30
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 3); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 3); // slow down right motor

				WallFollowingLab.rightMotor.forward(); // EV3 motor hack
				WallFollowingLab.leftMotor.forward();
			}
		}
		return;
	}

	/**
	 * A wrapper method which returns the value of the local private class variable
	 * {@code distance}
	 * 
	 * @return this.distance the distance measured by the ultrasonic sensor at ~45
	 *         degrees from the robot to the wall
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
