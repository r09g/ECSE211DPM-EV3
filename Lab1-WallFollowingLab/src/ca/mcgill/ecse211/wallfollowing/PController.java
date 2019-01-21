package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * <p>
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
	 * The constant is an integer applied to the error (difference between target
	 * band center and current distance value) for adjusting the left and right
	 * motor speed.
	 */
	private static final int CONSTANT = 9;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	/**
	 * This variable specifies the target distance (offset) the robot aims to keep
	 * from the wall. Note this distance is the distance in terms of the ultrasonic
	 * sensor's line of sight rather than the distance in terms of the robot itself.
	 * 
	 * @see WallFollowingLab
	 */
	private final int bandCenter;

	/**
	 * This variable specifies the range of deviation from the {@link #bandCenter}
	 * that the robot does not act upon. Within this range of distance, the robot
	 * will not make changes to the speed of its motors
	 */
	private final int bandWidth;

	/**
	 * This variable is used to store the distance read by the ultrasonic sensor
	 */
	private int distance;

	/**
	 * This variable plays an important role in implementing a filter mechanism. The
	 * variable stores the number of times the robot's ultrasonic sensor gives a
	 * reading larger than the {@link #BOUND}, consecutively. This variable helps
	 * the robot determine whether it is at a corner (and turn left), or simply
	 * passing a wall gap (and go straight).
	 */
	private int filter;

	/**
	 * This variable is used to store the difference between the current distance
	 * measured by the ultrasonic sensor and the target distance for the robot to
	 * maintain.
	 */
	private int error;

	/**
	 * Constructor that creates an instance of the PController class and initializes
	 * the class variables. The bandCenter is adjusted with respect to the initial
	 * input to tune the robot to its best performance. Note this adjustment should
	 * vary with different robot and physical conditions (i.e. tires, dirt on
	 * ground, ground material, and other factors)
	 * 
	 * @param bandCenter distance for the robot to keep from the wall, value is set
	 *                   in the WallFollowingLab class
	 * @param bandWidth  range where the robot does not make adjustments to the
	 *                   motor speed, value is set in the WallFollowingLab class
	 * 
	 * @see WallFollowingLab
	 */
	public PController(int bandCenter, int bandWidth) {
		this.bandCenter = bandCenter - 3;
		this.bandWidth = bandWidth;
		this.filter = 0;
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
			filter++;
		}

		// the distance reading is within a valid range
		if (this.distance < BOUND) {

			filter = 0; // clear filter value since an object is detected

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

				if (this.distance < 10) {
					try {
						Thread.sleep(175);
					} catch (Exception e) {

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
			// check filter
			if (filter > 30) {
				// out of bounds distance recorded more than 40 times, so there really is
				// nothing there
				// Make a sharper and
				error = Math.abs(this.distance - this.bandCenter);

				// Control upper bound of error to protect motor
				// When there is no object the sensor returns 2147483647
				// which may destroy the motor if rpm is too high
				if (error > 15) {
					error = 15;
				}

				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 30 - CONSTANT * error); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + CONSTANT * error); // speed up right motor
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else { // if sensor needs more time to complete filter
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED / 3); // slow down left motor
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED / 3); // slow down right motor
				// proceeds to slow robot down in straight line, giving it more time to complete
				// filter without it moving too far
				// away from wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
			// if filter < 15 then either it was a mistake and there is indeed a wall,
			// either
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
