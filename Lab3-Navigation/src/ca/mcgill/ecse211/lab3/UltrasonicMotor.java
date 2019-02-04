package ca.mcgill.ecse211.lab3;

// static import
import static ca.mcgill.ecse211.lab3.Lab3.SENSOR_MOTOR;

/**
 * This class specifies the implementation for the EV3 medium motor that
 * controls the sweeping of the ultrasonic sensor. The position of the sensor
 * depends on the state set in the USNav class. There is a setter method for
 * this reason. This class extends the Thread class to allow simultaneous
 * execution of sensor sweeping while performing navigating and other tasks.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 *
 */
public class UltrasonicMotor extends Thread {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	/**
	 * The sweep angle of the ultrasonic sensor in one direction (in degrees)
	 */
	private static final int SWITCH_ANGLE = 30;

	/**
	 * The angle (in degrees) the sensor turns to when in wall following mode
	 */
	private static final int WALL_FOLLOWING_ANGLE = 75;

	/**
	 * The original angle (in degrees) for the ultrasonic sensor
	 */
	private static final int ORIGIN = 0;

	/**
	 * A value for motor acceleration that makes the sweeping motion smooth
	 */
	private static final int SMOOTH_ACCELERATION = 500;

	/**
	 * The state corresponding to the robot not at obstacle avoiding mode. Used for
	 * wallfollowing variable to adjust ultrasonic sensor motor
	 */
	private static final int NOT_AVOIDING = 0;

	/**
	 * The state corresponding to the robot at right bangbang mode. Used for
	 * wallfollowing variable to adjust ultrasonic sensor motor
	 */
	private static final int RIGHT_BANGBANG = 1;

	/**
	 * The state corresponding to the robot at left bangbang mode. Used for
	 * wallfollowing variable to adjust ultrasonic sensor motor
	 */
	private static final int LEFT_BANGBANG = 2;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	/**
	 * A status to indicate the mode of wallfollowing the robot is in. Options are
	 * either not wallfollowing, left bangbang, or right bangbang. Used to determine
	 * ultrasonic sensor heading.
	 */
	private int wallfollowing;

	/**
	 * A flag to indicate whether the sensor is in the correct heading. This ensures
	 * the robot does not start bangbang when sensor is not in correct angle.
	 */
	private boolean ready4turn;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * Constructor for class. Initializes the wallfollowing state to not at
	 * wallfollowing.
	 */
	public UltrasonicMotor() {
		this.wallfollowing = 0;
	}

	// -----------------------------------------------------------------------------
	// Run Method
	// -----------------------------------------------------------------------------

	/**
	 * The run() method called when thread starts. Consists of a while loop that
	 * does not end. This turns the sensor in three angles repeatedly, and switches
	 * to special angles when in bangbang modes.
	 */
	public void run() {

		// smooth turn
		SENSOR_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

		// keep the ultrasonic sensor turning
		while (true) {
			ready4turn = false; // not in bangbang mode

			// sweeps the sensor
			// the false parameter forces sensor to finish one execution before proceeding
			// to the next
			SENSOR_MOTOR.rotateTo(-SWITCH_ANGLE, false);
			SENSOR_MOTOR.rotateTo(ORIGIN, false);
			SENSOR_MOTOR.rotateTo(SWITCH_ANGLE, false);

			if (wallfollowing == LEFT_BANGBANG) { // currently obstacle avoiding in right bangbang

				SENSOR_MOTOR.setAcceleration(SMOOTH_ACCELERATION); // ensure smoother turn

				// turn sensor RIGHT to 45 deg to wall
				SENSOR_MOTOR.rotateTo(-WALL_FOLLOWING_ANGLE, false);
				SENSOR_MOTOR.stop(false); // fix sensor at position
				ready4turn = true; // ready for bangbang

				// prevent sensor from going back to sweeping
				while (wallfollowing != NOT_AVOIDING) {
					// do nothing
				}
			} else if (wallfollowing == RIGHT_BANGBANG) { // currently obstacle avoiding in left bangbang

				SENSOR_MOTOR.setAcceleration(SMOOTH_ACCELERATION); // ensure smoother turn

				// turn sensor LEFT to 45 deg to wall
				SENSOR_MOTOR.rotateTo(WALL_FOLLOWING_ANGLE, false);
				SENSOR_MOTOR.stop(false); // ensure smoother turn
				ready4turn = true; // ready for bangbang

				// prevent sensor from going back to sweeping
				while (wallfollowing != NOT_AVOIDING) {
					// do nothing
				}
			}
		}
	}

	/**
	 * Setter method to allow setting of the status that determines the behaviour of
	 * the ultrasonic sensor motor. Different values result in the sensor facing
	 * different angles, either turning or not turning
	 * 
	 * @param type - left or right bangbang or not avoiding obstacles
	 */
	public void setstatus(int type) {
		this.wallfollowing = type;
	}

	/**
	 * Getter method to retrieve information on whether the sensor is facing the
	 * correct side and at the correct angle. Controls the start of the bangbang
	 * mode
	 * 
	 * @return a boolean value indicating whether sensor is ready and bangbang can
	 *         start. True = ready; False = not ready
	 */
	public boolean ready() {
		return this.ready4turn;
	}

}
