/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class describes the odometer function of the robot. The odometer records
 * the position data based on the degrees rotated of the left motor and the
 * right motor. The class contains constants that set the behaviour of the
 * odometer, and also public methods that facilitate the pass of control to
 * outside the class.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */

public class Odometer extends OdometerData implements Runnable {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	// odometer update period in ms
	private static final long ODOMETER_PERIOD = 25;

	// degrees -> radians conversion
	private static final double toRad = Math.PI / 180.0;

	// radians -> degrees conversion
	private static final double toDeg = 180.0 / Math.PI;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	// provides access to odometer data
	private OdometerData odoData;

	// Returned as singleton
	private static Odometer odo = null;

	// total degrees turned by left motor and right motor
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;

	// the left motor and right motor objects
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	// heading of the robot in radians
	private double Theta;

	// distance between left and right wheels
	private final double TRACK;

	// wheel radius
	private final double WHEEL_RAD;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * This is the default constructor of this class. It initiates all motors and
	 * variables once. It cannot be accessed externally. The position is set to
	 * (0,0,0) at start.
	 * 
	 * @param leftMotor  - the left motor object to be initialized
	 * @param rightMotor - the right motor object to be initialized
	 * @throws OdometerExceptions - throws the OdometerExceptions if fails to
	 *                            retrieve control for odometer object
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD) throws OdometerExceptions {

		// retrieves odometerData control, allows control of x,y,and theta values
		odoData = OdometerData.getOdometerData();

		// initializes the left and right motors
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and theta to 0
		odoData.setXYT(0, 0, 0);

		// resets the left motor and right motor tacho counts to 0
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		// resets heading to 0
		this.Theta = 0.0;

		// the width of the robot from center of left wheel to center of right wheel
		this.TRACK = TRACK;

		// the wheel radius
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used
	 * throughout the code.
	 * 
	 * @param leftMotor  - existing left motor instance
	 * @param rightMotor - existing right motor instance
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions - exception thrown if multiple odometer instances
	 *                            are created
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {

		// checks if a Odometer object current exists
		if (odo != null) {

			// Return existing object
			return odo;

		} else {

			// no OdometerData objects have been instantiated yet
			// create new instance of Odometer
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

			// returns the new odometer instance
			return odo;

		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created.
	 * 
	 * @throws OdometerExceptions - if no previous Odometer instance exists
	 * @return an existing odometer instance
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		// checks if a Odometer object current exists
		if (odo == null) {

			// if not instantiated, throw exception
			throw new OdometerExceptions("No previous Odometer exits.");

		}

		// if there exist an Odometer instance, return
		return odo;

	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer. The odometer
	 * records the current position of the robot by counting the difference in tacho
	 * counts of the left and right tachometers.
	 * 
	 * <p>
	 * The odometer uses the convention where the initial heading is the +Y
	 * direction with Theta = 0. Theta increases if robot turns right. To the right
	 * of the robot is the +X direction.
	 */
	public void run() {

		// variables to store the start and finish time
		long updateStart, updateEnd;

		while (true) {

			// get current time
			updateStart = System.currentTimeMillis();

			// get current TachoCount in degrees
			int leftMotorTachoCountNew = leftMotor.getTachoCount();
			int rightMotorTachoCountNew = rightMotor.getTachoCount();

			// degrees rotated for left and right
			int leftPhi = leftMotorTachoCountNew - leftMotorTachoCount;
			int rightPhi = rightMotorTachoCountNew - rightMotorTachoCount;

			// save counts for next iteration
			leftMotorTachoCount = leftMotorTachoCountNew;
			rightMotorTachoCount = rightMotorTachoCountNew;

			// convert angular displacement to linear displacement
			double leftDistance = WHEEL_RAD * leftPhi * toRad;
			double rightDistance = WHEEL_RAD * rightPhi * toRad;

			// change in displacement of vehicle
			double dDisp = 0.5 * (leftDistance + rightDistance);

			// change in heading in radians
			double radTheta = (leftDistance - rightDistance) / TRACK;

			// update heading
			Theta += radTheta;

			// compute x, y component of displacement
			// sin and cos uses radian
			double dX = Math.sin(Theta) * dDisp;
			double dY = Math.cos(Theta) * dDisp;

			// TODO Update odometer values with new calculated values
			// theta is in degrees
			odo.update(dX, dY, radTheta * toDeg);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();

			// if program runs less than target time
			// sleep thread for the missing time
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}
