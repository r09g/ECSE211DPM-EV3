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

// non-static import
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

	/**
	 * Odometer update period (in ms) 
	 */
	private static final long ODOMETER_PERIOD = 25;

	/**
	 * A constant factor that can be applied to convert angular units in degrees to
	 * radians
	 */
	private static final double TO_RAD = Math.PI / 180.0;

	/**
	 * A constant factor that can be applied to convert angular units in radians to
	 * degrees
	 */
	private static final double TO_DEG = 180.0 / Math.PI;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	// odometer instance
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
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be
	 * used only if an odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * This method is where the logic for the odometer will run. Use the methods
	 * provided from the OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
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
			double leftDistance = WHEEL_RAD * leftPhi * TO_RAD;
			double rightDistance = WHEEL_RAD * rightPhi * TO_RAD;

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

			// Update odometer values with new calculated values
			// theta is in degrees
			odo.update(dX, dY, radTheta * TO_DEG);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
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
