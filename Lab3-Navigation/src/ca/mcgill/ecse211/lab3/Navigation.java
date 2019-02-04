package ca.mcgill.ecse211.lab3;

// non-static imports
import ca.mcgill.ecse211.odometer.*;

// static imports from Lab3 class
import static ca.mcgill.ecse211.lab3.Lab3.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.SENSOR_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.TILE;
import static ca.mcgill.ecse211.lab3.Lab3.WHEEL_RAD;
import static ca.mcgill.ecse211.lab3.Lab3.TO_RAD;
import static ca.mcgill.ecse211.lab3.Lab3.TO_DEG;
import static ca.mcgill.ecse211.lab3.Lab3.FWDSPEED;
import static ca.mcgill.ecse211.lab3.Lab3.TRNSPEED;
import static ca.mcgill.ecse211.lab3.Lab3.PATH;

/**
 * <p>
 * This class implements the simple navigator. Most of the constants are
 * introduced through importing from the Lab3 class. This class extends the
 * Thread class to allow simultaneous execution, so that other classes can work
 * alongside this class. Helper methods are added at the end to make conversions
 * easier.
 * 
 * <p>
 * The robot completes a path with a total of 5 points, specified using a
 * coordinate system relative to the start of the robot. The robot is (0,0),
 * with the front set as 0 degrees Theta, the right as the +x direction, and the
 * front as the +y direction. Theta increases when turning in the clockwise
 * direction. The robot first turns to face the point it is travelling to, and
 * then travels to that point.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 * 
 */
public class Navigation extends Thread {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	/**
	 * The width (in cm) of the robot measured from the center of the left wheel to
	 * the center of the right wheel. Not imported from Lab3 in case tuning is
	 * needed specifically for simple navigation
	 */
	private static final double TRACK = 13.30;
	
	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	/**
	 * The odometer instance
	 */
	private Odometer odo;

	/**
	 * Records odometer data returned, in a double precision array, specifying X, Y,
	 * and Theta values
	 */
	private double position[];

	/**
	 * A volatile boolean variable to indicate whether robot is currently travelling
	 * along a path, from one way point to the next. The volatile keyword tells the
	 * thread to check the current value of this variable in the main memory first
	 */
	private volatile boolean isNavigating;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * Constructor for this class, sets up the odometer instance to allow access to
	 * position data and initializes the isNavigating flag to false
	 * 
	 * @param odometer - the odometer instance passed from Lab3, gives access to
	 *                 retrieve position data
	 */
	public Navigation(Odometer odometer) {
		this.odo = odometer;
		this.isNavigating = false;
	}

	// -----------------------------------------------------------------------------
	// Run Method
	// -----------------------------------------------------------------------------

	/**
	 * The run() method that is called when the thread is started
	 */
	public void run() {

		for (int[] inner : PATH) {
			travelTo(inner[0], inner[1]);
		}

	}

	/**
	 * 
	 * @param x - the x coordinate with the robot as the origin (0,0)
	 * @param y - the y coordinate with the robot as the origin (0,0)
	 */
	public void travelTo(double x, double y) {
		/*
		 * this method causes the robot to travel to the absolute field location (x, y)
		 * specified in the tile points. This method should continuously call
		 * turnTO(double theta) and then set the motor speed to forward(straight). this
		 * will make sure ur heading is updated until u reach ur exact goal. this method
		 * will poll odometer for info
		 */

		// Convert coordinates x, y to length in cm
		x = x * TILE;
		y = y * TILE;

		isNavigating = true; // update status

		position = odo.getXYT(); // current position

		// position[0] = x, position[1] = y, position[2] = theta
		double dx = x - position[0]; // displacement in x
		double dy = y - position[1]; // displacment in y
		double ds = Math.hypot(dx, dy); // calculates the hypotenuse of dx and dy --> gives the displacement robot will
										// need to travel to get to destination
		double dTheta = Math.atan(dy / dx) * TO_DEG; // calculates angle dTheta of new displacement
														// will be in the range of [-90,90] degrees

		// our convention being north = 0 degrees + increase clockwise, this new angle
		// is the absolute angle
		if (dTheta >= 0 && dx >= 0) {
			// first quadrant
			dTheta = 90 - dTheta; // absolute angle
		} else if (dTheta >= 0 && dx < 0) {
			// 3rd quadrant
			dTheta = 270 - dTheta; // absolute angle
		} else if (dTheta < 0 && dx >= 0) {
			// 4th quadrant
			dTheta = 90 - dTheta; // absolute angle
		} else if (dTheta < 0 && dx < 0) {
			// 2nd quadrant
			dTheta = 270 - dTheta; // absolute angle
		}

		turnTo(dTheta); // robot turns

		// sets to forward speed
		LEFT_MOTOR.setSpeed(FWDSPEED);
		RIGHT_MOTOR.setSpeed(FWDSPEED);

		// Smooth Acceleration (Test)
		LEFT_MOTOR.setAcceleration(500);
		RIGHT_MOTOR.setAcceleration(500);

		LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true); // from square driver, goes straight
		RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), false);

		isNavigating = false; // update status

	}

	private void turnTo(double Theta) {
		// causes the robot to turn on point to absolute heading theta
		// should turn at minimal angle to target

		isNavigating = true; // update status

		// Smooth Acceleration (Test)
		LEFT_MOTOR.setAcceleration(250);
		RIGHT_MOTOR.setAcceleration(250);

		double minTheta = ((Theta - position[2]) + 360) % 360; // right turn angle

		if (minTheta > 0 && minTheta <= 180) { // already min angle, turn right
			LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false); // from square driver
		} else if (minTheta > 180 && minTheta < 360) { // will not be minimal angle by turning right
			// turn left
			// opposite from square driver since we turn left
			minTheta = 360 - minTheta; // since we are turning in the opposite direction
			LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), false);
		}

		isNavigating = false; // update status

	}

	/**
	 * Wrapper method to determine whether robot is currently navigating by checking
	 * class variable {@code isNavigating}
	 * 
	 * @return true if another thread has called travelTo() or turnTo() and the
	 *         method has yet to return, false otherwise
	 */
	public boolean isNavigating() {
		return isNavigating;
	}

	/**
	 * This is a static method allows the conversion of a distance to the total
	 * rotation of each wheel need to cover that distance.
	 * 
	 * (Distance / Wheel Circumference) = Number of wheel rotations. Number of
	 * rotations * 360.0 degrees = Total number of degrees needed to turn.
	 * 
	 * @param radius   - Radius of the wheel
	 * @param distance - Distance of path
	 * @return an integer indicating the total rotation angle for wheel to cover the
	 *         distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This is a static method that converts the angle needed to turn at a corner to
	 * the equivalent total rotation. This method first converts the degrees of
	 * rotation, radius of wheels, and width of robot to distance needed to cover by
	 * the wheel, then the method calls another static method in process to convert
	 * distance to the number of degrees of rotation.
	 * 
	 * @param radius - the radius of the wheels
	 * @param width  - the track of the robot
	 * @param angle  - the angle for the turn
	 * @return an int indicating the total rotation sufficient for wheel to cover
	 *         turn angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
