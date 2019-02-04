package ca.mcgill.ecse211.lab3;

// non-static imports
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.robotics.SampleProvider;

// static imports from Lab3 class
import static ca.mcgill.ecse211.lab3.Lab3.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.TILE;
import static ca.mcgill.ecse211.lab3.Lab3.WHEEL_RAD;
import static ca.mcgill.ecse211.lab3.Lab3.PATH;

/**
 * <p>
 * This class implements the advanced navigator with the ultrasonic sensor. Some
 * of the existing motor instances and a few constants not dependent on
 * navigation. This class extends the Thread class to allow simultaneous
 * execution, so that other classes can work alongside this class. Helper
 * methods are added at the end to make conversions easier.
 * 
 * <p>
 * The robot completes a path with a total of 5 points, specified using a
 * coordinate system relative to the start of the robot. The robot is (0,0),
 * with the front set as 0 degrees Theta, the right as the +x direction, and the
 * front as the +y direction. Theta increases when turning in the clockwise
 * direction. The robot first turns to face the point it is travelling to, and
 * then travels to that point.
 * 
 * <p>
 * The robot uses the ultrasonic sensor to detect obstacles and avoid them if
 * necessary, using either the left bangbang mode or the right bangbang mode.
 * There are many constants in this class that are related to the bangbang
 * controllers.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 * 
 */
public class USNav extends Thread {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	/**
	 * The width (in cm) of the robot measured from the center of the left wheel to
	 * the center of the right wheel. Not imported from Lab3 because track needs to
	 * be tuned specifically for obstacle avoidance navigation
	 */
	private static final double TRACK = 13.21;

	/**
	 * A constant factor that can be applied to convert angular units in radians to
	 * degrees
	 */
	private static final double TO_DEG = 180.0 / Math.PI;

	/**
	 * The speed at which the robot moves straight (in rpm)
	 */
	private static final int FWDSPEED = 250;

	/**
	 * Angle correction for Quadrant 1 and 4. Arctan returns the correct angle and
	 * the only adjustment needed is to turn it into an angle that starts from Theta
	 * = 0 and increases clockwise
	 */
	private static final int Q1Q4COR = 90;

	/**
	 * Angle correction for Quadrant 2 and 3. Arctan returns the incorrect angle and
	 * the adjustment needed is to add pi to the angle and turn it into an angle
	 * that starts from Theta = 0 and increases clockwise
	 */
	private static final int Q2Q3COR = 270;

	/**
	 * The center of the board platform that the EV3 runs on. This is used in
	 * determining which quadrant the destination coordinate is in relative to the
	 * robot's current location and whether arctan needs correction
	 */
	private static final int CENTER = 0;

	/**
	 * A value for motor acceleration that prevents the wheels from slipping on the
	 * demo floor by accelerating and decelerating slowly
	 */
	private static final int SMOOTH_ACCELERATION = 500;

	/**
	 * A value for motor acceleration that prevents the wheels from slipping on the
	 * demo floor by accelerating and decelerating slowly
	 */
	private static final int TURN_ACCELERATION = 250;

	/**
	 * A revolution of half of a circle in degrees
	 */
	private static final int HALF_CIRCLE = 180;

	/**
	 * A full revolution of a circle in degrees
	 */
	private static final int FULL_CIRCLE = 360;

	/**
	 * The heading/Theta value of the robot initially
	 */
	private static final int INITIAL_ANGLE = 0;

	/**
	 * Robot's ideal offset from the wall (in cm)
	 */
	private static final int BANDCENTER = 12;

	/**
	 * Avoid hunting behaviour. Within this range (in cm) from the band center, the
	 * bangbang controller does not make adjustments
	 */
	private static final int BANDWIDTH = 3;

	/**
	 * The fast speed for the motor, used in bangbang controller (in deg/sec)
	 */
	private static final int MOTOR_HIGH = 275;

	/**
	 * The medium speed for the motor, used in bangbang controller (in deg/sec)
	 */
	private static final int MOTOR_MEDIUM = 137;

	/**
	 * The distance that triggers the bangbang controller to avoid the obstacle (in
	 * cm)
	 */
	private static final int AVOID_DISTANCE = 15;

	/**
	 * A value that controls the maximum distance (in cm) the robot travels in
	 * bangbang mode, allows exiting of obstacle avoidance state
	 */
	private static final int MAX_AVOID_DISTANCE = 55;

	/**
	 * A signal indicating the robot is currently not in obstacle avoidance mode.
	 * This is mainly used to signal the ultrasonic motor to sweep the sensor
	 */
	private static final int NOT_AVOIDING = 0;

	/**
	 * A signal indicating the robot is currently in right bangbang mode. This is
	 * mainly used to signal the ultrasonic motor to turn the motor left to face the
	 * wall and hold it there
	 */
	private static final int RIGHT_BANGBANG = 1;

	/**
	 * A signal indicating the robot is currently in left bangbang mode. This is
	 * mainly used to signal the ultrasonic motor to turn the motor right to face
	 * the wall and hold it there
	 */
	private static final int LEFT_BANGBANG = 2;

	/**
	 * The clockwise angle turned from Theta = 0 (north). This is the 45 degree line
	 * in quadrant 2. This is used in deciding whether to execute left bangbang or
	 * right bangbang
	 */
	private static final int TOP_LEFT = 315;

	/**
	 * The clockwise angle turned from Theta = 0 (north). This is the 45 degree line
	 * in quadrant 1. This is used in deciding whether to execute left bangbang or
	 * right bangbang
	 */
	private static final int TOP_RIGHT = 45;

	/**
	 * The clockwise angle turned from Theta = 0 (north). This is the 45 degree line
	 * in quadrant 3. This is used in deciding whether to execute left bangbang or
	 * right bangbang
	 */
	private static final int BOTTOM_LEFT = 225;

	/**
	 * The clockwise angle turned from Theta = 0 (north). This is the 45 degree line
	 * in quadrant 4. This is used in deciding whether to execute left bangbang or
	 * right bangbang
	 */
	private static final int BOTTOM_RIGHT = 135;

	/**
	 * This is the threshold for ultrasonic sensor readings. Values above this
	 * threshold activate the tally (counter) and tells the robot that it might need
	 * to make a turn rather than adjustments
	 */
	private static final int DISTANCE_THRESHOLD = 160;

	/**
	 * The threshold needed for the tally (counter). If this is satisfied then a
	 * left turn is executed rather than left adjustments
	 */
	private static final int MAX_TALLY = 30;

	/**
	 * Value of left adjustment needed for left motor in right bangbang mode
	 */
	private static final int R_LEFT_ADJUSTMENT = 90;

	/**
	 * Value of right adjustment needed for right motor in right bangbang mode
	 */
	private static final int R_RIGHT_ADJUSTMENT = 70;

	/**
	 * Value of left turn adjustment needed for left motor in right bangbang mode
	 */
	private static final int R_LT_LM_ADJUSTMENT = 150;

	/**
	 * Value of left turn adjustment needed for right motor in right bangbang mode
	 */
	private static final int R_LT_RM_ADJUSTMENT = 375;

	/**
	 * Value of right turn adjustment needed for left motor in right bangbang mode
	 */
	private static final int R_RT_LM_ADJUSTMENT = 395;

	/**
	 * Value of right turn adjustment needed for right motor in right bangbang mode
	 */
	private static final int R_RT_RM_ADJUSTMENT = 10;

	/**
	 * Value of left adjustment needed for left shift in left bangbang mode
	 */
	private static final int L_LEFT_ADJUSTMENT = 70;

	/**
	 * Value of right adjustment needed for right shift in left bangbang mode
	 */
	private static final int L_RIGHT_ADJUSTMENT = 110;

	/**
	 * Value of right turn adjustment needed for left motor in left bangbang mode
	 */
	private static final int L_RT_LM_ADJUSTMENT = 375;

	/**
	 * Value of right turn adjustment needed for right motor in left bangbang mode
	 */
	private static final int L_RT_RM_ADJUSTMENT = 150;

	/**
	 * Value of left turn adjustment needed for left motor in left bangbang mode
	 */
	private static final int L_LT_LM_ADJUSTMENT = 10;

	/**
	 * Value of left turn adjustment needed for right motor in left bangbang mode
	 */
	private static final int L_LT_RM_ADJUSTMENT = 455;

	/**
	 * The sleep time (in ms) to control ultrasonic ssensor sampling rate
	 */
	private static final int IDLE_TIME = 50;

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
	 * Access to fetch ultrasonic sensor readings
	 */
	private SampleProvider us;

	/**
	 * Buffer to store sensor reading data of type float
	 */
	private float[] usData;

	/**
	 * The motor instance that controls the sweeping of the ultrasonic sensor. Motor
	 * used is the EV3 medium motor
	 */
	private UltrasonicMotor usMotor;

	/**
	 * A volatile boolean variable to indicate whether robot is currently travelling
	 * along a path, from one way point to the next. The volatile keyword tells the
	 * thread to check the current value of this variable in the main memory first
	 */
	private volatile boolean isNavigating;

	/**
	 * Stores the distance reading, after applying a median medianFilter, from the
	 * ultrasonic sensor
	 */
	private double distance;

	/**
	 * Records the distance travelled (in cm) after the occurrence of an obstacle in
	 * the robot's path. This is used to control the length of the bangbang
	 * controller execution and assist in the exiting of the obstacle avoidance
	 * state
	 */
	private double turncount;

	/**
	 * This is a boolean flag which signals whether the robot has reached the
	 * destination coordinate. Useful for navigation after avoiding an obstacle
	 */
	private boolean completed;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * This constructor passes in the existing instances of the odometer and the
	 * ultrasonic sensor related instances. The values of turncount and navigating
	 * status are also initialized here to 0. Note that motor related instances are
	 * imported as static objects at the top of this class.
	 *
	 * @param odometer - the existing odometer instance, allows acquiring position
	 *                 data
	 * @param us       - access to acquiring ultrasonic sensor reading datas
	 * @param usData   - buffer to store the ultrasonic sensor readings
	 * @param usMotor  - access to controlling and checking states of the motor that
	 *                 controls the sweeping of sensor
	 */
	public USNav(Odometer odometer, SampleProvider us, float[] usData, UltrasonicMotor usMotor) {
		this.odo = odometer;
		this.us = us; // ultrasonic sensor
		this.usData = usData;
		this.usMotor = usMotor; // motor for ultrasonic sensors
		this.isNavigating = false; // navigating state
		this.turncount = 0; // distance travelled in bangbang
	}

	// -----------------------------------------------------------------------------
	// Run Method
	// -----------------------------------------------------------------------------

	/**
	 * The run() method that is called when the thread is started. The five
	 * coordinate points are used in order as parameters of the {@code travelTo}
	 * function, which is called a total of five times. There is a completed flag in
	 * this method to account for the case when the robot avoided an obstacle. The
	 * robot is not at the destination but the travelTo() method returned. Robot
	 * needs to re-navigate and go to the same destination coordinate again.
	 */
	public void run() {

		for (int[] inner : PATH) {
			// initialize flag for reaching destination coordinate
			this.completed = false;

			// inner[0]: X coordinate
			// inner[1]: Y coordinate
			int x = inner[0];
			int y = inner[1];

			travelTo(x, y); // go to coordinate

			while (this.completed == false) {
				// if the robot avoided an obstacle, it needs to re-navigate and go to the same
				// destination coordinate again
				travelTo(x, y);
			}
		}
	}

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------
	
	/**
	 * <p>
	 * Controls the robot to travel to the coordinate (x,y) with the robot's initial
	 * starting location as the origin. This is done by retrieving current position
	 * data of the robot and calculating the new heading the robot needs to have, as
	 * well as the distance the robot needs to travel to reach its next destination.
	 * A minimum angle approach is taken, meaning that the robot will turn the
	 * smallest angle possible to adjust its heading.
	 * 
	 * <p>
	 * There is a logic implemented in this method to determine the angle the robot
	 * needs to turn, clockwise, to reach its new heading. This logic is necessary
	 * largely due to the values returned by arctan function. The arctan function
	 * only returns values ranging from -pi/2 to pi/2, and real values can be + or -
	 * pi from the returned value.
	 * 
	 * <p>
	 * The robot will check for ultrasonic sensor readings when the left and right
	 * motors are moving, to identify any obstacles and avoid them using the
	 * bangbang controller. The robot stops when encountering an obstacle avoidance
	 * and follows bangbang controller procedure (robot avoids from either left side
	 * or right side). After obstacle avoidance ends, the ultrasonic sensor motor is
	 * signaled to turn to a specific angle to allow bangbang; the completed flag is
	 * set to signal re-navigation; and the turn count is reset to 0. If no obstacle
	 * is encountered, the robot behaves the same as in simple navigation
	 * 
	 * @param x - the x coordinate of the destination
	 * @param y - the y coordinate of the destination
	 */
	public void travelTo(double x, double y) {

		// Convert coordinates x, y to length in cm
		x = x * TILE;
		y = y * TILE;

		isNavigating = true; // update navigation status
		position = odo.getXYT(); // get current position data from odometer

		// position[0] = x, position[1] = y, position[2] = theta
		double dx = x - position[0]; // displacement in x
		double dy = y - position[1]; // displacment in y
		double ds = Math.hypot(dx, dy); // calculates the hypotenuse of dx and dy --> gives the displacement robot will
										// need to travel to get to destination
		double dTheta = Math.atan(dy / dx) * TO_DEG;// calculates the angle the new displacement will be. Value in the
													// range of [-90,90] degrees

		// Convention: north = 0 degrees; Theta increases clockwise
		// The following logic is used to determine the angle the robot needs to turn
		if (dTheta >= CENTER && dx >= CENTER) {
			// 1st quadrant
			dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
		} else if (dTheta >= CENTER && dx < CENTER) {
			// 3rd quadrant, need to correct arctan value
			dTheta = Q2Q3COR - dTheta; // clockwise angle robot needs to turn
		} else if (dTheta < CENTER && dx >= CENTER) {
			// 4th quadrant
			dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
		} else if (dTheta < CENTER && dx < CENTER) {
			// 2nd quadrant, need to correct arctan value
			dTheta = Q2Q3COR - dTheta; // absolute angle
		}

		turnTo(dTheta); // robot turns at minimum angle

		// smooth acceleration so that wheels do not slip
		LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
		RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

		// sets both motors to forward speed
		LEFT_MOTOR.setSpeed(FWDSPEED);
		RIGHT_MOTOR.setSpeed(FWDSPEED);

		// rotates both motors for a fixed number of degrees equivalent to ds, the
		// distance from the robot's current location to the next destination point,
		// equivalent as travelling straight. The boolean flag parameter indicates
		// whether method returns immediately, to allow simultaneous execution of both
		// rotate() methods. The method does not wait for motors to complete before
		// proceeding to next command to allow obstacle avoidance during travel
		LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true);
		RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true);

		// while the robot is travelling along the paths
		while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {

			// acquire filtered (median) distance reading
			this.distance = medianFilter();

			// if robot is too close to obstacle
			if (distance <= AVOID_DISTANCE) {

				// stop robot; stop both motors, setting the first motor to return immediately
				// to avoid unwanted change in heading
				LEFT_MOTOR.stop(true);
				RIGHT_MOTOR.stop(false);

				bangbang(); // activate bangbang

				// stop robot
				LEFT_MOTOR.stop(true);
				RIGHT_MOTOR.stop(false);

				// smoother acceleration to avoid wheel slipping
				LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
				RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

				turncount = 0; // reset turncount
				usMotor.setstatus(NOT_AVOIDING); // sweep ultrasonic motor
				this.completed = false; // did not reach destination

				return; // prevent further looping and re-navigates
			}
		}

		isNavigating = false; // set navigation status
		this.completed = true; // destination coordinate reached

		return;
	}

	/**
	 * Turns the robot at a fixed position to face the next destination, changes the
	 * robot heading. The minimum angle is calculated by determining whether the
	 * clockwise angle Theta is greater than half of a full circles
	 * 
	 * @param Theta - the clockwise angle to turn from Theta = 0
	 */
	public void turnTo(double Theta) {

		isNavigating = true; // update navigating status

		// smoother turn acceleration to avoid wheels slipping
		LEFT_MOTOR.setAcceleration(TURN_ACCELERATION);
		RIGHT_MOTOR.setAcceleration(TURN_ACCELERATION);

		// ensure angle is positive and within 360
		double minTheta = ((Theta - position[2]) + FULL_CIRCLE) % FULL_CIRCLE;

		if (minTheta > INITIAL_ANGLE && minTheta <= HALF_CIRCLE) {
			// angle is already minimum angle, robot should turn clockwise
			LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
		} else if (minTheta > HALF_CIRCLE && minTheta < FULL_CIRCLE) {
			// angle is not minimum angle, robot should turn counter-clockwise to the
			// complementary angle of a full circle 360 degrees
			minTheta = FULL_CIRCLE - minTheta;
			LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), false);
		}

		isNavigating = false; // update navigation status
	}

	/**
	 * Getter method to determine whether robot is currently navigating by checking
	 * class variable {@code isNavigating}
	 * 
	 * @return true if another thread has called travelTo() or turnTo() and the
	 *         method has yet to return, false otherwise
	 */
	public boolean isNavigating() {
		return isNavigating;
	}
	
	// -----------------------------------------------------------------------------
	// Private Method
	// -----------------------------------------------------------------------------

	/**
	 * This is a median filter. The filter takes 5 consecutive readings from the
	 * ultrasonic sensor, amplifies them to increase sensor sensitivity, sorts them,
	 * and picks the median to minimize the influence of false negatives and false
	 * positives in sensor readings, if any. The sensor is very likely to report
	 * false negatives.
	 * 
	 * @return the median of the five readings, sorted from small to large
	 */
	private double medianFilter() {
		double[] arr = new double[5]; // store readings
		for (int i = 0; i < 5; i++) { // take 5 readingss
			us.fetchSample(usData, 0); // store reading in buffer
			arr[i] = usData[0] * 100.0; // signal amplification
		}
		Arrays.sort(arr); // sort readingss
		return arr[2]; // take median value
	}

	/**
	 * This method consists of a set of if-else statements and is used to determine
	 * whether to avoid the obstacle from the left side or from the right side
	 */
	private void bangbang() {

		// get current X, Y, Theta position data
		double[] pos = odo.getXYT();
		double x = pos[0];
		double y = pos[1];
		double heading = pos[2];

		// The logic determines the robot's current position in the coordinate system,
		// and chooses the style of bangbang that allows the robot to go around the
		// obstacle from the inside (the side near the center point of the coordinate
		// system (1,1)
		if (heading > TOP_LEFT || heading < TOP_RIGHT) {
			// the robot is travelling north
			if (x < TILE) { // if the robot is on the left part of board
				Rbangbang(); // right bangbang
			} else { // if the robot is on the right part of board
				Lbangbang(); // left bangbang
			}
		} else if (heading > TOP_RIGHT && heading < BOTTOM_RIGHT) {
			// the robot is travelling east
			if (y > TILE) { // if the robot is on the upper part of board
				Rbangbang(); // right bangbang
			} else { // if the robot is on the lower part of board
				Lbangbang(); // left bangbang
			}
		} else if (heading > BOTTOM_RIGHT && heading < BOTTOM_LEFT) {
			// the robot is travelling south
			if (x > TILE) { // if the robot is on the right part of board
				Rbangbang(); // right bangbang
			} else { // if the robot is on the left part of board
				Lbangbang(); // left bangbang
			}
		} else if (heading > BOTTOM_LEFT && heading < TOP_LEFT) {
			// the robot is travelling west
			if (y > TILE) { // if the robot is on the upper part of board
				Lbangbang(); // left bangbang
			} else { // if the robot is on the lower part of board
				Rbangbang(); // right bangbang
			}
		}
	}

	/**
	 * Original version of bangbang controller that avoids the obstacle from the
	 * right side. The robot is turned 90 degrees to the right and the ultrasonic
	 * sensor is turned to face left before starting Rbangbang() mode
	 */
	private void Rbangbang() {

		// smooth acceleration to reduce wheel slip
		LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
		RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

		// turn robot 90 degrees to the right, left wheel forward and right wheel
		// backward
		LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);

		usMotor.setstatus(RIGHT_BANGBANG); // turn sensor motor to face the wall on left
		while (!usMotor.ready()) {
			// wait for sensor to get in position
		}
		int tally = 0; // a counter used in bangbang for turning around corners

		// implementation of bangbang controller
		while (turncount < MAX_AVOID_DISTANCE) { // check for total bangbang distance

			// smooth acceleration to reduce wheel slip
			LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
			RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

			distance = medianFilter(); // get filtered sensor reading

			// record starting tacho counts for computing distance travelled in bangbang
			int prevLcount = LEFT_MOTOR.getTachoCount();
			int prevRcount = RIGHT_MOTOR.getTachoCount();

			double error = distance - BANDCENTER; // deviation from expected band center

			if (Math.abs(error) > (BANDWIDTH) && error > 0) { // robot is too far
				if (Math.abs(error) > (BANDWIDTH) && error > 0 && error < DISTANCE_THRESHOLD) {
					// make left adjustments not left turn
					LEFT_MOTOR.setSpeed(MOTOR_HIGH - R_LEFT_ADJUSTMENT); // slow down left motor
					RIGHT_MOTOR.setSpeed(MOTOR_HIGH + R_RIGHT_ADJUSTMENT); // speed up right motor
					LEFT_MOTOR.forward(); // EV3 motor hack
					RIGHT_MOTOR.forward();
				} else {
					// make a left turns
					tally++; // increment counter
					if (tally > MAX_TALLY) {
						// at edge of wall, make left turn
						LEFT_MOTOR.setSpeed(R_LT_LM_ADJUSTMENT); // slow down left motor
						RIGHT_MOTOR.setSpeed(R_LT_RM_ADJUSTMENT); // speed up right motor
						LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();// EV3 motor hack
					} else {
						// robot to go straight slower while counter not satisfied
						LEFT_MOTOR.setSpeed(MOTOR_MEDIUM);
						RIGHT_MOTOR.setSpeed(MOTOR_MEDIUM);
						LEFT_MOTOR.forward(); // EV3 motor hack
						RIGHT_MOTOR.forward();
					}
				}
			} else if (Math.abs(error) > (BANDWIDTH) && error < 0) { // robot is too close
				// make right adjustments
				LEFT_MOTOR.setSpeed(R_RT_LM_ADJUSTMENT); // speed up left motor
				RIGHT_MOTOR.setSpeed(R_RT_RM_ADJUSTMENT); // slow down right motor
				LEFT_MOTOR.forward();
				RIGHT_MOTOR.forward();// EV3 motor hack
			} else { // error within dead band
				// robot to go straight while in deadband
				LEFT_MOTOR.setSpeed(MOTOR_MEDIUM);
				RIGHT_MOTOR.setSpeed(MOTOR_MEDIUM);
				LEFT_MOTOR.forward();
				RIGHT_MOTOR.forward(); // EV3 motor hack
			}

			// control sensor sampling rate
			try {
				Thread.sleep(IDLE_TIME);
			} catch (Exception e) {
				// Poor man's timed sampling
			}

			// change in tachocount, final - initial
			int diffL = LEFT_MOTOR.getTachoCount() - prevLcount;
			int diffR = RIGHT_MOTOR.getTachoCount() - prevRcount;

			// average distance travelled in bangbang
			this.turncount = this.turncount + deg2Distance(WHEEL_RAD, (diffL + diffR) / 2);
		}
	}

	/**
	 * Mirror version of bangbang controller that avoids the obstacle from the left
	 * side. The robot is turned 90 degrees to the left and the ultrasonic sensor is
	 * turned to face right before starting Lbangbang() mode
	 */
	private void Lbangbang() {

		LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
		RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

		// turn left 90 deg
		LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), false);

		usMotor.setstatus(LEFT_BANGBANG); // tell sensor motor: currently in bangbang
		while (!usMotor.ready()) {
			// wait for motor to get ready
		}
		int tally = 0; // a counter used in bangbang for turning around corners

		while (turncount < MAX_AVOID_DISTANCE) {

			LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
			RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

			distance = medianFilter();
			// previous tacho counts
			int prevLcount = LEFT_MOTOR.getTachoCount();
			int prevRcount = RIGHT_MOTOR.getTachoCount();

			double error = distance - BANDCENTER; // deviation from expected band center

			if (Math.abs(error) > (BANDWIDTH) && error > 0) { // robot is too far

				if (Math.abs(error) > (BANDWIDTH) && error > 0 && error < DISTANCE_THRESHOLD) {
					// turn right
					LEFT_MOTOR.setSpeed(MOTOR_HIGH + L_LEFT_ADJUSTMENT); // speed up left motor
					RIGHT_MOTOR.setSpeed(MOTOR_HIGH - L_RIGHT_ADJUSTMENT); // slow down right motor
					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward(); // EV3 motor hack
				} else {
					tally++; // increment counter
					if (tally > MAX_TALLY) {
						// at edge of wall, make right turn
						LEFT_MOTOR.setSpeed(L_RT_LM_ADJUSTMENT); // speed up left motor
						RIGHT_MOTOR.setSpeed(L_RT_RM_ADJUSTMENT); // slow down right motor
						LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();// EV3 motor hack
					} else {
						// robot to go straight slower while counter not satisfied
						LEFT_MOTOR.setSpeed(MOTOR_MEDIUM);
						RIGHT_MOTOR.setSpeed(MOTOR_MEDIUM);
						LEFT_MOTOR.forward(); // EV3 motor hack
						RIGHT_MOTOR.forward();
					}
				}
			} else if (Math.abs(error) > (BANDWIDTH) && error < 0) { // robot is too close
				// turn left
				LEFT_MOTOR.setSpeed(L_LT_LM_ADJUSTMENT); // slow down left motor
				RIGHT_MOTOR.setSpeed(L_LT_RM_ADJUSTMENT); // speed up right motor
				LEFT_MOTOR.forward(); // EV3 motor hack
				RIGHT_MOTOR.forward();
			} else { // error within dead band
				// robot to go straight
				LEFT_MOTOR.setSpeed(MOTOR_MEDIUM);
				RIGHT_MOTOR.setSpeed(MOTOR_MEDIUM);
				RIGHT_MOTOR.forward(); // EV3 motor hack
				LEFT_MOTOR.forward();
			}

			// control sensor sampling rate
			try {
				Thread.sleep(IDLE_TIME);
			} catch (Exception e) {
				// Poor man's timed sampling
			}

			// change in tachocount, final - initial
			int diffL = LEFT_MOTOR.getTachoCount() - prevLcount;
			int diffR = RIGHT_MOTOR.getTachoCount() - prevRcount;

			// average distance travelled in bangbang
			this.turncount = this.turncount + deg2Distance(WHEEL_RAD, (diffL + diffR) / 2);
		}
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

	/**
	 * Helper method. Converts degrees of wheel rotation to distance in cm travelled
	 * 
	 * @param radius    - radius of robot wheel
	 * @param turncount - total number of degrees turned
	 * @return an double indicating the total distance in cm travelled equivalent to
	 *         the degrees turned
	 */
	private static double deg2Distance(double radius, int turncount) {
		return turncount / 360.0 * 2 * Math.PI * radius;
	}

}
