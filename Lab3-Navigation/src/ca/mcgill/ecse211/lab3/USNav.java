package ca.mcgill.ecse211.lab3;

import java.util.Arrays;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USNav extends Thread {

	// degrees -> radians conversion
	private static final double toRad = Math.PI / 180.0;
	private static final double toDeg = 180.0 / Math.PI;

	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.21;
	public static final double TILE = 30.48;

	private static final int bandCenter = 32; // Offset from the wall (cm)
	private static final int bandwidth = 3; // Width of dead band (cm)
	private static final int motorLow = 175; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 275; // Speed of the faster rotating wheel (deg/sec)

	private static final int FWDSPEED = 250; // forward speed, might need to change later
	private static final int TRNSPEED = 150; // turn speed, migth need to change later

	private static final double AVVDIST = 15; // distance from which robot should stop in front of block

	private EV3LargeRegulatedMotor leftMotor; // left motor
	private EV3LargeRegulatedMotor rightMotor; // right motor
	private Odometer odo; // odometer
	private double position[]; // position data

	private SampleProvider us;
	private float[] usData;
	private UltrasonicMotor usMotor;

	private volatile boolean isNavigating;
	private double distance;
	private double turncount;
	private int numObs; // number of obstacles

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	public USNav(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			SampleProvider us, float[] usData, UltrasonicMotor usMotor) {
		// constructor
		this.leftMotor = leftMotor; // left motor
		this.rightMotor = rightMotor; // right motor
		this.odo = odometer; // odometer
		this.isNavigating = false; // navigating status

		this.us = us;
		this.usData = usData;
		this.usMotor = usMotor; // motor for ultrasonic sensor
		this.turncount = 0;
	}

	// -----------------------------------------------------------------------------
	// Run Method
	// -----------------------------------------------------------------------------

	public void run() {
		// TODO:
		travelTo(2, 2);
		travelTo(0, 0);

	}

	private void travelTo(double x, double y) {
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

		position = odo.getXYT(); // current position

		// position[0] = x, position[1] = y, position[2] = theta
		double dx = x - position[0]; // displacement in x
		double dy = y - position[1]; // displacment in y
		double ds = Math.hypot(dx, dy); // calculates the hypotenuse of dx and dy --> gives the displacement robot will
										// need to travel to get to destination
		double dTheta = Math.atan(dy / dx) * toDeg; // calculates angle dTheta of new displacement
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

		isNavigating = true; // update status

		leftMotor.setSpeed(FWDSPEED);
		rightMotor.setSpeed(FWDSPEED); // sets to forward speed

		leftMotor.rotate(convertDistance(WHEEL_RAD, ds), true); // from square driver, goes straight
		rightMotor.rotate(convertDistance(WHEEL_RAD, ds), true);

		while (leftMotor.isMoving() || rightMotor.isMoving()) {

			// while travelling
			// acquire filtered distance reading
			this.distance = filter();

			// while wall following conditions are met
			if (this.turncount < 60 && numObs < 2 && distance <= AVVDIST) {

				bangbang();
				this.numObs++; // navigated past one obstacle
				turncount = 0;	// reset turncount
				
				usMotor.setstatus(false); // signal motor

				travelTo(x, y); // recalculate
				return;
			}

		}

		isNavigating = false;

	}

	private void turnTo(double Theta) {
		// causes the robot to turn on point to absolute heading theta
		// should turn at minimal angle to target

		leftMotor.setSpeed(TRNSPEED);
		rightMotor.setSpeed(TRNSPEED);

		double minTheta = ((Theta - position[2]) + 360) % 360; // right turn angle

		if (minTheta > 0 && minTheta <= 180) { // already min angle, turn right
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false); // from square driver
		} else if (minTheta > 180 && minTheta < 360) { // will not be minimal angle by turning right
			// turn left
			// opposite from square driver since we turn left
			minTheta = 360 - minTheta; // since we are turning in the opposite direction
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
		}

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
	 * This is a median filter
	 * 
	 * @return
	 */
	private double filter() {

		double[] arr = new double[5];
		for (int i = 0; i < 5; i++) {
			us.fetchSample(usData, 0);
			arr[i] = usData[0] * 100.0; // signal amplification
		}
		Arrays.sort(arr);

		return arr[2]; // median value
	}

	private void bangbang() {

		double[] pos = odo.getXYT();
		double x, y, heading;
		x = pos[0];
		y = pos[1];
		heading = pos[2];

		if (heading > 315 || heading < 45) {
			// up
			if (x < TILE) {
				// right bangbang
			} else {
				// left bangbang
			}
		} else if (heading > 45 && heading < 135) {
			// right
			if (y > TILE) {
				// right bangbang
			} else {
				// left bangbang
			}
		} else if (heading > 135 && heading < 225) {
			// down
			if (x > TILE) {
				// right bangbang
			} else {
				// left bangbang
			}
		} else if (heading > 260 && heading < 280) {
			if (y > TILE) {
				// left bangbang
			} else {
				// right bangbang
			}
		}

	}

	/**
	 * Avoid obstacle from right side
	 */
	private void Rbangbang() {

		while (turncount < 60) {

			distance = filter();
			// previous tacho counts
			int prevLcount = leftMotor.getTachoCount();
			int prevRcount = rightMotor.getTachoCount();

			usMotor.setstatus(true); // tell sensor motor: currently in bangbang

			double error = distance - bandCenter; // deviation from expected band center

			if (Math.abs(error) > (bandwidth) && error > 0) { // robot is too far

				if (Math.abs(error) > (bandwidth) && error > 0 && error < 255) {
					// turn left
					leftMotor.setSpeed(motorHigh - 110); // slow down left motor
					rightMotor.setSpeed(motorHigh + 70); // speed up right motor

					rightMotor.forward(); // EV3 motor hack
					leftMotor.forward();
				} else {
					// at edge of wall, make sharp left turn
					leftMotor.setSpeed(30); // slow down left motor
					rightMotor.setSpeed(motorHigh + 160); // speed up right motor

					rightMotor.forward();// EV3 motor hack
					leftMotor.forward();
				}

			} else if (Math.abs(error) > (bandwidth) && error < 0) { // robot is too close

				// turn right
				leftMotor.setSpeed(motorHigh + 180); // speed up left motor
				rightMotor.setSpeed(10); // slow down right motor

				rightMotor.forward();// EV3 motor hack
				leftMotor.forward();

			} else { // error within dead band

				// robot to go straight
				leftMotor.setSpeed(motorHigh);
				rightMotor.setSpeed(motorHigh);

				rightMotor.forward(); // EV3 motor hack
				leftMotor.forward();

			}

			// control sensor sampling rate
			try {
				Thread.sleep(50);
			} catch (Exception e) {
				// Poor man's timed sampling
			}

			// change in tachocount
			int diffL = leftMotor.getTachoCount() - prevLcount;
			int diffR = rightMotor.getTachoCount() - prevRcount;

			// average degrees turned
			// convert to distance in cm
			this.turncount = this.turncount + deg2Distance(WHEEL_RAD, (diffL + diffR) / 2);
		}
	}

	/**
	 * Avoid obstable from left side
	 */
	private void Lbangbang() {
	
		while (turncount < 60) {

			distance = filter();
			// previous tacho counts
			int prevLcount = leftMotor.getTachoCount();
			int prevRcount = rightMotor.getTachoCount();

			usMotor.setstatus(true); // tell sensor motor: currently in bangbang

			double error = distance - bandCenter; // deviation from expected band center

			if (Math.abs(error) > (bandwidth) && error > 0) { // robot is too far

				if (Math.abs(error) > (bandwidth) && error > 0 && error < 255) {
					// turn right
					rightMotor.setSpeed(motorHigh - 110); // slow down right motor
					leftMotor.setSpeed(motorHigh + 70); // speed up left motor

					rightMotor.forward(); // EV3 motor hack
					leftMotor.forward();
				} else {
					// at edge of wall, make sharp left turn
					rightMotor.setSpeed(30); // slow down right motor
					leftMotor.setSpeed(motorHigh + 160); // speed up left motor

					rightMotor.forward();// EV3 motor hack
					leftMotor.forward();
				}

			} else if (Math.abs(error) > (bandwidth) && error < 0) { // robot is too close

				// turn left
				rightMotor.setSpeed(motorHigh + 180); // speed up right motor
				leftMotor.setSpeed(10); // slow down left motor

				rightMotor.forward();// EV3 motor hack
				leftMotor.forward();

			} else { // error within dead band

				// robot to go straight
				leftMotor.setSpeed(motorHigh);
				rightMotor.setSpeed(motorHigh);

				rightMotor.forward(); // EV3 motor hack
				leftMotor.forward();

			}

			// control sensor sampling rate
			try {
				Thread.sleep(50);
			} catch (Exception e) {
				// Poor man's timed sampling
			}

			// change in tachocount
			int diffL = leftMotor.getTachoCount() - prevLcount;
			int diffR = rightMotor.getTachoCount() - prevRcount;

			// average degrees turned
			// convert to distance in cm
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
	 * Converts degrees of wheel rotation in to cm travelled
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
