package ca.mcgill.ecse211.lab3;

// non-static imports
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

// static imports from Lab3 class
import static ca.mcgill.ecse211.lab3.Lab3.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.SENSOR_MOTOR;
import static ca.mcgill.ecse211.lab3.Lab3.TILE;
import static ca.mcgill.ecse211.lab3.Lab3.WHEEL_RAD;
import static ca.mcgill.ecse211.lab3.Lab3.FWDSPEED;
import static ca.mcgill.ecse211.lab3.Lab3.TRNSPEED;
import static ca.mcgill.ecse211.lab3.Lab3.TO_RAD;
import static ca.mcgill.ecse211.lab3.Lab3.TO_DEG;
import static ca.mcgill.ecse211.lab3.Lab3.PATH;

public class USNav extends Thread {

	// degrees -> radians conversion
	private static final double TRACK = 13.21;

	private static final int bandCenter = 12; // Offset from the wall (cm)
	private static final int bandwidth = 3; // Width of dead band (cm)
	private static final int motorLow = 175; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 275; // Speed of the faster rotating wheel (deg/sec)

	private static final double AVVDIST = 15; // distance from which robot should stop in front of block

	private Odometer odo; // odometer
	private double position[]; // position data

	private SampleProvider us;
	private float[] usData;
	private UltrasonicMotor usMotor;

	private volatile boolean isNavigating;
	private double distance;
	private double turncount;
	private boolean completed;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	public USNav(EV3LargeRegulatedMotor LEFT_MOTOR, EV3LargeRegulatedMotor RIGHT_MOTOR, Odometer odometer,
			SampleProvider us, float[] usData, UltrasonicMotor usMotor) {
		// constructor
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
		
		for (int[] inner : PATH) {
			
			this.completed = false;
			
			int x = inner[0];
			int y = inner[1];
			
			travelTo(x, y);
			while (this.completed == false) {
				travelTo(x, y);
			}

		}

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

		isNavigating = true; // update status

		LEFT_MOTOR.setAcceleration(500);
		RIGHT_MOTOR.setAcceleration(500);

		LEFT_MOTOR.setSpeed(FWDSPEED);
		RIGHT_MOTOR.setSpeed(FWDSPEED); // sets to forward speed

		LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true); // from square driver, goes straight
		RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true);

		while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {

			// while travelling
			// acquire filtered distance reading
			this.distance = filter();

			// while wall following conditions are met
			if (distance <= AVVDIST) {

				LEFT_MOTOR.stop(true);
				RIGHT_MOTOR.stop(false);

				bangbang(); // activate bangbang

				LEFT_MOTOR.stop(true);
				RIGHT_MOTOR.stop(false);
				
				LEFT_MOTOR.setAcceleration(500);
				RIGHT_MOTOR.setAcceleration(500);

				turncount = 0; // reset turncount

				usMotor.setstatus(0); // signal motor

				this.completed = false;
				
				return;
			}

		}

		isNavigating = false; // set status
		
		this.completed = true;

		return;
	}

	private void turnTo(double Theta) {
		// causes the robot to turn on point to absolute heading theta
		// should turn at minimal angle to target

		this.isNavigating = true;
		
		LEFT_MOTOR.setAcceleration(500);
		RIGHT_MOTOR.setAcceleration(500);

		LEFT_MOTOR.setSpeed(TRNSPEED);
		RIGHT_MOTOR.setSpeed(TRNSPEED);

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
		
		this.isNavigating = false;

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
				Rbangbang(); // right bangbang
			} else {
				Lbangbang(); // left bangbang
			}
		} else if (heading > 45 && heading < 135) {
			// right
			if (y > TILE) {
				Rbangbang(); // right bangbang
			} else {
				Lbangbang(); // left bangbang
			}
		} else if (heading > 135 && heading < 225) {
			// down
			if (x > TILE) {
				Rbangbang(); // right bangbang
			} else {
				Lbangbang(); // left bangbang
			}
		} else if (heading > 260 && heading < 280) {
			if (y > TILE) {
				Lbangbang(); // left bangbang
			} else {
				Rbangbang(); // right bangbang
			}
		}

	}

	/**
	 * Avoid obstacle from right side
	 */
	private void Rbangbang() {

		LEFT_MOTOR.setAcceleration(500);
		RIGHT_MOTOR.setAcceleration(500);

		// turn right 90 deg
		LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), false);

		usMotor.setstatus(1); // tell sensor motor: currently in bangbang

		while (!usMotor.ready()) {
			// wait for motor to get ready
		}

		int tally = 0;

		while (turncount < 50) {

			LEFT_MOTOR.setAcceleration(500);
			RIGHT_MOTOR.setAcceleration(500);

			distance = filter();
			// previous tacho counts
			int prevLcount = LEFT_MOTOR.getTachoCount();
			int prevRcount = RIGHT_MOTOR.getTachoCount();

			double error = distance - bandCenter; // deviation from expected band center

			if (Math.abs(error) > (bandwidth) && error > 0) { // robot is too far

				if (Math.abs(error) > (bandwidth) && error > 0 && error < 160) {
					// turn left
					LEFT_MOTOR.setSpeed(motorHigh - 90); // slow down left motor
					RIGHT_MOTOR.setSpeed(motorHigh + 70); // speed up right motor

					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward(); // EV3 motor hack

				} else {

					tally++;

					// robot to go straight
					LEFT_MOTOR.setSpeed(motorHigh / 2);
					RIGHT_MOTOR.setSpeed(motorHigh / 2);

					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward(); // EV3 motor hack

					if (tally > 30) {

						// at edge of wall, make sharp left turn
						LEFT_MOTOR.setSpeed(150); // slow down left motor
						RIGHT_MOTOR.setSpeed(motorHigh + 100); // speed up right motor

						LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();// EV3 motor hack

					}
				}

			} else if (Math.abs(error) > (bandwidth) && error < 0) { // robot is too close

				// turn right
				LEFT_MOTOR.setSpeed(motorHigh + 120); // speed up left motor
				RIGHT_MOTOR.setSpeed(10); // slow down right motor

				LEFT_MOTOR.forward();
				RIGHT_MOTOR.forward();// EV3 motor hack

			} else { // error within dead band

				// robot to go straight
				LEFT_MOTOR.setSpeed(motorHigh / 2);
				RIGHT_MOTOR.setSpeed(motorHigh / 2);

				LEFT_MOTOR.forward();
				RIGHT_MOTOR.forward(); // EV3 motor hack

			}

			// control sensor sampling rate
			try {
				Thread.sleep(50);
			} catch (Exception e) {
				// Poor man's timed sampling
			}

			// change in tachocount
			int diffL = LEFT_MOTOR.getTachoCount() - prevLcount;
			int diffR = RIGHT_MOTOR.getTachoCount() - prevRcount;

			// average degrees turned
			// convert to distance in cm
			this.turncount = this.turncount + deg2Distance(WHEEL_RAD, (diffL + diffR) / 2);
		}
	}

	/**
	 * Avoid obstable from left side
	 */
	private void Lbangbang() {

		LEFT_MOTOR.setAcceleration(500);
		RIGHT_MOTOR.setAcceleration(500);

		// turn left 90 deg
		LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, 90.0), true);
		RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, 90.0), false);

		usMotor.setstatus(2); // tell sensor motor: currently in bangbang

		while (!usMotor.ready()) {
			// wait for motor to get ready
		}

		int tally = 0;

		while (turncount < 60) {

			LEFT_MOTOR.setAcceleration(500);
			RIGHT_MOTOR.setAcceleration(500);

			distance = filter();
			// previous tacho counts
			int prevLcount = LEFT_MOTOR.getTachoCount();
			int prevRcount = RIGHT_MOTOR.getTachoCount();

			double error = distance - bandCenter; // deviation from expected band center

			if (Math.abs(error) > (bandwidth) && error > 0) { // robot is too far

				if (Math.abs(error) > (bandwidth) && error > 0 && error < 160) {
					// turn right
					LEFT_MOTOR.setSpeed(motorHigh + 70); // speed up left motor
					RIGHT_MOTOR.setSpeed(motorHigh - 110); // slow down right motor

					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward(); // EV3 motor hack
				} else {
					tally++;

					// robot to go straight
					LEFT_MOTOR.setSpeed(motorHigh / 2);
					RIGHT_MOTOR.setSpeed(motorHigh / 2);

					LEFT_MOTOR.forward();
					RIGHT_MOTOR.forward(); // EV3 motor hack

					if (tally > 30) {
						// at edge of wall, make sharp right turn
						LEFT_MOTOR.setSpeed(motorHigh + 100); // speed up left motor
						RIGHT_MOTOR.setSpeed(150); // slow down right motor

						LEFT_MOTOR.forward();
						RIGHT_MOTOR.forward();// EV3 motor hack
					}
				}

			} else if (Math.abs(error) > (bandwidth) && error < 0) { // robot is too close

				// turn left
				LEFT_MOTOR.setSpeed(10); // slow down left motor
				RIGHT_MOTOR.setSpeed(motorHigh + 180); // speed up right motor

				LEFT_MOTOR.forward();
				RIGHT_MOTOR.forward();// EV3 motor hack

			} else { // error within dead band

				// robot to go straight
				LEFT_MOTOR.setSpeed(motorHigh / 2);
				RIGHT_MOTOR.setSpeed(motorHigh / 2);

				RIGHT_MOTOR.forward(); // EV3 motor hack
				LEFT_MOTOR.forward();

			}

			// control sensor sampling rate
			try {
				Thread.sleep(50);
			} catch (Exception e) {
				// Poor man's timed sampling
			}

			// change in tachocount
			int diffL = LEFT_MOTOR.getTachoCount() - prevLcount;
			int diffR = RIGHT_MOTOR.getTachoCount() - prevRcount;

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
