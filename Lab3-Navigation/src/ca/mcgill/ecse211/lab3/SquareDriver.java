/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;

/**
 * This class is used to drive the robot on the demo floor in a square fashion.
 * This class specifies the constants that control the behaviour of the robot
 * along the square. The class also methods that drive the robot and helper
 * methods that performs unit conversions.
 */
public class SquareDriver {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	// this is the speed at which the robot moves forward (in rpm)
	private static final int FORWARD_SPEED = 250;

	// the rotating speed of the robot (in rpm)
	private static final int ROTATE_SPEED = 150;

	// the size of each tile on the demo floor (in cm0
	private static final double TILE_SIZE = 30.48;

	// -----------------------------------------------------------------------------
	// Public method
	// -----------------------------------------------------------------------------
	
	/**
	 * This method is meant to drive the robot in a square of size 3x3 Tiles on the
	 * lab wooden demo floor. It is to run in parallel with the odometer and
	 * Odometer correcton classes allow testing their functionality. The method
	 * controls the path of the robot by specifying the total degrees the left and
	 * right motors need to turn, thus achieving the purpose of going a fixed
	 * distance or turning a fixed angle. The motor's acceleration as well as the
	 * EV3 brick's signalling sound can be changed here.
	 * 
	 * @param leftMotor   - passes control of the left motor into this method
	 * @param rightMotor  - passes control of the right motor into this method
	 * @param leftRadius  - the radius of the left wheel in cm
	 * @param rightRadius - the radius of the right wheel in cm
	 * @param track       - the width of the robot in terms of the center of its
	 *                    left and right wheels
	 */
	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double track) {

		// stop both motors and set both their accelerations to 3000
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {

			motor.stop(); // stops the motors
			motor.setAcceleration(3000); // sets the acceleration

		}

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}

		// loops four times for four sides of the squares
		for (int i = 0; i < 4; i++) {

			// set driving speed for both motors
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			// drive forward for 3 (instead of 2.92) tiles. The 2.92 is to account for the
			// size of our
			// robot. The robot could potentially fall off the demo surface.
			// immediately pass control out and to next motor to ensure synchronization
			leftMotor.rotate(convertDistance(leftRadius, 2.92 * TILE_SIZE), true);
			rightMotor.rotate(convertDistance(rightRadius, 2.92 * TILE_SIZE), false);

			// set rotating speed
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			// rotate( angle in degrees, return immediately or not )
			// so both motors turn at same time
			// turn 90 degrees clockwise
			// immediately pass control out and to next motor to ensure synchronization
			leftMotor.rotate(convertAngle(leftRadius, track, 90.0), true);
			rightMotor.rotate(-convertAngle(rightRadius, track, 90.0), false);

		}

		// set the beeping volume of the EV3 brick
		Sound.setVolume(100);

	}

	// -----------------------------------------------------------------------------
	// Private Methods
	// -----------------------------------------------------------------------------
	
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
