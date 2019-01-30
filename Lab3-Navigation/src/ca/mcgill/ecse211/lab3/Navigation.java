package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

import java.math.*;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.lab3.SquareDriver;

public class Navigation {
	
	// degrees -> radians conversion
		private static final double toRad = Math.PI / 180.0;

		// radians -> degrees conversion
		private static final double toDeg = 180.0 / Math.PI;
		
		private static final int FWDSPEED = 200; //forward speed, might need to change later
		private static final int TRNSPEED = 100; // turn speed, migth need to change later
		
		private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); // left
		// motor
private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D")); // right
		// motor
		double position[] = null;
		
		// wheel radius of robot
		// this value reflects the actual value of the wheel radius
		public static final double WHEEL_RAD = 2.1;

		// distance between center of left and right wheels
		// this value is tweaked to optimize the behaviours of the robot in different
		// operation modes
		public static final double TRACK = 13.21;

		
	public Navigation() {
		//constructor
		try {
			position = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	void travelTo(double x, double y) {
		/*this method causes the robot to travel to the absolute field
		location (x, y) specified in the tile points. This method should continuously
		call turnTO(double theta) and then set the motor speed to forward(straight).
		this will make sure ur heading is updated until u reach ur exact goal. this method will
		poll odometer for info*/
		double position[] = null;
		try {
			position = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		//current position
		//position[0] = x, position[1] = y, position[2] = theta
		double dx = x - position[0]; //displacement in x
		double dy = y - position[1]; // displacment in y
		double ds = Math.hypot(dx, dy); //calculates the hypotenuse of dx and dy --> gives the displacement robot will need to travel to get to destination
		double dTheta = Math.atan(dy/dx)* toDeg; //calculates angle dTheta of new displacement
		//will be in the range of [-90,90] degrees
		if (dTheta >=0 && dx >= 0) { //first quadrant
			dTheta = 90-dTheta; //our convention being north = 0degrees + increase clockwise, this new angle is the absolute angle
		}
		else if (dTheta >=0 && dx < 0) { //3rd quadrant
			dTheta = 270 - dTheta; //absolute angle
		}
		else if (dTheta < 0 && dx >= 0) { //4th quadrant
			dTheta = 90 + dTheta; //absolute angle
		}
		else if (dTheta < 0 && dx < 0) { //2nd quadrant
			dTheta = 270 + dTheta; //absolute angle
		}
		turnTo(dTheta);
		
	}
	
	void turnTo(double Theta) {
		//causes the robot to turn on point to absolute heading theta
		//should turn at minimal angle to target
		leftMotor.setSpeed(TRNSPEED);
		rightMotor.setSpeed(TRNSPEED);
		
		double minTheta = ((Theta - position[2]) + 360)%360; //right turn angle
		
		if (minTheta > 0 && minTheta <=180) { //already min angle, turn right
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false); //from square driver
		}
		else if (minTheta > 0 && minTheta > 180) { //will not be minimal angle by turning right
			//turn left
			minTheta = minTheta - 180; //since we are turning in the opposite direction
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
		rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true); //opposite from square driver since we turn left
			
		}
		
	//	leftMotor.rotate(SquareDriver.convertAngle(leftRadius, track, 90.0), true);
	//	rightMotor.rotate(-SquareDriver.convertAngle(rightRadius, track, 90.0), false);
		
	}
	
	boolean isNavigating() {
		//returns true if another thread has called travelTo() or turnTo() and the method has yet to return
		//false otherwise
		return false;
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
