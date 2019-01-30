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
		
	public Navigation() {
		//constructor
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
		
		leftMotor.rotate(SquareDriver.convertAngle(leftRadius, track, 90.0), true);
		rightMotor.rotate(-SquareDriver.convertAngle(rightRadius, track, 90.0), false);
		
	}
	
	boolean isNavigating() {
		//returns true if another thread has called travelTo() or turnTo() and the method has yet to return
		//false otherwise
		return false;
	}
}
