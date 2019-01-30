package ca.mcgill.ecse211.odometer;
import ca.mcgill.ecse211.odometer.Odometer;
import java.math.*;

public class Navigation {
	
	// degrees -> radians conversion
		private static final double toRad = Math.PI / 180.0;

		// radians -> degrees conversion
		private static final double toDeg = 180.0 / Math.PI;
		
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
		} //current position
		//position[0] = x, position[1] = y, position[2] = theta
		double dx = x - position[0]; //displacement in x
		double dy = y - position[1]; // displacment in y
		double ds = Math.hypot(dx, dy); //calculates the hypotenuse of dx and dy --> gives the displacement robot will need to travel to get to destination
		double dTheta;
		//double dTheta = Math.tan(dy/dx)* toDeg; //calculates angle dTheta of new displacement
		
		//calculate angle of turn
		// determine direction
		if ((position[2] > 350 && position[2] < 360) || (position[2] > 0 && position[2] < 10)) { // moving +Y
			dTheta = Math.atan(dx/dy) * toDeg;
			if (dx >= 0) { //new destination on right of robot
				
				if (dTheta >= 0) {
					//TODO turn right
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn right
				}
			}
			else {
				if (dTheta >= 0) {
					//TODO turn left
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn left
				}
			}

		} else if (position[2] > 170 && position[2] < 190) { //robot is going in -Y direction
			dTheta = Math.atan(dx/dy) * toDeg;
			if (dx >= 0) { //new destination on right of robot
				if (dTheta >= 0) {
					//TODO turn left
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn left
				}
			}
			else {
				if (dTheta >= 0) {
					//TODO turn right
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn right
				}
			}

		} else if (position[2] > 80 && position[2] < 100) {	// robot going in +X direction
			dTheta = Math.atan(dy/dx) * toDeg;
			if (dy >= 0) { //new destination on right of robot
				
				if (dTheta >= 0) {
					//TODO turn right
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn right
				}
			}
			else {
				if (dTheta >= 0) {
					//TODO turn left
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn left
				}
			}
			

		} else if (position[2] > 260 && position[2] < 280) { // robot going in -X direction
			dTheta = Math.atan(dy/dx) * toDeg;
			if (dy >= 0) { //new destination on left of robot
				
				if (dTheta >= 0) {
					//TODO turn left
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn left
				}
			}
			else {
				if (dTheta >= 0) {
					//TODO turn right
				}
				else {
					dTheta = 180 - Math.abs(dTheta); //minimal angle
					//TODO turn right
				}
			}

		}
	}
	
	void turnTo(double Theta) {
		//causes the robot to turn on point to absolute heading theta
		//should turn at minimal angle to target
	}
	
	boolean isNavigating() {
		//returns true if another thread has called travelTo() or turnTo() and the method has yet to return
		//false otherwise
		return false;
	}
}
