package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import static ca.mcgill.ecse211.lab3.Lab3.SENSOR_MOTOR;

public class UltrasonicMotor extends Thread {

	private static final int USSPEED = 300; //speed of sensor motor, might need to change
	private static final int SWITCH_ANGLE = 30; //angle at which motor stops rotating in one direction and starts rotating
											//in the other
	private static final int WALL_FOLLOWING_ANGLE = 75;	// left side wallfollowing angle
	private static final int ORIGIN = 0;	// original position
	
	private int wallfollowing;	// wallfollowing status
	private boolean ready4turn;
	
	public UltrasonicMotor() {
		this.wallfollowing = 0;
	}
	
	public void run() {

		SENSOR_MOTOR.setAcceleration(500);

		while (true) {
			
			ready4turn = false;
			
			SENSOR_MOTOR.rotateTo(-SWITCH_ANGLE, false);
			SENSOR_MOTOR.rotateTo(ORIGIN, false);
			SENSOR_MOTOR.rotateTo(SWITCH_ANGLE, false);
			
			if(wallfollowing == 2) {
				// currently obstacle avoiding
				// turn sensor RIGHT to 45 deg to wall
				SENSOR_MOTOR.setAcceleration(500);
				SENSOR_MOTOR.rotateTo(-WALL_FOLLOWING_ANGLE, false);
				SENSOR_MOTOR.stop(false);

				ready4turn = true;
				
				while(wallfollowing != 0) {
					/* wait until wall following ends */
				};
			} else if(wallfollowing == 1) {
				// currently obstacle avoiding
				// turn sensor LEFT to 45 deg to wall
				SENSOR_MOTOR.setAcceleration(500);
				SENSOR_MOTOR.rotateTo(WALL_FOLLOWING_ANGLE, false);
				SENSOR_MOTOR.stop(false);

				ready4turn = true;
				
				while(wallfollowing != 0) {
					/* wait until wall following ends */
				};
			}
			
			
		}

	}
	
	/**
	 * setter
	 * 
	 * 0 = no bangbang; 1 = left bangbang; 2 = right bangbang
	 * @param type - left or right bangbang
	 */
	public void setstatus(int type) {
		this.wallfollowing = type;
	}
	
	public boolean ready() {
		return this.ready4turn;
	}
	
}
