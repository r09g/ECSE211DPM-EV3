package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class UltrasonicMotor extends Thread {

	private EV3MediumRegulatedMotor sensorMotor;
	private static final int USSPEED = 100; //speed of sensor motor, might need to change
	private static final int SWITCHANGLE = 30; //angle at which motor stops rotating in one direction and starts rotating
											//in the other
	private static final int WALL_FOLLOWING_ANGLE = 45;	// left side wallfollowing angle
	private static final int ORIGIN = 0;	// original position
	
	
	private boolean wallfollowing;	// wallfollowing status
	
	public UltrasonicMotor(EV3MediumRegulatedMotor sensorMotor) {
		this.sensorMotor = sensorMotor;
		this.wallfollowing = false;
	}
	
	public void run() {
		sensorMotor.forward();
		sensorMotor.setSpeed(USSPEED); //motor starts going forward
		
		while (true) {
			
			sensorMotor.rotateTo(-SWITCHANGLE, false);
			sensorMotor.rotateTo(ORIGIN, false);
			sensorMotor.rotateTo(SWITCHANGLE, false);
			
			if(wallfollowing) {
				// currently obstacle avoiding
				// turn sensor to 45 deg to wall
				sensorMotor.setAcceleration(3000);
				sensorMotor.rotateTo(-WALL_FOLLOWING_ANGLE, false);
				while(wallfollowing) {/* wait until wall following ends */};
			}
		}

	}
	
	/**
	 * setter
	 * @param wallfollowing
	 */
	public void setstatus(boolean wallfollowing) {
		this.wallfollowing = wallfollowing;
	}
	
}
