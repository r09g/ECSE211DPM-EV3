package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class UltrasonicMotor extends Thread {

	private EV3MediumRegulatedMotor sensorMotor;
	private static final int USSPEED = 300; //speed of sensor motor, might need to change
	private static final int SWITCH_ANGLE = 45; //angle at which motor stops rotating in one direction and starts rotating
											//in the other
	private static final int WALL_FOLLOWING_ANGLE = 75;	// left side wallfollowing angle
	private static final int ORIGIN = 0;	// original position
	
	private int wallfollowing;	// wallfollowing status
	private boolean ready4turn;
	
	public UltrasonicMotor(EV3MediumRegulatedMotor sensorMotor) {
		this.sensorMotor = sensorMotor;
		this.wallfollowing = 0;
	}
	
	public void run() {

		sensorMotor.setAcceleration(500);

		while (true) {
			
			ready4turn = false;
			
			sensorMotor.rotateTo(-SWITCH_ANGLE, false);
			sensorMotor.rotateTo(ORIGIN, false);
			sensorMotor.rotateTo(SWITCH_ANGLE, false);
			
			if(wallfollowing == 2) {
				// currently obstacle avoiding
				// turn sensor RIGHT to 45 deg to wall
				sensorMotor.setAcceleration(500);
				sensorMotor.rotateTo(-WALL_FOLLOWING_ANGLE, false);
				sensorMotor.stop(false);

				ready4turn = true;
				
				while(wallfollowing != 0) {
					/* wait until wall following ends */
				};
			} else if(wallfollowing == 1) {
				// currently obstacle avoiding
				// turn sensor LEFT to 45 deg to wall
				sensorMotor.setAcceleration(500);
				sensorMotor.rotateTo(WALL_FOLLOWING_ANGLE, false);
				sensorMotor.stop(false);

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
