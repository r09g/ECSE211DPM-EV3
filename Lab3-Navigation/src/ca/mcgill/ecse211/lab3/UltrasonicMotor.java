package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class UltrasonicMotor extends Thread {

	private EV3MediumRegulatedMotor sensorMotor;
	private static final int USSPEED = 100; //speed of sensor motor, might need to change
	private static final int SWITCHANG = 30; //angle at which motor stops rotating in one direction and starts rotating
											//in the other
	private boolean isForward; //to keep track of direction of motor
	
	public UltrasonicMotor(EV3MediumRegulatedMotor sensorMotor) {
		this.sensorMotor = sensorMotor;
	}
	
	public void run() {
		sensorMotor.forward();
		sensorMotor.setSpeed(USSPEED); //motor starts going forward
		isForward = true; //to keep track of direction
		
		while (true) {
			if (sensorMotor.getLimitAngle() <= SWITCHANG) { //if motor done sweeping in 1 direction
				sensorMotor.stop(); //stop motor
				if (isForward) { //motor going forward --> need to switch to backwards
					sensorMotor.backward();
					sensorMotor.setSpeed(USSPEED);
					isForward = false; //keep track of direction
				}
				else { //motor going backward --> need to switch to forward
					sensorMotor.forward();
					sensorMotor.setSpeed(USSPEED);
					isForward = true; //keep track of direction
				}		
				
			}			
		}

	}
	
}
