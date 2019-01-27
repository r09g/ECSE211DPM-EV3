/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private static final double TILE = 30.48; // length of 1 tile in cm

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		Port csPort = LocalEV3.get().getPort("S1");
		SensorModes csSensor = new EV3UltrasonicSensor(csPort); // usSensor is the instance
		SampleProvider csColor = csSensor.getMode("RGB"); // csColor provides samples from
		// this instance
		float[] csData = new float[csColor.sampleSize()]; // csData is the buffer in which data are
		// returned

		double dx = 0; // used to store correction in x axis
		double dy = 0; // used to store correction in x axis
		double dt = 0; // used to store correction in orientation (not sure we need that)
		int count = 0; // keeps count of gridline number in that portion of the rectangle
		double prevTheta = 0; // keeps track of previous Theta
		double difference = 0; // difference between gridline and calculated position

		while (true) {
			correctionStart = System.currentTimeMillis();

			double position[] = odometer.getXYT(); // get current values for x, y, and theta

			// TODO Trigger correction (When do I have information to correct?)
			// TODO Calculate new (accurate) robot position
			if (csData[0] * 1000 < 300) { // 1 refers to BLACK according to LeJOS API
				if (prevTheta == position[2]) { // if robot still in the same line
					count++;
					prevTheta = position[2];
				} else { // if the robot turned
					count = 1;
					prevTheta = position[2];
				}
				
				if ((position[2] > 350 && position[2] < 10) || (position[2] > 170 && position[2] < 190)) { // robot moving horizontally
					// adjust x coordinate
					difference = Math.abs((position[0] - TILE * count));
					if (position[0] < TILE * count) {
						dx = difference; // we will need to add the difference to the calculated position
					} else {
						dx = 0 - difference; // we will need to subtract the difference to the calculated position
					}
				} else if ((position[2] > 80 && position[2] < 100) || (position[2] > 260 && position[2] < 280)) { // robot moving vertically
					// adjust y coordinate
					difference = Math.abs((position[1] - TILE * count));
					if (position[1] < TILE * count) {
						dx = difference; // we will need to add the difference to the calculated position
					} else {
						dx = 0 - difference; // we will need to subtract the difference to the calculated position
					}
				}
			}

			// TODO Update odometer with new calculated (and more accurate) vales

			odometer.setXYT(dx, dy, dt);

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
