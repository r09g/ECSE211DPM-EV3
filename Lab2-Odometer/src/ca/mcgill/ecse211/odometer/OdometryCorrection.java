
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private static final double TILE = 30.48; // length of 1 tile in cm
	private static final Port csPort = LocalEV3.get().getPort("S1");
	private SensorModes csSensor;
	private SampleProvider cs;
	private float[] csData;

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

		this.csSensor = new EV3ColorSensor(csPort);
		this.cs = csSensor.getMode("Red");
		this.csData = new float[cs.sampleSize()];
		
		// counts black lines in the X and Y direction
		int countX = 0;
		int countY = 0;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// acquire color sensor data
			cs.fetchSample(csData, 0);

			// intensity ranges from 0 ~ 1
			double intensity = csData[0];

			// get current values for x, y, and theta
			double position[] = odometer.getXYT();

			// current X,Y,Theta
			double X = position[0];
			double Y = position[1];
			double Theta = position[2];
			double dX = 0.0;
			double dY = 0.0;

			double difference = 0.0;

			// TODO Trigger correction (When do I have information to correct?)
			// TODO Calculate new (accurate) robot position
			if (intensity <= 0.23) { // black line
				
				Sound.beep();	// signal occurrence
				
				// determine direction
				if (Theta > -10 && Theta < 10) {	

					// moving +Y
					// difference between theoretical distance and displayed distance
					difference = Y - (TILE * countY);
					Y = Y - difference; // correction

					countY++;	// black line
					
				} else if(Theta > 170 && Theta < 190) {
					
					// robot is going in -Y direction
					countY--;
					
					difference = Y - (TILE * countY);
					Y = Y - difference; // correction
					
				} else if(Theta > 80 && Theta < 100) {
					
					// robot going in +X direction
					countX++;
					
					difference = X - (TILE * countX);
					X = X - difference; // correction
					
				} else {
					
					// robot going in -X direction
					countX--;
					
					difference = X - (TILE * countX);
					X = X - difference; // correction
					
				}
				
				
			}

			// TODO Update odometer with new calculated (and more accurate) vales

			odometer.setXYT(dX, dY, Theta);

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
