
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;
import java.util.LinkedList;;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	private static final double TILE = 30.48; // length of 1 tile in cm

	private static final Port csPort = LocalEV3.get().getPort("S1");
	private SensorModes csSensor;
	private SampleProvider cs;
	private float[] csData;

	private double filterSum;
	private double STD;	// initial reading

	private double X;
	private double Y;
	private double Theta;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

		// initialization
		this.csSensor = new EV3ColorSensor(csPort);
		this.cs = csSensor.getMode("Red");
		this.csData = new float[cs.sampleSize()];
		this.filterSum = 0;
		this.STD = 0;

		Sound.setVolume(50);

	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		// counts black lines in the X and Y direction
		int countX = 0;
		int countY = 0;

		for(int i = 0; i < 100; i++) {
			cs.fetchSample(csData, 0);
			STD += csData[0] * 100;	// signal amplification
		}
		
		STD /= 100.0;
		
		while (true) {
			correctionStart = System.currentTimeMillis();

			// get current values for x, y, and theta
			double position[] = odometer.getXYT();

			double difference = 0.0;

			// TODO Trigger correction (When do I have information to correct?)
			// TODO Calculate new (accurate) robot position
			
			double intensity = meanFilter();
			
			if ( (intensity / STD) < 0.75) { // black line

				// current X,Y,Theta
				X = position[0];
				Y = position[1];
				Theta = position[2];

				Sound.beep(); // signal occurrence

				// determine direction
				if ((Theta > 350 && Theta < 360) || (Theta > 0 && Theta < 10)) {

					// moving +Y
					// difference between theoretical distance and displayed distance
					difference = Y - (TILE * countY);
					Y = Y - difference - 1.75; // correction
					
					countY++; // black line

					odometer.setY(Y);

				} else if (Theta > 170 && Theta < 190) {

					// robot is going in -Y direction
					countY--;

					difference = Y - (TILE * countY);
					Y = Y - difference + 1.75; // correction

					odometer.setY(Y);

				} else if (Theta > 80 && Theta < 100) {

					// robot going in +X direction
					difference = X - (TILE * countX);
					X = X - difference - 1.75; // correction

					countX++;

					odometer.setX(X);

				} else if (Theta > 260 && Theta < 280) {

					// robot going in -X direction
					countX--;

					difference = X - (TILE * countX);
					X = X - difference + 1.75; // correction

					odometer.setX(X);

				}
				
			}
			
			// clear loop values
			intensity = 0;
			filterSum = 0;

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

	private double meanFilter() {
		// for the first 5 readings
		for(int i = 0; i < 5; i++) {
			cs.fetchSample(csData, 0);
			filterSum += csData[0] * 100;	// amplify signal
		}
		
		return filterSum / 5.0;
	}

}
