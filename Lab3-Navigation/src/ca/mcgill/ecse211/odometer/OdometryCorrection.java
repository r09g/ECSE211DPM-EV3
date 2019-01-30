
package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * This class specifies the algorithm for correction the odometry mechanism on
 * the EV3 robot. This class contains class variables that provide control to
 * the motors, LCD display, and sensor. This class also has constants that
 * specifies the behaviour of the sensor and the robot movement.
 * 
 * @author Raymond Yang
 * @author Erica De Petrillo
 */

public class OdometryCorrection implements Runnable {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	// this sets the correction frequency, controlling each correction loop at 10 ms
	private static final long CORRECTION_PERIOD = 10;

	// this is the length of 1 tile in cm
	private static final double TILE = 30.48;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	// odometer object, this provides control of the odometer
	private Odometer odometer;

	// specifies the port of the color sensor
	//private static final Port csPort = LocalEV3.get().getPort("S1");

	// provides options to choose mode of sensor
	private SensorModes csSensor;

	// provides control to allow acquiring of color sensor data
	private SampleProvider cs;

	// a float array to store retrieved data from sensor
	private float[] csData;

	// variable used in the mean filter, stores the sum of 5 readings
	private double filterSum;

	// stores the initial reading taken at the start of trial
	private double STD;

	// stores the position data
	private double X;
	private double Y;
	private double Theta;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety. The sensor is initialized and a
	 * mode is selected. A float array is created to store data. Variables used in
	 * the filter are initialized.
	 * 
	 * @throws OdometerExceptions - in the case where odometer fails to obtain
	 *                            control of the odometer
	 */
	public OdometryCorrection() throws OdometerExceptions {

		// passes control of odometer
		this.odometer = Odometer.getOdometer();

		// initialization of the color sensor
	/*	this.csSensor = new EV3ColorSensor(csPort);

		// specifies the mode of operation of the sensor
		this.cs = csSensor.getMode("Red");

		// the array used to store the data
		this.csData = new float[cs.sampleSize()];

		// initializes the variables used in the mean filter
		this.filterSum = 0;
		this.STD = 0;*/

		// set the beeping volume of the robot
		Sound.setVolume(50);

	}

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * The implementation for the odometry correction class. The odometer is
	 * corrected when passing black lines on the demo board. A count is stored and
	 * the odometer values are corrected based on the count. A filter is also
	 * implemented in this method, initial readings are taken as the robot starts to
	 * move, and are used as a standard to decide whether the robot passes a black
	 * line.
	 * 
	 * @throws OdometerExceptions
	 */

	public void run() {

		// stores the start time and end time
		long correctionStart, correctionEnd;

		// counts the number of black lines in the X and Y direction
		int countX = 0;
		int countY = 0;

		// initial readings taken (100 times) and the average is used to distinguish
		// between the wooden board and the black line. Each reading is amplified to
		// enhance the sensitivity of the sensor
		for (int i = 0; i < 100; i++) {

			// acquires sample data
			cs.fetchSample(csData, 0);

			// sums the sample data
			// signal amplification
			STD += csData[0] * 100;
		}

		// take average of the standard
		STD /= 100.0;

		// correction loop
		while (true) {
			
			// records start time
			correctionStart = System.currentTimeMillis();

			// get current values for x, y, and theta
			double position[] = odometer.getXYT();

			// the actual correction applied to the X and Y values
			double difference = 0.0;
			
			// activates filter and takes the average
			double intensity = meanFilter();

			if ((intensity / STD) < 0.75) { // black line

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
					Y = Y - difference; // correction

					countY++; // black line

					odometer.setY(Y);

				} else if (Theta > 170 && Theta < 190) {

					// robot is going in -Y direction
					countY--;

					difference = Y - (TILE * countY);
					Y = Y - difference; // correction

					odometer.setY(Y);

				} else if (Theta > 80 && Theta < 100) {

					// robot going in +X direction
					difference = X - (TILE * countX);
					X = X - difference; // correction

					countX++;

					odometer.setX(X);

				} else if (Theta > 260 && Theta < 280) {

					// robot going in -X direction
					countX--;

					difference = X - (TILE * countX);
					X = X - difference; // correction

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
		for (int i = 0; i < 5; i++) {
			cs.fetchSample(csData, 0);
			filterSum += csData[0] * 100; // amplify signal
		}

		return filterSum / 5.0;
	}

}
