
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

	// the intensity to standard intensity ratio threshold
	// used for determining whether robot passed a black line
	private static final double THRESHOLD = 0.75;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	// odometer object, this provides control of the odometer
	private Odometer odometer;

	// specifies the port of the color sensor
	private static final Port csPort = LocalEV3.get().getPort("S1");

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

	// stores the position data X,Y,Theta
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
		this.csSensor = new EV3ColorSensor(csPort);

		// specifies the mode of operation of the sensor
		this.cs = csSensor.getMode("Red");

		// the array used to store the data
		this.csData = new float[cs.sampleSize()];

		// initializes the variables used in the mean filter
		this.filterSum = 0;
		this.STD = 0;

		// set the beeping volume of the robot
		Sound.setVolume(50);

	}

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * The implementation for the odometry correction class. The odometer is
	 * corrected when passing black lines on the demo board. A count is stored and
	 * the odometer values are corrected based on the count. An average filter is
	 * also implemented in this method, initial readings are taken as the robot
	 * starts to move, and are used as a standard to decide whether the robot passes
	 * a black line.
	 * 
	 * <p>
	 * The odometer uses the convention where the initial heading is the +Y
	 * direction with Theta = 0. Theta increases if robot turns right. To the right
	 * of the robot is the +X direction.
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

			// if the ratio of the new intensity is less than threshold
			// probably passed a black line
			if ((intensity / STD) < THRESHOLD) {

				// current X,Y,Theta obtained from position, a double array
				X = position[0];
				Y = position[1];
				Theta = position[2];

				// signal when passed black line
				Sound.beep();

				// determine direction
				if ((Theta > 350 && Theta < 360) || (Theta > 0 && Theta < 10)) {

					// robot is moving in the +Y direction
					// difference between theoretical distance and displayed distance
					difference = Y - (TILE * countY);

					// the new Y value after correction
					Y = Y - difference;

					// increment the number of black lines passed in the Y direction
					countY++;

					// update the Y value of the odometer
					odometer.setY(Y);

				} else if (Theta > 170 && Theta < 190) {

					// robot is going in -Y direction
					// decrement the number of black lines in the Y direction
					countY--;

					// difference between theoretical distance and displayed distance
					difference = Y - (TILE * countY);

					// the new Y value after correction
					Y = Y - difference;

					// update the Y value of the odometer
					odometer.setY(Y);

				} else if (Theta > 80 && Theta < 100) {

					// robot going in +X direction
					// difference between theoretical distance and displayed distance
					difference = X - (TILE * countX);

					// the new X value after correction
					X = X - difference;

					// increment the number of black lines passed in the X direction
					countX++;

					// update the X value of the odometer
					odometer.setX(X);

				} else if (Theta > 260 && Theta < 280) {

					// robot going in -X direction
					// decrement the number of black lines in the X direction
					countX--;

					// difference between theoretical distance and displayed distance
					difference = X - (TILE * countX);

					// the new X value after correction
					X = X - difference;

					// update the X value of the odometer
					odometer.setX(X);

				}

			}

			// clear and reset variable values for each loop
			intensity = 0;
			filterSum = 0;

			// record finish time
			correctionEnd = System.currentTimeMillis();

			// this ensure the odometry correction occurs only once every period
			// if time elasped is too short, wait for some time
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {

					// sleep for the time difference between required and actual time
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));

				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}

	/**
	 * This is a private method which functions as a mean or average filter. The
	 * filter ensures the correctness of the readings, filtering out the noise in
	 * the signal from the color sensor. The filter takes 5 readings and sums the
	 * amplified value of each reading.
	 * 
	 * @return returns the average of the amplified reading
	 */
	private double meanFilter() {

		// take 5 readings
		for (int i = 0; i < 5; i++) {

			// acquire sample data and read into array with no offset
			cs.fetchSample(csData, 0);

			// amplify signal for increased sensitivity
			filterSum += csData[0] * 100;

		}

		// return an amplified average
		return filterSum / 5.0;

	}

}
