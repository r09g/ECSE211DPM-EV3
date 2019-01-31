package ca.mcgill.ecse211.lab3;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller
 * thread. The while loop at the bottom executes in a loop. Assuming that the
 * us.fetchSample, and cont.processUSData methods operate in about 20mS, and
 * that the thread sleeps for 50 mS at the end of each loop, then one cycle
 * through the loop is approximately 70 mS. This corresponds to a sampling rate
 * of 1/70mS or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {

	// ----------------------------------------------------------------------------
	// Class Variables
	// ----------------------------------------------------------------------------

	/**
	 * Stores ultrasonic sensor instance
	 */
	private SampleProvider us;

	/**
	 * Controller instance, either bangbang or p-type
	 */
	private UltrasonicController cont;

	/**
	 * Stores sensor data read in as array of float
	 */
	private float[] usData;

	// ----------------------------------------------------------------------------
	// Constructor
	// ----------------------------------------------------------------------------

	/**
	 * Constructor creates a UltrasonicPoller class, initializes class variables.
	 * 
	 * @param us     the ultrasonic sensor instance
	 * @param usData the data obtained from the ultrasonic sensor
	 * @param cont   the controller instance
	 */
	public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
		this.us = us;
		this.cont = cont;
		this.usData = usData;
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result
	 * to an integer [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		int distance;

		while (true) { // operates continuously
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

			// TODO: pause navigation thread and let controller run while encountering
			// obstacle; resume nav thread after obstacle avoided;
			if (USNav.isNavigating() && distance < 15) {
				// robot on the move
				cont.processUSData(distance); // activate controller
				
			} else {
				// do nothing
			}

			try {
				Thread.sleep(50); // control sensor sampling rate
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

}
