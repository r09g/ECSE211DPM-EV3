package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
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
	 * Stores sensor data read in as array of float
	 */
	private float[] usData;

	private EV3LargeRegulatedMotor leftMotor; // left motor
	private EV3LargeRegulatedMotor rightMotor; // right motor
	private USNav nav;
	
	private boolean isAvoiding;
	private double distance;

	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.21;
	public static final double TILE = 30.48;
	private static final double AVDDIST = 15; // distance from which robot should stop in front of block

	private static final int bandCenter = 32; // Offset from the wall (cm)
	private static final int bandwidth = 3; // Width of dead band (cm)
	private static final int motorLow = 175; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 275; // Speed of the faster rotating wheel (deg/sec)
	
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
	public UltrasonicPoller(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider us,
			float[] usData, USNav nav) {
		this.us = us; // ultrasonic sensor
		this.usData = usData; // ultrasonic sensor data sampling control
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.nav = nav;	// navigator
		
		this.isAvoiding = false;
		
	}

	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result
	 * to an integer [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		
		// while travelling
		while (nav.isNavigating()) {

			// acquire filtered distance reading
			this.distance = filter();

			if (distance <= AVDDIST) { // robot too close to obstacle
				// TODO: implementation
				
				// let navigation wait for this to finish
				try {
					nav.join();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				isAvoiding = true;
				double error = distance - this.bandCenter; // deviation from expected band center

				if (Math.abs(error) > (this.bandwidth) && error > 0) { // robot is too far

					// turn left
					Lab3.leftMotor.setSpeed(motorHigh - 110); // slow down left motor
					Lab3.rightMotor.setSpeed(motorHigh + 70); // speed up right motor

					Lab3.rightMotor.forward(); // EV3 motor hack
					Lab3.leftMotor.forward();

				} else if (Math.abs(error) > (this.bandwidth) && error < 0) { // robot is too close

					// turn right
					Lab3.leftMotor.setSpeed(motorHigh + 180); // speed up left motor
					Lab3.rightMotor.setSpeed(10); // slow down right motor

					Lab3.rightMotor.forward();// EV3 motor hack
					Lab3.leftMotor.forward();

				} else { // error within dead band

					// robot to go straight
					Lab3.leftMotor.setSpeed(motorHigh);
					Lab3.rightMotor.setSpeed(motorHigh);

					Lab3.rightMotor.forward(); // EV3 motor hack
					Lab3.leftMotor.forward();

				}
			} else if(distance > 255) {
				
				leftMotor.stop();
				rightMotor.stop();
				
				isAvoiding = false;
				
				nav.notify();	// resume navigator thread
			}
			
			// control sensor sampling rate
			try {
				Thread.sleep(50);
			} catch (Exception e) {
				// Poor man's timed sampling	
			} 
		}
	}

	/**
	 * 
	 * @return
	 */
	private double filter() {

		double value = 0;
		for (int i = 0; i < 5; i++) {
			us.fetchSample(usData, 0);
			value += usData[0] * 100.0;
		}

		return value /= 5.0;
	}
	
	public double readDistance() {
		return this.distance;
	}

}
