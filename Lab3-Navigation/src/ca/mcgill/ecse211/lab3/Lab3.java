package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import ca.mcgill.ecse211.lab3.BangBangController;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

/**
 * This class contains Lab2 Odometry Lab implemented on the EV3 platform. This
 * class specifies constants that control the behaviour of the robot. This class
 * also creates instances of the left and right motors, and also the LCD display
 * and the information to be displayed. Users are given the option of choosing a
 * mode of operation. The Odometer, OdometerDisplay, and odometryCorrection
 * threads are started based on the button clicked by the user. The robot drives
 * a square path after program starts. The program exits upon clicking any
 * button after starting the program and choosing a mode of operation.
 */

public class Lab3 {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	// wheel radius of robot
	// this value reflects the actual value of the wheel radius
	private static final double WHEEL_RAD = 2.1;

	// distance between center of left and right wheels
	// this value is tweaked to optimize the behaviours of the robot in different
	// operation modes
	private static final double TRACK = 13.21;

	// length of tile in cm
	private static final double TILE = 30.48;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	// Motor Objects, and Robot related parameters
	private static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); // left
																													// motor
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D")); // right
																														// motor
	private static final TextLCD lcd = LocalEV3.get().getTextLCD(); // display screen
	private static final int bandCenter = 32; // Offset from the wall (cm)
	private static final int bandWidth = 3; // Width of dead band (cm)
	private static final int motorLow = 175; // Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 275; // Speed of the faster rotating wheel (deg/seec)

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * The main method for Lab2 class. In this class, the user options are specified
	 * and formatted. Responses to button clicks are specified. The instances of
	 * odometer, odometryCorrection, and odometerDisplay are created. The odometer,
	 * odometerDisplay, odometryCorrection, and SquareDrive threads are started.
	 * 
	 * @param args - command-line argument inputs, this will not be used here
	 * @throws OdometerExceptions - this method may throw OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		// records button clicked by user
		int buttonChoice;

		// Odometer instance and odometry correction instance initialization
		final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // LCD display instance initialization

		// ask user again if button clicked is not left or right
		do {
			// clear the display
			lcd.clear();

			lcd.drawString("< Left  |  Right >", 0, 0);
			lcd.drawString("        |         ", 0, 1);
			lcd.drawString("Simple  |  Nav w/ ", 0, 2);
			lcd.drawString("  Nav   |    obs  ", 0, 3);
			lcd.drawString("        |   avoid ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ESCAPE);

		// left button chosen
		if (buttonChoice == Button.ID_LEFT) {

			// simple navigation
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Navigation nav = new Navigation(leftMotor, rightMotor, odometer);

			// navigation thread
			(new Thread() {
				public void run() {
					// choose travelling sequence
					SNpath(1);
				}
			}).start(); // starts thread

		} else if (buttonChoice == Button.ID_RIGHT) { // navigation with obstacle avoidance

			// ---------------------------------------------------------------------------------
			// Ultrasonic Sensor Setup		
			// ---------------------------------------------------------------------------------
			BangBangController bangbangController = new BangBangController(bandCenter, bandWidth, motorLow, motorHigh);

			/*
			 * Setup ultrasonic sensor There are 4 steps involved: 1. Create a port object
			 * attached to a physical port (done already above) 2. Create a sensor instance
			 * and attach to port 3. Create a sample provider instance for the above and
			 * initialize operating mode 4. Create a buffer for the sensor data
			 */

			@SuppressWarnings("resource") // Because we don't bother to close this resource
			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
			// this instance
			float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
			// returned
			UltrasonicPoller usPoller = null; // the selected controller on each cycle
			usPoller = new UltrasonicPoller(usDistance, usData, bangbangController);
			
			// ---------------------------------------------------------------------------------
			// ---------------------------------------------------------------------------------

			
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer); // creates new odometer thread
			odoThread.start(); // starts thread
			Thread odoDisplayThread = new Thread(odometryDisplay); // new display thread for odometer
			odoDisplayThread.start();// starts thread
			usPoller.start();
			
			USNav usnav = new USNav(leftMotor, rightMotor, odometer, usDistance, usData);
			
			USNavpath(1);	// select path

		} else {
			// exits upon pressing esc button
			System.exit(0);
		}

		// keep the program from ending
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0); // exit program
	}

	private static void SNpath(int num) {
		switch (num) {
		case 1:
			Navigation.travelTo(0, 2);
			Navigation.travelTo(1, 1);
			Navigation.travelTo(2, 2);
			Navigation.travelTo(2, 1);
			Navigation.travelTo(1, 0);
			break;
		case 2:
			Navigation.travelTo(1, 1);
			Navigation.travelTo(0, 2);
			Navigation.travelTo(2, 2);
			Navigation.travelTo(2, 1);
			Navigation.travelTo(1, 0);
			break;
		case 3:
			Navigation.travelTo(1, 0);
			Navigation.travelTo(2, 1);
			Navigation.travelTo(2, 2);
			Navigation.travelTo(0, 2);
			Navigation.travelTo(1, 1);
			break;
		case 4:
			Navigation.travelTo(0, 1);
			Navigation.travelTo(2, 1);
			Navigation.travelTo(1, 0);
			Navigation.travelTo(2, 1);
			Navigation.travelTo(2, 2);
			break;
		default:
			break;
		}
	}

	private static void USNavpath(int num) {
		switch (num) {
		case 1:
			USNav.run(0, 2);
			USNav.run(1, 1);
			USNav.run(2, 2);
			USNav.run(2, 1);
			USNav.run(1, 0);
			break;
		case 2:
			USNav.run(1, 1);
			USNav.run(0, 2);
			USNav.run(2, 2);
			USNav.run(2, 1);
			USNav.run(1, 0);
			break;
		case 3:
			USNav.run(1, 0);
			USNav.run(2, 1);
			USNav.run(2, 2);
			USNav.run(0, 2);
			USNav.run(1, 1);
			break;
		case 4:
			USNav.run(0, 1);
			USNav.run(2, 1);
			USNav.run(1, 0);
			USNav.run(2, 1);
			USNav.run(2, 2);
			break;
		default:
			break;
		}

	}

}
