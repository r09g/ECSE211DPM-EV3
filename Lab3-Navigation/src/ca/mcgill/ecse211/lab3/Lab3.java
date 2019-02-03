package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
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

	private static final double WHEEL_RAD = 2.1;
	private static final double TRACK = 13.18;
	private static final double TILE = 30.48;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	private static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); 
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
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

		// get odometer
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

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
			Navigation navThread = new Navigation(leftMotor, rightMotor, odometer);
			Thread odoThread = new Thread(odometer);
			Thread odoDisplayThread = new Thread(odometryDisplay);
			
			odoThread.start();
			odoDisplayThread.start();
			
			navThread.start();	// **Update: paths are specified in the Navigation class

		} else if (buttonChoice == Button.ID_RIGHT) { // navigation with obstacle avoidance
			
			@SuppressWarnings("resource") // Because we don't bother to close this resource
			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
			float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are

			UltrasonicMotor usMotor = new UltrasonicMotor(sensorMotor);
			USNav usnav = new USNav(leftMotor, rightMotor, odometer, usDistance, usData, usMotor);	// navigators
			
			
			// -------------------------------------------------------------------------------s--
			// ---------------------------------------------------------------------------------

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer); // creates new odometer thread
			Thread odoDisplayThread = new Thread(odometryDisplay); // new display thread for odometer
			Thread usnavThread = new Thread(usnav);
			Thread usThread = new Thread(usMotor); 
			
			odoThread.start(); // starts thread
			odoDisplayThread.start();// starts thread
			usnavThread.start();	// **Update: paths are specified in the Navigation class
			usThread.start();
			
		} else {
			// exits upon pressing esc button
			System.exit(0);
		}

		// keep the program from ending
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0); // exit program
	}

}
