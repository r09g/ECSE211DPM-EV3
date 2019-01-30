package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
	public static final double WHEEL_RAD = 2.1;

	// distance between center of left and right wheels
	// this value is tweaked to optimize the behaviours of the robot in different
	// operation modes
	public static final double TRACK = 13.21;

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); // left
																														// motor
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D")); // right
																														// motor
	private static final TextLCD lcd = LocalEV3.get().getTextLCD(); // display screen

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

			// navigation thread
			(new Thread() {
				public void run() {

					Navigation nav = new Navigation(leftMotor, rightMotor, odometer);
//					Navigation.travelTo(30.48*2, 30.48*2);
//					Navigation.travelTo(30.48, 30.48*2);
//					Navigation.travelTo(0, 30.48);
//					Navigation.travelTo(30.48*2, 30.48);
//					Navigation.travelTo(0, 0);
					Navigation.travelTo(-30.48 * 2, -30.48 * 2);

				}
			}).start(); // starts thread

		} else if(buttonChoice == Button.ID_RIGHT) { // navigation with obstacle avoidance

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer); // creates new odometer thread
			odoThread.start(); // starts thread
			Thread odoDisplayThread = new Thread(odometryDisplay); // new display thread for odometer
			odoDisplayThread.start();// starts thread

		} else {
			System.exit(0); // exit
		}

		// keep the program from ending
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0); // exit program
	}
}
