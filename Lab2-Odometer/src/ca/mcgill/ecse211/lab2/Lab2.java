package ca.mcgill.ecse211.lab2;

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

public class Lab2 {

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
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		OdometryCorrection odometryCorrection = new OdometryCorrection();

		Display odometryDisplay = new Display(lcd); // LCD display instance initialization

		// ask user again if button clicked is not left or right
		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" Float | Drive  ", 0, 2);
			lcd.drawString("motors | in a   ", 0, 3);
			lcd.drawString("       | square ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// left button chosen
		if (buttonChoice == Button.ID_LEFT) {
			// both motors are started and set to float mode
			leftMotor.forward();
			leftMotor.flt();
			rightMotor.forward();
			rightMotor.flt();

			// starts threads relating to floating
			// Display changes in position as wheels are (manually) moved
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

		} else {// any other button chosen

			// clear the display
			lcd.clear();

			// ask the user whether odometery correction should be run or not
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("  No   | with   ", 0, 1);
			lcd.drawString(" corr- | corr-  ", 0, 2);
			lcd.drawString(" ection| ection ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer); // creates new odometer thread
			odoThread.start(); // starts thread
			Thread odoDisplayThread = new Thread(odometryDisplay); // new display thread for odometer
			odoDisplayThread.start();// starts thread

			// Start correction if right button was pressed
			if (buttonChoice == Button.ID_RIGHT) {
				// starts the odometer correction class
				Thread odoCorrectionThread = new Thread(odometryCorrection);
				odoCorrectionThread.start();
			}

			// spawn a new Thread to avoid SquareDriver.drive() from blocking
			(new Thread() {
				public void run() {
					SquareDriver.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
				}
			}).start(); // starts thread
		}

		// keep the program from ending
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0); // exit program
	}
}
