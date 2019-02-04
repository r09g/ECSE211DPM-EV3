package ca.mcgill.ecse211.lab3;

// non-static imports
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

/**
 * This class contains Lab3 Navigation Lab implemented on the EV3 platform. This
 * class specifies constants that control the behaviour of the robot. This class
 * also creates instances of the left and right motors (two Large EV3 motors),
 * and also the LCD display, and the medium EV3 motor that turns the ultrasonic
 * sensor and the information to be displayed. Users are given the option of
 * choosing a mode of operation, whether to only navigate or navigate with
 * obstacle avoidance feature. The Odometer, OdometerDisplay, navigation, and
 * sensor motor threads are started based on the button clicked by the user. The
 * robot drives to 5 waypoints defined by the user after program starts. The
 * program exits upon clicking any button after starting the program and
 * choosing a mode of operation.
 */
public class Lab3 {

	// -----------------------------------------------------------------------------
	// Public Constants
	// -----------------------------------------------------------------------------

	/**
	 * The radius (in cm) of the left/right wheels of the EV3 robot
	 */
	public static final double WHEEL_RAD = 2.2;

	/**
	 * The width (in cm) of the robot measured from the center of the left wheel to
	 * the center of the right wheel
	 */
	public static final double TRACK = 13.21;

	/**
	 * The length (in cm) of a tile
	 */
	public static final double TILE = 30.48;

	/**
	 * The instance of the left wheel large EV3 motor. The left motor is connected
	 * to port A on the EV3 brick.
	 */
	public static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * The instance of the right wheel large EV3 motor. The right motor is connected
	 * to port D on the EV3 brick.
	 */
	public static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	/**
	 * The instance of the medium EV3 motor that controls the turning of the
	 * ultrasonic sensor. The motor is connected to port C on the EV3 brick.
	 */
	public static final EV3MediumRegulatedMotor SENSOR_MOTOR = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * This 2D array specifies the path for the robot to travel. Each coordinate
	 * point is stored as an integer array, and the five coordinates are stored in a
	 * 2D int array
	 */
	public static final int[][] PATH = new int[][] {
//		{ 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 }
		{ 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 }
//		{ 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 }
//		{ 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } 
	};

	// -----------------------------------------------------------------------------
	// Private Constants
	// -----------------------------------------------------------------------------

	/**
	 * The port that the ultrasonic sensor is connected to, which is port S1 on the
	 * EV3 brick.
	 */
	private static final Port US_PORT = LocalEV3.get().getPort("S1");

	/**
	 * The LCD display instance, provides access and control of the material
	 * displayed on the screen.
	 */
	private static final TextLCD LCD = LocalEV3.get().getTextLCD();

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * The main method for Lab3 class. In this class, the user options are specified
	 * and formatted. Responses to button clicks are specified. The odometer, LCD
	 * display, ultrasonic sensor, navigator (simple or with obstacle avoidance),
	 * and ultrasonic sensor motor are created. The corresponding threads are
	 * started, allowing simultaneous execution of multiple tasks (rotating sensor,
	 * going forward, etc.)
	 * 
	 * @param args - command-line argument inputs, this will not be used here
	 * @throws OdometerExceptions - this method may throw OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice; // variable to record button clicked by user

		// retrieves the odometer instance
		// calling getOdometer ensures only one instance of Odometer class is created
		Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

		// used for displaying odometer data on the LCD screen of EV3 brick
		Display odometryDisplay = new Display(LCD);

		// ask user for navigation mode
		// repeat asking if button clicked is not left or right or esc
		do {
			// set up display format and options for navigation modes
			// LEFT for simple navigation and RIGHT for navigation with obstacle avoidance
			LCD.clear(); // clear LCD screen
			LCD.drawString("< Left  |  Right >", 0, 0);
			LCD.drawString("        |         ", 0, 1);
			LCD.drawString("Simple  |  Nav w/ ", 0, 2);
			LCD.drawString("  Nav   |    obs  ", 0, 3);
			LCD.drawString("        |   avoid ", 0, 4);
			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ESCAPE);

		// left button chosen: simple navigation selected
		if (buttonChoice == Button.ID_LEFT) {

			// Create navigator, odometer, and LCD display threads
			Navigation navThread = new Navigation(odometer);
			Thread odoThread = new Thread(odometer);
			Thread odoDisplayThread = new Thread(odometryDisplay);

			// start the threads created above
			odoThread.start();
			odoDisplayThread.start();
			navThread.start();

		} else if (buttonChoice == Button.ID_RIGHT) { // RIGHT button chosen: navigation with obstacle avoidance

			@SuppressWarnings("resource") // Because we don't bother to close this resource
			SensorModes usSensor = new EV3UltrasonicSensor(US_PORT); // provide access to selecting a mode
			SampleProvider usDistance = usSensor.getMode("Distance"); // provides access to acquire sensor data
			float[] usData = new float[usDistance.sampleSize()]; // buffer to store sensor data acquired

			// instance of the motor that turns the ultrasonic sensor
			UltrasonicMotor usMotor = new UltrasonicMotor();

			// instance of the navigator with obstacle avoidance
			USNav usnav = new USNav(odometer, usDistance, usData, usMotor);

			// Create odometer, LCD display, navigator, and ultrasonic sensor motor threads
			// ultrasonic sensor motor thread is here to make the motor turn the ultrasonic
			// sensor as the robot is travelling
			Thread odoThread = new Thread(odometer);
			Thread odoDisplayThread = new Thread(odometryDisplay);
			Thread usnavThread = new Thread(usnav);
			Thread usThread = new Thread(usMotor);

			// starts the threads created above
			odoThread.start();
			odoDisplayThread.start();
			usnavThread.start();
			usThread.start();

		} else { // if the esc button is presseds
			System.exit(0); // exit program
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
			// keep the program from ending unless esc button is pressed
		}
		System.exit(0); // exit program after esc pressed
	}

}
