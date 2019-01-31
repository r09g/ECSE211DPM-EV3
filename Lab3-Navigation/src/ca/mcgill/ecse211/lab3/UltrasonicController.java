package ca.mcgill.ecse211.lab3;

/**
 * interface for ultrasonic controller types, specifies the method required to
 * implement by classes implementing this interface.
 */

public interface UltrasonicController {

	/**
	 * Processes the ultrasonic sensor data and make adjustments to motor speeds
	 * with respect to an input distance of int type.
	 * 
	 * @param distance an int type value indicating distance measured by sensor
	 */
	public void processUSData(int distance);

	/**
	 * A wrapper method which returns the value of the local variable distance
	 * 
	 * @return an integer type value indicating the ultrasonic sensor distance
	 */
	public int readUSDistance();
}
