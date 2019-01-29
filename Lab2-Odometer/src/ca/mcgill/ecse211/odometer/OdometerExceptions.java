package ca.mcgill.ecse211.odometer;

/**
 * This is an exception class and is used to handle errors regarding the
 * singleton pattern used for the odometer and odometerData
 *
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {

	/**
	 * This is a constructor for the OdometerExceptions object. The super class is
	 * called with the error message.
	 * 
	 * @param Error - the error that occurs
	 */
	public OdometerExceptions(String Error) {

		// calls super constructor
		super(Error);

	}

}
