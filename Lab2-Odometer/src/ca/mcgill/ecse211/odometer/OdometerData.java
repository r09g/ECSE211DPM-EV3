package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class stores and provides thread safe access to the odometer data. This
 * class contains synchronized methods and Locks to ensure thread
 * synchronization. This class comprises of many getters and setters to
 * manipulate the X,Y,Theta of the odometer.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

public class OdometerData {

	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------

	// Maximum number of OdometerData instances allowed
	private static final int MAX_INSTANCES = 1;

	// -----------------------------------------------------------------------------
	// Class Variable
	// -----------------------------------------------------------------------------

	// threads that access a volatile variable will first check the variable's
	// current value in main memory
	// records number of OdometerData objects instantiated so far
	private volatile static int numberOfIntances = 0;

	// Position parameters
	private volatile double x; // x-axis position
	private volatile double y; // y-axis position
	private volatile double theta; // Head angle

	// Thread control tools
	// Fair lock for concurrent writing
	private static Lock lock = new ReentrantLock(true);

	// indicates if a thread is trying to reset any position parameters
	private volatile boolean isReseting = false;

	// signals other threads that a reset operation is over
	private Condition doneReseting = lock.newCondition();

	// OdometerData object
	private static OdometerData odoData = null;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------

	/**
	 * Default constructor. The constructor is private. A factory is used instead
	 * such that only one instance of this class is ever created. Constructor
	 * initializes the position data.
	 */
	protected OdometerData() {
		this.x = 0;
		this.y = 0;
		this.theta = 0;
	}

	/**
	 * OdometerData factory. Returns an OdometerData instance and makes sure that
	 * only one instance is ever created. If the user tries to instantiate multiple
	 * objects, the method throws a MultipleOdometerDataException.
	 * 
	 * @return An OdometerData object providing control to position data
	 * @throws OdometerExceptions - throws exception if user tries to instantiate
	 *                            multiple objects
	 */
	public synchronized static OdometerData getOdometerData() throws OdometerExceptions {

		// checks if a OdometerData object current exists
		if (odoData != null) {

			// Return existing object
			return odoData;

		} else if (numberOfIntances < MAX_INSTANCES) {

			// no OdometerData objects have been instantiated yet
			odoData = new OdometerData();
			numberOfIntances += 1; // increment count
			return odoData;

		} else {

			// more than one OdometerData object exists
			throw new OdometerExceptions("Only one intance of the Odometer can be created.");

		}

	}

	// -----------------------------------------------------------------------------
	// Public Method
	// -----------------------------------------------------------------------------

	/**
	 * Return the Odomometer data in a double precision array.
	 * 
	 * <p>
	 * Writes the current position and orientation of the robot onto the odoData
	 * array. odoData[0] = X; odoData[1] = Y; odoData[2] = theta.
	 * 
	 * @return position - a double precision array containing position data: X, Y,
	 *         and Theta
	 */
	public double[] getXYT() {

		// stores position data
		double[] position = new double[3];

		// acquire lock and control access to synchronize threads
		lock.lock();

		try {
			while (isReseting) {

				// If a reset operation is being executed, wait until it is over.
				// Using await() is lighter on the CPU than simple busy wait.
				doneReseting.await();

			}

			// retrieved position data X, Y, and Theta
			position[0] = x;
			position[1] = y;
			position[2] = theta;

		} catch (InterruptedException e) {

			// Print exception to screen
			e.printStackTrace();

		} finally {

			// release lock to and unblock access before exiting try-catch
			lock.unlock();

		}

		return position;

	}

	/**
	 * Adds dx, dy and dtheta to the current values of x, y and theta, respectively.
	 * Useful for odometry.
	 * 
	 * @param dx     - new value of X to be incremented
	 * @param dy     - new value of Y to be incremented
	 * @param dtheta - new value of Theta to be incremented
	 */
	public void update(double dx, double dy, double dtheta) {

		// acquire lock and control access to synchronize threads
		lock.lock();

		// flag to indicate currently resetting
		isReseting = true;

		// update position values
		try {

			// updates X,Y position values
			x += dx;
			y += dy;
			theta = (theta + (360 + dtheta) % 360) % 360; // keeps Theta value within 360

			isReseting = false; // Done reseting

			// Let the other threads know that you are done resetting
			doneReseting.signalAll();

		} finally {

			// release lock before exiting try-catch
			lock.unlock();

		}

	}

	/**
	 * Setter for position variable X, Y, and Theta. Use for odometry correction.
	 * 
	 * @param x     - the value of x
	 * @param y     - the value of y
	 * @param theta - the value of theta
	 */
	public void setXYT(double x, double y, double theta) {

		// acquire lock and control access to synchronize threads
		lock.lock();

		// flag to indicate currently resetting
		isReseting = true;

		try {

			// overrides X,Y,Theta position values
			this.x = x;
			this.y = y;
			this.theta = theta;

			// indicates done reseting
			isReseting = false;

			// Let the other threads know that you are done reseting
			doneReseting.signalAll();

		} finally {

			// release lock before exiting try-catch
			lock.unlock();

		}
	}

	/**
	 * Setter for position variable X. Use for odometry correction.
	 * 
	 * @param x - the new value of x
	 */
	public void setX(double x) {
		
		// acquire lock and control access to synchronize threads
		lock.lock();
		
		// flag to indicate currently resetting
		isReseting = true;
		
		try {
			
			// sets the value of the position variable X
			this.x = x;

			// indicates done reseting
			isReseting = false; 
			
			// Let the other threads know that you are done reseting
			doneReseting.signalAll();
			
		} finally {
			
			// release lock before exiting try-catch
			lock.unlock();
			
		}
	}

	/**
	 * Setter for position variable Y. Use for odometry correction.
	 * 
	 * @param y - the new value of y
	 */
	public void setY(double y) {
		
		// acquire lock and control access to synchronize threads
		lock.lock();
		
		// flag to indicate currently resetting
		isReseting = true;
		
		try {
			
			// sets the value of the position variable Y
			this.y = y;
			
			// indicates done reseting
			isReseting = false;
			
			// Let the other threads know that you are done reseting
			doneReseting.signalAll(); 
			
		} finally {
			
			// release lock before exiting try-catch
			lock.unlock();
			
		}
	}

	/**
	 * Setter for position variable Y. Use for odometry correction.
	 * 
	 * @param theta - the new value of theta
	 */
	public void setTheta(double theta) {
		
		// acquire lock and control access to synchronize threads
		lock.lock();
		
		// flag to indicate currently resetting
		isReseting = true;
		
		try {

			// sets the value of the position variable Theta
			this.theta = theta;
			
			// indicates done reseting
			isReseting = false; 
			
			// Let the other threads know that you are done reseting
			doneReseting.signalAll(); 
			
		} finally {
			
			// release lock before exiting try-catch
			lock.unlock();
			
		}
	}

}
