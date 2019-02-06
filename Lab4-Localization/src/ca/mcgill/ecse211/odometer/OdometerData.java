package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class stores and provides thread safe access to the odometer data.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

public class OdometerData {

	// -----------------------------------------------------------------------------
	// Class Variables
	// -----------------------------------------------------------------------------
	
	// Position parameters
	private volatile double x; // x-axis position
	private volatile double y; // y-axis position
	private volatile double theta; // Head angle

	// Number of OdometerData objects instantiated so far
	private volatile static int numberOfIntances = 0;

	// Maximum number of OdometerData instances
	private static final int MAX_INSTANCES = 1;

	// Thread control tools
	// Fair lock for concurrent writing
	private static Lock lock = new ReentrantLock(true);

	// Indicates if a thread is trying to reset any position parameters
	private volatile boolean isReseting = false;

	// Let other threads know that a reset operation is over.
	private Condition doneReseting = lock.newCondition();

	// OdometerData instance
	private static OdometerData odoData = null;

	/**
	 * Default constructor. The constructor is private. A factory is used instead
	 * such that only one instance of this class is ever created.
	 */
	protected OdometerData() {
		this.x = 0;
		this.y = 0;
		this.theta = 0;
	}

	// -----------------------------------------------------------------------------
	// Public Methods
	// -----------------------------------------------------------------------------

	/**
	 * OdometerData factory. Returns an OdometerData instance and makes sure that
	 * only one instance is ever created. If the user tries to instantiate multiple
	 * objects, the method throws a MultipleOdometerDataException.
	 * 
	 * @return An OdometerData object
	 * @throws OdometerExceptions - Exception if more than one odometer instance
	 *                            exists
	 */
	public synchronized static OdometerData getOdometerData() throws OdometerExceptions {
		if (odoData != null) { // Return existing object
			return odoData;
		} else if (numberOfIntances < MAX_INSTANCES) { // create object and
														// return it
			odoData = new OdometerData();
			numberOfIntances += 1;
			return odoData;
		} else {
			throw new OdometerExceptions("Only one intance of the Odometer can be created.");
		}

	}

	/**
	 * Return the Odomometer data.
	 * <p>
	 * Writes the current position and orientation of the robot onto the odoData
	 * array. odoData[0] = x, odoData[1] = y; odoData[2] = theta;
	 * 
	 * @param position the array to store the odometer data
	 * @return the odometer data.
	 */
	public double[] getXYT() {
		double[] position = new double[3]; // stores position data
		lock.lock(); // acquire lock and control access to synchronize threads
		try {
			while (isReseting) { // If a reset operation is being executed, wait
				// until it is over.
				doneReseting.await(); // Using await() is lighter on the CPU
				// than simple busy wait.
			}

			// retrieved data
			position[0] = x;
			position[1] = y;
			position[2] = theta;

		} catch (InterruptedException e) {
			// Print exception to screen
			e.printStackTrace();
		} finally {
			lock.unlock(); // release lock to and unblock access
		}

		return position;

	}

	/**
	 * Adds dx, dy and dtheta to the current values of x, y and theta, respectively.
	 * Useful for odometry.
	 * 
	 * @param dx
	 * @param dy
	 * @param dtheta
	 */
	public void update(double dx, double dy, double dtheta) {

		// thread synchronization
		lock.lock(); // acquire lock
		isReseting = true; // flag to indicate currently resetting

		// update position values
		try {
			x += dx;
			y += dy;
			theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
															// within 360
															// degrees

			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock(); // release lock
		}

	}

	/**
	 * Overrides the values of x, y and theta. Use for odometry correction.
	 * 
	 * @param x     the value of x
	 * @param y     the value of y
	 * @param theta the value of theta
	 */
	public void setXYT(double x, double y, double theta) {
		lock.lock(); // acquire lock
		isReseting = true;
		try {
			this.x = x;
			this.y = y;
			this.theta = theta;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides x. Use for odometry correction.
	 * 
	 * @param x the value of x
	 */
	public void setX(double x) {
		lock.lock(); // acquire lock
		isReseting = true;
		try {
			this.x = x;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides y. Use for odometry correction.
	 * 
	 * @param y the value of y
	 */
	public void setY(double y) {
		lock.lock();
		isReseting = true;
		try {
			this.y = y;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

	/**
	 * Overrides theta. Use for odometry correction.
	 * 
	 * @param theta the value of theta
	 */
	public void setTheta(double theta) {
		lock.lock(); // acquire lock
		isReseting = true;
		try {
			this.theta = theta;
			isReseting = false; // Done reseting
			doneReseting.signalAll(); // Let the other threads know that you are
										// done reseting
		} finally {
			lock.unlock();
		}
	}

}
