package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int tally;

  public PController(int bandCenter, int bandWidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandWidth;
    this.tally = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
  }

  @Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		// count to decide if at corner
		// check if distance stays max
		if(this.distance > 160) {
			this.tally++;
			WallFollowingLab.leftMotor.setSpeed(100);
			WallFollowingLab.rightMotor.setSpeed(100);
		}

		// check valid distance value
		if(this.distance < 160) {
			this.tally = 0;		// clear tally value

			if(this.distance > this.bandCenter) {
				// this is for far from wall
				// turn left, sharply if far from wall
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 6*(Math.abs(this.distance-bandWidth)));
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 6*(Math.abs(this.distance-bandWidth)));

			} else if(this.distance < this.bandCenter) {
				// this is for close to wall
				// turn right
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + 8*(Math.abs(this.distance-bandWidth)));
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - 6*(Math.abs(this.distance-bandWidth)));
			
			}
		} else {
			// this is for corners
			// turn left faster, robot at edge
			// check tally
			if(tally > 50) {
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - 40);
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + 140);
			}
		}

		return;
	}

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
