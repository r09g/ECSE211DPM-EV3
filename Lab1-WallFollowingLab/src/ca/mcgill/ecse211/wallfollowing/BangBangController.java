package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;
import java.math.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter; //offset from the wall
	private final int bandwidth; //width of dead band
	private final int motorLow; //low speed
	private final int motorHigh; //high speed
	private int distance; //distance recorded by sensor
	private int tally; //count the number of consecutive times out of bounds distances were recorded
	private final int bound = 160; //any distance above 160 will be considered out of bounds

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter - 3;
		this.bandwidth = bandwidth - 1;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.tally = 0;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
	}
	
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		int error = this.distance - this.bandCenter; //difference between actual distance and required distance from wall

		if(this.distance > bound) { //if sensor records out of bounds distance
			this.tally++;		
		}
				
		if(this.distance < bound) { //if sensor records valid distance
			this.tally = 0;		// clears tally value

			if(Math.abs(error) > (this.bandwidth) && error > 0) {// if robot too far from wall by more than dead band value
				
				WallFollowingLab.leftMotor.setSpeed(motorHigh - 110); //slow down left motor
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 70); //speed up right motor
				//proceed to turn left, moving closer to wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();

			} else if(Math.abs(error) > (this.bandwidth) && error < 0) { //if robot too close to wall by more than dead band value
				
				WallFollowingLab.leftMotor.setSpeed(motorHigh + 180); // speed up left motor
				WallFollowingLab.rightMotor.setSpeed(10); //slow down right motor
				//proceed to turn right, moving away from wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			
			} else { //error within dead band
				WallFollowingLab.leftMotor.setSpeed(motorHigh);
				WallFollowingLab.rightMotor.setSpeed(motorHigh);
				//robot keeps going straight
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
		} else { //if sensor records an out of bound distance
			
			if(tally > 45) { //out of bounds distance recorded more than 45 times, so there really is nothing there
				
				WallFollowingLab.leftMotor.setSpeed(30); //slow down left motor
				WallFollowingLab.rightMotor.setSpeed(motorHigh + 160); //speed up right motor
				// proceed to make corner left turn: sharper and faster than usual left turn to keep fix distance from wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			} else if (tally > 15 && tally < 45) { //if sensor needs more time to complete tally
				
				WallFollowingLab.leftMotor.setSpeed(motorLow); //slow down left motor
				WallFollowingLab.rightMotor.setSpeed(motorLow); //slow down right motor
				//proceeds to slow robot down in straight line, giving it more time to complete tally without it moving too far
				//away from wall
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
			//if tally < 15 then either it was a mistake and there is indeed a wall, either it was a small gap
			//do nothing, keep going straight
		}		
		return;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
