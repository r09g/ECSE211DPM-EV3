package ca.mcgill.ecse211.HelloWorld;

import lejos.hardware.Button;

/*
 * Hello world program test
 * Display "Hello World!" on EV3 and quit when any button pressed
 */
public class HelloWorld {
	
	public static void main(String args[]) {
		
		System.out.println("Hello World!");
		Button.waitForAnyEvent();
	}
	
	
}
