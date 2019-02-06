package ca.mcgill.ecse211.lab4;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Lab4 {

  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  
  private static final Port US_PORT = LocalEV3.get().getPort("S1");
  
  private static final TextLCD LCD = LocalEV3.get().getTextLCD();
  
  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------
  
  public static void main(String[] args) {
    
    int buttonChoice; // variable to record button clicked by user
    
    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString(" Rising | Falling ", 0, 2);
      LCD.drawString("  Edge  |   Edge  ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);

    if(buttonChoice == Button.ID_LEFT) {
      // TODO: rising edge 
      
    } else if(buttonChoice == Button.ID_RIGHT) {
      // TODO: falling edge
      
    } else {
      System.exit(0); 
    }
    
    // keep the program from ending unless esc button is pressed
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      
    }
    System.exit(0); // exit program after esc pressed
  }
}
