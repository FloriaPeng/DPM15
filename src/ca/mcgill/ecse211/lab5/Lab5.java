// Lab5.java
package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * @author Floria Peng
 * 
 *         This class implements the control of the EV3 robot. It will ask for user choice (left
 *         button for falling edge and right button for rising edge) for re-orientation. The robot
 *         will then correct to 0 degree with respect to the x-y coordinate. It will also start the
 *         display thread and the odometer thread after any of the user choice.
 *
 */
public class Lab5 { // TODO missing comment

  // Motor and Sensor Objects, and Robot related parameters
  // Connect all the motors and sensor to its port
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final EV3MediumRegulatedMotor sensorMotor =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));

  private static final Port usPort = LocalEV3.get().getPort("S4"); // Ultrasonic sensor port
  // private static final Port portColor1 = LocalEV3.get().getPort("S2"); // Light sensor port1
  // private static final Port portColor2 = LocalEV3.get().getPort("S3"); // Light sensor port2
  private static final Port colorPort = LocalEV3.get().getPort("S1"); // Light sensor port for color
                                                                      // detection

  private static final TextLCD lcd = LocalEV3.get().getTextLCD(); // The LCD display
  public static final double WHEEL_RAD = 2.1; // The radius of the wheel measured
  public static final double TRACK = 11.35; // The width of the robot measured
  public static final int FULL_TURN = 360; // 360 degree for a circle
  private static final double SCAN_DISTANCE = 5; // The detect a can distance TODO
  private static final double SCAN_OUT_OF_BOUND = 255; // Too close, out of bound TODO

  /**
   * @param args
   * @throws OdometerExceptions
   * 
   *         The main method for Lab3 class. This class will pass the user choice to the
   *         UltrasonicLocalizer class, and start the threads used for the UltrasonicLocalizer.
   * 
   */
  @SuppressWarnings("resource")
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    // Create the odometer of the robot
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

    // Sensor related objects
    // Necessary for creating a ultrasonic sensor that reads in distance
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // Create usSensor instance
    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
                                                              // the instance
    float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer where data is
                                                         // stored

    /*// Sensor1 related objects
    // Necessary for creating a light sensor that reads the color
    SensorModes myColor1 = new EV3ColorSensor(portColor1); // Get sensor instance
    SampleProvider myColorStatus1 = myColor1.getMode("RGB"); // Get sample provider as "RGB"
    float[] sampleColor1 = new float[myColorStatus1.sampleSize()]; // Create a data buffer

    // Sensor1 related objects
    // Necessary for creating a light sensor that reads the color
    SensorModes myColor2 = new EV3ColorSensor(portColor2); // Get sensor instance
    SampleProvider myColorStatus2 = myColor2.getMode("RGB"); // Get sample provider as "RGB"
    float[] sampleColor2 = new float[myColorStatus2.sampleSize()]; // Create a data buffer
*/
    // Sensor related objects
    // Necessary for creating a light sensor that reads the color
    SensorModes colorSensor = new EV3ColorSensor(colorPort); // Get sensor instance
    SampleProvider colorReading = colorSensor.getMode("RGB"); // Get sample provider as "RGB"
    float[] colorData = new float[colorReading.sampleSize()]; // Create a data buffer

    ColorClassification colorclassification =
        new ColorClassification(usDistance, usData, colorReading, colorData);

    /*LineCorrection linecorrection =
        new LineCorrection(myColorStatus1, sampleColor1, myColorStatus2, sampleColor2);

    // Navigation related objects
    // The navigation instance for creating a thread
    Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, sensorMotor,
        colorclassification, linecorrection, WHEEL_RAD, WHEEL_RAD, TRACK);

    SearchCan searchcan = new SearchCan(TRACK, odometer, navigation, colorclassification);

    // LightLocalizer related objects
    // The lightlocalizer instance for creating a thread
    LightLocalizer lightlocalizer = new LightLocalizer(odometer, leftMotor, rightMotor, WHEEL_RAD,
        WHEEL_RAD, TRACK, linecorrection, navigation, searchcan);

    // UltrasonicLocalizer related objects
    // The uslocalizer instance for creating a thread
    UltrasonicLocalizer uslocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor,
        WHEEL_RAD, WHEEL_RAD, TRACK, usDistance, usData, navigation, lightlocalizer, lcd);

    // Display related objects
    // The display instance for updating the odometer reading to the LCD display
    Display odometryDisplay = new Display(lcd); // No need to change
*/
    // navigation.goTo(0, 30.48, 4);
    /*sensorMotor.setSpeed(200 / 4);
    sensorMotor.rotate(FULL_TURN);*/

    // The color classification
    while (Button.readButtons() != Button.ID_ESCAPE) {
      if (colorclassification.median_filter() < SCAN_DISTANCE
          || colorclassification.median_filter() > SCAN_OUT_OF_BOUND) {
        lcd.drawString("Object Detected", 0, 0);
        if (colorclassification.colorDetect(1)) { // Blue detect
          lcd.drawString("Blue", 0, 1);
        } else if (colorclassification.colorDetect(2)) { // Green detect
          lcd.drawString("Green", 0, 1);
        } else if (colorclassification.colorDetect(3)) { // Yellow detect
          lcd.drawString("Yellow", 0, 1);
        } else if (colorclassification.colorDetect(4)) { // Red detect
          lcd.drawString("Red", 0, 1);
        }
        try {
          Thread.sleep(2000);
        } catch (Exception e) {
        }
      }
      lcd.clear();
    }

    /*// Asking for user choice
    do {
      // clear the display
      lcd.clear();

      // Ask the user whether the robot avoid the obstacles or not
      lcd.drawString("<  Left  |  Right  >", 0, 0);
      lcd.drawString("         |         ", 0, 1);
      lcd.drawString(" Falling | Rising  ", 0, 2);
      lcd.drawString("  Edge   |   Edge  ", 0, 3);
      lcd.drawString("         |         ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) { // Falling edge

      UltrasonicLocalizer.OPTION = true; // The user choosing falling edge

      // Starts the thread for odometer and display
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      // Starts the thread for us localizer
      Thread usThread = new Thread(uslocalizer);
      usThread.start();

    } else { // Rising edge

      UltrasonicLocalizer.OPTION = false; // The user is choosing rising edge

      // Starts odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      // Starts the thread for us localizer
      Thread usThread = new Thread(uslocalizer);
      usThread.start();

    }

    // Press escape button to exit the program
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);*/
  }
}
