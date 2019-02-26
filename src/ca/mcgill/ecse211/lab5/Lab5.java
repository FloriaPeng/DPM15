package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
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
 * This is the main class of the program for "Lab 5: Search and Localize"
 * Starting in a known corner, localize to the grid, and perform 
 * a search in the prescribed area for a can of specified color.
 * It will then detect the can and identify its color.
 * The prescribed area is given to us by a lower left corner and an upper right corner.
 * 
 * @author Floria Peng
 *
 */
public class Lab5 {

	/*STATIC FIELDS*/

	//Motors
	//Instantiate motors; left right and sensor motor to rotate the color sensor.
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3MediumRegulatedMotor sensorMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));

	//Ports
	private static final Port usPort = LocalEV3.get().getPort("S4"); // Ultrasonic sensor port
	private static final Port portColor1 = LocalEV3.get().getPort("S2"); // Light sensor port1
	private static final Port portColor2 = LocalEV3.get().getPort("S3"); // Light sensor port2
	private static final Port colorPort = LocalEV3.get().getPort("S1"); // Light sensor port for color detection

	private static final TextLCD lcd = LocalEV3.get().getTextLCD(); // The LCD display

	/* CONSTANTS */
	public static final double WHEEL_RAD = 2.1; // The radius of the wheel
	public static final double TRACK = 11.35; // The width of the robot measured
	public static final int FULL_TURN = 360; // 360 degree for a circle
	private static final double SCAN_DISTANCE = 5; // The detect a can distance 
	private static final double SCAN_OUT_OF_BOUND = 255; // Too close, out of bound 

	/**
	 * @param args
	 * 
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 * 
	 *         The main method from Lab3 class. This class will start the threads used for the program.
	 * 
	 */
	@SuppressWarnings("resource")
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		/* Sensor related objects */

		// US Sensor (Obstacle Detection, Front)
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // Create usSensor instance
		SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from the instance
		float[] usData = new float[usDistance.sampleSize()]; // usData is the buffer where data is stored

		// Color Sensor (Line Detection, Left)
		SensorModes myColor1 = new EV3ColorSensor(portColor1); // Get sensor instance
		SampleProvider myColorStatus1 = myColor1.getMode("Red"); // Get sample provider as "RGB"
		float[] sampleColor1 = new float[myColorStatus1.sampleSize()]; // Create a data buffer

		// Color Sensor (Line Detection, Right)
		SensorModes myColor2 = new EV3ColorSensor(portColor2); // Get sensor instance
		SampleProvider myColorStatus2 = myColor2.getMode("Red"); // Get sample provider as "RGB"
		float[] sampleColor2 = new float[myColorStatus2.sampleSize()]; // Create a data buffer

		// Color Sensor (Color Classification, Front)
		SensorModes colorSensor = new EV3ColorSensor(colorPort); // Get sensor instance
		SampleProvider colorReading = colorSensor.getMode("RGB"); // Get sample provider as "RGB"
		float[] colorData = new float[colorReading.sampleSize()]; // Create a data buffer


		/* Obtaining Instances */

		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); //instance of odometer

		Display odometryDisplay = new Display(lcd); //instance of Display

		ColorClassification colorclassification =
				new ColorClassification(usDistance, usData, colorReading, colorData); //instance of ColorClassification

		LineCorrection linecorrection =
				new LineCorrection(myColorStatus1, sampleColor1, myColorStatus2, sampleColor2); //instance of LineCorrection

		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, sensorMotor,
				colorclassification, linecorrection, WHEEL_RAD, WHEEL_RAD, TRACK); //instance of Navigation

		UltrasonicLocalizer uslocalizer = new UltrasonicLocalizer(odometer, leftMotor, rightMotor,
				WHEEL_RAD, WHEEL_RAD, TRACK, usDistance, usData, navigation); //instance of UltrasonicLocalizer

		LightLocalizer lightlocalizer = new LightLocalizer(odometer, leftMotor, rightMotor, WHEEL_RAD,
				WHEEL_RAD, TRACK, navigation, linecorrection); //instance of LightLocalizer

		SearchCan searchcan = new SearchCan(TRACK, odometer, navigation, colorclassification); //instance of SearchCan

		Sound.beepSequence();
		// The color classification until ESC pressed
		while (Button.readButtons() != Button.ID_ESCAPE) {

			// Print instruction
			lcd.drawString("Press ESC to start searching", 0, 2);
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
					Thread.sleep(1000);
				} catch (Exception e) {
				}
			}
			lcd.clear();
		}

		Sound.beepSequenceUp();

		/* STARTING THREADS */

		//Starting odometer thread
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		//Starting display thread
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Starting localization thread
		UltrasonicLocalizer.OPTION = false; // The user is choosing rising edge

		// Start the thread for us localizer
		Thread usThread = new Thread(uslocalizer);
		usThread.start();
		usThread.join();

		// Start the thread for light localizer
		Thread lightThread = new Thread(lightlocalizer);
		lightThread.start();
		lightThread.join();

		// Start the thread for can searching
		Thread scThread = new Thread(searchcan);
		scThread.start();

		// Wait here forever until button pressed to terminate the robot
		Button.waitForAnyPress();
		System.exit(0);
	}
}
