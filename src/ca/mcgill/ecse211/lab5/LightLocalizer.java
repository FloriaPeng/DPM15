package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author Floria Peng
 * @author Jamie Li
 * 
 *         This class implements the Light localization of the robot. With the use of two light sensors
 *         one on each side of the robot, the robot can localize by stopping the motor that detects a line
 *         and moving the other until it detects a line.     
 *
 */
public class LightLocalizer implements Runnable {

	/*PRIVATE FIELDS*/

	private Odometer odometer; // The odometer instance
	private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
	private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
	double leftRadius; // The left wheel radius of the robot
	double rightRadius; // The right wheel radius of the robot
	double track; // The track of the robot (by measuring the distance between the center of both
	// wheel)

	private LineCorrection linecorrection; // The instance of line correction
	private Navigation navigation; // The instance of sensor rotation

	/*CONSTANTS*/
	public static final double TILE_SIZE = 30.48; // The tile size used for demo
	public static final int FACING_CORNER = 225; // Angle facing the corner
	public static final int FULL_TURN = 360; // 360 degree for a circle
	private static final double SENSOR_TO_CENTER = 11; // The distance from the light sensor to the
	// rotation sensor
	private static final double BACK_DIST = 8.8; // Travel back distance (distance between wheels and sensors)
	private static final int ACCELERATION = 3000; // The acceleration of the motor

	/*NON-PRIVATE FIELDS*/
	double last = Math.PI; // Initialize the last variable to a specific number
	double current = 0; // last and current are both used for differential filter
	double[] detect1 = new double[4]; // The x and y tile line detect angle, clockwise
	double[] detect2 = new double[4]; // The x and y tile line detect angle, clockwise
	long[] time = new long[2]; // The time of the light sensor
	boolean[] line = {false, false}; // The detection of the line of the two light sensors
	double xerror = 0; // The localization error in the x direction
	double yerror = 0; // The localization error in the y direction
	double terror = 0; // The localization error in angle
	double before = 0; // The before line correction angle

	/**
	 * The default constructor of this class
	 * 
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftRadius
	 * @param rightRadius
	 * @param track
	 * @param navigation
	 * @param linecorrection
	 * @throws OdometerExceptions
	 */
	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track, Navigation navigation, LineCorrection linecorrection)
					throws OdometerExceptions {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.track = track;
		this.navigation = navigation;
		this.linecorrection = linecorrection;
	}

	/**
	 * The run method of this class. It will travel to 45 degree forward a specific distance, detect
	 * the black line, retreat for a pre-defined distance, starts rotation and records four angles of
	 * the robot when the black lines are detected, and finally travel to the origin point and adjust
	 * itself based on the x and y error
	 * 
	 * @see java.lang.Runnable#run()
	 */
	public void run() {

		// The robot will first travel 45 degree front-right first until the light sensor detects a line
		navigation.move(TILE_SIZE); //move forward (until you detect a line) to correct Y odometer reading
		correctAngle(); //when a line is detected, correct angle
		navigation.back(0, BACK_DIST); // Go back the offset distance between the wheels and sensors
		navigation.rotate(FULL_TURN / 4); //Rotate 90 degrees clockwise

		navigation.move(TILE_SIZE); //move forward (until you detect a line) to correct X odometer reading
		correctAngle(); //when a line is detected, correct angle
		navigation.back(BACK_DIST, 0); //Move back the offset distance between the wheels and sensors
		navigation.rotate(-FULL_TURN / 4); //Rotate 90 degrees anti-clockwise

		navigation.move(TILE_SIZE); //move forward (until you detect a line) to correct Y value (double check)
		correctAngle();//when a line is detected, correct angle 
		navigation.back(0, BACK_DIST); // Go back offset distance, you reach the origin

		//Depending on the starting corner, set the Theta value accordingly
		switch (SearchCan.SC) {
		case 0:
			odometer.setXYT(1 * TILE_SIZE, 1 * TILE_SIZE, 0);
			odometer.position[2] = Math.toRadians(0);
			break;
		case 1:
			odometer.setXYT(7 * TILE_SIZE, 1 * TILE_SIZE, 270);
			odometer.position[2] = Math.toRadians(270);
			break;
		case 2:
			odometer.setXYT(7 * TILE_SIZE, 7 * TILE_SIZE, 180);
			odometer.position[2] = Math.toRadians(180);
			break;
		case 3:
			odometer.setXYT(1 * TILE_SIZE, 7 * TILE_SIZE, 90);
			odometer.position[2] = Math.toRadians(90);
			break;
		}
	}

	/**
	 * This helper method is used to correct the Angle 
	 */
	void correctAngle() {
		while (true) {
			/* Line 0 corresponds to boolean if left motor detected a line
			 * Line 1 corresponds to boolean if right motor detected a line
			 */
			line[0] = linecorrection.filter1(); //set line[0] to whether or not a line was detected
			line[1] = linecorrection.filter2(); //set line[1] to whether or not a line was detected
			if (line[0]) { // If the black line is detected, the robot will stop
				leftMotor.stop(true);
			}
			if (line[1]) {
				rightMotor.stop(true);
			}
			if (!leftMotor.isMoving() && !rightMotor.isMoving()) {
				line[0] = false;
				line[1] = false;
				leftMotor.stop(true);
				rightMotor.stop(false);
				if (odometer.getXYT()[2] < 30 || odometer.getXYT()[2] > 330) {
					odometer.setTheta(0);
					odometer.position[2] = Math.toRadians(0);
				} else if (Math.abs(odometer.getXYT()[2] - 90) < 60) {
					odometer.setTheta(90);
					odometer.position[2] = Math.toRadians(90);
				}

				break;
			}
		}
	}

}
