package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author Floria Peng
 * @author Jamie Li
 * 
 *         This class implements the Light locatizaion of the robot. There are a few processes in
 *         this class. This method thread is implemented after the ultrasonic localization is done.
 *         It firstly travel to the point (1,1) using Navigation class. When the light sensor
 *         detects the black line, it means the robot is almost at the corner of the (1,1), so it
 *         could starts adjusting. The robot would retreat for a pre-defined distance and starts
 *         rotation. The detect should detects four black lines, and record the angles of them. Then
 *         the robot would calculate the x-error and y-error based the four angles. Finally, the
 *         robot travels to (1,1) again and adjust its angle based on x-error and y error.
 *
 */
public class LightLocalizer implements Runnable { // TODO missing comment

  private Odometer odometer; // The odometer instance
  private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
  private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
  double leftRadius; // The left wheel radius of the robot
  double rightRadius; // The right wheel radius of the robot
  double track; // The track of the robot (by measuring the distance between the center of both
                // wheel)

  private LineCorrection linecorrection; // The instance of line correction
  private Navigation navigation; // The instance of sensor rotation

  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  public static final int FACING_CORNER = 225; // Angle facing the corner
  public static final int FULL_TURN = 360; // 360 degree for a circle
  private static final double SENSOR_TO_CENTER = 11; // The distance from the light sensor to the
                                                     // rotation sensor
  private static final int BACK_DIST = 13; // Travel back distance TODO

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
   * The constructor of this class
   * 
   * @param odometer
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param track
   * @param myColorStatus
   * @param sampleColor
   * @param navigation
   * @throws OdometerExceptions
   */
  public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track)
      throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;
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
    navigation.move(TILE_SIZE);
    correctAngle();
    navigation.back(0, BACK_DIST); // And then go back a certain distance
    navigation.rotate(FULL_TURN / 4);

    navigation.move(TILE_SIZE);
    correctAngle();
    navigation.back(BACK_DIST, 0);

    navigation.turn(FULL_TURN); // Then starts rotating clockwise

    for (int i = 0; i < 4;) { // It will record 4 angles
      if (linecorrection.filter1()) {
        detect1[i] = odometer.getXYT()[2];
      }
      if (linecorrection.filter2()) {
        detect1[i] = odometer.getXYT()[2];
        i++;
      }
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }
    leftMotor.stop(true); // Stop both motors
    rightMotor.stop(false);

    // Calculate the x error, y error and theta error using the cos of the angle detected
    xerror = SENSOR_TO_CENTER * Math.cos(
        Math.toRadians((((detect1[3] + detect2[3]) / 2) - ((detect1[1] + detect2[1]) / 2)) / 2));
    yerror = SENSOR_TO_CENTER * Math.cos(
        Math.toRadians((((detect1[2] + detect2[2]) / 2) - ((detect1[0] + detect2[0]) / 2)) / 2));

    terror = 270 - (((detect1[3] + detect2[3]) / 2) + ((detect1[1] + detect2[1]) / 2)) / 2;

    // Correcting the position of the robot
    odometer.position[2] = odometer.position[2] + terror;
    odometer.setXYT(-xerror, -yerror, odometer.position[2]);
    navigation.travelTo(0, 0);
    navigation.turnTo(0);

    switch (SearchCan.SC) {
      case 0:
        odometer.setXYT(1 * TILE_SIZE, 1 * TILE_SIZE, 0);
        odometer.position[2] = 0;
        break;
      case 1:
        odometer.setXYT(7 * TILE_SIZE, 1 * TILE_SIZE, 270);
        odometer.position[2] = 270;
        break;
      case 2:
        odometer.setXYT(7 * TILE_SIZE, 7 * TILE_SIZE, 180);
        odometer.position[2] = 180;
        break;
      case 3:
        odometer.setXYT(1 * TILE_SIZE, 7 * TILE_SIZE, 90);
        odometer.position[2] = 90;
        break;
    }
  }

  void correctAngle() {
    while (rightMotor.isMoving() || leftMotor.isMoving()) {
      if (linecorrection.filter1()) { // If the black line is detected, the robot will stop
        time[0] = System.currentTimeMillis();
        line[0] = true;
      }
      if (linecorrection.filter2()) {
        time[1] = System.currentTimeMillis();
        line[1] = true;
      }
      if (line[0] && line[1]) {
        line[0] = line[1] = false;
        leftMotor.stop(true);
        rightMotor.stop(false);
        double dtheta = Math.atan(((time[1] - time[0]) * Navigation.FORWARD_SPEED) / track);
        before = odometer.getXYT()[2];
        navigation.rotate(-dtheta);
        odometer.setTheta(before);
        odometer.position[2] = before;
      }
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }
  }

}
