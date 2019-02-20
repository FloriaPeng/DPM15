package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

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

  private SampleProvider myColorStatus; // The sample provider for the color sensor
  private float[] sampleColor; // The data buffer for the color sensor reading
  private Navigation navigation; // The instance of sensor rotation
  private SearchCan searchcan;

  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  public static final int FACING_CORNER = 225; // Angle facing the corner
  public static final int FULL_TURN = 360; // 360 degree for a circle
  private static final double SENSOR_TO_CENTER = 11; // The distance from the light sensor to the
                                                       // rotation sensor
  private static final double TURNING_ADJUSTMENT = 10; // The light localization adjustment
  private static final int BACK_DIST = 15; // Travel back distance

  double last = Math.PI; // Initialize the last variable to a specific number
  double current = 0; // last and current are both used for differential filter
  double[] detect = new double[4]; // The x and y tile line detect angle, clockwise
  double xerror = 0; // The localization error in the x direction
  double yerror = 0; // The localization error in the y direction
  double terror = 0; // The localization error in angle

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
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track,
      SampleProvider myColorStatus, float[] sampleColor, Navigation navigation, SearchCan searchcan)
      throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;

    this.myColorStatus = myColorStatus;
    this.sampleColor = sampleColor;
    this.navigation = navigation;
    this.searchcan = searchcan;
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
    navigation.goTo(TILE_SIZE * Math.sqrt(2), TILE_SIZE * Math.sqrt(2), 0); // TODO probably should change method

    while (rightMotor.isMoving() || leftMotor.isMoving()) {
      if (filter()) { // If the black line is detected, the robot will stop
        leftMotor.stop(true);
        rightMotor.stop(false);
      }
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }

    navigation.back(BACK_DIST, BACK_DIST); // And then go back a certain distance
    navigation.turn(FULL_TURN); // Then starts rotating clockwise

    for (int i = 0; i < 4;) { // It will record 4 angles
      if (filter()) {
        detect[i] = odometer.getXYT()[2];
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
    xerror = SENSOR_TO_CENTER * Math.cos(Math.toRadians((detect[3] - detect[1]) / 2));
    yerror = SENSOR_TO_CENTER * Math.cos(Math.toRadians((detect[2] - detect[0]) / 2));

    terror = 270 - (detect[3] + detect[1]) / 2;

    // Correcting the position of the robot
    odometer.position[2] = odometer.position[2] + terror;
    odometer.setTheta(odometer.position[2]);
    double turnAngle = 360 - detect[3];
    turnAngle += Math.toDegrees(Math.abs(Math.atan(xerror / yerror)));
    navigation.rotate(turnAngle);
    navigation.forward(xerror, yerror);
    navigation.rotate(-TURNING_ADJUSTMENT - Math.toDegrees(Math.abs(Math.atan(xerror / yerror))));

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

    // Thread scThread = new Thread(searchcan);
    // scThread.start();

  }

  /**
   * The differential filter of the light sensor, it will consider detecting a line if there is a
   * huge increase of the reading (the derivative if large)
   * 
   * @return - true for detecting an line, vice versa
   */
  boolean filter() { // Differential filter

    myColorStatus.fetchSample(sampleColor, 0); // Used for obtaining color reading from the
                                               // SampleProvider

    if (Math.abs(last - Math.PI) < Math.pow(0.1, 5)) { // If last has not been assigned for any
                                                       // number yet
      last = current = sampleColor[0];
    } else {
      last = current; // Update the last
      current = sampleColor[0]; // Update the current
    }

    if ((current - last) / 0.01 < -0.7) { // If there is a black line detected
      Sound.beepSequenceUp();
      return true;
    }
    return false;
  }

}
