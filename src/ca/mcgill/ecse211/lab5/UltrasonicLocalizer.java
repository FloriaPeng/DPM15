package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * @author Floria Peng
 * 
 *         This class implements the ultrasonic localization of the robot. There are two methods in
 *         this class: falling edge implementation and rising edge implementation. For the the
 *         falling edge the robot starts from not facing the wall, and the sensor first detect of
 *         from the back wall and then the left wall, and record the angle of the robot if the
 *         sensor detect a certain distance. The rising edge is basically the same except the robot
 *         starts from facing the wall.
 *
 */
public class UltrasonicLocalizer implements Runnable { // TODO missing comment

  public static final int INFINITY_DISTANCE = 50; // The distance that the sensor consider the robot
                                                  // is not facing the wall
  public static final int WALL_DISTANCE = 30; // The distance that the sensor consider the robot is
                                              // facing the wall
  public static boolean OPTION = true; // true for falling edge, false for rising edge
  public static final int FULL_TURN = 360; // 360 degree for a circle
  private static final int ACCELERATION = 3000; // The acceleration of the motor

  private TextLCD lcd; // The lcd display
  private Odometer odometer; // The odometer instance
  private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
  private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
  double leftRadius; // The left wheel radius of the robot
  double rightRadius; // The right wheel radius of the robot
  double track; // The track of the robot (by measuring the distance between the center of both
                // wheel)

  private SampleProvider us; // The sample provider for the ultrasonic sensor
  private float[] usData; // The data buffer for the ultrasonic sensor reading
  private Navigation navigation; // The instance of navigation
  private LightLocalizer lightlocalizer; // The instance of light localizer

  int d = 40; // An arbitrary distance that the robot record the angle (first/last below)
  int k = 1; // To eliminate the noise
  double first = 0; // The first angle that falls into the band (d+/-k)
  double last = 0; // The last angle that falls into the band (d+/-k)
  int count = 0; // +1 for having already obtained a first or last
  double alpha = 0; // The angle when detecting the back wall
  double beta = 0; // The angle when detecting the left wall
  double error = 0; // The angle error

  /**
   * The constructor of this class.
   * 
   * @param odometer
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param track
   * @param us
   * @param usData
   * @param navigation
   * @param lightlocalizer
   * @param lcd
   * @throws OdometerExceptions
   */
  public UltrasonicLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track,
      SampleProvider us, float[] usData, Navigation navigation, LightLocalizer lightlocalizer,
      TextLCD lcd) throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;

    this.us = us;
    this.usData = usData;
    this.navigation = navigation;
    this.lightlocalizer = lightlocalizer;
    this.lcd = lcd;
  }

  /**
   * The run method of this class. It will rotate the robot to a proper angle (facing/not facing the
   * wall), and then start the falling edge or rising edge method according to the user choice.
   * 
   * @see java.lang.Runnable#run()
   */
  public void run() {
    if (OPTION) { // Falling edge
      navigation.turn(FULL_TURN); // The robot will rotate clockwise for a full turn until disrupted
      while (rightMotor.isMoving() || leftMotor.isMoving()) {
        if (filter() > INFINITY_DISTANCE) { // If the robot is not facing the wall
          leftMotor.setAcceleration(ACCELERATION);
          rightMotor.setAcceleration(ACCELERATION);
          leftMotor.stop(true); // Stop the motors
          rightMotor.stop(false);
          Sound.beepSequenceUp();
          break;
        }
        try {
          Thread.sleep(50);
        } catch (Exception e) {
        }
      }
      fallingEdge(); // Call the fallingEdge method
    } else { // Rising edge
      navigation.turn(-FULL_TURN); // The robot will rotate counter-clockwise for a full turn until
                                   // disrupted
      while (rightMotor.isMoving() || leftMotor.isMoving()) {
        if (filter() < WALL_DISTANCE) { // If the robot is facing the wall
          leftMotor.setAcceleration(ACCELERATION);
          rightMotor.setAcceleration(ACCELERATION);
          leftMotor.stop(true); // Stop the motors
          rightMotor.stop(false);
          Sound.beepSequenceUp();
          break;
        }
        try {
          Thread.sleep(50);
        } catch (Exception e) {
        }
      }
      risingEdge(); // Call the risingEdge method
    }

    navigation.turnTo(-error); // Correct the angle of the robot
    odometer.position[2] = 0; // Reset the angle of the odometer
    odometer.setTheta(0); // Reset the angle of the odometerData

    lcd.drawString("Waiting for a press", 0, 3); // Show the user the program is waiting for input
                                                 // to continue
    Button.waitForAnyPress();
    lcd.clear(3); // Clear the sentence

    // Call light sensor
    Thread lightThread = new Thread(lightlocalizer);
    lightThread.start();
  }

  /**
   * The fallingEdge method that will continuously call the fallingDetect to detect the alpha and
   * beta angle the robot needs to correct the angle.
   */
  void fallingEdge() {

    // Detect back wall
    navigation.turn(FULL_TURN); // The robot will rotate counter-clockwise for a full turn until
                                // disrupted
    fallingDetect(); // Calling the fallingDetect method
    leftMotor.stop(true); // Stop the motors
    rightMotor.stop(false);
    alpha = (first + last) / 2;

    navigation.rotate(-FULL_TURN / 4); // The robot will rotate 90 degrees first before the next
    // detect, to avoid detecting the same wall twice

    // Detect left wall
    navigation.turn(-FULL_TURN);
    fallingDetect();
    leftMotor.stop(true);
    rightMotor.stop(false);
    beta = (first + last) / 2;

    error = 225 - (alpha + beta) / 2; // The angle error to correct
  }

  /**
   * The fallingEdge method that will continuously call the risingDetect to detect the alpha and
   * beta angle the robot needs to correct the angle.
   */
  void risingEdge() { // Same as fallingEdge, please refer to the fallingEdge method

    // Detect back wall
    navigation.turn(-FULL_TURN);
    risingDetect();
    leftMotor.stop(true);
    rightMotor.stop(false);
    alpha = (first + last) / 2;

    navigation.turn(FULL_TURN / 4);
    while (navigation.isNavigating()) {
      // Wait for completion
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }

    // Detect left wall
    navigation.turn(FULL_TURN);
    risingDetect();
    leftMotor.stop(true);
    rightMotor.stop(false);
    beta = (first + last) / 2;

    error = 45 - (alpha + beta) / 2;
  }

  /**
   * The detect for the falling edge (as the sensor is approaching the wall) The method that will
   * record first and last angle used for calculating the alpha and beta
   */
  void fallingDetect() {
    count = 0;
    while (count < 2) {
      if (filter() < d - k && count == 1) {
        last = odometer.getXYT()[2];
        count++;
      } else if (filter() < d + k && count == 0) {
        first = odometer.getXYT()[2];
        count++;
      }
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }
  }

  /**
   * The detect for the rising edge (as the sensor is turning away from the wall) The method that
   * will record first and last angle used for calculating the alpha and beta
   */
  void risingDetect() {
    count = 0;
    while (count < 2) {
      if (filter() > d + k && count == 1) {
        last = odometer.getXYT()[2];
        count++;
      } else if (filter() > d - k && count == 0) {
        first = odometer.getXYT()[2];
        count++;
      }
      try {
        Thread.sleep(50);
      } catch (Exception e) {
      }
    }
  }

  /**
   * The median filter of the distance detected to ignore the noice
   * 
   * @return
   */
  double filter() { // TODO debug
    double[] arr = new double[5]; // store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      us.fetchSample(usData, 0); // store reading in buffer
      arr[i] = usData[0] * 100.0; // signal amplification
    }
    Arrays.sort(arr); // sort readings
    return arr[2]; // take median value
  }

}
