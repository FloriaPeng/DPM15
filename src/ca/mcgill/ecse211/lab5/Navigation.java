package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * @author Floria Peng
 * 
 *         This class contains the basic method for the navigation. All the movements of the robot
 *         is controlled by the this class. The methods will be called by UltrasonicLocalizer or
 *         LightLocalizer.
 *
 */
public class Navigation {

  private Odometer odometer; // The odometer instance
  public static final int FORWARD_SPEED = 180; // The forward speed for the robot
  public static final int ROTATE_SPEED = 100; // The rotation speed for the robot
  private static final int ACCELERATION = 3000; // The acceleration of the motor
  private static final double SCAN_DISTANCE = 7; // The detect a can distance TODO
  public static final int FULL_TURN = 360; // 360 degree for a circle
  private static final double PREPARE_SQUARE = 30.48 / 2; // TODO
  private static final double SQUARE_LENGTH = 30.48 / 2; // TODO

  private LineCorrection linecorrection; // The instance of line correction
  private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
  private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
  private EV3MediumRegulatedMotor sensorMotor; // The sensor motor of the robot
  private ColorClassification colorclassification; // The ColorClassification instance
  double leftRadius; // The left wheel radius of the robot
  double rightRadius; // The right wheel radius of the robot
  double track; // The track of the robot (by measuring the distance between the center of both
                // wheel)

  double lastx; // The last x position of the robot
  double lasty; // The last y position of the robot
  double lasttheta; // The last angle of the robot
  double travel; // The traveling distance between the current point with the next map point
  double distance; // The distance calculated to go backs
  double angle; // The angle to which the robot should turn to get to the next map point
  double theta; // The angle that the robot should actually rotate
  double warning; // The front ultrasonic sensor distance detected
  double enter; // The entering avoidance angle
  boolean isNavigating = false; // True for the robot is traveling to next point

  long[] time = new long[2]; // The time of the light sensor
  boolean[] line = {false, false}; // The detection of the line of the two light sensors
  boolean corrected = false;
  double before = 0;
  int flag = 0;

  /**
   * The constructor for the Navigation class
   * 
   * @param odometer - The odometer of the robot
   * @param leftMotor - The leftMotor of the robot
   * @param rightMotor - The rightMotor of the robot
   * @param leftRadius - The left wheel radius of the robot
   * @param rightRadius - The right wheel radius of the robot
   * @param track - The track of the robot measured form the distance between the center of both
   *        wheels
   * 
   * @throws OdometerExceptions
   */
  public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, EV3MediumRegulatedMotor sensorMotor,
      ColorClassification colorclassification, LineCorrection linecorrection, double leftRadius,
      double rightRadius, double track) throws OdometerExceptions {
    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.sensorMotor = sensorMotor;
    this.colorclassification = colorclassification;
    this.linecorrection = linecorrection;
    this.leftRadius = leftRadius;
    this.rightRadius = rightRadius;
    this.track = track;
  }

  /**
   * <p>
   * This method causes the robot to travel to the absolute field location (x, y), specified in tile
   * points. This method should continuously call turnTo(double theta) and then set the motor speed
   * to forward(straight). This will make sure that your heading is updated until you reach your
   * exact goal. This method will poll the odometer for information.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x coordinate for the next point
   * @param y - The y coordinate for the next point
   * 
   * @return - void method, no return
   */
  void travelTo(double x, double y) {

    lastx = odometer.getXYT()[0]; // The last x position of the robot
    lasty = odometer.getXYT()[1]; // The last y position of the robot

    travel = Math.sqrt(Math.pow(x - lastx, 2) + Math.pow(y - lasty, 2)); // The travel distance
    angle = Math.atan2(x - lastx, y - lasty) * 180 / Math.PI; // The angle that the robot should
                                                              // rotate to

    turnTo(angle); // Call the turnTo method

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // Travel the robot to the destination point
    leftMotor.rotate(convertDistance(leftRadius, travel), true);
    rightMotor.rotate(convertDistance(rightRadius, travel), false);

  }

  /**
   * <p>
   * This method causes the robot to travel to the absolute field location (x, y), specified in tile
   * points. This method should continuously call turnTo(double theta) and then set the motor speed
   * to forward(straight). This will make sure that your heading is updated until you reach your
   * exact goal. This method will poll the odometer for information.
   * 
   * <p>
   * This method can be break
   * 
   * @param x - The x coordinate for the next point
   * @param y - The y coordinate for the next point
   * 
   * @return - void method, no return
   */
  void goTo(double x, double y, int position) {

    if (flag == 0) {
      corrected = false;
    }

    lastx = odometer.getXYT()[0]; // The last x position of the robot
    lasty = odometer.getXYT()[1]; // The last y position of the robot

    travel = Math.sqrt(Math.pow(x - lastx, 2) + Math.pow(y - lasty, 2)); // The travel distance
    angle = Math.atan2(x - lastx, y - lasty) * 180 / Math.PI; // The angle that the robot should
                                                              // rotate to

    turnTo(angle); // Call the turnTo method

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    // Travel the robot to the destination point
    leftMotor.rotate(convertDistance(leftRadius, travel), true);
    rightMotor.rotate(convertDistance(rightRadius, travel), true);

    while (leftMotor.isMoving() || rightMotor.isMoving()) { // If the robot is moving

      if (!corrected) {
        correctAngle(x, y, position);
        flag = 1;
      }
      
      warning = colorclassification.median_filter();
      if (warning < SCAN_DISTANCE) { // TODO
        Sound.beepSequenceUp();

        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.stop(true);
        rightMotor.stop(false);

        move(3);

        Thread classificationThread = new Thread(colorclassification);
        classificationThread.start();
        sensorMotor.setSpeed(ROTATE_SPEED / 6);
        sensorMotor.rotate(FULL_TURN - 10, true);
        while (sensorMotor.isMoving()) {
          try {
            Thread.sleep(50);
          } catch (Exception e) {
          }
        }
        colorclassification.stop = true;
        try {
          classificationThread.join();
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        if (colorclassification.color == SearchCan.TR) {
          colorclassification.found = true;
          Sound.beep();
          sensorMotor.setSpeed(ROTATE_SPEED);
          sensorMotor.rotate(-FULL_TURN, false);
          return;
        } else {
          Sound.twoBeeps();
          sensorMotor.setSpeed(ROTATE_SPEED);
          sensorMotor.rotate(-FULL_TURN + 10, false);
          canAvoidance(position);
        }
      }
    }

  }

  void correctAngle(double x, double y, int position) {
    if (linecorrection.filter1()) { // If the black line is detected, the robot will stop
      time[0] = System.currentTimeMillis();
      line[0] = true;
    }
    if (linecorrection.filter2()) {
      time[1] = System.currentTimeMillis();
      line[1] = true;
    }
    
    if (line[0] && line[1]) {
      line[0] = false;
      line[1] = false;
      leftMotor.stop(true);
      rightMotor.stop(false);
      double dtheta = Math.atan(((time[1] - time[0]) * FORWARD_SPEED * Math.PI * leftRadius) / (track * 180));
      before = odometer.getXYT()[2];
      rotate(-dtheta);
      odometer.setTheta(before);
      odometer.position[2] = before;
      corrected = true;
      try {
        Thread.sleep(5000);
      } catch (Exception e) {
      }
      goTo(x, y, position);
    }
  }

  void canAvoidance(int position) {
    back((PREPARE_SQUARE - 2.9), 0); // diameter of can = 5.8cm
    if (position == 0) { // right side can
      rotate(-90);
      forward(SQUARE_LENGTH, 0);
      rotate(90);
      forward(SQUARE_LENGTH * 2 + 2.9, 0);
      rotate(-90);
    } else if (position == 1) { // left side can
      rotate(90);
      forward(SQUARE_LENGTH, 0);
      rotate(-90);
      forward(SQUARE_LENGTH * 2 + 2.9, 0);
      rotate(90);
    } else if (position == 2) { // straight line can
      rotate(90);
      forward(SQUARE_LENGTH, 0);
      rotate(-90);
      forward(SQUARE_LENGTH * 2 + 2.9, 0);
      rotate(-90);
      forward(SQUARE_LENGTH, 0);
      rotate(90);
    }
  }

  void move(double distance) {

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), true);

  }

  /**
   * <p>
   * This method is the forward method of the robot. The forward distance is calculated by the x and
   * y parameter passed to this method (Euclidean distance).
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x distance the robot should move
   * @param y - The y distance the robot should move
   */
  void forward(double x, double y) {

    distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // The travel distance
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(leftRadius, distance), true);
    rightMotor.rotate(convertDistance(rightRadius, distance), false);

  }

  /**
   * <p>
   * This method is the back method of the robot. The forward distance is calculated by the x and y
   * parameter passed to this method (Euclidean distance). And the robot will travel back this
   * distance.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param x - The x distance the robot should move
   * @param y - The y distance the robot should move
   */
  void back(double x, double y) {

    distance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // The travel distance

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(-convertDistance(leftRadius, distance), true);
    rightMotor.rotate(-convertDistance(rightRadius, distance), false);

  }

  /**
   * <p>
   * This method causes the robot to turn (on point) to the absolute heading theta. This method
   * should turn a MINIMAL angle to its target.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param theta - The angle that the robot should rotate to
   */
  void turnTo(double angle) {

    lasttheta = odometer.getXYT()[2]; // Update the last theta of the robot
    theta = angle - lasttheta; // The angle that the robot should actually rotate

    if (theta > 180) { // Convert the angle from MAXIMAL to MINIMAL
      theta = -(360 - theta);
    }
    if (theta < -180) { // Convert the angle from MAXIMAL to MINIMAL
      theta = 360 + theta;
    }

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);

  }

  /**
   * <p>
   * This method turns the robot an angle passed by the method call. And the method can be
   * interrupted if the caller decide the turning angle is enough.
   * 
   * <p>
   * This method can be break
   * 
   * @param theta - The angle to turn
   */
  void turn(double theta) {

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), true); // The true is to ensure the
                                                                       // method can be interrupted.

  }

  /**
   * <p>
   * This method rotates the robot an angle passed by the method call. And the method cannot be
   * interrupted and the robot will make sure to finish rotating the angle.
   * 
   * <p>
   * This method cannot be break
   * 
   * @param theta - The angle to turn
   */
  void rotate(double theta) {

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);
    // The false is to ensure the rotation finish before continuing.

  }

  /**
   * This method returns true if another thread has called travelTo(), forward(), back(), turn(),
   * rotate() or turnTo() and the method has yet to return; false otherwise.
   * 
   * @return
   */
  boolean isNavigating() {

    if (leftMotor.isMoving() || rightMotor.isMoving()) {
      return true;
    } else {
      return false;
    }

  }

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * @param radius
   * @param width
   * @param angle
   * @return
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

}
