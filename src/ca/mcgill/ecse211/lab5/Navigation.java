package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
  private static final int FORWARD_SPEED = 250; // The forward speed for the robot
  private static final int ROTATE_SPEED = 150; // The rotation speed for the robot
  private static final int ACCELERATION = 300; // The acceleration of the motor
  public static final int FULL_TURN = 360; // 360 degree for a circle

  private EV3LargeRegulatedMotor leftMotor; // The left motor of the robot
  private EV3LargeRegulatedMotor rightMotor; // The right motor of the robot
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
  void goTo(double x, double y) {

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
    
    enter = 0;
    while (leftMotor.isMoving() || rightMotor.isMoving()) { // If the robot is moving
      warning = UltrasonicLocalizer.filter();
      if (warning < 10) {
        enter = odometer.getXYT()[2];
        
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.stop(true);
        rightMotor.stop(false);
        
        // goAround();
      }
    }

  }
  
  void goAround() {
    rotate(FULL_TURN / 4);
    leftMotor.setSpeed(ROTATE_SPEED / 2); // Slow down left motor
    rightMotor.setSpeed(FORWARD_SPEED / 2); // Speed up right motor
    leftMotor.forward();
    rightMotor.forward();
    if (Math.abs(Math.abs(odometer.getXYT()[2] - enter) - 180) < 5) {
      return;
    }
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
  void forword(double x, double y) {

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
    rightMotor.rotate(-convertAngle(rightRadius, track, theta), false); // The false is to ensure
                                                                        // the rotation finish
                                                                        // before continuing.

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
