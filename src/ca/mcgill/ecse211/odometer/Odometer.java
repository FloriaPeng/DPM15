/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author Floria Peng
 *
 *         This class implements the odometer of the robot. (Completed during Lab2) This class
 *         enables the robot to get its location related to the convention of the x and y
 *         coordinates
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  public double[] position = new double[3];

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   * 
   * @param position[2] - The angle of the robot
   * @param lastleftMotorTachoCount - The last reading of the left motor angle
   * @param lastrightMotorTachoCount - The last reading of the right motor angle
   * 
   * @return - void method, no return
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
    int lastleftMotorTachoCount = 0; // Save the last angle of the left motor
    int lastrightMotorTachoCount = 0; // Same the last angle of the right motor

    while (true) {
      updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      // TODO Calculate new robot position based on tachometer counts
      double dx, dy, dl, dr, dc, dtheta; // change in x, change in y, change in left motor
                                         // change in right motor, change in the center of the motor
                                         // change in angle of the robot

      dl = Math.PI / 180 * WHEEL_RAD * (leftMotorTachoCount - lastleftMotorTachoCount);
      // The distance the left wheel goes
      dr = Math.PI / 180 * WHEEL_RAD * (rightMotorTachoCount - lastrightMotorTachoCount);
      // The distance the right wheel goes

      lastleftMotorTachoCount = leftMotorTachoCount; // Update the saved left motor degree
      lastrightMotorTachoCount = rightMotorTachoCount; // Update the saved right motor degree

      dc = (dl + dr) * 0.5; // The distance change of the center (the middle point)
      dtheta = (dl - dr) / TRACK; // The change in theta

      position[2] = position[2] + dtheta; // Update the new theta
      dx = dc * Math.sin(position[2]); // Change in x calculated by sin
      dy = dc * Math.cos(position[2]); // Change in y calculated by cos

      // TODO Update odometer values with new calculated values
      odo.update(dx, dy, Math.toDegrees(dtheta)); // Update and store the calculated value to the
                                                  // OdometerData

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
