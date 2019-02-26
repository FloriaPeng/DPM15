package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;

/**
 * @author Floria Peng
 * 
 *         This class contains the basic method for the searching the can. The map is interpreted in this class and calls navigation class to perform the move.  
 */

public class SearchCan implements Runnable {
  
  /*
   * Constant (Settings)
   */

  public static final int[] LOWER_LEFT = {1, 1}; // The lower-left corner of the search region [0,
                                                 // 8]
  public static final int[] UPPER_RIGHT = {4, 4}; // The upper-right corner of the search region [0,
                                                  // 8]
  public static final int TR = 3; // Blue, Green, Yellow, Red [1, 4]
  public static final int SC = 0; // [0, 3]

  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  private static final int SLEEP_TIME = 1000; // Reach the lower-left time
  public static final int FULL_TURN = 360; // 360 degree for a circle

  /*
   * Instances
   */
  private Odometer odometer;
  private Navigation navigation; // The instance of sensor rotation
  private ColorClassification colorclassification;

  /*
   * Variables
   */
  double track; // The track of the robot
  double halfTrack; // The half track of the robot TODO
  int[][] map; // The search map

  /**
   * This is a constructor for SearchCan class, called by Lab5.java after localization is complete. 
   * @param track
   * @param odometer
   * @param navigation
   * @param colorclassification
   */
  public SearchCan(double track, Odometer odometer, Navigation navigation,
      ColorClassification colorclassification) {
    this.track = track;
    this.halfTrack = track / 2;
    this.odometer = odometer;
    this.navigation = navigation;
    this.colorclassification = colorclassification;
  }

  /**
   * The run method of this class. It calls searchMap() to generate the map, interprets the map and calls navigation to perform the move. 
   * initialize() is called before performing the search to move the robot to the starting point. 
   * 
   * @see java.lang.Runnable#run()
   */
  public void run() {
    initialize(); // Navigate the robot to the Lower-Left corner of the search region
    searchMap(); // Generate search map
    int x, y, position;
    for (int i = 1; i < map.length; i++) {
      x = map[i][0];
      y = map[i][1];
      position = map[i][2];
      System.out.println("from x = " + odometer.getXYT()[0] + " y = " + odometer.getXYT()[1] + " T = " + odometer.getXYT()[2]);
      System.out.println("to x " + x * TILE_SIZE + " y = " + y * TILE_SIZE);
      navigation.goTo(x * TILE_SIZE, y * TILE_SIZE, position);
      navigation.flag = 0;
      if (colorclassification.found) {
        break;
      }
    }
    navigation.back(TILE_SIZE / 3, 0);
    navigation.travelTo(UPPER_RIGHT[0] * TILE_SIZE, UPPER_RIGHT[1] * TILE_SIZE);
  }

  /**
   * This method let the robot to move to the starting point of the map. 
   * It depends on the variable SC to determine where is the destination. 
   */
  private void initialize() {
    switch (SC) {
      case 0:
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE, LOWER_LEFT[1] * TILE_SIZE); // 0
        ready();
        break;
      case 1:
        navigation.travelTo(UPPER_RIGHT[0] * TILE_SIZE + halfTrack,
            LOWER_LEFT[1] * TILE_SIZE - halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE - halfTrack,
            LOWER_LEFT[1] * TILE_SIZE - halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE, LOWER_LEFT[1] * TILE_SIZE);
        ready();
        break;
      case 2:
        navigation.travelTo(UPPER_RIGHT[0] * TILE_SIZE + halfTrack,
            UPPER_RIGHT[1] * TILE_SIZE + halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE - halfTrack,
            UPPER_RIGHT[1] * TILE_SIZE + halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE - halfTrack,
            LOWER_LEFT[1] * TILE_SIZE - halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE, LOWER_LEFT[1] * TILE_SIZE);
        ready();
        break;
      case 3:
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE - halfTrack,
            UPPER_RIGHT[1] * TILE_SIZE + halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE - halfTrack,
            UPPER_RIGHT[1] * TILE_SIZE - halfTrack);
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE, LOWER_LEFT[1] * TILE_SIZE);
        ready();
        break;
    }
  }

  /*
   * This method triggers beep. 
   */
  private void ready() {
    Sound.beep();
    try {
      Thread.sleep(SLEEP_TIME);
    } catch (InterruptedException e) {
    }
  }

  /*
   * This method interprets the input and generate the map to search. 
   * Used from run() method. 
   */
  private void searchMap() {
    int horizontal = UPPER_RIGHT[0] - LOWER_LEFT[0] + 1;
    int vertical = UPPER_RIGHT[1] - LOWER_LEFT[1] + 1;
    map = new int[horizontal * vertical][3];
    int direction = 1;
    for (int i = 0; i < vertical; i++) {
      for (int j = 0; j < horizontal; j++) {
        if (direction == 1) {
          map[i * horizontal + j][0] = LOWER_LEFT[0] + j;
          map[i * horizontal + j][1] = LOWER_LEFT[1] + i;
        } else {
          map[i * horizontal + j][0] = UPPER_RIGHT[0] - j;
          map[i * horizontal + j][1] = LOWER_LEFT[1] + i;
        }
      }
      direction *= -1;
    }
    for (int i = 0; i < map.length; i++) {
      if (i % (2 * horizontal) == horizontal - 1 || i % (2 * horizontal) == horizontal) {
        // right side can
        map[i][2] = 0;
      } else if (i % (2 * horizontal) == 2 * horizontal - 1 || i % (2 * horizontal) == 0) {
        // left side can
        map[i][2] = 1;
      } else { // straight line can
        map[i][2] = 2;
      }
    }
  }

}
