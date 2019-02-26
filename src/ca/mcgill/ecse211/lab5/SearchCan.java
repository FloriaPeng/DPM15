package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;

/*
This class provides the functionality for the robot to search a given area.
The robot firstly travels to the given corner, and generates a map based on 
given corner. Then it starts travel through the map tile by tile.

###not understand why when it finds a target, it breaks the for loop and go back
*/

public class SearchCan implements Runnable {

  public static final int[] LOWER_LEFT = {1, 1}; // The lower-left corner of the search region [0,
                                                 // 8]
  public static final int[] UPPER_RIGHT = {4, 4}; // The upper-right corner of the search region [0,
                                                  // 8]
  public static final int TR = 3; // Blue, Green, Yellow, Red [1, 4] Target
  public static final int SC = 0; // [0, 3] Search Corner

  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  private static final int SLEEP_TIME = 1000; // Reach the lower-left time
  public static final int FULL_TURN = 360; // 360 degree for a circle

  private Odometer odometer;
  private Navigation navigation; // The instance of sensor rotation
  private ColorClassification colorclassification;

  double track; // The track of the robot
  double halfTrack; // The half track of the robot TODO
  int[][] map; // The search map

  //class constructor

  public SearchCan(double track, Odometer odometer, Navigation navigation,
      ColorClassification colorclassification) {
    this.track = track;
    this.halfTrack = track / 2;
    this.odometer = odometer;
    this.navigation = navigation;
    this.colorclassification = colorclassification;
  }

  public void run() {
    initialize(); // Navigate the robot to the Lower-Left corner of the search region
    searchMap(); // Once ready() is complete, the robot will generate search map
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

  private void initialize() {  //based on different search Corner, the robot travel to accoridng corner and get ready
    switch (SC) {
      case 0:
        navigation.travelTo(LOWER_LEFT[0] * TILE_SIZE, LOWER_LEFT[1] * TILE_SIZE); 
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

  private void ready() { //when it's ready at the according corner, the robot will beep once to indicate
    Sound.beep();
    try {
      Thread.sleep(SLEEP_TIME);
    } catch (InterruptedException e) {
    }
  }

  private void searchMap() {  //the robot will generate search map based on given corner
    int horizontal = UPPER_RIGHT[0] - LOWER_LEFT[0] + 1;  //set initial X origin
    int vertical = UPPER_RIGHT[1] - LOWER_LEFT[1] + 1; //set inital Y origin
    map = new int[horizontal * vertical][3]; //set up a 2D array of map
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
