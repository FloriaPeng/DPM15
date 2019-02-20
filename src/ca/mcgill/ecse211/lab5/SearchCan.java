package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;

public class SearchCan implements Runnable { // TODO missing comment

  public static final int[] LOWER_LEFT = {1, 1}; // The lower-left corner of the search region [0,
                                                 // 8]
  public static final int[] UPPER_RIGHT = {5, 5}; // The upper-right corner of the search region [0,
                                                  // 8]
  public static final int TR = 1; // Blue, Green, Yellow, Red [1, 4]
  public static final int SC = 0; // [0, 3]

  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  private static final int SLEEP_TIME = 1000; // Reach the lower-left time
  public static final int FULL_TURN = 360; // 360 degree for a circle

  private Odometer odometer;
  private Navigation navigation; // The instance of sensor rotation
  private ColorClassification colorclassification;

  double track; // The track of the robot
  double halfTrack; // The half track of the robot TODO
  int[][] map; // The search map

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
    searchMap(); // Generate search map
    int x, y, position;
    for (int i = 1; i < map.length; i++) {
      x = map[i][0];
      y = map[i][1];
      position = map[i][2];
      navigation.goTo(x, y, position);
      if (colorclassification.found) {
        break;
      }
    }
    if (colorclassification.found) {
      navigation.rotate(FULL_TURN / 4);
      navigation.forward(0, halfTrack);
      navigation.travelTo(odometer.getXYT()[0], UPPER_RIGHT[1] + halfTrack);
      navigation.travelTo(UPPER_RIGHT[0], UPPER_RIGHT[1] + halfTrack);
      navigation.travelTo(UPPER_RIGHT[0], UPPER_RIGHT[1]);
    } else if ((UPPER_RIGHT[1] - LOWER_LEFT[1] + 1) % 2 == 0) {
      if (Math.abs(odometer.getXYT()[2] - FULL_TURN * 3 / 4) < (FULL_TURN / 8)) {
        navigation.rotate(FULL_TURN / 4);
        navigation.forward(0, halfTrack);
      }
      navigation.travelTo(UPPER_RIGHT[0], UPPER_RIGHT[1] + halfTrack);
      navigation.travelTo(UPPER_RIGHT[0], UPPER_RIGHT[1]);
    }
  }

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

  private void ready() {
    navigation.turnTo(45);
    Sound.beep();
    sleep(SLEEP_TIME);
  }

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
        map[i][2] = 2;
      } else if (i % (2 * horizontal) == 2 * horizontal - 1 || i % (2 * horizontal) == 0) {
        map[i][2] = 4;
      } else {
        map[i][2] = 5;
      }
    }
  }

  private void sleep(int time) {
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
    }
  }

}
