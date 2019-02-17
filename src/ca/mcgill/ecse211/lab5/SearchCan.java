package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import lejos.robotics.SampleProvider;

public class SearchCan implements Runnable {

  public static final double TILE_SIZE = 30.48; // The tile size used for demo
  
  // The mean value of the normal plot for blue, green, yellow, red
  public static final float[] MEAN_BLUE = {0, 0, 0}; // R, G, B
  public static final float[] MEAN_GREEN = {0, 0, 0};
  public static final float[] MEAN_YELLOW = {0, 0, 0};
  public static final float[] MEAN_RED = {0, 0, 0};
  // The standard deviation of the normal plot for blue, green, yellow, red
  public static final float[] STD_BLUE = {0, 0, 0};
  public static final float[] STD_GREEN = {0, 0, 0};
  public static final float[] STD_YELLOW = {0, 0, 0};
  public static final float[] STD_RED = {0, 0, 0};

  public static final int PROPER_DISTANCE = 0; // The proper distance between the color sensor and
                                               // the can, so that the reading is valid

  private static SampleProvider myUSStatus; // The sample provider for the ultrasonic sensor
  private static float[] sampleUS; // The data buffer for the ultrasonic sensor reading
  
  private static SampleProvider colorReading; // The sample provider for the color sensor
  private static float[] colorData; // The data buffer for the color sensor reading
  
  private Navigation navigation;

  static double color;

  // 1. The ultrasonic will continue rotating all the time, report if there is a can in front of it
  // 2. If the ultrasonic sensor report a can, turn the robot 90 degrees to make the light sensor
  // facing the can
  // 3. The light sensor will keep reading the RGB value from the can while the robot is going
  // around the can for 180 degrees
  // 4. Use the data collected by the light sensor to determine if it is the correct can being
  // scanned
  // 5. Go back for a length of the a (TODO test for an appropriate a), turn right for 45 degree, go
  // for squareRoot(2) * a of a robot and turn 45 degree right
  // 6. Continue the previous route.

  public SearchCan(SampleProvider myUSStatus, float[] sampleUS, SampleProvider colorReading, float[] colorData, Navigation navigation) {
    SearchCan.myUSStatus = myUSStatus;
    SearchCan.sampleUS = sampleUS;
    SearchCan.colorReading = colorReading;
    SearchCan.colorData = colorData;
    
    this.navigation = navigation;
  }

  public void run() {
    navigation.goTo(Lab5.UPPER_RIGHT[0] * TILE_SIZE, Lab5.LOWER_LEFT[1] * TILE_SIZE); // 1
    navigation.goTo(Lab5.UPPER_RIGHT[0] * TILE_SIZE, Lab5.UPPER_RIGHT[1] * TILE_SIZE); // 2
    navigation.goTo(Lab5.LOWER_LEFT[0] * TILE_SIZE, Lab5.UPPER_RIGHT[1] * TILE_SIZE); // 3
  }

  static boolean colorDetect(int colorID) {

    float[] target_mean = {0, 0, 0};
    float[] target_std = {0, 0, 0};

    switch (colorID) {
      case 1:
        target_mean = MEAN_BLUE;
        target_std = STD_BLUE;
        break;
      case 2:
        target_mean = MEAN_GREEN;
        target_std = STD_GREEN;
        break;
      case 3:
        target_mean = MEAN_YELLOW;
        target_std = STD_YELLOW;
        break;
      case 4:
        target_mean = MEAN_RED;
        target_std = STD_RED;
        break;
    }
    float[] reading = mean_filter();
    color = Math.sqrt(Math.pow((reading[0] - target_mean[0]) / target_std[0], 2)
        + Math.pow((reading[1] - target_mean[1]) / target_std[1], 2)
        + Math.pow((reading[2] - target_mean[2]) / target_std[2], 2)); // TODO
    if (color < Math.sqrt(3)) {
      return true;
    } else {
      return false;
    }
    
  }
  
  /**
   * The median filter of the distance detected to ignore the noice
   * 
   * @return
   */
  static double median_filter() { // TODO debug
    double[] arr = new double[5]; // store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      myUSStatus.fetchSample(sampleUS, 0); // store reading in buffer
      arr[i] = sampleUS[0] * 100.0; // signal amplification
    }
    Arrays.sort(arr); // sort readings
    return arr[2]; // take median value
  }
  
  static float[] mean_filter() { // TODO debug
    float[][] arr = new float[5][3]; // store readings
    float[] RGB = {0, 0, 0};
    for (int i = 0; i < 5; i++) { // take 5 readings
      colorReading.fetchSample(colorData, 0); // store reading in buffer
      arr[i] = colorData; // signal amplification
    }
    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 3; j++) {
        RGB[j] += arr[i][i] / 5;
      }
    }
    return RGB; // take median value
  }

}
