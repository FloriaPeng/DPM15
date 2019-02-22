package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class ColorClassification implements Runnable { // TODO missing comment

  // The mean value of the normal plot for blue, green, yellow, red
  // R, G, B
  public static final float[] MEAN_BLUE = {(float) 0.005526, (float) 0.013250, (float) 0.010755};
  public static final float[] MEAN_GREEN = {(float) 0.003129, (float) 0.010527, (float) 0.003218};
  public static final float[] MEAN_YELLOW = {(float) 0.037928, (float) 0.022688, (float) 0.005595};
  public static final float[] MEAN_RED = {(float) 0.018667, (float) 0.002129, (float) 0.001089};
  // The standard deviation of the normal plot for blue, green, yellow, red
  public static final float[] STD_BLUE = {(float) 0.000585, (float) 0.000567, (float) 0.000649};
  public static final float[] STD_GREEN = {(float) 0.000518, (float) 0.000477, (float) 0.000561};
  public static final float[] STD_YELLOW = {(float) 0.000621, (float) 0.000672, (float) 0.000744};
  public static final float[] STD_RED = {(float) 0.001196, (float) 0.000578, (float) 0.000609};

  // The mean value of the normal plot for blue, green, yellow, red
  // R, G, B
  public static final float[] MEAN_BLUE_HAT = {(float) 0.308062, (float) 0.738658, (float) 0.599568};
  public static final float[] MEAN_GREEN_HAT = {(float) 0.273420, (float) 0.919875, (float) 0.281197};
  public static final float[] MEAN_YELLOW_HAT = {(float) 0.851384, (float) 0.509286, (float) 0.125593};
  public static final float[] MEAN_RED_HAT = {(float) 0.991894, (float) 0.113127, (float) 0.057865};
  // The standard deviation of the normal plot for blue, green, yellow, red
  public static final float[] STD_BLUE_HAT = {(float) 0.561640, (float) 0.544359, (float) 0.623084};
  public static final float[] STD_GREEN_HAT = {(float) 0.575352, (float) 0.529812, (float) 0.623113};
  public static final float[] STD_YELLOW_HAT = {(float) 0.526581, (float) 0.569827, (float) 0.630880};
  public static final float[] STD_RED_HAT = {(float) 0.818453, (float) 0.395540, (float) 0.416754};

  public static final int SCAN_DISTANCE = 7; // The proper distance between the color sensor and
                                             // the can, so that the reading is valid TODO

  private SampleProvider usDistance; // The sample provider for the ultrasonic sensor
  private float[] usData; // The data buffer for the ultrasonic sensor reading

  private SampleProvider colorReading; // The sample provider for the color sensor
  private float[] colorData; // The data buffer for the color sensor reading

  int detected = 0; // +1 if target color is detected
  boolean notfound = false; // false for this can is not the target can
  boolean found = false; // true for target color found

  public ColorClassification(SampleProvider usDistance, float[] usData, SampleProvider colorReading,
      float[] colorData) {
    this.usDistance = usDistance;
    this.usData = usData;
    this.colorReading = colorReading;
    this.colorData = colorData;
  }

  public void run() {
    detected = 0;
    while (true) {
      if (Math.abs(median_filter() - SCAN_DISTANCE) < 0.5) {
        LCD.clear();
        LCD.drawString("R: " + mean_filter()[0], 0, 0);
        LCD.drawString("G: " + mean_filter()[1], 0, 1);
        LCD.drawString("B: " + mean_filter()[2], 0, 2);
        
        if (colorDetect(SearchCan.TR)) {
          detected++;
        }
      }
      if (detected > 2) {
        found = true;
        Sound.beep();
        break;
      } else if (notfound) {
        Sound.twoBeeps();
        break;
      }
    }
  }

  boolean colorDetect(int colorID) {

    float[] target_mean = {0, 0, 0};
    float[] target_std = {(float)0.2, (float)0.2, (float)0.2};

    switch (colorID) {
      case 1:
        target_mean = MEAN_BLUE_HAT;
        // target_std = STD_BLUE_HAT;
        break;
      case 2:
        target_mean = MEAN_GREEN_HAT;
        // target_std = STD_GREEN_HAT;
        break;
      case 3:
        target_mean = MEAN_YELLOW_HAT;
        // target_std = STD_YELLOW_HAT;
        break;
      case 4:
        target_mean = MEAN_RED_HAT;
        // target_std = STD_RED_HAT;
        break;
    }
    float[] reading = mean_filter();
    for (int i = 0; i < 3; i++) {
      reading[i] /= Math.sqrt(Math.pow(reading[0], 2) + Math.pow(reading[1], 2) + Math.pow(reading[2], 2));
    }
    if (Math.abs(reading[0] - target_mean[0]) < target_std[0]
        && Math.abs(reading[1] - target_mean[1]) < target_std[1]
        && Math.abs(reading[2] - target_mean[2]) < target_std[2]) {
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
  double median_filter() { // The distance filter TODO debug
    double[] arr = new double[5]; // store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      usDistance.fetchSample(usData, 0); // store reading in buffer
      arr[i] = usData[0] * 100.0; // signal amplification
    }
    Arrays.sort(arr); // sort readings
    return arr[2]; // take median value
  }

  float[] mean_filter() { // The color filter TODO debug
    float[][] arr = new float[5][3]; // store readings
    float[] RGB = {0, 0, 0};
    for (int i = 0; i < 5; i++) { // take 5 readings
      colorReading.fetchSample(colorData, 0); // store reading in buffer
      arr[i] = colorData; // signal amplification
    }
    for (int i = 0; i < 5; i++) {
      for (int j = 0; j < 3; j++) {
        RGB[j] += arr[i][j] / 5;
      }
    }
    return RGB; // take median value
  }
}
