package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class ColorClassification implements Runnable {

  // The mean value of the normal plot for blue, green, yellow, red
  // R, G, B
  public static final float[] MEAN_BLUE = {(float) 0.004901, (float) 0.016666, (float) 0.016666};
  public static final float[] MEAN_GREEN = {(float) 0.002941, (float) 0.012745, (float) 0.002941};
  public static final float[] MEAN_YELLOW = {(float) 0.032352, (float) 0.018627, (float) 0.003921};
  public static final float[] MEAN_RED = {(float) 0.033333, (float) 0.008823, (float) 0.005882};
  // The standard deviation of the normal plot for blue, green, yellow, red
  public static final float[] STD_BLUE = {(float) 0.002, (float) 0.002, (float) 0.002};
  public static final float[] STD_GREEN = {(float) 0.002, (float) 0.002, (float) 0.002};
  public static final float[] STD_YELLOW = {(float) 0.002, (float) 0.002, (float) 0.002};
  public static final float[] STD_RED = {(float) 0.002, (float) 0.002, (float) 0.002};

  public static final int PROPER_DISTANCE = 3; // The proper distance between the color sensor and
                                               // the can, so that the reading is valid

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
      if (colorDetect(SearchCan.TR)) {
        detected++;
      }
      if (detected == 20) {
        Sound.beep();
        found = true;
        break;
      } else if (notfound) {
        Sound.twoBeeps();
        break;
      }
    }
    try {
      Thread.sleep(500);
    } catch (InterruptedException e) {
    }
  }

  boolean colorDetect(int colorID) {

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
    if ((reading[0] - target_mean[0]) < target_std[0]
        && (reading[1] - target_mean[1]) < target_std[1]
        && (reading[2] - target_mean[2]) < target_std[2]) {
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