package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class ColorClassification implements Runnable {

  // The mean value of the normal plot for blue, green, yellow, red
  // R, G, B
  /*public static final float[] MEAN_BLUE_HAT =
      {(float) 0.259460, (float) 0.634070, (float) 0.728403};
  public static final float[] MEAN_GREEN_HAT =
      {(float) 0.245649, (float) 0.947591, (float) 0.203922};
  public static final float[] MEAN_YELLOW_HAT =
      {(float) 0.916237, (float) 0.316426, (float) 0.243150};
  public static final float[] MEAN_RED_HAT = {(float) 0.990418, (float) 0.105019, (float) 0.089437};*/
  // The standard deviation of the normal plot for blue, green, yellow, red
  /*public static final float[] STD_BLUE_HAT = {(float) 0.006316, (float) 0.003392, (float) 0.003607};
  public static final float[] STD_GREEN_HAT =
      {(float) 0.007863, (float) 0.003210, (float) 0.008531};
  public static final float[] STD_YELLOW_HAT =
      {(float) 0.001904, (float) 0.002950, (float) 0.004110};
  public static final float[] STD_RED_HAT = {(float) 0.000783, (float) 0.004657, (float) 0.004712};*/
  
  public static final float[] MEAN_BLUE_HAT =
      {(float) 0.269460, (float) 0.794070, (float) 0.588403};
  public static final float[] MEAN_GREEN_HAT =
      {(float) 0.245649, (float) 0.947591, (float) 0.203922};
  public static final float[] MEAN_YELLOW_HAT =
      {(float) 0.836237, (float) 0.536426, (float) 0.127190};
  public static final float[] MEAN_RED_HAT = {(float) 0.990418, (float) 0.105019, (float) 0.089437};
  
  public static final float[] STD_BLUE_HAT = {(float) 0.030395, (float) 0.038519, (float) 0.018570};
  public static final float[] STD_GREEN_HAT =
      {(float) 0.034945, (float) 0.017382, (float) 0.042392};
  public static final float[] STD_YELLOW_HAT =
      {(float) 0.009065, (float) 0.014057, (float) 0.019677};
  public static final float[] STD_RED_HAT = {(float) 0.004000, (float) 0.025304, (float) 0.024357};

  private SampleProvider usDistance; // The sample provider for the ultrasonic sensor
  private float[] usData; // The data buffer for the ultrasonic sensor reading

  private SampleProvider colorReading; // The sample provider for the color sensor
  private float[] colorData; // The data buffer for the color sensor reading

  int[] detected = new int[4]; // +1 if target color is detected
  boolean stop = false;
  int color = -1;
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

    // Initialize the thread
    stop = false;
    color = -1;
    for (int i = 0; i < 4; i++) {
      detected[i] = 0;
    }

    while (true) {

      LCD.clear();
      LCD.drawString("R: " + mean_filter()[0], 0, 0);
      LCD.drawString("G: " + mean_filter()[1], 0, 1);
      LCD.drawString("B: " + mean_filter()[2], 0, 2);

      if (colorDetect(0)) {
        detected[0]++;
      } else if (colorDetect(1)) {
        detected[1]++;
      } else if (colorDetect(2)) {
        detected[2]++;
      } else if (colorDetect(3)) {
        detected[3]++;
      }
      if (stop) {
        int[] arr = Arrays.copyOf(detected, detected.length);
        Arrays.sort(arr);
        color = -1;
        for (; color < arr.length; color++) {
          if (detected[color] == arr[3]) {
            break;
          }
        }
        break;
      }
    }

  }

  boolean colorDetect(int colorID) {

    float[] target_mean = {0, 0, 0};
    float[] target_std = {0, 0, 0};

    switch (colorID) {
      case 1:
        target_mean = MEAN_BLUE_HAT;
        target_std = STD_BLUE_HAT;
        break;
      case 2:
        target_mean = MEAN_GREEN_HAT;
        target_std = STD_GREEN_HAT;
        break;
      case 3:
        target_mean = MEAN_YELLOW_HAT;
        target_std = STD_YELLOW_HAT;
        break;
      case 4:
        target_mean = MEAN_RED_HAT;
        target_std = STD_RED_HAT;
        break;
    }
    float[] reading = mean_filter();
    // LCD.clear();
    /*LCD.drawString("R: " + reading[0], 0, 2);
    LCD.drawString("G: " + reading[1], 0, 3);
    LCD.drawString("B: " + reading[2], 0, 4);*/
    if (Math.abs(reading[0] - target_mean[0]) < 3 * target_std[0]
        && Math.abs(reading[1] - target_mean[1]) < 3 * target_std[1]
        && Math.abs(reading[2] - target_mean[2]) < 3 * target_std[2]) {
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
    float[][] temp = new float[5][3];
    float[] RGB = {0, 0, 0};
    for (int i = 0; i < 5; i++) { // take 5 readings
      colorReading.fetchSample(colorData, 0); // store reading in buffer
      arr[i] = Arrays.copyOf(colorData, 3); // signal amplification
      temp[i] = Arrays.copyOf(arr[i], 3);
    }
    for (int i = 0; i < 5; i++) { // Normalization
      float norm = (float) Math.sqrt(Math.pow(temp[i][0], 2) + Math.pow(temp[i][1], 2) + Math.pow(temp[i][2], 2));
      for (int j = 0; j < 3; j++) {
        arr[i][j] = arr[i][j] / norm;
      }
    }
    for (int i = 0; i < 5; i++) { // Taking the average
      for (int j = 0; j < 3; j++) {
        RGB[j] += arr[i][j] / 5;
      }
    }
    return RGB; // take median value
  }

}
