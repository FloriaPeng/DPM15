package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

/**
 * This class allows the identification of the color of a 222 mL can. The can can either be blue,
 * red, green or yellow.
 * 
 * @author Floria Peng
 *
 */
public class ColorClassification implements Runnable {

  /* CONSTANTS */
  /**
   * The normalized mean value for each color. The values were found after taking 100 sample point
   * for each color, normalizing them and taking the average of these normalized values. The first
   * number in the array is the R component value. The second number in the array is the G component
   * value. The third number in the array is the B component value.
   */
  public static final float[] MEAN_BLUE_HAT = {(float) 0.265, (float) 0.775, (float) 0.585};
  public static final float[] MEAN_GREEN_HAT = {(float) 0.240, (float) 0.950, (float) 0.200};
  public static final float[] MEAN_YELLOW_HAT = {(float) 0.830, (float) 0.525, (float) 0.125};
  public static final float[] MEAN_RED_HAT = {(float) 0.960, (float) 0.100, (float) 0.060};

  /**
   * The normalized standard deviation value for each color. The values were found after taking 100
   * sample point for each color, normalizing them and taking the standard deviation of these
   * normalized values. The first number in the array is the R component value. The second number in
   * the array is the G component value. The third number in the array is the B component value.
   */
  public static final float[] STD_BLUE_HAT = {(float) 0.030395, (float) 0.025519, (float) 0.036570};
  public static final float[] STD_GREEN_HAT =
      {(float) 0.034945, (float) 0.017382, (float) 0.042392};
  public static final float[] STD_YELLOW_HAT =
      {(float) 0.020065, (float) 0.024057, (float) 0.019677};
  public static final float[] STD_RED_HAT = {(float) 0.013000, (float) 0.030304, (float) 0.024357};

  /* FIELDS */
  private SampleProvider usDistance; // The sample provider for the ultrasonic sensor
  private float[] usData; // The data buffer for the ultrasonic sensor reading
  private SampleProvider colorReading; // The sample provider for the color sensor
  private float[] colorData; // The data buffer for the color sensor reading
  int[] detected = new int[4]; // +1 if target color is detected
  boolean stop = false;
  int color = -1; // initialize color to -1 (which is none of the colors)
  boolean notfound = false; // false for this can is not the target can
  boolean found = false; // true for target color found

  /**
   * This is the default constructor. It takes:
   * 
   * @param usDistance: reading of distance for US sensor
   * @param usData: array to store ultrasonic readings
   * @param colorReading: color reading for the color sensor
   * @param colorData: color array data to store color sensor readings
   */
  public ColorClassification(SampleProvider usDistance, float[] usData, SampleProvider colorReading,
      float[] colorData) {
    this.usDistance = usDistance;
    this.usData = usData;
    this.colorReading = colorReading;
    this.colorData = colorData;
  }

  /**
   * This is the method that is called when the thread is called.
   */
  public void run() {

    // Initialize the thread
    stop = false;
    color = -1;

    // Initialize detected array values to zero
    for (int i = 0; i < 4; i++) {
      detected[i] = 0;
    }

    while (true) {

      /**
       * If a color is detected, then you increase the array at the corresponding value. Number 1
       * detects blue color and increases detected at position 0. Number 2 detects green color and
       * increases detected at position 1. Number 3 detects yellow color and increases detected at
       * position 2. Number 4 detects red color and increases detected at position 3.
       */
      if (colorDetect(1)) {
        detected[0]++;
      } else if (colorDetect(2)) {
        detected[1]++;
      } else if (colorDetect(3)) {
        detected[2]++;
      } else if (colorDetect(4)) {
        detected[3]++;
      }

      if (stop) {
        LCD.clear();
        LCD.drawString("detected: " + detected[0] + "==" + detected[1] + "==" + detected[2] + "=="
            + "detected[3]", 0, 3);
        int[] arr = Arrays.copyOf(detected, detected.length);
        Arrays.sort(arr);
        color = 0;
        for (; color < arr.length; color++) {
          if (detected[color] == arr[3]) {
            color += 1;
            break;
          }
        }
        break;
      }
    }

  }

  /**
   * This method is used to detect the color of a can
   * 
   * @param colorID: 1 --> Blue, 2 --> Green, 3 --> Yellow, 4 --> Red
   * @return if a color was detected
   */
  boolean colorDetect(int colorID) {

    // Initialize target mean and standard deviation
    float[] target_mean = {0, 0, 0};
    float[] target_std = {0, 0, 0};

    // Depending on the colorID, this switch statements correctly sets the target mean and standard
    // deviation value.
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

    float[] reading = mean_filter(); // Read value from filter

    // If the reading value is within 3 standard deviations from the target mean for the 3
    // components, then return that color is identified.
    if (Math.abs(reading[0] - target_mean[0]) < 3 * target_std[0]
        && Math.abs(reading[1] - target_mean[1]) < 3 * target_std[1]
        && Math.abs(reading[2] - target_mean[2]) < 3 * target_std[2]) {
      return true;
    } else {
      return false;
    }

  }

  /**
   * The median filter of the distance detected to ignore the noise. Window size is set to 5.
   * 
   * @return filtered distance reading
   */
  double median_filter() {
    double[] arr = new double[5];
    for (int i = 0; i < 5; i++) { // take 5 readings
      usDistance.fetchSample(usData, 0); // store reading in buffer
      arr[i] = usData[0] * 100.0; // amplify signal
    }
    Arrays.sort(arr); // sort readings
    return arr[2]; // take median value
  }

  /**
   * The mean filter to guess the correct color. Window size is set to 5
   * 
   * @return filtered RGB reading
   */
  float[] mean_filter() {

    float[][] arr = new float[5][3]; // store readings
    float[][] temp = new float[5][3];
    float[] RGB = {0, 0, 0};

    for (int i = 0; i < 5; i++) { // take 5 readings
      colorReading.fetchSample(colorData, 0); // store reading in buffer
      arr[i] = Arrays.copyOf(colorData, 3);
      temp[i] = Arrays.copyOf(arr[i], 3);
    }
    for (int i = 0; i < 5; i++) { // Normalize readings
      float norm = (float) Math
          .sqrt(Math.pow(temp[i][0], 2) + Math.pow(temp[i][1], 2) + Math.pow(temp[i][2], 2));
      for (int j = 0; j < 3; j++) {
        arr[i][j] = arr[i][j] / norm;
      }

    }
    for (int i = 0; i < 5; i++) { // Taking the average
      for (int j = 0; j < 3; j++) {
        RGB[j] += arr[i][j] / 5;
      }
    }
    return RGB; // return average value
  }
}
