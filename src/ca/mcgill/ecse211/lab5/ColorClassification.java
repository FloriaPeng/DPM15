package ca.mcgill.ecse211.lab5;

import java.util.Arrays;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class ColorClassification implements Runnable { // TODO missing comment

  // The mean value of the normal plot for blue, green, yellow, red
  // R, G, B
  public static final float[] MEAN_BLUE_HAT = {(float) 0.307388, (float) 0.738585, (float) 0.599052};
  public static final float[] MEAN_GREEN_HAT = {(float) 0.263362, (float) 0.911141, (float) 0.300717};
  public static final float[] MEAN_YELLOW_HAT = {(float) 0.851280, (float) 0.509135, (float) 0.125418};
  public static final float[] MEAN_RED_HAT = {(float) 0.916237, (float) 0.316426, (float) 0.243150};
  
  // The standard deviation of the normal plot for blue, green, yellow, red
  public static final float[] STD_BLUE_HAT = {(float) 0.024030, (float) 0.016221, (float) 0.017660};
  public static final float[] STD_GREEN_HAT = {(float) 0.076144, (float) 0.063255, (float) 0.018292};
  public static final float[] STD_YELLOW_HAT = {(float) 0.006869, (float) 0.009795, (float) 0.015301};
  public static final float[] STD_RED_HAT = {(float) 0.012793, (float) 0.026668, (float) 0.019988};

  private SampleProvider usDistance; // The sample provider for the ultrasonic sensor
  private float[] usData; // The data buffer for the ultrasonic sensor reading

  private SampleProvider colorReading; // The sample provider for the color sensor
  private float[] colorData; // The data buffer for the color sensor reading

  int[] detected = new int[4]; // +1 if target color is detected
  boolean stop = false;
  int color = -1;
  boolean notfound = false; // false for this can is not the target can
  boolean found = false; // true for target color found
  
  
  float[] sensorValues = new float[3];
  int sensorReadCount = 0;

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
        
        for (color = 0; color < arr.length; color++) {
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
    if (Math.abs(reading[0] - target_mean[0]) < 2 * target_std[0]
        && Math.abs(reading[1] - target_mean[1]) < 2 * target_std[1]
        && Math.abs(reading[2] - target_mean[2]) < 2 * target_std[2]) {
      return true;
    } else {
      return false;
    }

  }
  
  
  
  /*
   * Read the sensor data, compute the average and store it as long as count <5
   */
  float[] mean_filter () {
    
    while (sensorReadCount < 5) {
      // Fetch sample
      colorReading.fetchSample(colorData, 0);
      
      // Store average value
      sensorValues[0] = (sensorValues[0] + colorData[0]) / 2;
      sensorValues[1] = (sensorValues[1] + colorData[1]) / 2;
      sensorValues[2] = (sensorValues[2] + colorData[2]) / 2;

      sensorReadCount++;
    }
    
    float square = (float) Math.sqrt((Math.pow(sensorValues[0],2) + Math.pow(sensorValues[1],2) + Math.pow(sensorValues[2],2)));
    
    float r = sensorValues[0] / square;
    float g = sensorValues[1] / square;
    float b = sensorValues[2] / square;
    
    LCD.drawString("R: " + r, 0, 2);
    LCD.drawString("G: " + g, 0, 3);
    LCD.drawString("B: " + b, 0, 4);
    
    Arrays.fill(sensorValues, 0);
    sensorReadCount = 0;
    
    float[] RGB = {r,g,b};
    
    return RGB;
  }
  

  /**
   * The median filter of the distance detected to ignore the noise
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

}
