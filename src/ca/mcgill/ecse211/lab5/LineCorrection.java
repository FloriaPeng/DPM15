package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class LineCorrection {

  private SampleProvider myColorStatus1; // The sample provider for the color sensor
  private float[] sampleColor1; // The data buffer for the color sensor reading
  private SampleProvider myColorStatus2; // The sample provider for the color sensor
  private float[] sampleColor2; // The data buffer for the color sensor reading

  double[] last = {Math.PI, Math.PI}; // Initialize the last variable to a specific number
  double[] current = {0, 0}; // last and current are both used for differential filter

  public LineCorrection(SampleProvider myColorStatus1, float[] sampleColor1,
      SampleProvider myColorStatus2, float[] sampleColor2) {

    this.myColorStatus1 = myColorStatus1;
    this.sampleColor1 = sampleColor1;
    this.myColorStatus2 = myColorStatus2;
    this.sampleColor2 = sampleColor2;

  }

  /**
   * The differential filter of the light sensor, it will consider detecting a line if there is a
   * huge increase of the reading (the derivative if large)
   * 
   * @return - true for detecting an line, vice versa
   */
  boolean filter1() { // Differential filter

    myColorStatus1.fetchSample(sampleColor1, 0); // Used for obtaining color reading from the
                                                 // SampleProvider

    if (Math.abs(last[0] - Math.PI) < Math.pow(0.1, 5)) { // If last has not been assigned for any number yet
      last[0] = current[0] = sampleColor1[0];
    } else {
      last[0] = current[0]; // Update the last
      current[0] = sampleColor1[0]; // Update the current
    }

    if ((current[0] - last[0]) / 0.01 < -0.7) { // If there is a black line detected
      Sound.beep();
      return true;
    }
    return false;
  }

  /**
   * The differential filter of the light sensor, it will consider detecting a line if there is a
   * huge increase of the reading (the derivative if large)
   * 
   * @return - true for detecting an line, vice versa
   */
  boolean filter2() { // Differential filter

    myColorStatus2.fetchSample(sampleColor2, 0); // Used for obtaining color reading from the
                                                 // SampleProvider

    if (Math.abs(last[1] - Math.PI) < Math.pow(0.1, 5)) { // If last has not been assigned for any number yet
      last[1] = current[1] = sampleColor2[0];
    } else {
      last[1] = current[1]; // Update the last
      current[1] = sampleColor2[0]; // Update the current
    }

    if ((current[1] - last[1]) / 0.01 < -0.6) { // If there is a black line detected
      Sound.beep();
      return true;
    }
    return false;
  }

}
