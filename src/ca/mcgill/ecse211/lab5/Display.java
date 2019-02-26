package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display implements Runnable {

	/*FIELDS*/
	private Odometer odo; //odometer
	private TextLCD lcd; //lcd screen
	private double[] position; //X, Y and theta position
	private final long DISPLAY_PERIOD = 25; //Time to display
	private long timeout = Long.MAX_VALUE; //timeout value to end

	/**
	 * This is the default constructor.
	 * It takes a TextLCD and throws an OdometerExceptions
	 * @param lcd
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.lcd = lcd;
	}

	/**
	 * This is an overloaded Display constructor
	 * In addition to the default constructor, you can set a timeout.
	 * @param lcd
	 * @param timeout
	 * @throws OdometerExceptions
	 */
	public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.timeout = timeout;
		this.lcd = lcd;
	}

	/**
	 * This is the main method that will run when the Thread is started
	 */
	public void run() {

		lcd.clear(); //Clear the screen

		long updateStart, updateEnd; //start and end time

		long tStart = System.currentTimeMillis();
		do {
			updateStart = System.currentTimeMillis();

			// Retrieve x, y and Theta information
			position = odo.getXYT();

			// Print x,y, and theta information
			DecimalFormat numberFormat = new DecimalFormat("######0.00"); //Check number formatting
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);

			// This ensures that the data is updated only once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} while ((updateEnd - tStart) <= timeout);

	}

}
