package org.usfirst.frc.team449.robot.oi.buttons;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.I2C;
import org.usfirst.frc.team449.robot.PracticeFrame;
import org.usfirst.frc.team449.robot.jacksonWrappers.MappedButton;

/**
 * A button that gets triggered by a reaching a certain color threshold.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class ColorSensorButton extends MappedButton {

	/**
	 * The I2C this class is a wrapper on.
	 */
	protected final I2C i2c;

	/**
	 * Thresholds for red and blue values from color sensor, over which get()
	 * will return true depending on alliance color
	 */
	private final int redThreshold, blueThreshold;

	/**
	 * Cached color values, since the RIOduino can't be pinged every robot tick
	 */
	private int cachedRed, cachedBlue;

	/**
	 * For delaying how often the I2C is pinged
	 */
	private int ticks;

	/**
	 * How many robot ticks to wait before pinging the RIOdunio
	 */
	private static final int TICKS_PER_DUINO_PING = 20;

	/**
	 * Default constructor.
	 *
	 * @param port          The I2C port the device is connected to.
	 * @param deviceAddress The address of the device on the I2C bus.
	 * @param redThreshold  The threshold for red, over which the sensed object will be identified as red.
	 * @param blueThreshold The threshold for blue, over which the sensed object will be identified as blue.
	 */
	@JsonCreator
	public ColorSensorButton(@JsonProperty(required = true) I2C.Port port,
	                         @JsonProperty(required = true) Integer deviceAddress,
	                         @JsonProperty(required = true) Integer redThreshold,
	                         @JsonProperty(required = true) Integer blueThreshold) {
		i2c = new I2C(port, deviceAddress);
		this.redThreshold = redThreshold;
		this.blueThreshold = blueThreshold;
		ticks = 0;
	}

	private int[] readRGB() {
		byte[] receivedData = new byte[3];
		i2c.transaction(new byte[0], 0, receivedData, 3);

		int[] rgb = new int[3];
		for (int i = 0; i < receivedData.length; i++) {
			rgb[i] = (0x000000FF) & receivedData[i];
		}

		return rgb;
	}

	/**
	 * Returns whether or not the trigger is active.
	 * <p>
	 * <p>This method will be called repeatedly a command is linked to the Trigger.
	 *
	 * @return whether or not the trigger condition is active.
	 */
	@Override
	public boolean get() {
		if (ticks % TICKS_PER_DUINO_PING == 0) {
			int[] rgb = readRGB();
			cachedRed = rgb[0];
			cachedBlue = rgb[2];
		}

		ticks++;

		if (false/*PracticeFrame.onRed*/) {
			return cachedBlue > blueThreshold;
		} else {
			return cachedRed > redThreshold;
		}
	}

}
