package org.usfirst.frc.team449.robot.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.usfirst.frc.team449.robot.PracticeFrame;

public class AllianceDigitalInput extends MappedDigitalInput {

    /**
     * Create an instance of a Digital Input class. Creates a digital input given a channel.
     *
     * @param channel the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
     */
    @JsonCreator
    public AllianceDigitalInput(@JsonProperty(required = true) int channel) {
        super(channel);
    }

    /**
     * Get the value from a digital input channel. Retrieve the value of a single digital input channel from the FPGA.
     *
     * @return the status of the digital input
     */
    @Override
    public boolean get() {
        boolean onRed = super.get();
        PracticeFrame.onRed = onRed;
        return onRed;
    }
}
