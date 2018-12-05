package org.usfirst.frc.team449.robot.drive.omnidirectional.commands;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.drive.omnidirectional.DriveOmnidirectional;
import org.usfirst.frc.team449.robot.oi.omnidirectional.OIOmnidirectional;
import org.usfirst.frc.team449.robot.other.Logger;

/**
 * An omnidirectional drive control.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class OmnidirectionalDrive<T extends Subsystem & DriveOmnidirectional> extends Command {

	/**
	 * The OI used for input.
	 */
	@NotNull
	public final OIOmnidirectional oi;


	/**
	 * The subsystem to execute this command on.
	 */
	@NotNull
	private final T subsystem;

	/**
	 * Default constructor
	 *
	 * @param subsystem The subsystem to execute this command on
	 * @param oi        The OI that gives the input to this command.
	 */
	@JsonCreator
	public OmnidirectionalDrive(@NotNull @JsonProperty(required = true) T subsystem,
	                            @NotNull @JsonProperty(required = true) OIOmnidirectional oi) {
		this.oi = oi;
		this.subsystem = subsystem;

		//Default commands need to require their subsystems.
		requires(subsystem);
	}

	/**
	 * Stop the drive for safety reasons.
	 */
	@Override
	protected void initialize() {
		subsystem.fullStop();
	}

	/**
	 * Give output to the motors based on the stick inputs.
	 */
	@Override
	protected void execute() {
		double[] motionCommands = oi.getMotionCommandsCached();
		subsystem.setDirection(motionCommands[0], motionCommands[1], motionCommands[2]);
	}

	/**
	 * Run constantly because this is a default drive
	 *
	 * @return false
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Log and brake when interrupted.
	 */
	@Override
	protected void interrupted() {
		Logger.addEvent("OmnidirectionalDrive Interrupted! Stopping the robot.", this.getClass());
		//Brake for safety!
		subsystem.fullStop();
	}

}
