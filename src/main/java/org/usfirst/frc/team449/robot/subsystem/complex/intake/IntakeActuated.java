package org.usfirst.frc.team449.robot.subsystem.complex.intake;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.generalInterfaces.simpleMotor.SimpleMotor;
import org.usfirst.frc.team449.robot.jacksonWrappers.MappedDoubleSolenoid;
import org.usfirst.frc.team449.robot.subsystem.interfaces.intake.SubsystemIntake;
import org.usfirst.frc.team449.robot.subsystem.interfaces.solenoid.SubsystemSolenoid;

/**
 * An intake that goes up and down with a piston.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class IntakeActuated extends Subsystem implements SubsystemSolenoid, SubsystemIntake {

    /**
     * The piston for actuating the intake.
     */
    private final DoubleSolenoid piston;
    /**
     * The motor for the intake.
     */
    private final SimpleMotor motor;
    /**
     * The speed to run the motor at going fast and slow, respectively.
     */
    private final double fastSpeed, slowSpeed;
    /**
     * The current state of the intake.
     */
    private IntakeMode currentMode;
    /**
     * The current position of the piston
     */
    private DoubleSolenoid.Value currentPistonPos;

    /**
     * Default constructor.
     *
     * @param piston    The piston for actuating the intake.
     * @param motor     The motor for the intake.
     * @param fastSpeed The speed to run the motor at going fast.
     * @param slowSpeed The speed to run the motor at going slow.
     */
    @JsonCreator
    public IntakeActuated(@NotNull @JsonProperty(required = true) MappedDoubleSolenoid piston,
                          @NotNull @JsonProperty(required = true) SimpleMotor motor,
                          @JsonProperty(required = true) double fastSpeed,
                          @JsonProperty(required = true) double slowSpeed) {
        this.piston = piston;
        this.motor = motor;
        this.fastSpeed = fastSpeed;
        this.slowSpeed = slowSpeed;
    }

    /**
     * Initialize the default command for a subsystem By default subsystems have no default command, but if they do, the
     * default command is set with this method. It is called on all Subsystems by CommandBase in the users program after
     * all the Subsystems are created.
     */
    @Override
    protected void initDefaultCommand() {
        //Do nothing
    }

    /**
     * @return the current mode of the intake.
     */
    @Override
    public @NotNull IntakeMode getMode() {
        return currentMode;
    }

    /**
     * @param mode The mode to switch the intake to.
     */
    @Override
    public void setMode(@NotNull IntakeMode mode) {
        this.currentMode = mode;
        switch (mode) {
            case OFF:
                motor.disable();
                break;
            case IN_FAST:
                motor.setVelocity(fastSpeed);
                break;
            case IN_SLOW:
                motor.setVelocity(slowSpeed);
                break;
            case OUT_FAST:
                motor.setVelocity(-fastSpeed);
                break;
            case OUT_SLOW:
                motor.setVelocity(-slowSpeed);
                break;
        }
    }

    /**
     * @param value The position to set the solenoid to.
     */
    @Override
    public void setSolenoid(@NotNull DoubleSolenoid.Value value) {
        currentPistonPos = value;
        piston.set(value);
    }

    /**
     * @return the current position of the solenoid.
     */
    @Override
    @NotNull
    public DoubleSolenoid.Value getSolenoidPosition() {
        return currentPistonPos;
    }
}
