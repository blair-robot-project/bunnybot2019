package org.usfirst.frc.team449.robot.subsystem.interfaces.analogMotor.commands;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.subsystem.interfaces.analogMotor.SubsystemAnalogMotor;

@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class carouselCommand<T extends Subsystem & SubsystemAnalogMotor> extends Command {

    /**
     * The subsystem to execute this command on
     */
    @NotNull
    private final T subsystem;

    @JsonCreator
    public carouselCommand(@NotNull T subsystem){
        this.subsystem = subsystem;
    }

    @Override
    protected void initialize(){

    }

    @Override
    protected void execute(){

    }

    @Override
    protected void end(){

    }

    @Override
    protected void interrupted(){

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
