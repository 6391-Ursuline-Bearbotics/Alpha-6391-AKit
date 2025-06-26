package frc.robot.subsystems.GenericDigitalSensorSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public abstract class GenericDigitalSensorSubsystem
    extends SubsystemBase {

    private final String name;
    private final GenericDigitalSensorSubsystemIO io;
    protected final DigitalSensorIOInputsAutoLogged inputs = new DigitalSensorIOInputsAutoLogged();

    @Getter
    private boolean triggered;

    public GenericDigitalSensorSubsystem(String name, GenericDigitalSensorSubsystemIO io)
    {
        this.name = name;
        this.io = io;
    }

    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);

        triggered = inputs.tripped;

        displayInfo();
    }

    private void displayInfo()
    {

        if (Constants.tuningMode) {
            Logger.recordOutput(this.name + "/Triggered", triggered);
        }
    }
}
