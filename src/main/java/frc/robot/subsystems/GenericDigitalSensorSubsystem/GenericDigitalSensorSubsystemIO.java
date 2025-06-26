package frc.robot.subsystems.GenericDigitalSensorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface GenericDigitalSensorSubsystemIO {
    @AutoLog
    abstract class DigitalSensorIOInputs {
        public boolean tripped;
    }

    default void updateInputs(DigitalSensorIOInputs inputs)
    {}
}
