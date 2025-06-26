package frc.robot.subsystems.GenericDigitalSensorSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Generic motion IO implementation for any motion mechanism using a TalonFX motor controller, an
 * optional follower motor, and an optional remote CANcoder encoder.
 */
public class GenericDigitalSensorSubsystemIOImpl implements GenericDigitalSensorSubsystemIO {

    private DigitalInput di = null;
    private String name;

    /*
     * Constructor
     */
    public GenericDigitalSensorSubsystemIOImpl(
        GenericDigitalSensorSubsystemConstants constants, boolean isSim)
    {
        name = constants.kName;

        di = new DigitalInput(constants.digitalPort);
    }

    public Boolean getMeasurement()
    {
        return !di.get();
    }

    @Override
    public void updateInputs(DigitalSensorIOInputs inputs)
    {
        inputs.tripped = getMeasurement();
    }
}
