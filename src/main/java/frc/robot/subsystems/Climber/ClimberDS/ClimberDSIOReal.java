package frc.robot.subsystems.Climber.ClimberDS;

import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemIOImpl;

public class ClimberDSIOReal extends GenericDigitalSensorSubsystemIOImpl
    implements ClimberDSIO {

    public ClimberDSIOReal()
    {
        super(ClimberDSConstants.kSubSysConstants, false);
    }
}
