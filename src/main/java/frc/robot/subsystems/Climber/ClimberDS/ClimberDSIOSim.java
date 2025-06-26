package frc.robot.subsystems.Climber.ClimberDS;

import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemIOImpl;

public class ClimberDSIOSim extends GenericDigitalSensorSubsystemIOImpl
    implements ClimberDSIO {

    public ClimberDSIOSim()
    {
        super(ClimberDSConstants.kSubSysConstants, true);
    }
}
