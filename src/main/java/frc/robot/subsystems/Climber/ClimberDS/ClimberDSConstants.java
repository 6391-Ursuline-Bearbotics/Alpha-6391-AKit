package frc.robot.subsystems.Climber.ClimberDS;

import frc.robot.Ports;
import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemConstants;

public class ClimberDSConstants {
    public static final GenericDigitalSensorSubsystemConstants kSubSysConstants =
        new GenericDigitalSensorSubsystemConstants();

    static {
        kSubSysConstants.kName = "ClimberDS";
        kSubSysConstants.digitalPort = Ports.CLIMBER_DIGITAL;
    }
}
