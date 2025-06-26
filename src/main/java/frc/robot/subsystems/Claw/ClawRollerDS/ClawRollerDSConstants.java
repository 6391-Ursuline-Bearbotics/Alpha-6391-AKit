package frc.robot.subsystems.Claw.ClawRollerDS;

import frc.robot.Ports;
import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemConstants;

public class ClawRollerDSConstants {
    public static final GenericDigitalSensorSubsystemConstants kSubSysConstants =
        new GenericDigitalSensorSubsystemConstants();

    static {
        kSubSysConstants.kName = "ClawRollerDS";
        kSubSysConstants.digitalPort = Ports.CLAW_DIGITAL;
    }
}
