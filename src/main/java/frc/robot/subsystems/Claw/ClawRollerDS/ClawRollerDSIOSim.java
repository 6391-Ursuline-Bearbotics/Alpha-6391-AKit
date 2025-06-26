package frc.robot.subsystems.Claw.ClawRollerDS;

import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemIOImpl;

public class ClawRollerDSIOSim extends GenericDigitalSensorSubsystemIOImpl
    implements ClawRollerDSIO {

    public ClawRollerDSIOSim()
    {
        super(ClawRollerDSConstants.kSubSysConstants, true);
    }
}
