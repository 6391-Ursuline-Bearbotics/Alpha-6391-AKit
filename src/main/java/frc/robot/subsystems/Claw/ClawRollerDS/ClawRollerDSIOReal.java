package frc.robot.subsystems.Claw.ClawRollerDS;

import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemIOImpl;

public class ClawRollerDSIOReal extends GenericDigitalSensorSubsystemIOImpl
    implements ClawRollerDSIO {

    public ClawRollerDSIOReal()
    {
        super(ClawRollerDSConstants.kSubSysConstants, false);
    }
}
