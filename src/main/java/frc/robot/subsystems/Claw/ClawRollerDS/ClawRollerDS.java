package frc.robot.subsystems.Claw.ClawRollerDS;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ClawRollerDS extends GenericDigitalSensorSubsystem {

    public Trigger triggered = new Trigger(() -> super.isTriggered());

    public ClawRollerDS(ClawRollerDSIO io)
    {
        super(ClawRollerDSConstants.kSubSysConstants.kName, io);
    }

}
