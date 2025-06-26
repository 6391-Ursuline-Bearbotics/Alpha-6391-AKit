package frc.robot.subsystems.Climber.ClimberDS;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class ClimberDS extends GenericDigitalSensorSubsystem {

    public Trigger triggered = new Trigger(() -> super.isTriggered());

    public ClimberDS(ClimberDSIO io)
    {
        super(ClimberDSConstants.kSubSysConstants.kName, io);
    }

}
