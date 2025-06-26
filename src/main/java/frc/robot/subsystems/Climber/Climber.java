package frc.robot.subsystems.Climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Climber extends GenericMotionProfiledSubsystem<Climber.State> {

    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Climber/PositionTuningSP", 0.0);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOME(new ProfileType.DISABLED_BRAKE()),
        UNCLIMB(new ProfileType.OPEN_VOLTAGE(() -> -6.0)),
        CLIMB(new ProfileType.OPEN_VOLTAGE(() -> 6.0)),
        TUNING(new ProfileType.MM_POSITION(
            () -> Units.degreesToRotations(positionTuning.getAsDouble())));

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.HOME;

    @Getter
    public final Alert climbedAlert = new Alert("CLIMB COMPLETE", Alert.AlertType.kInfo);

    /** Constructor */
    public Climber(ClimberIO io, boolean isSim)
    {
        super(State.HOME.profileType, ClimberConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return startEnd(() -> this.state = state, () -> this.state = State.HOME);
    }

    public Command climbedAlertCommand()
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> climbedAlert.set(true)),
            Commands.waitSeconds(1),
            new InstantCommand(() -> climbedAlert.set(false)));
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

}
