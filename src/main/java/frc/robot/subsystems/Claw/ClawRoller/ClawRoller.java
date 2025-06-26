package frc.robot.subsystems.Claw.ClawRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClawRoller
    extends GenericMotionProfiledSubsystem<ClawRoller.State> {

    static LoggedTunableNumber shotSpeed =
        new LoggedTunableNumber("ClawRoller/ShotSpeed", 6.0);
    static LoggedTunableNumber shotSpeedL1 =
        new LoggedTunableNumber("ClawRoller/ShotSpeedL1", 4.5);
    static LoggedTunableNumber shotSpeedL4 =
        new LoggedTunableNumber("ClawRoller/ShotSpeedL4", 2.2);
    static LoggedTunableNumber shotSpeedAlgae =
        new LoggedTunableNumber("ClawRoller/ShotSpeedAlgae", -3.0);

    public final Trigger stalled = new Trigger(() -> super.inputs.torqueCurrentAmps[0] <= -60);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        INTAKE(new ProfileType.OPEN_VOLTAGE(() -> 1.0)),
        EJECT(new ProfileType.OPEN_VOLTAGE(() -> shotSpeedAlgae.get())),
        SCORE(new ProfileType.OPEN_VOLTAGE(() -> shotSpeed.get())),
        SCORE_L1(new ProfileType.OPEN_VOLTAGE(() -> shotSpeedL1.get())),
        SCORE_L4(new ProfileType.OPEN_VOLTAGE(() -> shotSpeedL4.get())),
        HOLDCORAL(new ProfileType.DISABLED_BRAKE());

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.HOLDCORAL;

    /** Constructor */
    public ClawRoller(ClawRollerIO io, boolean isSim)
    {
        super(State.HOLDCORAL.profileType, ClawRollerConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public boolean notIntaking()
    {
        return state != State.INTAKE;
    }
}
