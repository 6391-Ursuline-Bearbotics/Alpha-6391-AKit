package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Elevator extends GenericMotionProfiledSubsystem<Elevator.State> {

    static LoggedTunableNumber homingTuning =
        new LoggedTunableNumber("Elevator/HomingVoltageSP", -1);
    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Elevator/PositionTuningSP", 0.05);

    private static final double BOTTOM = 0.065;

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> homingTuning.getAsDouble())),
        CORAL_INTAKE(new ProfileType.MM_POSITION(() -> BOTTOM)),
        LEVEL_1(new ProfileType.MM_POSITION(() -> BOTTOM)),
        LEVEL_2(new ProfileType.MM_POSITION(() -> 1.9)), // 2.1
        LEVEL_3(new ProfileType.MM_POSITION(() -> 5.0)), // 5.1
        LEVEL_4(new ProfileType.MM_POSITION(() -> 9.50)),
        ALGAE_LOW(new ProfileType.MM_POSITION(() -> 5.3)), // 5.5
        ALGAE_HIGH(new ProfileType.MM_POSITION(() -> 8.1)), // 8.2
        TUNING(new ProfileType.MM_POSITION(() -> positionTuning.getAsDouble())),
        CHARACTERIZATION(new ProfileType.CHARACTERIZATION()),
        COAST(new ProfileType.DISABLED_COAST()),
        BRAKE(new ProfileType.DISABLED_BRAKE());

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.CORAL_INTAKE;

    @Getter
    public final Alert homedAlert = new Alert("NEW ELEVATOR HOME SET", Alert.AlertType.kInfo);

    /* For adjusting the Arm's static characterization velocity threshold */
    private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
        new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.1);

    /** Constructor */
    public Elevator(ElevatorIO io, boolean isSim)
    {
        super(State.CORAL_INTAKE.profileType, ElevatorConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Elevator Coast Command", setCoastStateCommand());
        SmartDashboard.putData("Elevator Brake Command", setBrakeStateCommand());
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public Command setCoastStateCommand()
    {
        return this.runOnce(() -> this.state = State.COAST);
    }

    public Command setBrakeStateCommand()
    {
        return this.runOnce(() -> this.state = State.BRAKE);
    }

    public boolean isElevated()
    {
        switch (this.getState()) {
            case LEVEL_1:
            case LEVEL_2:
            case LEVEL_3:
            case LEVEL_4:
            case ALGAE_LOW:
            case ALGAE_HIGH:
                return true;

            default:
                return false;
        }
    }

    public boolean isL1()
    {
        // return this.getState() == Elevator.State.CORAL_INTAKE;
        return this.getState() == Elevator.State.LEVEL_1;

    }

    public boolean isL2()
    {
        return this.getState() == Elevator.State.LEVEL_2;
    }

    public boolean isL3()
    {
        return this.getState() == Elevator.State.LEVEL_3;
    }

    public boolean isL4()
    {
        return this.getState() == Elevator.State.LEVEL_4;
    }

    public boolean isOther()
    {
        return this.getState() == Elevator.State.CORAL_INTAKE
            || this.getState() == Elevator.State.ALGAE_HIGH
            || this.getState() == Elevator.State.ALGAE_LOW;
    }

    public boolean isAlgae()
    {
        return this.getState() == Elevator.State.ALGAE_HIGH
            || this.getState() == Elevator.State.ALGAE_LOW;
    }

    private Debouncer homedDebouncer = new Debouncer(1, DebounceType.kRising);

    public Trigger homedTrigger =
        new Trigger(
            () -> homedDebouncer.calculate(
                (this.state == State.HOMING && Math.abs(io.getVelocity()) < .01)));

    public Command zeroSensorCommand()
    {
        return new InstantCommand(() -> io.zeroSensors());
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    public Command homedAlertCommand()
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> homedAlert.set(true)),
            Commands.waitSeconds(1),
            new InstantCommand(() -> homedAlert.set(false)));
    }

    public Command staticCharacterization(double outputRampRate)
    {
        final StaticCharacterizationState state = new StaticCharacterizationState();
        Timer timer = new Timer();
        return Commands.startRun(
            () -> {
                this.state = State.CHARACTERIZATION;
                timer.restart(); // Starts the timer that tracks the time of the characterization
            },
            () -> {
                state.characterizationOutput = outputRampRate * timer.get();
                io.runCurrent(state.characterizationOutput, 1);
                Logger.recordOutput(
                    "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
            .until(() -> inputs.velocityRps * 2 * Math.PI >= staticCharacterizationVelocityThresh
                .get())
            .finallyDo(
                () -> {
                    timer.stop();
                    Logger.recordOutput("Elevator/CharacterizationOutput",
                        state.characterizationOutput);
                    this.state = State.CORAL_INTAKE;
                });
    }

    private static class StaticCharacterizationState {
        public double characterizationOutput = 0.0;
    }
}
