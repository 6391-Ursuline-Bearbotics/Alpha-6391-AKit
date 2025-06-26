// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIO;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOSim;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOTalonFX;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDS;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDSIO;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDSIOReal;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDSIOSim;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Climber.ClimberDS.ClimberDS;
import frc.robot.subsystems.Climber.ClimberDS.ClimberDSIO;
import frc.robot.subsystems.Climber.ClimberDS.ClimberDSIOReal;
import frc.robot.subsystems.Climber.ClimberDS.ClimberDSIOSim;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.WindupXboxController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final WindupXboxController m_driver = new WindupXboxController(0);
    private final WindupXboxController m_operator = new WindupXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // Maple Sim
    private SwerveDriveSimulation m_driveSimulation = null;

    // AK-enabled Subsystems
    public final Drive m_drive;
    public final Arm m_profiledArm;
    public final Elevator m_profiledElevator;
    private final Climber m_profiledClimber;
    public final ClawRoller m_clawRoller;
    public final ClawRollerDS m_ClawRollerDS;
    public final ClimberDS m_ClimberDS;
    public final Superstructure m_superStruct;

    public final Vision m_vision;

    private double speedMultiplier = 1.0;
    private boolean rumbleReady = false;
    private Supplier<Double> speedScalar = () -> speedMultiplier;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight));

                m_vision = new Vision(m_drive, new VisionIOQuestNav(VisionConstants.questName),
                    new VisionIOLimelight(VisionConstants.camera0Name, m_drive::getRotation),
                    new VisionIOLimelight(VisionConstants.camera1Name, m_drive::getRotation));
                m_drive.addQuestZero(m_vision::zeroQuest);
                m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                m_profiledClimber = new Climber(new ClimberIOTalonFX(), false);
                m_clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIOReal());
                m_ClimberDS = new ClimberDS(new ClimberDSIOReal());

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_driveSimulation =
                    new SwerveDriveSimulation(Drive.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(m_driveSimulation);
                m_drive =
                    new Drive(
                        new GyroIOSim(this.m_driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                m_profiledArm = new Arm(new ArmIOSim(), true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                m_profiledClimber = new Climber(new ClimberIOSim(), true);
                m_clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIOSim());
                m_ClimberDS = new ClimberDS(new ClimberDSIOSim());

                m_vision = new Vision(m_drive);
                m_drive.addQuestZero(m_vision::zeroQuest);
                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});

                m_profiledArm = new Arm(new ArmIO() {}, true);
                m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
                m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIO() {});
                m_ClimberDS = new ClimberDS(new ClimberDSIO() {});

                m_vision = new Vision(m_drive);
                m_drive.addQuestZero(m_vision::zeroQuest);
                break;
        }

        // Superstructure coordinates Arm and Elevator motions
        m_superStruct = new Superstructure(m_profiledArm, m_profiledElevator);

        // Logic Triggers
        registerNamedCommands();

        // Set up auto routines
        m_autoChooser =
            new LoggedDashboardChooser<>("Auto Choices",
                AutoBuilder.buildAutoChooser("L2 Right"));


        // for (String auto : AutoBuilder.getAllAutoNames()) {
        // if (auto.contains("Right")) {
        // m_autoChooser.addOption(auto.replace("Right", "Left"),
        // new PathPlannerAuto(auto, true));
        // }
        // }

        // Set up SysId routines
        /*
         * m_autoChooser.addOption( "Drive Wheel Radius Characterization",
         * DriveCommands.wheelRadiusCharacterization(m_drive)); m_autoChooser.addOption(
         * "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
         * m_autoChooser.addOption( "Drive SysId (Quasistatic Forward)",
         * m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)); m_autoChooser.addOption(
         * "Drive SysId (Quasistatic Reverse)",
         * m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); m_autoChooser.addOption(
         * "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
         * m_autoChooser.addOption( "Drive SysId (Dynamic Reverse)",
         * m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
         */

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driver.getLeftY() * speedScalar.get(),
            () -> -m_driver.getLeftX() * speedScalar.get(),
            () -> -m_driver.getRightX() * 0.75);
    }

    private Command joystickDriveAtAngle(Supplier<Rotation2d> angle)
    {
        return DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            () -> -m_driver.getLeftX() * speedMultiplier,
            angle);
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> -m_driver.getLeftY() * 0.5,
            approachPose,
            () -> m_drive.getPose());
    }

    private Command autoApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> 0.27,
            approachPose,
            () -> m_drive.getPose()).withTimeout(1.2);
    }

    /** Button and Command mappings */
    private void configureControllerBindings()
    {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(joystickDrive());

        // Reset gyro to 0° when start button is pressed
        final Runnable resetGyro =
            Constants.currentMode == Constants.Mode.SIM
                ? () -> m_drive.setPose(
                    m_driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
                // simulation
                : () -> m_drive.setPose(
                    new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        m_driver.start().onTrue(Commands.runOnce(resetGyro, m_drive).ignoringDisable(true));

        // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        m_driver.rightBumper().and(() -> !m_profiledElevator.isAlgae())
            .whileTrue(Commands.runOnce(() -> m_vision.useLeft(true)).andThen(Commands.either(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)),
                joystickDriveAtAngle(
                    () -> FieldConstants.getNearestCoralStation(m_drive.getPose()).getRotation()),
                m_ClawRollerDS.triggered)));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        m_driver.leftBumper().and(() -> !m_profiledElevator.isAlgae())
            .whileTrue(Commands.runOnce(() -> m_vision.useLeft(false)).andThen(Commands.either(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT)),
                joystickDriveAtAngle(
                    () -> FieldConstants.getNearestCoralStation(m_drive.getPose()).getRotation()),
                m_ClawRollerDS.triggered)));

        // Driver Right Bumper and Algae mode: Approach Nearest Reef Face
        m_driver.rightBumper().or(m_driver.leftBumper()).and(() -> m_profiledElevator.isAlgae())
            .whileTrue(
                joystickApproach(() -> FieldConstants.getNearestReefFace(m_drive.getPose())));

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver.a().or(m_operator.a())
            .and(() -> m_clawRoller.notIntaking())
            .onTrue(Commands.runOnce(() -> speedMultiplier = 1.0)
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1)));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver.x().or(m_operator.x())
            .and(() -> m_clawRoller.notIntaking())
            .onTrue(Commands.runOnce(() -> speedMultiplier = 1.0)
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2)));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver.b().or(m_operator.b())
            .and(() -> m_clawRoller.notIntaking())
            .onTrue(Commands.runOnce(() -> speedMultiplier = 0.5)
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3)));

        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver.y().or(m_operator.y())
            .and(() -> m_clawRoller.notIntaking())
            .onTrue(Commands.runOnce(() -> speedMultiplier = 0.5)
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_4,
                        Elevator.State.LEVEL_4)));

        // Driver Right Trigger: Any other level
        m_driver.rightTrigger().and(() -> m_profiledElevator.isOther())
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL));

        // Driver Right Trigger: Place Coral on L2/L3
        m_driver.rightTrigger()
            .and(() -> m_profiledElevator.isL2() || m_profiledElevator.isL3())
            .and(m_operator.rightBumper().or(m_operator.leftBumper()).negate())
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.SCORE)
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate()))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                // backup after shoot
                .andThen(DriveCommands.joystickDriveRobot(m_drive, () -> -0.15, () -> 0,
                    () -> 0).withTimeout(0.4).andThen(Commands.runOnce(() -> m_drive.stop()))
                    .asProxy())
                .andThen(Commands.runOnce(() -> speedMultiplier = 1.0))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.INTAKE))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                .andThen(() -> rumbleReady = true)
                .andThen(Commands.waitSeconds(0.25))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2)));

        // Driver Right Trigger: Place Coral on L2/L3 then Algae
        m_driver.rightTrigger()
            .and(() -> m_profiledElevator.isL2() || m_profiledElevator.isL3())
            .and(m_operator.rightBumper().or(m_operator.leftBumper()))
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.SCORE)
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate()))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                // backup after shoot
                .andThen(DriveCommands.joystickDriveRobot(m_drive, () -> -0.15, () -> 0,
                    () -> 0).withTimeout(0.4).andThen(Commands.runOnce(() -> m_drive.stop()))
                    .asProxy())
                .andThen(Commands.runOnce(() -> speedMultiplier = 0.5))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.EJECT))
                .andThen(Commands.either(m_superStruct.getTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2),
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_3,
                        Elevator.State.LEVEL_3),
                    m_operator.leftBumper())));

        // Place Coral on L1
        m_driver.rightTrigger().and(() -> m_profiledElevator.isL1())
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L1)
                .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_1_FLIP))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate()))
                .andThen(m_profiledElevator.setStateCommand(Elevator.State.LEVEL_1))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.INTAKE))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE))
                .andThen(Commands.runOnce(() -> speedMultiplier = 1.0))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                .andThen(() -> rumbleReady = true)
                .andThen(Commands.waitSeconds(0.25))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2)));

        // Place Coral on L4
        m_driver.rightTrigger().and(() -> m_profiledElevator.isL4())
            .and(m_operator.rightBumper().or(m_operator.leftBumper()).negate())
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L4)
                .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4_BACK))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                    .withTimeout(1))
                // backup after shoot
                .andThen(DriveCommands.joystickDriveRobot(m_drive, () -> -0.15, () -> 0,
                    () -> 0).withTimeout(0.4).andThen(Commands.runOnce(() -> m_drive.stop()))
                    .asProxy())
                .andThen(Commands.runOnce(() -> speedMultiplier = 1.0))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.INTAKE))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                .andThen(() -> rumbleReady = true)
                .andThen(Commands.waitSeconds(0.25))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2)));

        // Place Coral on L4 OP bumper held so go straight to algae descore
        m_driver.rightTrigger().and(() -> m_profiledElevator.isL4())
            .and(m_operator.rightBumper().or(m_operator.leftBumper()))
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L4)
                .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4_BACK))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                    .withTimeout(1))
                // backup after shoot
                .andThen(DriveCommands.joystickDriveRobot(m_drive, () -> -0.15, () -> 0,
                    () -> 0).withTimeout(0.4).andThen(Commands.runOnce(() -> m_drive.stop()))
                    .asProxy())
                .andThen(Commands.runOnce(() -> speedMultiplier = 0.5))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.EJECT))
                .andThen(Commands.either(m_superStruct.getTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2),
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_3,
                        Elevator.State.LEVEL_3),
                    m_operator.leftBumper())));

        // Intake coral from chute if you don't already have one
        m_driver.leftTrigger().or(m_operator.leftTrigger())
            .and(m_ClawRollerDS.triggered.negate())
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE))
                .andThen(Commands.runOnce(() -> speedMultiplier = 1.0))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                .andThen(() -> rumbleReady = true)
                .andThen(Commands.waitSeconds(0.25))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.LEVEL_2,
                    Elevator.State.LEVEL_2)));

        // Driver or Operator POV Down: Zero the Elevator (HOMING)
        m_driver.povDown().or(m_operator.povDown()).whileTrue(
            Commands.sequence(
                // Always move Arm to STOW position before moving Elevator
                m_profiledArm.setStateCommand(Arm.State.STOW),
                // Move Elevator to homing position
                Commands.waitUntil(() -> m_profiledArm.atPosition(0.1)),
                m_profiledElevator.setStateCommand(Elevator.State.HOMING),
                Commands.waitUntil(m_profiledElevator.getHomedTrigger()),
                m_profiledElevator.zeroSensorCommand(),
                m_profiledElevator.setStateCommand(Elevator.State.CORAL_INTAKE)));

        // Driver POV Right: Zero the Arm (HOMING)
        m_driver.povRight().or(m_operator.povRight()).whileTrue(
            Commands.sequence(
                // Move Arm to CORAL_INTAKE position before homing
                m_profiledArm.setStateCommand(Arm.State.CORAL_INTAKE),
                Commands.waitUntil(() -> m_profiledArm.atPosition(0.1)),
                m_profiledArm.setStateCommand(Arm.State.HOMING),
                Commands.waitUntil(m_profiledArm.getHomedTrigger()),
                m_profiledArm.zeroSensorCommand(),
                m_profiledArm.setStateCommand(Arm.State.CORAL_INTAKE)));

        // Driver POV Left Button: Send Arm and Elevator to ALGAE_LOW position
        m_driver.povLeft().or(m_operator.povLeft())
            .onTrue(Commands.parallel(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_LOW,
                    Elevator.State.ALGAE_LOW),
                m_clawRoller.setStateCommand(ClawRoller.State.EJECT)));

        // Driver POV Up Button: Send Arm and Elevator to ALGAE_HIGH position
        m_driver.povUp().or(m_operator.povUp())
            .onTrue(Commands.parallel(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_HIGH,
                    Elevator.State.ALGAE_HIGH),
                m_clawRoller.setStateCommand(ClawRoller.State.EJECT)));

        // While operator start held winch in the climber
        m_operator.start().onTrue(m_profiledArm.setStateCommand(Arm.State.CLIMB));
        m_operator.start().whileTrue(m_profiledClimber.setStateCommand(Climber.State.CLIMB))
            .onFalse(m_profiledClimber.setStateCommand(Climber.State.HOME));

        // While operator back held release the climber
        m_operator.back().whileTrue(m_profiledClimber.setStateCommand(Climber.State.UNCLIMB))
            .onFalse(m_profiledClimber.setStateCommand(Climber.State.HOME));

        // For custom tuning in AdvantageScope
        m_operator.rightTrigger().onTrue(m_profiledElevator.setStateCommand(Elevator.State.TUNING));
        // m_operator.leftBumper().onTrue(m_profiledArm.setStateCommand(Arm.State.TUNING));

        Trigger driverRumble = new Trigger(() -> m_vision.hasNewTarget && rumbleReady);
        driverRumble.onTrue(m_driver.rumbleForTime(1).andThen(() -> rumbleReady = false));

        Trigger opRumble = new Trigger(() -> m_ClimberDS.isTriggered());
        opRumble.onTrue(m_operator.rumbleForTime(1));
    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        // Go to the L4 Position
        NamedCommands.registerCommand("L4",
            Commands.waitUntil(m_ClawRollerDS.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4)));

        // Go to the L2 Position
        NamedCommands.registerCommand("L2",
            Commands.waitUntil(m_ClawRollerDS.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2)));

        // Go to the Home Position
        NamedCommands.registerCommand("Home",
            m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                Elevator.State.CORAL_INTAKE));

        // Move Elevator and Arm then wait for EE sensor to be triggered
        NamedCommands.registerCommand("Intake",
            m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)));

        // Score Coral on L4
        NamedCommands.registerCommand("Shoot",
            m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L4)
                .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4_BACK))
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                    .withTimeout(1)));

        // Score Coral on L2
        NamedCommands.registerCommand("ShootL2",
            m_clawRoller.setStateCommand(ClawRoller.State.SCORE));

        NamedCommands.registerCommand("ApproachRight",
            Commands.runOnce(() -> m_vision.useLeft(true)).andThen(autoApproach(
                () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT))));

        NamedCommands.registerCommand("ApproachLeft",
            Commands.runOnce(() -> m_vision.useLeft(false)).andThen(autoApproach(
                () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autoChooser.get();
    }

    /*
     * Simulation-specific routines
     */
    public void resetSimulation()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        m_drive.setPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void resetSimulationField()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
    }

    public void displaySimFieldToAdvantageScope()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        // SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,
        // 2)));
        Logger.recordOutput(
            "FieldSimulation/RobotPosition", m_driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
