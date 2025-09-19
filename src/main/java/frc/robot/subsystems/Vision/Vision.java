// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    public boolean visionHasTarget = false;
    public boolean hasNewTarget = false;
    private boolean seesThisTarget = false;
    private double shiftTrust = 1.0;
    private Pose2d pose = new Pose2d();

    public Vision(VisionConsumer consumer, VisionIO... io)
    {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.",
                    AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex)
    {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    /**
     * Zeros the Quest.
     *
     * @param startingPose The auto starting pose.
     */
    public void zeroQuest(Pose2d startingPose)
    {
        for (int i = 0; i < io.length; i++) {
            io[i].zeroQuest(startingPose);
        }
    }

    @Override
    public void periodic()
    {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // If useLeft skip right camera (2) if not skip left camera (1)

            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent() && !rejectedTags.contains(tagId)) {
                    tagPoses.add(tagPose.get());
                    seesThisTarget = true;
                }
            }

            // Report to visionhas Target whether or not vision sees at least one tag
            hasNewTarget = false;
            if (seesThisTarget) {
                if (!visionHasTarget) {
                    hasNewTarget = true;
                }
                visionHasTarget = true;
                // Now reset seesThisTarget for next periodic loop
                seesThisTarget = false;
            } else {
                visionHasTarget = false;
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                    observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                            && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z
                        // coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations

                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) /
                    observation.tagCount();
                double linearStdDev = linearStdDevBaseline *
                    stdDevFactor;
                if (cameraIndex < cameraStdDevFactors.length) {
                    if (shiftTrust < 0) {
                        if (cameraIndex == 0) {
                            linearStdDev *=
                            cameraStdDevFactors[cameraIndex] * (1/Math.abs(shiftTrust));
                        } else {
                            linearStdDev *=
                            cameraStdDevFactors[cameraIndex];
                        }
                    } else {
                        if (cameraIndex == 1) {
                            linearStdDev *=
                            cameraStdDevFactors[cameraIndex] * (1/Math.abs(shiftTrust));
                        } else {
                            linearStdDev *=
                            cameraStdDevFactors[cameraIndex];
                        }
                    }
                }

                if (cameraIndex != 0) {
                    // Send vision observation
                    consumer.accept(observation.pose().toPose2d(), observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, 999999));
                }
            }

            // Log camera datadata
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
            "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }

    public Pose2d getPose()
    {
        return pose;
    }

    // Positive numbers |<1| mean to trust the right camera less as that percentage
    // -0.5 would mean trust the left camera at 50%
    public void shiftTrust(double trust)
    {
        shiftTrust = trust;
    }
}
