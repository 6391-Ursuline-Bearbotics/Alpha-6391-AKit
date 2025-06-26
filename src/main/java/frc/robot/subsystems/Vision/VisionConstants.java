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

import java.util.Arrays;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-left";
    public static String camera1Name = "limelight-right";
    public static String questName = "quest";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(Units.inchesToMeters(9.287), Units.inchesToMeters(10.9704),
            Units.inchesToMeters(7.9167),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(-30)));
    public static Transform3d robotToCamera1 =
        new Transform3d(Units.inchesToMeters(9.287), Units.inchesToMeters(-10.9704),
            Units.inchesToMeters(7.9167),
            new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(30)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
                1.0, // Quest
                1.0, // Left
                1.0 // Right
        };

    public static List<Integer> rejectedTags = Arrays.asList(3, 4, 5, 14, 15, 16);
}
