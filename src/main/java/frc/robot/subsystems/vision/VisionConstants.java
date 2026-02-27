// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
	// AprilTag layout
	public static AprilTagFieldLayout aprilTagLayout =
		AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	// Camera names, must match names configured on coprocessor
	// TODO: name these better
	public static String camera1Name = "limelight-one"; // back limelight
	public static String camera2Name = "limelight-two"; // other limelight
	public static String camera3Name = "limelight-three"; // limelight 3


	// Basic filtering thresholds
	public static double maxAmbiguity = 0.3;
	public static double maxZError = 0.75;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static double linearStdDevBaseline = 0.02; // Meters
	public static double angularStdDevBaseline = 0.06; // Radians

	// Standard deviation multipliers for each camera
	// (Adjust to trust some cameras more than others)
	// value greater than one, trust cameras less, value less than one, trust cameras more
	// TODO: check the above statement ^^
	public static double[] cameraStdDevFactors =
		new double[] {
			1.0, // Camera 0
			1.0 // Camera 1
		};

	// Multipliers to apply for MegaTag 2 observations
	public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
	public static double angularStdDevMegatag2Factor =
		Double.POSITIVE_INFINITY; // No rotation data available
}
