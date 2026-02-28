package frc.robot.subsystems.vision;
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;


public class Vision extends SubsystemBase {
	private final VisionConsumer consumer; // lamda expression that takes in values and records a vision measurement
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;


	// Initialize logging values
	// only in debug mode
	private final List<Pose3d> allTagPoses = RobotContainer.VISION_DEBUG ? new ArrayList<>() : null;
	private final List<Pose3d> allRobotPoses = RobotContainer.VISION_DEBUG ? new ArrayList<>() : null;
	private final List<Pose3d> allRobotPosesAccepted = RobotContainer.VISION_DEBUG ? new ArrayList<>() : null;
	private final List<Pose3d> allRobotPosesRejected = RobotContainer.VISION_DEBUG ? new ArrayList<>() : null;

	// always
	private final List<Pose3d> tagPoses = new ArrayList<>();
	private final List<Pose3d> robotPoses = new ArrayList<>();
	private final List<Pose3d> robotPosesAccepted = new ArrayList<>();
	private final List<Pose3d> robotPosesRejected = new ArrayList<>();

	private int logCounter = 0;
	private final int loopsPerLog;

	public Vision(VisionConsumer consumer, VisionIO... io) {
		this.consumer = consumer;
		this.io = io;

		// 5 hz logging normally, up to 10 hz when in debug
		// 50 hz / 5 loops per log = 10 hz
		// 50 hz / 10 loops per log = 5 hz
		loopsPerLog = RobotContainer.VISION_DEBUG ? 5 : 10;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] =new Alert(
				"Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning
			);
		}
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing with vision.
	 *
	 * @param cameraIndex The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		logCounter++;

		// debug mode specific
		if(RobotContainer.VISION_DEBUG) {
			allRobotPoses.clear();
			allTagPoses.clear();
			allRobotPosesAccepted.clear();
			allRobotPosesRejected.clear();
		}

		// always
		tagPoses.clear();
		robotPoses.clear();
		robotPosesAccepted.clear();
		robotPosesRejected.clear();

		// loops through the different cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {

			io[cameraIndex].updateInputs(inputs[cameraIndex]);

			if (logCounter % loopsPerLog == 0) {
				Logger.processInputs("Vision/Camera" + Integer.toString(cameraIndex), inputs[cameraIndex]);
			}

			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// clear camera-specific arrays every time
			robotPoses.clear();
			tagPoses.clear();
			robotPosesAccepted.clear();
			robotPosesRejected.clear();

			// Add tag poses
			if (logCounter % loopsPerLog == 0) {
				for (int tagId : inputs[cameraIndex].tagIds) {
					var tagPose = aprilTagLayout.getTagPose(tagId);
					if (tagPose.isPresent()) {
						tagPoses.add(tagPose.get());
					}
				}
			}

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				// Check whether to reject pose
				boolean rejectPose =
					observation.tagCount() == 0 // Must have at least one tag
					|| (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
					|| Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

					// Must be within the field boundaries
					|| observation.pose().getX() < 0.0
					|| observation.pose().getX() > aprilTagLayout.getFieldLength()
					|| observation.pose().getY() < 0.0
					|| observation.pose().getY() > aprilTagLayout.getFieldWidth()

					// max angular rate
					|| RobotContainer.drivetrain.getAngularSpeed().in(DegreesPerSecond) > 720
					// max tag distance
					|| observation.averageTagDistance() > 5.0;

				// Add pose to log
				if (logCounter % loopsPerLog == 0) {
					robotPoses.add(observation.pose());
					if (rejectPose) {
						robotPosesRejected.add(observation.pose());
					} else {
						robotPosesAccepted.add(observation.pose());
					}
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double d = observation.averageTagDistance();
				double stdDevFactor = (d * d) / Math.pow(observation.tagCount(), 1.5);
				double linearStdDev = linearStdDevBaseline * stdDevFactor;
				double angularStdDev = angularStdDevBaseline * stdDevFactor;
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= linearStdDevMegatag2Factor;
					angularStdDev *= angularStdDevMegatag2Factor;
				}
				if (cameraIndex < cameraStdDevFactors.length) {
					linearStdDev *= cameraStdDevFactors[cameraIndex];
					angularStdDev *= cameraStdDevFactors[cameraIndex];
				}

				// Send vision observation
				consumer.accept(
					observation.pose().toPose2d(),
					observation.timestamp(),
					VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			if(logCounter % loopsPerLog == 0) {
				Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[0]));
				Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[0]));
				Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[0]));
				Logger.recordOutput(
					"Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[0]));
			}

			if(RobotContainer.VISION_DEBUG) {
				allTagPoses.addAll(tagPoses);
				allRobotPoses.addAll(robotPoses);
				allRobotPosesAccepted.addAll(robotPosesAccepted);
				allRobotPosesRejected.addAll(robotPosesRejected);
			}

		}

		// Log summary data
		if (RobotContainer.VISION_DEBUG && logCounter % loopsPerLog == 0) {
			Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
			Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
			Logger.recordOutput(
				"Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
			Logger.recordOutput(
				"Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
		}
	}


	@FunctionalInterface
	public static interface VisionConsumer {
		public void accept(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs
		);
	}
}
