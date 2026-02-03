package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class TurretVisualization {

	public Pose2d turretPose = new Pose2d();

	public void update() {
		turretPose = new Pose2d(
			0,
			0,
			Rotation2d.fromRotations(RobotContainer.turret.io.getRobotRelativeAngle().in(Rotations))
		);
	}

	public void log() {
		Logger.recordOutput(
			"Turret/TurretPose",
			new Pose3d(0, 0, 0, new Rotation3d(turretPose.getRotation()))
		);
	}
}
