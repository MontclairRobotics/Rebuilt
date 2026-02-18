package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;

public class TurretVisualization {

	public Pose3d turretPose = new Pose3d();

	public void update() {
		// turretPose = new Pose2d(
		// 	0,
		// 	0,
		// 	Rotation2d.fromRotations(RobotContainer.turret.io.getRobotRelativeAngle().in(Rotations))
		// );
		turretPose = new Pose3d(TurretConstants.ORIGIN_TO_TURRET, new Rotation3d(Rotations.zero(), Rotations.zero(), RobotContainer.turret.getRobotRelativeAngle()));
	}

	public void log() {
		Logger.recordOutput(
			"Turret/TurretPose",
			turretPose
		);
	}
}
