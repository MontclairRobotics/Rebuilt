package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.TURRET_TO_HOOD;
import static frc.robot.constants.TurretConstants.ORIGIN_TO_TURRET;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;

public class HoodVisualization {
public Pose3d hoodPose = new Pose3d();

	public void update() {
		Translation3d turretToHoodFieldRelative = new Translation3d(
			TURRET_TO_HOOD.toTranslation2d()
			.rotateBy(Rotation2d.fromRotations(RobotContainer.turret.getRobotRelativeAngle().in(Rotations))))
			.plus(new Translation3d(0, 0, TURRET_TO_HOOD.getZ())
		);

		hoodPose = new Pose3d(
			ORIGIN_TO_TURRET.plus(turretToHoodFieldRelative),
			new Rotation3d(
				Rotations.zero(),
				Rotations.of(-RobotContainer.hood.getAngle().in(Rotations)),
				RobotContainer.turret.getRobotRelativeAngle()
			)
		);
	}

	public void log() {
		Logger.recordOutput("Hood/Hood Pose", hoodPose);
	}
}
