package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class TurretVisualization {

	static int number = 0;
	public Pose2d turretPose = new Pose2d();

	public void update() {
		turretPose = new Pose2d(
			0,
			0, 
			Rotation2d.fromRotations(RobotContainer.turret.io.getRobotRelativeAngle())
		);
	}

	public void log() {
		Logger.recordOutput("Turret/turret_internal", "turret logging" + number);
		number++;
		
		Logger.recordOutput(
			"Turret/TurretPose", 
			new Pose3d(0, 0, 0, new Rotation3d(turretPose.getRotation()))
		);
	}
}
