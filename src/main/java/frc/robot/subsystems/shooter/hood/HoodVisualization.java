package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class HoodVisualization {

	public Pose3d hoodPose = new Pose3d();

	public void update() {
		hoodPose =
			new Pose3d(
				new Translation3d(0, 0, 0.3),
				new Rotation3d(Math.PI/2 - RobotContainer.hood.io.getAngle().in(Radians), 0, RobotContainer.turret.io.getRobotRelativeAngle().in(Radians)));
	}

	public void log() {
		Logger.recordOutput("Hood/Hood Pose", hoodPose);
	}
}