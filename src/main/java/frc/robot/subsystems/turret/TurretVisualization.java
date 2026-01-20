package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import org.littletonrobotics.junction.Logger;

public class TurretVisualization {
  private Pose2d turretPose;

  public void update(TurretIOInputs inputs) {
    turretPose = new Pose2d(0, 0, Rotation2d.fromRotations(inputs.robotRelativeAngle));
  }

  public void log() {
    Logger.recordOutput("Turret/TurretPose", turretPose);
  }
}
