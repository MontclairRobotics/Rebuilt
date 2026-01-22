package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class HoodVisualization {

  public Pose3d hoodPose;

  public void update() {
    hoodPose =
        new Pose3d(
            0,
            0,
            0,
            new Rotation3d(0, Rotations.of(RobotContainer.hood.io.getAngle()).in(Radians), 0));
  }

  public void log() {
    Logger.recordOutput("Hood/HoodPose", hoodPose);
  }
}
