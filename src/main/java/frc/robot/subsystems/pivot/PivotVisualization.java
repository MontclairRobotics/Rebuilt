package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class PivotVisualization {
  // pivot 3d pose (x, y, z pos and rotations)
  public Pose3d pivotPose = new Pose3d();

  // update current place (always middle of robot) plus the rotation OF the intake pose
  public void update() {
    pivotPose =
        new Pose3d(
            0,
            0,
            0,
            new Rotation3d(
                Angle.ofBaseUnits(0, Rotations),
                RobotContainer.pivot.io.getPivotAngle(),
                Angle.ofBaseUnits(0, Rotations)));
  }

  // log the pose
  public void log() {
    Logger.recordOutput("Pivot/PivotPose", pivotPose);
  }
}
