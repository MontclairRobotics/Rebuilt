package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.ShootingLUT.*;
import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;

public class StaticPositionShooting {
    public static Translation2d getFieldRelativePosition() {
        Pose2d robotPose = RobotContainer.drivetrain.getRobotPose();
        return robotPose.getTranslation();
    }
    public static double getDistanceToHub() {
        Translation2d hublocation = PoseUtils.flipTranslationAlliance(HUB_LOCATION);
        return getFieldRelativePosition().getDistance(hublocation); // TODO: clean this up
    }
  public Distance getDistance() {
    double distance = getDistanceToHub();
    return Meters.of(distance);
  }

  public Command staticShoot() {
    return RobotContainer.hood.setAngleCommand(PARAMETER_MAP.get(getDistance().in(Meters)).angle());
  }
}
