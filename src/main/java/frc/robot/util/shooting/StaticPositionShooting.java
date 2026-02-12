package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.ShootingLUT.*;
import static frc.robot.util.FieldConstants.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;

public class StaticPositionShooting {
    public static Translation2d getFieldRelativePosition() {
        return RobotContainer.turret.getFieldRelativePosition();
    }
    public static double getDistanceToHub() {
        Translation2d hublocation = PoseUtils.flipTranslationAlliance(Hub.HUB_LOCATION);
        return getFieldRelativePosition().getDistance(hublocation);
    }
  public Distance getDistance() {
    double distance = getDistanceToHub();
    return Meters.of(distance);
  }

  public Command staticShoot() {
    return RobotContainer.hood.setAngleCommand(getParametersFor(getDistance()).angle());
  }
  public static ShooterParameters getParametersFor(Distance distance) {
    return PARAMETER_MAP.get(distance.in(Meters));
  }
}
