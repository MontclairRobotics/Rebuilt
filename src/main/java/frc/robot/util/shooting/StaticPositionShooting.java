package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.ShootingLUT.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StaticPositionShooting {
  public Distance getDistance() {
    double distance = 0; // TODO: IMPLEMENT
    return Meters.of(distance);
  }

  public Command staticShoot() {
    return RobotContainer.hood.setAngleCommand(PARAMETER_MAP.get(getDistance().in(Meters)).angle());
  }
}
