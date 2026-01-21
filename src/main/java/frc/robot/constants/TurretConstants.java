package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {
  // TODO: IMPLEMENT
  public static final int CAN_ID = -1;
  public static final Angle MAX_ANGLE = Rotations.of(1);
  public static final double GEAR_RATIO =
      0; // how many rotations of the output shaft per rotation of the turret
  public static final double MOMENT_OF_INERTIA = 0;
  public static final double LENGTH = 0;
  public static final double SIM_KP = 0;
  public static final double SIM_KI = 0;
  public static final double SIM_KD = 0;
  public static final double TALON_KS = 0;
  public static final double TALON_KV = 0;
  public static final double TALON_KA = 0;
  public static final double TALON_KP = 0;
  public static final double TALON_KI = 0;
  public static final double TALON_KD = 0;
  public static final AngularVelocity TURRET_CRUISE_VELOCITY =
      RotationsPerSecond.of(4); // target cruise velocity of the turret, 4 rps
  public static final AngularAcceleration TURRET_ACCELERATION =
      RotationsPerSecondPerSecond.of(16); // target acceleration of the turret, 16 rps^2
  public static final double TURRET_JERK =
      160; // target jerk of the turret, _160 rps^3 -> there's no AngularJerk sadly
  public static final Angle ANGLE_TOLERANCE = Rotations.of(0.001);
}
