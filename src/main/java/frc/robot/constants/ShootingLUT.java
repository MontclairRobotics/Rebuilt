package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ShootingLUT {
  public record ShooterParameters(Angle angle, LinearVelocity speed, Time timeOfFlight)
        implements Interpolatable<ShooterParameters> {
      public ShooterParameters(Angle angle, LinearVelocity speed, Time timeOfFlight) {
        this.angle = angle;
        this.speed = speed;
        this.timeOfFlight = timeOfFlight;
      }

      @Override
      public ShooterParameters interpolate(ShooterParameters endValue, double t) {
        return new ShooterParameters(
            Radians.of(MathUtil.interpolate(this.angle.in(Radians), endValue.angle.in(Radians), t)),
            MetersPerSecond.of(MathUtil.interpolate(this.speed.in(MetersPerSecond), endValue.speed.in(MetersPerSecond), t)),
            Seconds.of(MathUtil.interpolate(this.timeOfFlight.in(Seconds), endValue.timeOfFlight.in(Seconds), t)));
      }
    }

  public static final InterpolatingTreeMap<Double, ShooterParameters> PARAMETER_MAP =
      new InterpolatingTreeMap<Double, ShooterParameters>(
          InverseInterpolator.forDouble(), ShooterParameters::interpolate);
  public static final InterpolatingTreeMap<Double, ShooterParameters> CONSTANT_VELOCITY_MAP =
      new InterpolatingTreeMap<Double, ShooterParameters>(
          InverseInterpolator.forDouble(), ShooterParameters::interpolate);
  static {
    PARAMETER_MAP.put(0.0, new ShooterParameters(Radians.of(0), MetersPerSecond.of(0), Seconds.of(0)));
  }
}
