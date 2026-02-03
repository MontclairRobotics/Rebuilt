package frc.robot.constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShootingLUT {
  public record ShooterParameters(double angle, double speed, double timeOfFlight)
      implements Interpolatable<ShooterParameters> {
    public ShooterParameters(double angle, double speed, double timeOfFlight) {
      this.angle = angle;
      this.speed = speed;
      this.timeOfFlight = timeOfFlight;
    }

    @Override
    public ShooterParameters interpolate(ShooterParameters endValue, double t) {
      return new ShooterParameters(
          MathUtil.interpolate(this.angle, endValue.angle, t),
          MathUtil.interpolate(this.speed, endValue.speed, t),
          MathUtil.interpolate(this.timeOfFlight, endValue.timeOfFlight, t));
    }
  }

  public static final InterpolatingTreeMap<Double, ShooterParameters> PARAMETER_MAP =
      new InterpolatingTreeMap<Double, ShooterParameters>(
          InverseInterpolator.forDouble(), ShooterParameters::interpolate);

  static {
    PARAMETER_MAP.put(0.0, new ShooterParameters(0, 0, 0));
  }
}
