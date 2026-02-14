package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class AimingConstants {

	public static final double LATENCY = 0.04; // seconds it takes to reach desired state once state is set

	public record ShotSettings(Angle angle, AngularVelocity flywheelVelocity, Time timeOfFlight, boolean withConstantVelocity) implements Interpolatable<ShotSettings> {
		public ShotSettings(Angle angle, AngularVelocity flywheelVelocity, Time timeOfFlight, boolean withConstantVelocity) {
			this.angle = angle;
			this.flywheelVelocity = flywheelVelocity;
			this.timeOfFlight = timeOfFlight;
			this.withConstantVelocity = withConstantVelocity;
		}

		@Override
		public ShotSettings interpolate(ShotSettings endValue, double t) {
			return new ShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				withConstantVelocity ? flywheelVelocity : RotationsPerSecond.of(MathUtil.interpolate(this.flywheelVelocity.in(RotationsPerSecond), endValue.flywheelVelocity.in(RotationsPerSecond), t)),
				Seconds.of(MathUtil.interpolate(this.timeOfFlight.in(Seconds), endValue.timeOfFlight.in(Seconds), t)),
				withConstantVelocity
			);
		}
    }

	public record SimShotSettings(Angle angle, LinearVelocity exitVelocity, Time timeOfFlight, boolean withConstantVelocity) implements Interpolatable<SimShotSettings> {
		public SimShotSettings(Angle angle, LinearVelocity exitVelocity, Time timeOfFlight, boolean withConstantVelocity) {
			this.angle = angle;
			this.exitVelocity = exitVelocity;
			this.timeOfFlight = timeOfFlight;
			this.withConstantVelocity = withConstantVelocity;
		}

		@Override
		public SimShotSettings interpolate(SimShotSettings endValue, double t) {
			return new SimShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				withConstantVelocity ? exitVelocity : MetersPerSecond.of(MathUtil.interpolate(this.exitVelocity.in(MetersPerSecond), endValue.exitVelocity.in(MetersPerSecond), t)),
				Seconds.of(MathUtil.interpolate(this.timeOfFlight.in(Seconds), endValue.timeOfFlight.in(Seconds), t)),
				withConstantVelocity
			);
		}
    }

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_CONSTANT_VELOCITY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_CONSTANT_VELOCITY_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_CONSTANT_VELOCITY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_CONSTANT_VELOCITY_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	static {
		REAL_MAP.put(0.0, new ShotSettings(Rotations.of(0), RotationsPerSecond.of(0), Seconds.of(0), false));
	}

	public record ShootingParameters(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
		public ShootingParameters(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
			this.turretAngle = turretAngle;
			this.hoodAngle = hoodAngle;
			this.flywheelVelocity = flywheelVelocity;
		}
	}

	public record SimShootingParameters(Angle turretAngle, Angle hoodAngle, LinearVelocity exitVelocity) {
		public SimShootingParameters(Angle turretAngle, Angle hoodAngle, LinearVelocity exitVelocity) {
			this.turretAngle = turretAngle;
			this.hoodAngle = hoodAngle;
			this.exitVelocity = exitVelocity;
		}
	}

}
