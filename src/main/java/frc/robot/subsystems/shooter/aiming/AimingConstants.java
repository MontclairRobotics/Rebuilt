package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class AimingConstants {

	public static double LATENCY = 0.02; // seconds it takes to reach desired state once state is set

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

		SIM_CONSTANT_VELOCITY_MAP.put(0.8516690912634933, new SimShotSettings(Degrees.of(3), MetersPerSecond.of(9), Seconds.of(1.52), true));
		SIM_CONSTANT_VELOCITY_MAP.put(2.0927103773901443, new SimShotSettings(Degrees.of(10), MetersPerSecond.of(9), Seconds.of(1.504), true));
		SIM_CONSTANT_VELOCITY_MAP.put(3.3339369176475455, new SimShotSettings(Degrees.of(15), MetersPerSecond.of(9), Seconds.of(1.512), true));
		SIM_CONSTANT_VELOCITY_MAP.put(4.5789466791916436, new SimShotSettings(Degrees.of(25), MetersPerSecond.of(9), Seconds.of(1.34), true));
		SIM_CONSTANT_VELOCITY_MAP.put(5.8242079384087, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(9), Seconds.of(1.04), true));

		SIM_MAP.put(0.99912, new SimShotSettings(Degrees.of(7), MetersPerSecond.of(7), Seconds.of(1.12), false));
		SIM_MAP.put(2.005294, new SimShotSettings(Degrees.of(16), MetersPerSecond.of(7), Seconds.of(1.12), false));
		SIM_MAP.put(3.000977, new SimShotSettings(Degrees.of(22), MetersPerSecond.of(7.4), Seconds.of(1), false));
		SIM_MAP.put(3.993633, new SimShotSettings(Degrees.of(26), MetersPerSecond.of(8.2), Seconds.of(1.16), false));
		SIM_MAP.put(4.998351, new SimShotSettings(Degrees.of(33), MetersPerSecond.of(8.6), Seconds.of(1.1), false));
		SIM_MAP.put(5.714192, new SimShotSettings(Degrees.of(37), MetersPerSecond.of(9), Seconds.of(1.14), false));
		SIM_FERRY_MAP.put(9.59, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(10.75), Seconds.of(1.8), false));
		SIM_FERRY_MAP.put(3.6669131055202966, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(6.6), Seconds.of(1.06), false));//
		SIM_FERRY_MAP.put(8.666543502404746, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(11), Seconds.of(1.78), false));//
		SIM_FERRY_MAP.put(6.628582671689323, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(8.5), Seconds.of(1.38), false));//
		SIM_FERRY_MAP.put(5.147707889525594, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(7.5), Seconds.of(1.22), false));//
		SIM_CONSTANT_VELOCITY_FERRY_MAP.put(0.0, new SimShotSettings(Degrees.zero(), MetersPerSecond.zero(), Seconds.zero(), true));
	}

	public record ShootingParameters(Angle robotRelativeTurretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
		public ShootingParameters(Angle robotRelativeTurretAngle, Angle hoodAngle, AngularVelocity flywheelVelocity) {
			this.robotRelativeTurretAngle = robotRelativeTurretAngle;
			this.hoodAngle = hoodAngle;
			this.flywheelVelocity = flywheelVelocity;
		}
	}

	public record SimShootingParameters(Angle robotRelativeTurretAngle, Angle hoodAngle, LinearVelocity exitVelocity) {
		public SimShootingParameters(Angle robotRelativeTurretAngle, Angle hoodAngle, LinearVelocity exitVelocity) {
			this.robotRelativeTurretAngle = robotRelativeTurretAngle;
			this.hoodAngle = hoodAngle;
			this.exitVelocity = exitVelocity;
		}
	}

}
