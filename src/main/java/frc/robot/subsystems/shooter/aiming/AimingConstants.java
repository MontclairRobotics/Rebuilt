package frc.robot.subsystems.shooter.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.tunables.Tunable;

public class AimingConstants {

	private static double latency = 0.08; // seconds it takes to reach desired state once state is set
	public static double getLatency() { return latency; }
	public static Tunable latencyTunable = new Tunable("latency", latency, (input) -> latency = input);

	public record ShotSettings(Angle angle, AngularVelocity flywheelVelocity) implements Interpolatable<ShotSettings> {
		@Override
		public ShotSettings interpolate(ShotSettings endValue, double t) {
			return new ShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				RotationsPerSecond.of(MathUtil.interpolate(this.flywheelVelocity.in(RotationsPerSecond), endValue.flywheelVelocity.in(RotationsPerSecond), t))
			);
		}
    }

	public record SimShotSettings(Angle angle, LinearVelocity exitVelocity) implements Interpolatable<SimShotSettings> {
		@Override
		public SimShotSettings interpolate(SimShotSettings endValue, double t) {
			return new SimShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				MetersPerSecond.of(MathUtil.interpolate(this.exitVelocity.in(MetersPerSecond), endValue.exitVelocity.in(MetersPerSecond), t))
			);
		}
    }

	// parameter maps

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	// time of flight maps

	public static final InterpolatingDoubleTreeMap REAL_TOF_MAP =
		new InterpolatingDoubleTreeMap();

	public static final InterpolatingDoubleTreeMap REAL_FERRY_TOF_MAP =
		new InterpolatingDoubleTreeMap();

	public static final InterpolatingDoubleTreeMap SIM_TOF_MAP =
		new InterpolatingDoubleTreeMap();

	public static final InterpolatingDoubleTreeMap SIM_FERRY_TOF_MAP =
		new InterpolatingDoubleTreeMap();

	static {

		REAL_MAP.put(0.00, new ShotSettings(Degrees.of(0.0), RotationsPerSecond.of(19)));
		REAL_MAP.put(1.50, new ShotSettings(Degrees.of(0), RotationsPerSecond.of(19.4)));
		REAL_MAP.put(2.49, new ShotSettings(Degrees.of(9.5), RotationsPerSecond.of(21.3)));
		REAL_MAP.put(3.49, new ShotSettings(Degrees.of(10), RotationsPerSecond.of(22.8)));
		REAL_MAP.put(3.98, new ShotSettings(Degrees.of(10.8), RotationsPerSecond.of(23.4)));
		REAL_MAP.put(4.28, new ShotSettings(Degrees.of(12.5), RotationsPerSecond.of(24)));
		REAL_MAP.put(4.74, new ShotSettings(Degrees.of(13), RotationsPerSecond.of(25.1)));
		REAL_MAP.put(5.07, new ShotSettings(Degrees.of(14.7), RotationsPerSecond.of(25.7)));
		REAL_MAP.put(5.73, new ShotSettings(Degrees.of(16), RotationsPerSecond.of(26.7)));
		REAL_MAP.put(7.00, new ShotSettings(Degrees.of(16), RotationsPerSecond.of(28.2)));

		//double x = 0.0;

		// REAL_MAP.put(7.00, new ShotSettings(Degrees.of(20), RotationsPerSecond.of(29)));
		// REAL_MAP.put(5.56895, new ShotSettings(Degrees.of(20), RotationsPerSecond.of(27.84)));
		// REAL_MAP.put(5.08635, new ShotSettings(Degrees.of(20), RotationsPerSecond.of(26.37)));
		// REAL_MAP.put(4.70535, new ShotSettings(Degrees.of(19), RotationsPerSecond.of(25.8)));
		// REAL_MAP.put(4.2164, new ShotSettings(Degrees.of(18), RotationsPerSecond.of(24.99)));
		// REAL_MAP.put(3.58648, new ShotSettings(Degrees.of(17), RotationsPerSecond.of(24.36)));
		// REAL_MAP.put(2.92862, new ShotSettings(Degrees.of(11), RotationsPerSecond.of(23.15)));
		// REAL_MAP.put(2.49, new ShotSettings(Degrees.of(9.5), RotationsPerSecond.of(22)));
		// REAL_MAP.put(1.50, new ShotSettings(Degrees.of(0), RotationsPerSecond.of(20)));
		// REAL_MAP.put(0.00, new ShotSettings(Degrees.of(0.0), RotationsPerSecond.of(19.5)));

		REAL_TOF_MAP.put(0.00, 0.8);
		REAL_TOF_MAP.put(1.50, 0.81);
		REAL_TOF_MAP.put(2.50, 0.86);
		REAL_TOF_MAP.put(3.45, 0.98);
		REAL_TOF_MAP.put(4.52, 1.1);
		REAL_TOF_MAP.put(5.11, 1.14);
		REAL_TOF_MAP.put(7.00, 1.2);

		REAL_FERRY_MAP.put(20.0, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(40)));
		REAL_FERRY_MAP.put(9.59, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(34)));
		REAL_FERRY_MAP.put(8.67, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(30.4)));
		REAL_FERRY_MAP.put(6.63, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(26.9)));
		REAL_FERRY_MAP.put(5.15, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(24.5)));
		REAL_FERRY_MAP.put(3.67, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(22.44)));
		REAL_FERRY_MAP.put(0.0, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(15)));

		REAL_FERRY_TOF_MAP.put(20.0, 3.0);
		REAL_FERRY_TOF_MAP.put(9.59, 1.8);
		REAL_FERRY_TOF_MAP.put(8.67, 1.78);
		REAL_FERRY_TOF_MAP.put(6.63, 1.38);
		REAL_FERRY_TOF_MAP.put(5.15, 1.22);
		REAL_FERRY_TOF_MAP.put(3.67, 1.06);
		REAL_FERRY_TOF_MAP.put(0.0, 1.0);

		SIM_MAP.put(0.99912, new SimShotSettings(Degrees.of(7), MetersPerSecond.of(7)));
		SIM_MAP.put(2.005294, new SimShotSettings(Degrees.of(16), MetersPerSecond.of(7)));
		SIM_MAP.put(3.000977, new SimShotSettings(Degrees.of(22), MetersPerSecond.of(7.4)));
		SIM_MAP.put(3.993633, new SimShotSettings(Degrees.of(26), MetersPerSecond.of(8.2)));
		SIM_MAP.put(4.998351, new SimShotSettings(Degrees.of(33), MetersPerSecond.of(8.6)));
		SIM_MAP.put(5.714192, new SimShotSettings(Degrees.of(37), MetersPerSecond.of(9)));

		SIM_TOF_MAP.put(0.99912, 1.12);
		SIM_TOF_MAP.put(2.005294, 1.12);
		SIM_TOF_MAP.put(3.000977, 1.0);
		SIM_TOF_MAP.put(3.993633, 1.16);
		SIM_TOF_MAP.put(4.998351, 1.1);
		SIM_TOF_MAP.put(5.714192, 1.14);

		SIM_FERRY_MAP.put(9.59, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(10.75)));
		SIM_FERRY_MAP.put(3.6669131055202966, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(6.6)));//
		SIM_FERRY_MAP.put(8.666543502404746, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(11)));//
		SIM_FERRY_MAP.put(6.628582671689323, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(8.5)));//
		SIM_FERRY_MAP.put(5.147707889525594, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(7.5)));//

		SIM_FERRY_TOF_MAP.put(9.59, 1.8);
		SIM_FERRY_TOF_MAP.put(3.6669131055202966, 1.06);
		SIM_FERRY_TOF_MAP.put(8.666543502404746, 1.78);
		SIM_FERRY_TOF_MAP.put(6.628582671689323, 1.38);
		SIM_FERRY_TOF_MAP.put(5.147707889525594, 1.22);

	}

	public record ShootingParameters(
		Angle robotRelativeTurretAngle,
		Angle hoodAngle,
		AngularVelocity flywheelVelocity,
		double timeSecondsForSetpoint) {}

	public record SimShootingParameters(
		Angle robotRelativeTurretAngle,
		Angle hoodAngle,
		LinearVelocity exitVelocity) {}

}
