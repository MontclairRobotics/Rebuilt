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
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class AimingConstants {

	public static double LATENCY = 0.02; // seconds it takes to reach desired state once state is set

	public record ShotSettings(Angle angle, AngularVelocity flywheelVelocity) implements Interpolatable<ShotSettings> {
		public ShotSettings(Angle angle, AngularVelocity flywheelVelocity) {
			this.angle = angle;
			this.flywheelVelocity = flywheelVelocity;
		}

		@Override
		public ShotSettings interpolate(ShotSettings endValue, double t) {
			return new ShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				RotationsPerSecond.of(MathUtil.interpolate(this.flywheelVelocity.in(RotationsPerSecond), endValue.flywheelVelocity.in(RotationsPerSecond), t))
			);
		}
    }

	public record SimShotSettings(Angle angle, LinearVelocity exitVelocity) implements Interpolatable<SimShotSettings> {
		public SimShotSettings(Angle angle, LinearVelocity exitVelocity) {
			this.angle = angle;
			this.exitVelocity = exitVelocity;
		}

		@Override
		public SimShotSettings interpolate(SimShotSettings endValue, double t) {
			return new SimShotSettings(
				Rotations.of(MathUtil.interpolate(this.angle.in(Rotations), endValue.angle.in(Rotations), t)),
				MetersPerSecond.of(MathUtil.interpolate(this.exitVelocity.in(MetersPerSecond), endValue.exitVelocity.in(MetersPerSecond), t))
			);
		}
    }

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingDoubleTreeMap REAL_TIME_MAP = 
		new InterpolatingDoubleTreeMap();

	public static final InterpolatingTreeMap<Double, ShotSettings> REAL_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotSettings::interpolate);

	public static final InterpolatingDoubleTreeMap REAL_FERRY_TIME_MAP = 
		new InterpolatingDoubleTreeMap();

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	public static final InterpolatingDoubleTreeMap SIM_TIME_MAP = 
		new InterpolatingDoubleTreeMap();

	public static final InterpolatingTreeMap<Double, SimShotSettings> SIM_FERRY_MAP =
		new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), SimShotSettings::interpolate);

	public static final InterpolatingDoubleTreeMap SIM_FERRY_TIME_MAP = 
		new InterpolatingDoubleTreeMap();

	static {

		/** SCORING MAP PARAMETERS */

		// fake start point, avoids code crashes
		REAL_MAP.put(0.00, new ShotSettings(Degrees.of(0.0), RotationsPerSecond.of(20)));
		
		// real data
		REAL_MAP.put(1.21, new ShotSettings(Degrees.of(0), RotationsPerSecond.of(21.2)));
		REAL_MAP.put(2.56, new ShotSettings(Degrees.of(9.5), RotationsPerSecond.of(23)));
		REAL_MAP.put(3.78, new ShotSettings(Degrees.of(16.5), RotationsPerSecond.of(24)));
		REAL_MAP.put(5.48, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(27)));
		
		// fake end point, avoids code crashes
		REAL_MAP.put(7.00, new ShotSettings(Degrees.of(24.5), RotationsPerSecond.of(33)));

		/** SCORING MAP TIME OF FLIGHT VALUES */

		// fake start point
		REAL_TIME_MAP.put(0.0, 1.0);

		// real data
		REAL_TIME_MAP.put(1.21, 0.968);
		REAL_TIME_MAP.put(2.56, 0.9);
		REAL_TIME_MAP.put(3.78, 0.9);
		REAL_TIME_MAP.put(5.48, 1.027);

		// fake end point
		REAL_TIME_MAP.put(7.00, 1.05);

		/** FERRY MAP PARAMETERS */

		// fake start point
		REAL_FERRY_MAP.put(20.0, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(50)));
		
		// real data
		REAL_FERRY_MAP.put(9.59, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(40)));
		REAL_FERRY_MAP.put(8.67, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(37.4)));
		REAL_FERRY_MAP.put(6.63, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(28.9)));
		REAL_FERRY_MAP.put(5.15, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(25.5)));
		REAL_FERRY_MAP.put(3.67, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(22.44)));
		
		// fake end point
		REAL_FERRY_MAP.put(0.0, new ShotSettings(Degrees.of(22.5), RotationsPerSecond.of(15)));

		/** FERRY MAP TIME OF FLIGHT VALUES */

		// fake start point
		REAL_FERRY_TIME_MAP.put(0.0, 3.0);

		// real data
		REAL_FERRY_TIME_MAP.put(0.0, 1.8);
		REAL_FERRY_TIME_MAP.put(0.0, 1.78);
		REAL_FERRY_TIME_MAP.put(0.0, 1.38);
		REAL_FERRY_TIME_MAP.put(0.0, 1.22);
		REAL_FERRY_TIME_MAP.put(0.0, 1.06);

		// fake end point
		REAL_FERRY_TIME_MAP.put(0.0, 1.0);

		/** SIMULATION SCORING MAP PARAMETERS */

		// real data
		SIM_MAP.put(0.99912, new SimShotSettings(Degrees.of(7), MetersPerSecond.of(7)));
		SIM_MAP.put(2.005294, new SimShotSettings(Degrees.of(16), MetersPerSecond.of(7)));
		SIM_MAP.put(3.000977, new SimShotSettings(Degrees.of(22), MetersPerSecond.of(7.4)));
		SIM_MAP.put(3.993633, new SimShotSettings(Degrees.of(26), MetersPerSecond.of(8.2)));
		SIM_MAP.put(4.998351, new SimShotSettings(Degrees.of(33), MetersPerSecond.of(8.6)));
		SIM_MAP.put(5.714192, new SimShotSettings(Degrees.of(37), MetersPerSecond.of(9)));

		/** SIMULATION SCORING MAP TIME OF FLIGHT VALUES */

		// real data
		SIM_TIME_MAP.put(0.99912, 1.12);
		SIM_TIME_MAP.put(2.005294, 1.12);
		SIM_TIME_MAP.put(3.000977, 1.0);
		SIM_TIME_MAP.put(3.993633, 1.16);
		SIM_TIME_MAP.put(4.998351, 1.1);
		SIM_TIME_MAP.put(5.714192, 1.14);

		/** SIMULATION FERRY MAP PARAMETERS */

		// real data
		SIM_FERRY_MAP.put(9.59, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(10.75)));
		SIM_FERRY_MAP.put(3.6669131055202966, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(6.6)));
		SIM_FERRY_MAP.put(8.666543502404746, new SimShotSettings(Degrees.of(30), MetersPerSecond.of(11)));
		SIM_FERRY_MAP.put(6.628582671689323, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(8.5)));
		SIM_FERRY_MAP.put(5.147707889525594, new SimShotSettings(Degrees.of(40), MetersPerSecond.of(7.5)));
	
		/** SIMULATION FERRY MAP TIME OF FLIGHT VALUES */

		// real data
		SIM_FERRY_TIME_MAP.put(9.59, 1.8);
		SIM_FERRY_TIME_MAP.put(3.6669131055202966, 1.06);
		SIM_FERRY_TIME_MAP.put(8.666543502404746, 1.78);
		SIM_FERRY_TIME_MAP.put(6.628582671689323, 1.38);
		SIM_FERRY_TIME_MAP.put(5.147707889525594, 1.22);

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
