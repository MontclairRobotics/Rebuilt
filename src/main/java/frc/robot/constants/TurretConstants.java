package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.tunables.ControlConstants;
import frc.robot.util.tunables.TunableControlConstants;

public class TurretConstants {

	// CAD transformations
	public static final Translation3d ORIGIN_TO_TURRET = new Translation3d(-0.1651, 0.1651, 0.3582);

	// ports
	public static final int CAN_ID = -1;

	// constraints
	public static final Angle MIN_ANGLE = Rotations.of(-0.5);
	public static final Angle MAX_ANGLE = Rotations.of(0.5);
	public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(4);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(16);
	public static final Angle ANGLE_TOLERANCE = Rotations.of(0.005);

	// physical properties
	public static final double MOMENT_OF_INERTIA = 0.154244993; //From CAD on 2026.02.12
	public static final Distance LENGTH = Meters.of(0.3);
	public static final Translation2d TURRET_OFFSET = ORIGIN_TO_TURRET.toTranslation2d();
	public static final double GEARING = 70.0/9.0; // how many rotations of the output shaft per rotation of the turret

	public static final double LATENCY = 0.04; // seconds

	// the angle between the zero of the gyro and the robot relative zero of the turret
	public static final Angle ANGLE_OFFSET = Rotations.of(0.5); // turret zero is perpendicular to gyro zero, pointed to the left

	// pid + ff gains
	public static final double kP = 138; // Tuned PID in Simulation
	public static final double kI = 10;
	public static final double kD = 5;

	public static final ControlConstants GAINS = new ControlConstants()
		.withPID(kP, kI, kD)
		.withProfiled(false)
		.withTolerance(ANGLE_TOLERANCE.in(Rotations));

	public static final TunableControlConstants TUNABLE_GAINS = new TunableControlConstants(
		"Turret", GAINS
	);
}
