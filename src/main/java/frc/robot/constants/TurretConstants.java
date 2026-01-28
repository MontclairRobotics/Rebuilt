package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretConstants {

	public static final Translation2d HUB_LOCATION = new Translation2d(4.6256, 4.034);

	// ports
	public static final int CAN_ID = -1;

	// constraints
	public static final Angle MIN_ANGLE = Rotations.of(-0.5);
	public static final Angle MAX_ANGLE = Rotations.of(0.5);
	public static final AngularVelocity TURRET_CRUISE_VELOCITY = RotationsPerSecond.of(4); // target cruise velocity of the turret, 4 rps
	public static final AngularAcceleration TURRET_ACCELERATION = RotationsPerSecondPerSecond.of(16); // target acceleration of the turret, 16 rps^2
	public static final double TURRET_JERK = 160; // target jerk of the turret, 160 rps^3
	public static final Angle ANGLE_TOLERANCE = Rotations.of(0.001);

	public static final double GEAR_RATIO = 12; // how many rotations of the output shaft per rotation of the turret

	// physical properties
	public static final double MOMENT_OF_INERTIA = 0.35;
	public static final double LENGTH = 0.3;
	public static final Translation2d TURRET_OFFSET = new Translation2d(0.1, 0.1);
	// pid + ff gains
	public static double SIM_KP = 100;
	public static double SIM_KI = 10;
	public static double SIM_KD = 5;

	public static final double TALON_KS = 0;
	public static final double TALON_KV = 0;
	public static final double TALON_KA = 0;
	public static final double TALON_KP = 0;
	public static final double TALON_KI = 0;
	public static final double TALON_KD = 0;

}
