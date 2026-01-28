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
	public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(4);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(16);
	public static final Angle ANGLE_TOLERANCE = Rotations.of(0.002);
	public static final AngularVelocity ANGULAR_VELOCITY_TOLERANCE = RotationsPerSecond.of(0.002);

	// physical properties
	public static final double MOMENT_OF_INERTIA = 0.35;
	public static final double LENGTH = 0.3;
	public static final double GEARING = 12; // how many rotations of the output shaft per rotation of the turret

	// the angle between the zero of the gyro and the robot relative zero of the turret
	public static final Angle OFFSET = Rotations.of(0.25); // turret zero is perpendicular to gyro zero, pointed to the left

	// pid + ff gains
	public static final double kP = 100;
	public static final double kI = 10;
	public static final double kD = 5;
}
