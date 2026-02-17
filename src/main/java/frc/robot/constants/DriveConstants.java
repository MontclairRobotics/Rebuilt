package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.TunerConstants;

public class DriveConstants {

	public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
	public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(8);

	public static final double JOYSTICK_INPUT_GAIN = 1.5;
	public static final double JOYSTICK_INPUT_ROT_GAIN = 2;
	public static final double JOYSTICK_DEADBAND = 0.1;

	public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.624);
	public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(4);

	public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
		MAX_SPEED,
		MAX_ACCELERATION,
		MAX_ANGULAR_SPEED,
		MAX_ANGULAR_ACCELERATION
	);

	public static final double ROTATION_kP = 16;
	public static final double ROTATION_kI = 0;
	public static final double ROTATION_kD = 0.1;
	public static final Angle ROTATION_TOLERANCE = Degrees.of(1);

	public static final double TRENCH_TRANSLATION_kP = 4;
	public static final double TRENCH_TRANSLATION_kI = 0;
	public static final double TRENCH_TRANSLATION_kD = 0;

	public static final LinearVelocity SIGNIFICANT_VELOCITY = MetersPerSecond.of(1);

}
