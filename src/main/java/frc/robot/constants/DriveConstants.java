package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.TunerConstants;
import frc.robot.util.tunables.ControlConstants;
import frc.robot.util.tunables.TunableControlConstants;

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

	public static final ControlConstants ROTATION_GAINS = new ControlConstants()
		.withPID(16, 0, 0.1)
		.withContinuous(-Math.PI, Math.PI)
		.withTolerance(Degrees.of(1).in(Radians));

	public static final TunableControlConstants TUNABLE_ROTATION_GAINS = new TunableControlConstants(
		"Drive/Rotation", ROTATION_GAINS
	);

	public static final ControlConstants TRENCH_TRANSLATION_GAINS = new ControlConstants()
		.withPID(4, 0, 0);

	public static final TunableControlConstants TUNABLE_TRENCH_TRANSLATION_GAINS = new TunableControlConstants(
		"Drive/Trench Translation", TRENCH_TRANSLATION_GAINS
	);

	public static final LinearVelocity SIGNIFICANT_VELOCITY = MetersPerSecond.of(2);

}
