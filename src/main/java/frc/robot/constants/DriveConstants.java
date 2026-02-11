package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;
// import frc.robot.util.TunerConstants;

public class DriveConstants {

	// public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
	public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(8);

	public static final double JOYSTICK_INPUT_GAIN = 1;
	public static final double JOYSTICK_INPUT_ROT_GAIN = 2;
	public static final double JOYSTICK_DEADBAND = 0.1;

	public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(1.624);
	public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(4);

	// public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
	// 	MAX_SPEED,
	// 	MAX_ACCELERATION,
	// 	MAX_ANGULAR_SPEED,
	// 	MAX_ANGULAR_ACCELERATION
	// );

	public static final ControlConstants ROTATION_BASE_CONSTANTS = new ControlConstants()
		.withPID(16, 0, 0)
		.withContinuous(-Math.PI, Math.PI);
	public static final ControlConstants TRENCH_TRANSLATION_BASE_CONSTANTS = new ControlConstants()
		.withPID(4, 0, 0);

	public static final TunableControlConstants TRENCH_TRANSLATION_CONSTANTS =
        new TunableControlConstants("Drive/Trench Translation", TRENCH_TRANSLATION_BASE_CONSTANTS);
	public static final TunableControlConstants ROTATION_CONSTANTS =
		new TunableControlConstants("Drive/Rotation", ROTATION_BASE_CONSTANTS);

	public static final LinearVelocity MIN_VELOCITY_FOR_TRENCH_AND_BUMP_LOCKS = MetersPerSecond.of(1);
}
