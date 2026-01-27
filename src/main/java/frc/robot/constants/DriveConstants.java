package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.TunerConstants;

public class DriveConstants {

	public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
	public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(8);

	/*
	 * Power that the joystick input is raised to -> controls how smooth or snappy driving is
	 * Between 1.5 and 2.5 is ideal
	 * higher = less mid speed control, more precision
	 * lower = more mid speed control, less precision
	 */
	public static final double joystickInputGain = 1.5;
	public static final double joystickInputRotGain = 2;

	public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(2);
	public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RotationsPerSecondPerSecond.of(5);

	public static final PathConstraints DEFAULT_CONSTRAINTS =
		new PathConstraints(
			MAX_SPEED,
			MAX_ACCELERATION,
			MAX_ANGULAR_SPEED,
			MAX_ANGULAR_ACCELERATION
		);
}
