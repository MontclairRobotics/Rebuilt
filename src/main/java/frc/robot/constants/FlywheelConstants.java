package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelConstants {

	public static final int LEFT_CAN_ID = 0;
	public static final int RIGHT_CAN_ID = 0;

	public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(1);

	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final double kV = 0;
	public static final double kA = 0;

	public static final double MOMENT_OF_INERTIA = 0.00233846427; // From CAD on 2026.02.11
	public static final double GEARING = 1; // gearing of motors to the BIG flywheel

}
