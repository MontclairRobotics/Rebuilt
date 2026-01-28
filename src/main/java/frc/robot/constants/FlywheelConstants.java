package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelConstants {

	public static final int CAN_ID = 0; // TODO: set in Phoenix Tuner X

	public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(0.5); // TODO: decide what this should be

	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final double kS = 0;
	public static final double kG = 0;
	public static final double kV = 0;

	public static final double MOMENT_OF_INERTIA = 0; // TODO: estimate or get from CAD
	public static final double GEARING = 1; // TODO: get from CAD

}
