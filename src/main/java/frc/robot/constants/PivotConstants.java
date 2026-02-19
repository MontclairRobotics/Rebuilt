package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class PivotConstants {

	// ports & offsets
	public static final int CAN_ID = -1;
	public static final int ENCODER_PORT = 0; //TODO: set this number
	public static final double ENCODER_OFFSET = 0;

	// constraints
	public static final Angle MIN_ANGLE = Degrees.of(0);
	public static final Angle MAX_ANGLE = Degrees.of(90);

	// physical properties
	public static final double GEARING = 50; // rotations of the motor shaft per rotations of the intake pivot
	public static final Distance ARM_LENGTH = Meters.of(0.5);
	public static final double MOMENT_OF_INERTIA = 1; // TODO: set

	// pid gains
	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;

	// ff gains
	public static final double kS = 0;
	public static final double kG = 0;
	public static final double kV = 0;

	public static final Angle TOLERANCE = Rotations.of(0.002);
}
