package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.tunables.ControlConstants;
import frc.robot.util.tunables.TunableControlConstants;

public class HoodConstants {

	// CAD transformations
	public static final Translation3d ORIGIN_TO_HOOD = new Translation3d(-0.1651, 0.2782, 0.4395);
	public static final Translation3d TURRET_TO_HOOD = new Translation3d(-0.113, 0, 0.08128);

	// ports
	public static final int CAN_ID = -1;
	public static final int ENCODER_PORT = -1;

	// constraints
	public static final Angle MIN_ANGLE = Degrees.of(0);
	public static final Angle MAX_ANGLE = Degrees.of(28);

	// physical properties
	public static final double GEARING = 10.8; // for every rotation of the hood the motor shaft rotates this amount
	public static final double MOMENT_OF_INERTIA = 0.03038161694; // from CAD on 3/3/2026
	public static final Angle HOOD_ENCODER_OFFSET = Rotations.of(0); // absolute encoder
	public static final Distance HOOD_LENGTH = Meters.of(0.25);

	// pid
	public static final double kP = 20;
	public static final double kI = 0;
	public static final double kD = 1;
	public static final double kS = 0;
	public static final double kV = 0;
	public static final double kG = 0;
	public static final double kA = 0;

	public static final Angle TOLERANCE = Angle.ofBaseUnits(0.002, Rotations);

	public static final ControlConstants GAINS = new ControlConstants()
		.withPID(kP, kI, kD)
		.withFeedforward(kV, kA)
		.withPhysical(kS, kG)
		.withTolerance(TOLERANCE.in(Rotations));

	public static final TunableControlConstants TUNABLE_GAINS = new TunableControlConstants(
		"Hood", GAINS
	);
}
