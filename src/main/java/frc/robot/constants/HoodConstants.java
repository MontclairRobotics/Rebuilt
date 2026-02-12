package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class HoodConstants {

	public static final Translation3d ORIGIN_TO_HOOD = new Translation3d(-0.1651, 0.2782, 0.4395);
	public static final Translation3d TURRET_TO_HOOD = new Translation3d(-0.113, 0, 0.08128);

	// ports
	public static final int CAN_ID = 37;
	public static final int ENCODER_PORT = 4;

	// constraints
	public static final Angle MIN_ANGLE = Degrees.of(0);
	public static final Angle MAX_ANGLE = Degrees.of(36);
	public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(3);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(8);

	// physical properties
	public static final double GEARING = 10.8; // for every rotation of the hood the motor shaft rotates this amount
	public static final double MOMENT_OF_INERTIA = 0.03038161694; //MOE from CAD around the pivot of the hoot on 2026.02.03
	public static final Angle HOOD_ENCODER_OFFSET = Rotations.of(0); // absolute encoder
	public static final Distance HOOD_LENGTH = Meters.of(0.305);
	public static final double VOLTAGE_LIMIT = 12;


	// pid
	public static final double kP = 20;
	public static final double kI = 0;
	public static final double kD = 1;
	public static final double kS = 0;
	public static final double kV = 0;
	public static final double kG = 0;

	public static final Angle TOLERANCE = Angle.ofBaseUnits(0.002, Rotations);

}
