package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class HoodConstants {

	// CAD transformations
	public static final Translation3d ORIGIN_TO_HOOD = new Translation3d(-0.1651, 0.2782, 0.4395);
	public static final Translation3d TURRET_TO_HOOD = new Translation3d(-0.113, 0, 0.08128);

	// ports
	public static final int CAN_ID = -1;
	public static final int ENCODER_PORT = -1;

	// constraints
	public static final Angle MIN_ANGLE = Degrees.of(0);
	public static final Angle MAX_ANGLE = Degrees.of(40);

	// physical properties
	public static final double GEARING = 10.8; // for every rotation of the hood the motor shaft rotates this amount
	public static final double MOMENT_OF_INERTIA = 0.03038161694; // from CAD on 3/3/2026
	public static final Angle HOOD_ENCODER_OFFSET = Rotations.of(0); // absolute encoder
	public static final Distance HOOD_LENGTH = Meters.of(0.25);
	public static final double HOOD_LOWER_TIME = 0.6;

	// pid
	public static final double kP = 20;
	public static final double kD = 1;
	public static final double kS = 0;
	public static final double kG = 0.482952;

	public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD)
		.withKS(kS).withKG(kG);

	public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();

	public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withInverted(InvertedValue.CounterClockwise_Positive)
		.withNeutralMode(NeutralModeValue.Brake);

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final MagnetSensorConfigs ENCODER_CONFIGS = new MagnetSensorConfigs();

	public static final Angle TOLERANCE = Degrees.of(1);

}
