package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class HoodConstants {

	// CAD values
	public static final Translation3d ORIGIN_TO_HOOD = new Translation3d(-0.1651, 0.2782, 0.4395);
	public static final Translation3d TURRET_TO_HOOD = new Translation3d(-0.08, 0.08, 0.08128);

	// CAN
	public static final int CAN_ID = 33;
	public static final int ENCODER_ID = 34;
	public static final CANBus CAN_BUS = new CANBus("turret");

	// Constraints
	public static final Angle MIN_ANGLE = Degrees.of(0);
	public static final Angle MAX_ANGLE = Rotations.of(0.93);

	// Physical properties
	public static final double GEARING = 176; // from Max Pearson on 2026.02.20
	public static final double SENSOR_TO_MECHANISM_RATIO = 176.0 / 10;
	public static final double ROTOR_TO_SENSOR_RATIO = 10;
	public static final double MOMENT_OF_INERTIA = 0.03038161694; // from CAD on 3/3/2026
	public static final Angle HOOD_ENCODER_OFFSET = Rotations.of(0.251944); // absolute encoder
	public static final Distance HOOD_LENGTH = Meters.of(0.25);
	public static final double HOOD_LOWER_TIME = 0.6;

	// Gains (Tuned 2/22)
	public static final double kP = 1600;
	public static final double kD = 0;
	public static final double kS = 0;
	public static final double kG = 0.3;

	public static final Angle TOLERANCE = Degrees.of(1);

	public static final double STATOR_CURRENT_LIMIT = 80; // Amps
	public static final double SUPPLY_CURRENT_LIMIT = 40; // Amps

	public static final AngularVelocity MOTION_MAGIC_CRUISE_VELOCITY = RotationsPerSecond.of(1);
	public static final AngularAcceleration MOTION_MAGIC_ACCELERATION = RotationsPerSecondPerSecond.of(2);
	public static final double MOTION_MAGIC_JERK = 40; // Rotations Per Second Per Second Per Second
	public static AngularVelocity MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(0.05);

	// Configs
	public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD)
		.withKS(kS).withKG(kG)
		.withGravityType(GravityTypeValue.Arm_Cosine);

	public static final Slot0Configs SIM_SLOT0_CONFIGS = new Slot0Configs()
		.withKP(2000).withKD(0)
		.withKS(0).withKG(0.3);

	public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
		.withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
		.withStatorCurrentLimitEnable(true)
		.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
		.withSupplyCurrentLimitEnable(true);

	public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withInverted(InvertedValue.Clockwise_Positive)
		.withNeutralMode(NeutralModeValue.Brake);

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
		.withFeedbackRemoteSensorID(ENCODER_ID)
		.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
		.withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
		.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

	public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
		.withMotionMagicCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY)
		.withMotionMagicAcceleration(MOTION_MAGIC_ACCELERATION)
		.withMotionMagicJerk(MOTION_MAGIC_JERK);

	public static final CANcoderConfiguration ENCODER_CONFIGS = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(HOOD_ENCODER_OFFSET)
        );

}
