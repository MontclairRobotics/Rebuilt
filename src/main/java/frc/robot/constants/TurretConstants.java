package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class TurretConstants {

	// CAD transformations
	public static final Translation3d ORIGIN_TO_TURRET = new Translation3d(-0.1651, 0.1651, 0.3582);
	public static final Translation2d TURRET_OFFSET = ORIGIN_TO_TURRET.toTranslation2d();

	// ports
	public static final int CAN_ID = 30;
	public static final int ENCODER_ID = 29;
	public static final CANBus CAN_BUS = new CANBus(""); // on RoboRio CAN Bus

	// constraints
	public static final Angle MIN_ANGLE = Rotations.of(-0.5);
	public static final Angle MAX_ANGLE = Rotations.of(0.47);
	public static final Angle ANGLE_TOLERANCE = Degrees.of(3);

	// physical properties
	public static final double MOMENT_OF_INERTIA = 0.154244993; //From CAD on 2026.02.12
	public static final Distance LENGTH = Meters.of(0.3);
	public static final double ROTOR_TO_SENSOR_RATIO = 3;
	public static final double SENSOR_TO_MECHANISM_RATIO = 128.0 / 17; //128.0 / 16.0
	public static final double GEARING = ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECHANISM_RATIO; // from Max Pearson on 2026.02.20

	// the angle between the zero of the gyro and the robot relative zero of the turret
	public static final Angle ANGLE_OFFSET = Rotations.of(0.375);
	public static final Angle ENCODER_OFFSET = Rotations.of(-0.14306640625);

	// pid + ff gains
	public static final double kP = 130.0555;
	public static final double kD = 0;
	public static final double kS = 2;

	public static final AngularVelocity MOTION_MAGIC_CRUISE_VELOCITY = RotationsPerSecond.of(3);
	public static final AngularAcceleration MOTION_MAGIC_ACCELERATION = RotationsPerSecondPerSecond.of(6);
	public static final double MOTION_MAGIC_JERK = 50; // Rotations Per Second Per Second Per Second
	public static AngularVelocity MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(0.1);

	public static final double STATOR_CURRENT_LIMIT = 60; // Amps
	public static final double SUPPLY_CURRENT_LIMIT = 40; // Amps

	public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
		.withMotionMagicCruiseVelocity(MOTION_MAGIC_CRUISE_VELOCITY)
		.withMotionMagicAcceleration(MOTION_MAGIC_ACCELERATION)
		.withMotionMagicJerk(MOTION_MAGIC_JERK);

	public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD).withKS(kS);

	public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
		.withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
		.withStatorCurrentLimitEnable(true)
		.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
		.withSupplyCurrentLimitEnable(true);

	public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withInverted(InvertedValue.CounterClockwise_Positive)
		.withNeutralMode(NeutralModeValue.Brake);

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
		.withFeedbackRemoteSensorID(ENCODER_ID)
		.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
		.withRotorToSensorRatio(ROTOR_TO_SENSOR_RATIO)
		.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO);

	public static final CANcoderConfiguration ENCODER_CONFIGS = new CANcoderConfiguration()
		.withMagnetSensor(
			new MagnetSensorConfigs()
				.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
				.withMagnetOffset(ENCODER_OFFSET)
		);
}
