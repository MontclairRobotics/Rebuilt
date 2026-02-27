package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
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

public class PivotConstants {

	// ports & offsets
	public static final int CAN_ID = 40;
	public static final int ENCODER_ID = 39;

	public static final CANBus CAN_BUS = new CANBus("drivetrain"); // on the canivore

	public static final double ROTOR_TO_SENSOR_RATIO = 64; 
	public static final double SENSOR_TO_MECHANISM_RATIO = 1; // CANcoder is on the mechanism
	public static final double ENCODER_OFFSET = -0.607689453125; 

	// constraints
	public static final Angle MIN_ANGLE = Degrees.of(0);
	public static final Angle MAX_ANGLE = Degrees.of(120);

	// physical properties
	public static final double GEARING = ROTOR_TO_SENSOR_RATIO * SENSOR_TO_MECHANISM_RATIO; // rotations of the motor shaft per rotations of the intake pivot
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

	public static final Angle TOLERANCE = Degrees.of(1);

	public static final double STATOR_CURRENT_LIMIT = 80; // Amps
	public static final double SUPPLY_CURRENT_LIMIT = 40; // Amps

	public static final AngularVelocity MOTION_MAGIC_CRUISE_VELOCITY = RotationsPerSecond.of(4);
	public static final AngularAcceleration MOTION_MAGIC_ACCELERATION = RotationsPerSecondPerSecond.of(40);
	public static final double MOTION_MAGIC_JERK = 100; // Rotations Per Second Per Second Per Second
	public static final AngularVelocity MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(0.05);

	// Configs
	public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD)
		.withKS(kS).withKG(kG)
		.withGravityType(GravityTypeValue.Arm_Cosine);

	public static final Slot0Configs SIM_SLOT0_CONFIGS = new Slot0Configs()
		.withKP(0).withKD(0)
		.withKS(0).withKG(0); //TODO: SET THESE

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
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(ENCODER_OFFSET)
        );
}
