package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

public class FlywheelConstants {

	public static final int LEFT_CAN_ID = 32;
	public static final int RIGHT_CAN_ID = 31;

	public static final CANBus CAN_BUS = new CANBus("turret");

	public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(1);

	public static final double kP = 0;
	public static final double kD = 0;
	public static final double kS = 0;
	public static final double kV = 0;

	public static final double STATOR_CURRENT_LIMIT = 100; // Amps
	public static final double SUPPLY_CURRENT_LIMIT = 60; // Amps

	public static final double MOMENT_OF_INERTIA = 0.00233846427; // From CAD on 2026.02.11
	public static final double GEARING = 1.10526315785; // to the big flywheel, from Max Pearson on 2026.02.20

	public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD)
		.withKS(kS).withKV(kV);

	public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
		.withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
		.withStatorCurrentLimitEnable(true)
		.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
		.withSupplyCurrentLimitEnable(true);

	public static final MotorOutputConfigs LEFT_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withInverted(InvertedValue.Clockwise_Positive)
		.withNeutralMode(NeutralModeValue.Coast);

	public static final MotorOutputConfigs RIGHT_MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withNeutralMode(NeutralModeValue.Coast);

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
		.withVelocityFilterTimeConstant(Seconds.of(0.01))
		.withSensorToMechanismRatio(GEARING);

}
