package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class SerializerConstants {

    public static final int CAN_ID = 20;
    public static final CANBus CAN_BUS = new CANBus("drivetrain"); // on the canivore

    public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(0.1);
	public static final AngularVelocity SPIN_VELOCITY = RotationsPerSecond.of(2.5);

    public static final double kP = 4;
	public static final double kD = 0.2;
	public static final double kS = 3;
	public static final double kV = 1.6;

    public static final double STATOR_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LIMIT = 40;

    public static final double GEARING = 20; // TODO: get
    public static final double MOMENT_OF_INERTIA = 0.02;

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD)
		.withKS(kS).withKV(kV);

	public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
		.withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
		.withStatorCurrentLimitEnable(true)
		.withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
		.withSupplyCurrentLimitEnable(true);

	public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withInverted(InvertedValue.CounterClockwise_Positive)
		.withNeutralMode(NeutralModeValue.Coast);

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
		.withVelocityFilterTimeConstant(Seconds.of(0.01))
		.withSensorToMechanismRatio(GEARING);

}
