package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class TurretConstants {

	// CAD transformations
	public static final Translation3d ORIGIN_TO_TURRET = new Translation3d(-0.1651, 0.1651, 0.3582);
	public static final Translation2d TURRET_OFFSET = ORIGIN_TO_TURRET.toTranslation2d();

	// ports
	public static final int CAN_ID = -1;
	public static final int ENCODER_PORT = -1;
	public static final CANBus CAN_BUS = new CANBus("");

	// constraints
	public static final Angle MIN_ANGLE = Rotations.of(-0.5);
	public static final Angle MAX_ANGLE = Rotations.of(0.5);
	public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(4);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(16);
	public static final Angle ANGLE_TOLERANCE = Degrees.of(3);

	// physical properties
	public static final double MOMENT_OF_INERTIA = 0.154244993; //From CAD on 2026.02.12
	public static final Distance LENGTH = Meters.of(0.3);
	public static final double GEARING = 70.0/9.0; // how many rotations of the output shaft per rotation of the turret

	// the angle between the zero of the gyro and the robot relative zero of the turret
	public static final Angle ANGLE_OFFSET = Rotations.of(0.5); // turret zero is perpendicular to gyro zero, pointed to the left

	// pid + ff gains
	public static final double kP = 0; // Tuned PID in Simulation
	public static final double kD = 0;
	public static final double kS = 0;

	public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
		.withKP(kP).withKD(kD).withKS(kS);

	public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs();

	public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
		.withInverted(InvertedValue.Clockwise_Positive)
		.withNeutralMode(NeutralModeValue.Brake);

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();

	public static final MagnetSensorConfigs ENCODER_CONFIGS = new MagnetSensorConfigs();

}
