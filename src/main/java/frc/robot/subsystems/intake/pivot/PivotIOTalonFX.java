package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.PivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class PivotIOTalonFX implements PivotIO {

	private TalonFX motor;

	private CANcoder encoder;
	private TalonFXConfiguration configs = new TalonFXConfiguration();

    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);


	public PivotIOTalonFX() {
        encoder = new CANcoder(ENCODER_PORT);

        configs = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);

        encoder.getConfigurator().apply(ENCODER_CONFIGS);
        motor.getConfigurator().apply(configs);
        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

		motor = new TalonFX(MOTOR_PORT);
		motor.setNeutralMode(NeutralModeValue.Brake);
	}

	public void updateInputs(PivotIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
		inputs.angle = getAngle().in(Rotations);
		inputs.encoderConnected = encoder.isConnected();
		inputs.isAtSetpoint = isAtSetpoint();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public void setAngle(Angle angle) {
		motor.setControl(request.withPosition(angle));
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

    @Override
    public void resetEncoderPosition() {
        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void setGains(double kP, double kD, double kS, double kG) {
        configs.Slot0.kP = kP;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        configs.Slot0.kG = kG;

        motor.getConfigurator().apply(configs.Slot0);
    }

	public Angle getAngle() {
		if (encoder.isConnected()) {
			return encoder.getPosition().getValue();
		} else {
			return motor.getPosition().getValue().div(GEARING);
		}
	}

    @Override
    public void setMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration, double jerk) {
        configs.MotionMagic.MotionMagicCruiseVelocity = velocity.in(RotationsPerSecond);
        configs.MotionMagic.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        configs.MotionMagic.MotionMagicJerk = jerk;

        motor.getConfigurator().apply(configs.MotionMagic);
    }

	@Override
	public boolean isAtSetpoint() {
        double error = motor.getClosedLoopError().getValueAsDouble();
        return Math.abs(error) < TOLERANCE.in(Rotations)
            && Math.abs(motor.getVelocity().getValueAsDouble()) < MAX_VELOCITY_AT_SETPOINT.in(RotationsPerSecond);
	}
}
