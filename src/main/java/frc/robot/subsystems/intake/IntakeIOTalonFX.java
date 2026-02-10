package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import static frc.robot.constants.IntakeConstants.GEARING;
import static frc.robot.constants.IntakeConstants.MOTOR_ID;

public class IntakeIOTalonFX implements IntakeIO {

	private TalonFX motor;
	TalonFXConfiguration config;

	public IntakeIOTalonFX() {
		motor = new TalonFX(MOTOR_ID);
		config = new TalonFXConfiguration();

		// motor setup and inverted setting
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: needed?
		motor.getConfigurator().apply(config);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		// get double (ints) as outputs
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
		inputs.velocity = getVelocity().in(RotationsPerSecond);
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public AngularVelocity getVelocity() {
		return motor.getVelocity().getValue().div(GEARING);
	}
}
