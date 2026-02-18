package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import static frc.robot.constants.RollersConstants.GEARING;
import static frc.robot.constants.RollersConstants.MOTOR_ID;

public class RollersIOTalonFX implements RollersIO {

	private TalonFX motor;
	TalonFXConfiguration config;

	public RollersIOTalonFX() {
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
