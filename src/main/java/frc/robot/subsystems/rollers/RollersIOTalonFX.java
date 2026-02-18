package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import static frc.robot.constants.RollersConstants.CAN_ID;

public class RollersIOTalonFX implements RollersIO {

	private TalonFX motor;
	TalonFXConfiguration config;

	public RollersIOTalonFX() {
		motor = new TalonFX(CAN_ID);

		TalonFXConfiguration config = new TalonFXConfiguration();
		config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // default
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motor.getConfigurator().apply(config);
	}

	@Override
	public void updateInputs(RollersIOInputs inputs) {
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
		inputs.motorVelocity = getMotorVelocity().in(RotationsPerSecond);
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
	public AngularVelocity getMotorVelocity() {
		return motor.getVelocity().getValue();
	}
}
