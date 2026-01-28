package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.constants.FlywheelConstants.*;

public class FlywheelIOTalonFX implements FlywheelIO {

	private TalonFX motor;

	public FlywheelIOTalonFX() {
		motor = new TalonFX(CAN_ID);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.appliedVoltage = getMotorVoltage();
		inputs.tempCelcius = getMotorTemp();
		inputs.motorVelocity = getMotorVelocity();
		inputs.flywheelVelocity = getFlywheelVelocity();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public double getMotorVelocity() {
		return motor.getVelocity().getValueAsDouble();
	}

	@Override
	public double getFlywheelVelocity() {
		return getMotorVelocity() / GEARING;
	}

	@Override
	public double getMotorVoltage() {
		return motor.getMotorVoltage().getValueAsDouble();
	}

	@Override
	public double getMotorTemp() {
		return motor.getDeviceTemp().getValueAsDouble();
	}
}
