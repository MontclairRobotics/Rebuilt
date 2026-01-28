package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.constants.HoodConstants.*;

public class HoodIOTalonFX implements HoodIO {

	public TalonFX motor;

	public HoodIOTalonFX() {
		motor = new TalonFX(CAN_ID);
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
		inputs.angle = getAngle();
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
	public double getAngle() {
		return motor.getPosition().getValueAsDouble() / GEAR_RATIO;
	}
}
