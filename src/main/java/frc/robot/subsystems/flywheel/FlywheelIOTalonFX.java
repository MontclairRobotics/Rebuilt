package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RotationsPerSecond;
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
		inputs.motorVelocity = getMotorVelocity().in(RotationsPerSecond);
		inputs.flywheelVelocity = getFlywheelVelocity().in(RotationsPerSecond);
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
	public AngularVelocity getMotorVelocity() {
		return motor.getVelocity().getValue();
	}

	@Override
	public AngularVelocity getFlywheelVelocity() {
		return RotationsPerSecond.of(getMotorVelocity().in(RotationsPerSecond) / GEARING);
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
