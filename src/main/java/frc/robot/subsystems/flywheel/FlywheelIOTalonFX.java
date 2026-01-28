package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {

	TalonFX motor;

	public FlywheelIOTalonFX() {
		motor = new TalonFX(FlywheelConstants.CAN_ID);
	}

	public void stop() {
		motor.stopMotor();
	}

	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	public double getMotorVelocity() {
		return motor.getVelocity().getValueAsDouble() / FlywheelConstants.GEARING; //This provides rotations per second
	}

	public double getMotorVoltage() {
		return motor.getMotorVoltage().getValueAsDouble();
	}

	public double getMotorTemp() {
		return motor.getDeviceTemp().getValueAsDouble();
	}

}
