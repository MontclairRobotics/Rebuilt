package frc.robot.subsystems.shooter.spindexer;

import static frc.robot.constants.SpindexerConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class SpindexerIOTalonFX implements SpindexerIO {

	private TalonFX motor;

	public SpindexerIOTalonFX() {
		motor = new TalonFX(CAN_ID);
	}

	public void updateInputs(SpindexerIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble(); // celsius
		inputs.velocity = motor.getVelocity().getValueAsDouble(); // RPS
	}

	public void stop() {
		motor.stopMotor();
	}

	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

}
