package frc.robot.subsystems.turret;

import static frc.robot.constants.TurretConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotContainer;

public class TurretIOTalonFX implements TurretIO {

	private TalonFX motor;

	public TurretIOTalonFX() {
		motor = new TalonFX(CAN_ID);
	}

	@Override
	public void updateInputs(TurretIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.motorVelocity = getMotorVelocity();
		inputs.turretVelocity = getTurretVelocity();
		inputs.robotRelativeAngle = getRobotRelativeAngle();
		inputs.fieldRelativeAngle = getFieldRelativeAngle();
	}

	@Override
	public void setVoltage(double volts) {
		motor.setVoltage(volts);
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
	public double getTurretVelocity() {
		return getMotorVelocity() / GEARING;
	}

	@Override
	public double getRobotRelativeAngle() {
		return motor.getPosition().getValueAsDouble() / GEARING;
	}

	@Override
	public double getFieldRelativeAngle() {
		return RobotContainer.turret.toFieldRelativeAngle(getRobotRelativeAngle());
	}

	@Override
	public void zero() {
		motor.setPosition(0);
	}
}
