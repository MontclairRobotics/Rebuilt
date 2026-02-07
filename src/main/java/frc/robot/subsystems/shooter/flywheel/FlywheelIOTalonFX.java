package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.FlywheelConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.FlywheelConstants.*;

public class FlywheelIOTalonFX implements FlywheelIO {

	private TalonFX leftMotor;
	private TalonFX rightMotor;

	public FlywheelIOTalonFX() {
		leftMotor = new TalonFX(LEFT_CAN_ID);
		rightMotor = new TalonFX(RIGHT_CAN_ID);

		rightMotor.setControl(new Follower(FlywheelConstants.LEFT_CAN_ID, MotorAlignmentValue.Aligned));
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
		leftMotor.setVoltage(voltage);
	}

	@Override
	public void stop() {
		leftMotor.stopMotor();
	}

	@Override
	public AngularVelocity getMotorVelocity() {
		return leftMotor.getVelocity().getValue();
	}

	@Override
	public AngularVelocity getFlywheelVelocity() {
		return RotationsPerSecond.of(getMotorVelocity().in(RotationsPerSecond) / GEARING);
	}

	@Override
	public double getMotorVoltage() {
		return leftMotor.getMotorVoltage().getValueAsDouble();
	}

	@Override
	public double getMotorTemp() {
		return leftMotor.getDeviceTemp().getValueAsDouble();
	}
}