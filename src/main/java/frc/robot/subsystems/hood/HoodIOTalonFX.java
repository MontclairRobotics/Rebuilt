package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.HoodConstants;
import java.util.function.DoubleSupplier;

public class HoodIOTalonFX implements HoodIO {

	public TalonFX motor;
	public PIDController pidController;
	public ArmFeedforward feedforward;

	public HoodIOTalonFX() {
		motor = new TalonFX(HoodConstants.CAN_ID);
		pidController = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
		feedforward = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV);
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
		inputs.angle = getAngle();
		inputs.angleSetpoint = pidController.getSetpoint();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public double getAngle() {
		return motor.getPosition().getValueAsDouble() / HoodConstants.GEAR_RATIO;
	}

	@Override
	public void setAngle(double goal) {
		double pidVoltage = pidController.calculate(getAngle(), goal);
		double feedforwardVoltage = feedforward.calculate(Rotations.of(getAngle()).in(Radians), 0);
		setVoltage(pidVoltage + feedforwardVoltage);
	}

	@Override
	public void setAngle(DoubleSupplier goalSupplier) {
		double currentGoal = goalSupplier.getAsDouble();
		setAngle(currentGoal);
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}
}
