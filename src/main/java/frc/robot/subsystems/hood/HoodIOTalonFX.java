package frc.robot.subsystems.hood;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;
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
		inputs.angle = getAngle().in(Rotations);
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
	public Angle getAngle() {
		return Angle.ofBaseUnits(motor.getPosition().getValueAsDouble() / GEAR_RATIO, Rotations);
	}
}
