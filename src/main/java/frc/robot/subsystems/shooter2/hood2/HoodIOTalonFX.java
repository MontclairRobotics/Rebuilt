package frc.robot.subsystems.shooter2.hood2;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

public class HoodIOTalonFX implements HoodIO {

	private TalonFX motor;
	private DutyCycleEncoder absoluteEncoder;

	public HoodIOTalonFX() {
		motor = new TalonFX(CAN_ID);
		absoluteEncoder = new DutyCycleEncoder(ENCODER_PORT, 1, HOOD_ENCODER_OFFSET.in(Rotations));
		motor.setPosition(Rotations.of(absoluteEncoder.get() * GEARING));
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
		inputs.angle = getAngle().in(Rotations);
		inputs.encoderConnected = absoluteEncoder.isConnected();
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
		if(absoluteEncoder.isConnected()) {
			return Rotations.of(absoluteEncoder.get());
		} else {
			return Rotations.of(motor.getPosition().getValueAsDouble() / GEARING); // default back to relative encoder
		}
	}
}
