package frc.robot.subsystems.shooter.spindexer;

import static frc.robot.constants.SpindexerConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class SpindexerIOTalonFX implements SpindexerIO {

	private TalonFX spinMotor;
	private TalonFX indexMotor;

	public SpindexerIOTalonFX() {
		spinMotor = new TalonFX(SPIN_ID);
		indexMotor = new TalonFX(INDEX_ID);
	}

	@Override
	public void updateInputs(SpindexerIOInputs inputs) {
		inputs.spinAppliedVoltage = spinMotor.getMotorVoltage().getValueAsDouble();
		inputs.indexAppliedVoltage = indexMotor.getMotorVoltage().getValueAsDouble();
		inputs.spinTempCelcius = spinMotor.getDeviceTemp().getValueAsDouble();
		inputs.indexTempCelcius = indexMotor.getDeviceTemp().getValueAsDouble();
		inputs.indexVelocity = indexMotor.getVelocity().getValue();
		inputs.spinVelocity = spinMotor.getVelocity().getValue();
	}

	@Override
	public void stopSpin() {
		spinMotor.stopMotor();
	}

	@Override
	public void stopIndex() {
		indexMotor.stopMotor();
	}

	@Override
	public void setSpinVoltage(double voltage) {
		spinMotor.setVoltage(voltage);
	}

	@Override
	public void setIndexVoltage(double voltage) {
		indexMotor.setVoltage(voltage);
	}
}