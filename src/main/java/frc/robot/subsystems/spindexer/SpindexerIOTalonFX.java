package frc.robot.subsystems.spindexer;

import static frc.robot.constants.SpindexerConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;

public class SpindexerIOTalonFX implements SpindexerIO {

	private TalonFX spinMotor;
	private TalonFX transMotor;

	public SpindexerIOTalonFX() {
		spinMotor = new TalonFX(SPIN_ID);
		transMotor = new TalonFX(TRANS_ID);
	}

	@Override
	public void updateInputs(SpindexerIOInputs inputs) {
		inputs.spinAppliedVoltage = spinMotor.getMotorVoltage().getValueAsDouble();
		inputs.transAppliedVoltage = transMotor.getMotorVoltage().getValueAsDouble();
		inputs.spinTempCelcius = spinMotor.getDeviceTemp().getValueAsDouble();
		inputs.transTempCelcius = transMotor.getDeviceTemp().getValueAsDouble();
	}

	@Override
	public void stopSpin() {
		spinMotor.stopMotor();
	}

	@Override
	public void stopTrans() {
		transMotor.stopMotor();
	}

	@Override
	public void setSpinVoltage(double voltage) {
		spinMotor.setVoltage(voltage);
	}

	@Override
	public void setTransVoltage(double voltage) {
		transMotor.setVoltage(voltage);
	}
}
