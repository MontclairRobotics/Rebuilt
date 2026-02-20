package frc.robot.subsystems.shooter.spindexer;


import static frc.robot.constants.SpindexerConstants.*;

public class SpindexerIOSim implements SpindexerIO {

	private double appliedSpinVoltage;
	private double appliedIndexVoltage;

	@Override
	public void updateInputs(SpindexerIOInputs inputs) {

		inputs.appliedSpinVoltage = appliedSpinVoltage;
		inputs.appliedIndexVoltage = appliedIndexVoltage;
		inputs.spinTempCelcius = 0;
		inputs.indexTempCelcius = 0;
		inputs.spinVelocity = MAX_SPIN_VELOCITY.times(appliedSpinVoltage / 12.0);
		inputs.indexVelocity = MAX_INDEX_VELOCITY.times(appliedIndexVoltage / 12.0);
	}

	@Override
	public void setSpinVoltage(double voltage) {
		appliedSpinVoltage = voltage;
	}

	@Override
	public void setIndexVoltage(double voltage) {
		appliedIndexVoltage = voltage;
	}

	@Override
	public void stopSpin() {
		setSpinVoltage(0.0);
	}

	@Override
	public void stopIndex() {
		setIndexVoltage(0.0);
	}
}
