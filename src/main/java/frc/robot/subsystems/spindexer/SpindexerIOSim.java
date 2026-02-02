package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static frc.robot.constants.SpindexerConstants.*;

public class SpindexerIOSim implements SpindexerIO {

	private FlywheelSim transSim = new FlywheelSim(
		LinearSystemId.createFlywheelSystem(
			DCMotor.getKrakenX60(1),
			TRANS_MOMENT_OF_INERTIA,
			TRANS_GEARING
		),
		DCMotor.getKrakenX60(1),
		0.0
	);

	private FlywheelSim spinSim = new FlywheelSim(
		LinearSystemId.createFlywheelSystem(
			DCMotor.getKrakenX60(1),
			SPIN_MOMENT_OF_INERTIA,
			SPIN_GEARING
		),
		DCMotor.getKrakenX60(1),
		0.0
	);

	@Override
	public void updateInputs(SpindexerIOInputs inputs) {
		inputs.spinAppliedVoltage = spinSim.getInputVoltage();
		inputs.transAppliedVoltage = transSim.getInputVoltage();
		inputs.spinTempCelcius = 0;
		inputs.transTempCelcius = 0;
	}

	@Override
	public void setSpinVoltage(double currentVoltage) {
		spinSim.setInputVoltage(currentVoltage);
	}

	@Override
	public void setTransVoltage(double currentVoltage) {
		transSim.setInputVoltage(currentVoltage);
	}

	@Override
	public void stopSpin() {
		setSpinVoltage(0.0);
	}

	@Override
	public void stopTrans() {
		setTransVoltage(0.0);
	}
}
