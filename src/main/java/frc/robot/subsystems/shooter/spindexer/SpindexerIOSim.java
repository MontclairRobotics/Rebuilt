package frc.robot.subsystems.shooter.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static frc.robot.constants.SpindexerConstants.*;

public class SpindexerIOSim implements SpindexerIO {

	private FlywheelSim indexSim = new FlywheelSim(
		LinearSystemId.createFlywheelSystem(
			DCMotor.getKrakenX60(1),
			INDEX_MOMENT_OF_INERTIA,
			INDEX_GEARING
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
		inputs.indexAppliedVoltage = indexSim.getInputVoltage();
		inputs.spinTempCelcius = 0;
		inputs.indexTempCelcius = 0;
		inputs.spinVelocity = spinSim.getAngularVelocity();
		inputs.indexVelocity = indexSim.getAngularVelocity();
	}

	@Override
	public void setSpinVoltage(double currentVoltage) {
		spinSim.setInputVoltage(currentVoltage);
	}

	@Override
	public void setIndexVoltage(double currentVoltage) {
		indexSim.setInputVoltage(currentVoltage);
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
