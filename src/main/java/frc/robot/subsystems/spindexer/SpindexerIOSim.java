package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.SpindexerConstants;

public class SpindexerIOSim implements SpindexerIO {

	private FlywheelSim sim = new FlywheelSim(
		LinearSystemId.createFlywheelSystem(
			DCMotor.getKrakenX60(1),
			SpindexerConstants.MOMENT_OF_INERTIA,
			SpindexerConstants.GEARING
		),
		DCMotor.getKrakenX60(1),
		0.0
	);

	public void updateInputs(SpindexerIOInputs inputs) {
		inputs.appliedVoltage = sim.getInputVoltage();
		inputs.velocity = (sim.getAngularVelocityRadPerSec() / 2 * Math.PI);
		inputs.tempCelcius = 0;
	}

	@Override
	public void setVoltage(double currentVoltage) {
		sim.setInputVoltage(currentVoltage);
	}

	@Override
	public void stop() {
		setVoltage(0.0);
	}
}
