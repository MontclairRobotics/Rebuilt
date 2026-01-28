package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.FlywheelConstants;

public class FlywheelIOSim implements FlywheelIO {

	private FlywheelSim sim =
		new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60(1),
				FlywheelConstants.MOMENT_OF_INERTIA,
				FlywheelConstants.GEARING),
			DCMotor.getKrakenX60(1),
			0.0
		);

	public void setVoltage(double currentVoltage) {
		sim.setInputVoltage(currentVoltage);
	}

	public void stop() {
		setVoltage(0);
	}

	public double getMotorVelocity() {
		return sim.getAngularVelocity().in(RotationsPerSecond);
	}

	public double getMotorVoltage() {
		return sim.getInputVoltage();
	}

	public double getMotorTemp() {
		return 0; //The simulation does not account for the temperature of the motor and that is fine with me!
	}
}
