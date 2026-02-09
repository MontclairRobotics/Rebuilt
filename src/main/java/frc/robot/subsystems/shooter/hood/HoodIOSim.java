package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
public class HoodIOSim implements HoodIO {

	public DCMotor motor;
	public SingleJointedArmSim sim;

	private double appliedVoltage;

	public HoodIOSim() {
		motor = DCMotor.getKrakenX44(1);

		sim =
			new SingleJointedArmSim(
				motor,
				GEARING,
				MOMENT_OF_INERTIA,
				HOOD_LENGTH.in(Meters),
				MIN_ANGLE.in(Radians),
				MAX_ANGLE.in(Radians),
				true,
				MIN_ANGLE.in(Radians),
				0.0,
				0.0
			);
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {

		sim.setInputVoltage(appliedVoltage);
		sim.update(0.02);

		inputs.appliedVoltage = appliedVoltage;
		inputs.current = sim.getCurrentDrawAmps();
		inputs.angle = getAngle().in(Rotations);
		inputs.tempCelsius = 0;
		inputs.encoderConnected = false;

	}

	@Override
	public void setVoltage(double voltage) {
		appliedVoltage = voltage;
	}

	@Override
	public void stop() {
		setVoltage(0);
	}

	@Override
	public Angle getAngle() {
		return Radians.of(sim.getAngleRads());
	}
}
