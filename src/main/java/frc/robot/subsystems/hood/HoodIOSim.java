package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
public class HoodIOSim implements HoodIO {

	public DCMotor motor;
	public SingleJointedArmSim sim;

	public DutyCycleEncoder realEncoder;
	public DutyCycleEncoderSim encoder;

	private double appliedVoltage;

	public HoodIOSim() {
		motor = DCMotor.getKrakenX44(1);

		realEncoder = new DutyCycleEncoder(
			ENCODER_PORT, 1, HOOD_ENCODER_OFFSET.in(Rotations)
		);
		encoder = new DutyCycleEncoderSim(realEncoder);
		encoder.setConnected(true);

		sim =
			new SingleJointedArmSim(
				motor,
				GEAR_RATIO,
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

		inputs.appliedVoltage = appliedVoltage;
		inputs.current = sim.getCurrentDrawAmps();
		inputs.angle = getAngle().in(Rotations);
		// encoder.set(Radians.of(sim.getAngleRads()).in(Rotations));

		sim.update(0.02);
	}

	@Override
	public void setVoltage(double voltage) {
		appliedVoltage = voltage;
		// encoder.set(Radians.of(sim.getAngleRads()).in(Rotations));
	}

	@Override
	public void stop() {
		setVoltage(0);
	}

	@Override
	public Angle getAngle() {
		return Angle.ofBaseUnits(sim.getAngleRads(), Radians);
	}
}
