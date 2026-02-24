package frc.robot.subsystems.shooter.spindexer.serializer;


import static frc.robot.constants.SerializerConstants.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SerializerIOSim implements SerializerIO {

	private final FlywheelSim sim;
	private double appliedVoltage;

	private PIDController pidController;
	private SimpleMotorFeedforward feedforward;

	public SerializerIOSim() {
		sim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60Foc(1), 
				MOMENT_OF_INERTIA, 
				GEARING), 
				DCMotor.getKrakenX60Foc(CAN_ID), null)
	}

	@Override
	public void updateInputs(SerializerIOInputs inputs) {

		inputs.spinAppliedVoltage = appliedSpinVoltage;
		inputs.indexAppliedVoltage = appliedIndexVoltage;
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
