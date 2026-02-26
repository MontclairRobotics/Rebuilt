package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.PivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {

	private SingleJointedArmSim sim;
	private PIDController pidController = new  PIDController(0, 0, 0);
	private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

	private double appliedVoltage;

	public PivotIOSim() {

		sim = new SingleJointedArmSim(
			DCMotor.getKrakenX44Foc(1),
			GEARING,
			MOMENT_OF_INERTIA,
			ARM_LENGTH.in(Meters),
			MIN_ANGLE.in(Radians),
			MAX_ANGLE.in(Radians),
			true,
			MIN_ANGLE.in(Radians),
			0.0,
			0.0
		);
	}

	@Override
	public void updateInputs(PivotIOInputs inputs) {

		sim.setInputVoltage(appliedVoltage);
		sim.update(0.02);

		inputs.appliedVoltage = appliedVoltage;
		inputs.current = sim.getCurrentDrawAmps();
		inputs.angle = getAngle().in(Rotations);
		inputs.tempCelsius = 0;
		inputs.encoderConnected = false;
		inputs.isAtSetpoint = isAtSetpoint();
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

	@Override
	public void setAngle(Angle angle) {

	}

	@Override
	public boolean isAtSetpoint() {
		return pidController.atSetpoint();
	}

    @Override
    public void setGains(double kP, double kD, double kS, double kG) {
        pidController.setP(kP);
        pidController.setD(kD);
        feedforward.setKs(kS);
        feedforward.setKg(kG);
    }

    @Override
    public void setMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration, double jerk) {
        // does nothing, not necessary
    }

    @Override
    public void resetEncoderPosition() {

    }
}
