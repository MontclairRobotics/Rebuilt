package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;
import frc.robot.util.Tunable;
import java.util.function.DoubleSupplier;

public class TurretIOSim implements TurretIO {
	private SingleJointedArmSim sim;
	ProfiledPIDController pidController;
	double appliedVoltage = 0;
	double fieldRelativeSetpoint = 0;

	private Tunable kp, ki, kd;

	public TurretIOSim() {

		kp = new Tunable("turret kP", SIM_KP, (value) -> pidController.setP(value));
		ki = new Tunable("turret kI", SIM_KI, (value) -> pidController.setI(value));
		kd = new Tunable("turret kD", SIM_KD, (value) -> pidController.setD(value));

		sim = new SingleJointedArmSim(
			DCMotor.getKrakenX60(1),
			GEAR_RATIO,
			MOMENT_OF_INERTIA,
			LENGTH,
			0,
			MAX_ANGLE.in(Radians),
			false,
			0,
			0,
			0
		);

		pidController = new ProfiledPIDController(
			SIM_KP,
			SIM_KI,
			SIM_KD,
			new TrapezoidProfile.Constraints(
				TURRET_CRUISE_VELOCITY.in(RotationsPerSecond),
				TURRET_ACCELERATION.in(RotationsPerSecondPerSecond)
			)
		);

		pidController.disableContinuousInput();
	}

	@Override
	public void updateInputs(TurretIOInputs input) {

		sim.setInputVoltage(appliedVoltage);
		sim.update(0.02);

		input.velocity = sim.getVelocityRadPerSec() / (2 * Math.PI);
		input.appliedVoltage = appliedVoltage;
		input.robotRelativeAngle = getRobotRelativeAngle();
		input.robotRelativeAngleSetpoint = pidController.getSetpoint().position;
		input.fieldRelativeAngle = getFieldRelativeAngle();
		input.fieldRelativeAngleSetpoint = fieldRelativeSetpoint;
	}

	@Override
	public void setRobotRelativeAngle(double angle) {
		pidController.setGoal(
			new TrapezoidProfile.State(
				wrapAngleSetpoint(angle),
				RadiansPerSecond.of(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond).in(RotationsPerSecond)
			)
		);

		setVoltage(pidController.calculate(getRobotRelativeAngle()));
	}

	@Override
	public double getRobotRelativeAngle() {
		return Radians.of(sim.getAngleRads()).in(Rotations);
	}

	@Override
	public double getFieldRelativeAngle() {
		return getRobotRelativeAngle() + RobotContainer.drivetrain.getWrappedHeading().getRotations();
	}

	@Override
	public void zeroRelativeEncoder() {
		sim.setState(0, 0);
	}

	@Override
	public void setRobotRelativeAngle(DoubleSupplier supplier) {
		setRobotRelativeAngle(supplier.getAsDouble());
	}

	@Override
	public void setVoltage(double voltage) {
		appliedVoltage = MathUtil.clamp(voltage, -12, 12);
	}

	@Override
	public void stop() {
		setVoltage(0);
	}

	@Override
	public void setFieldRelativeAngle(double angle) {
		double robotRelativeAngleSetpoint = angle - RobotContainer.drivetrain.getWrappedHeading().getRotations();
		fieldRelativeSetpoint = angle;
		setRobotRelativeAngle(robotRelativeAngleSetpoint);
	}

	@Override
	public void setFieldRelativeAngle(DoubleSupplier supplier) {
		setFieldRelativeAngle(supplier.getAsDouble());
	}

	@Override
	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	@Override
	public double wrapAngleSetpoint(double angle) {
		if (angle > MAX_ANGLE.in(Rotations)) {
			return angle - 1;
		} else if (angle < 0) {
			return angle + 1;
		} else {
			return angle;
		}
	}
}
