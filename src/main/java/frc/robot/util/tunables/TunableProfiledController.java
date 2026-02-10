package frc.robot.util.tunables;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class TunableProfiledController {
	private final ProfiledPIDController profiledPIDController;
	private final TunableControlConstants params;

	private double previousVelocity = 0;

	/**
	 * Creates a new TunableProfiledController with the given
	 * {@link TunableControlConstants}.
	 *
	 * @param tunableParams The {@link TunableControlConstants} to configure this
	 *                      controller.
	 */
	public TunableProfiledController(TunableControlConstants tunableParams) {
		this.params = tunableParams;

		if (!tunableParams.profiled) {
			throw new IllegalArgumentException(
					"TunableControlConstants must be profiled to use TunableProfiledController");
		}

		profiledPIDController = new ProfiledPIDController(
				tunableParams.kP.get(),
				tunableParams.kI.get(),
				tunableParams.kD.get(),
				new TrapezoidProfile.Constraints(tunableParams.maxVel.get(), tunableParams.maxAcc.get()),
				tunableParams.period);

		profiledPIDController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

		if (tunableParams.isContinuous) {
			profiledPIDController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
		}
	}

	/**
	 * Returns the {@link TunableControlConstants} used to configure this
	 * controller.
	 *
	 * @return The {@link TunableControlConstants} of this controller.
	 */
	public TunableControlConstants getParams() {
		return params;
	}

	/**
	 * Updates the controller's parameters based on the current values in the
	 * {@link TunableControlConstants}.
	 */
	public void updateParams() {
		profiledPIDController.setP(params.kP.get());
		profiledPIDController.setI(params.kI.get());
		profiledPIDController.setD(params.kD.get());

		profiledPIDController.setConstraints(
				new TrapezoidProfile.Constraints(params.maxVel.get(), params.maxAcc.get()));
		profiledPIDController.setIZone(params.iZone.get());
		profiledPIDController.setIntegratorRange(params.iMin.get(), params.iMax.get());
		profiledPIDController.setTolerance(params.tolerance.get(), params.velTolerance.get());
	}

	/**
	 * Returns the underlying {@link ProfiledPIDController}. Only use if you know what you are doing!
	 *
	 * @return The ProfiledPIDController of this controller.
	 */
	public ProfiledPIDController getProfiledPIDController() {
		return profiledPIDController;
	}

	/**
	 * Returns the accumulated error used in the integral calculation of this
	 * controller.
	 *
	 * @return The accumulated error of this controller.
	 * @see ProfiledPIDController#getAccumulatedError()
	 */
	public double getAccumulatedError() {
		return profiledPIDController.getAccumulatedError();
	}

	/**
	 * Sets the goal for the ProfiledPIDController.
	 *
	 * @param goal The desired goal.
	 * @param goalVel The desired goal velocity.
	 */
	public void setGoal(double goal, double goalVel) {
		profiledPIDController.setGoal(new TrapezoidProfile.State(goal, goalVel));
		previousVelocity = profiledPIDController.getSetpoint().velocity;
		updateParams();
	}

	/**
	 * Sets the goal for the ProfiledPIDController.
	 *
	 * @param goal The desired goal.
	 */
	public void setGoal(double goal) {
		setGoal(goal, 0);
	}

	/**
	 * Returns the current goal of the ProfiledPIDController.
	 *
	 * @return The current goal.
	 * @see ProfiledPIDController#getGoal()
	 */
	public double getGoal() {
		return profiledPIDController.getGoal().position;
	}

	/**
	 * Returns the current setpoint of the ProfiledPIDController.
	 *
	 * @return The current setpoint.
	 * @see ProfiledPIDController#getSetpoint()
	 */
	public State getSetpoint() {
		return profiledPIDController.getSetpoint();
	}

	/**
	 * @return Whether the error is within the acceptable bounds.
	 * @see ProfiledPIDController#atGoal()
	 */
	public boolean atGoal() {
		return profiledPIDController.atGoal();
	}

	/**
	 * Returns the difference between the goal and the measurement.
	 *
	 * @return The error.
	 * @see ProfiledPIDController#getPositionError()
	 */
	public double getPositionError() {
		return profiledPIDController.getPositionError();
	}

	/**
	 * Returns the error derivative.
	 *
	 * @return The error derivative.
	 * @see ProfiledPIDController#getVelocityError()
	 */
	public double getVelocityError() {
		return profiledPIDController.getVelocityError();
	}

	/**
	 * Calculates the feedforward output based on the current setpoint and kS, kG,
	 * kV, and kA.
	 *
	 * @return The feedforward output.
	 */
	public double calculateFeedforward() {
		State setpoint = profiledPIDController.getSetpoint();
		double velocity = params.isControllingVelocity ? setpoint.position : setpoint.velocity;
		double accel = (velocity - previousVelocity) / profiledPIDController.getPeriod();
		previousVelocity = velocity;

		return params.kS.get() * Math.signum(velocity)
				+ params.kG.get()
				+ params.kV.get() * velocity
				+ params.kA.get() * accel;
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @param goal        The new goal of the controller.
	 * @return The next controller output.
	 *
	 * @see ProfiledPIDController#calculate(double, double)
	 * @see #calculateFeedforward()
	 */
	public double calculate(double measurement, double goal) {
		return profiledPIDController.calculate(measurement, goal) + calculateFeedforward();
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @return The next controller output.
	 *
	 * @see ProfiledPIDController#calculate(double)
	 * @see #calculateFeedforward()
	 */
	public double calculate(double measurement) {
		return profiledPIDController.calculate(measurement) + calculateFeedforward();
	}

	/**
	 * Resets the previous error and the integral term.
	 *
	 * @see ProfiledPIDController#reset(double, double)
	 */
	public void reset(double measuredPos, double measuredVel) {
		profiledPIDController.reset(measuredPos, measuredVel);
		previousVelocity = measuredVel;
	}

	/**
	 * Resets the previous error and the integral term.
	 *
	 * @see ProfiledPIDController#reset(double)
	 */
	public void reset(double measuredPos) {
		reset(measuredPos, 0);
	}

	/**
	 * Logs goal, setpoint (pos+vel), position error, accumulated error, and velocity error
	 * @param tableKey NetworkTable key (no trailing "/")
	 */
	public void logData(String tableKey) {
		Logger.recordOutput(tableKey + "/Goal", getGoal());
		Logger.recordOutput(tableKey + "/Setpoint", getSetpoint());
		Logger.recordOutput(tableKey + "/Position error", getPositionError());
		Logger.recordOutput(tableKey + "/Accumulated error", getAccumulatedError());
		Logger.recordOutput(tableKey + "/Velocity error", getVelocityError());
	}
}
