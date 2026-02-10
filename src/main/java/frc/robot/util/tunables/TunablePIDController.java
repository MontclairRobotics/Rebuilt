package frc.robot.util.tunables;

import edu.wpi.first.math.controller.PIDController;

public class TunablePIDController {
	private final TunableControlConstants params;
	private final PIDController pidController;

	/**
	 * Creates a new TunablePIDController with the given
	 * {@link TunableControlConstants}.
	 *
	 * @param tunableParams The {@link TunableControlConstants} to configure this
	 *                      controller.
	 */
	public TunablePIDController(TunableControlConstants tunableParams) {
		this.params = tunableParams;

		pidController = new PIDController(
				tunableParams.kP.get(), tunableParams.kI.get(), tunableParams.kD.get(), tunableParams.period);

		pidController.setTolerance(tunableParams.tolerance.get(), tunableParams.velTolerance.get());

		if (tunableParams.isContinuous) {
			pidController.enableContinuousInput(tunableParams.minInput, tunableParams.maxInput);
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
	 * Updates the {@link PIDController}'s parameters from the
	 * {@link TunableControlConstants}.
	 */
	public void updateParams() {
		pidController.setP(params.kP.get());
		pidController.setI(params.kI.get());
		pidController.setD(params.kD.get());
		pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
		pidController.setIZone(params.iZone.get());
		pidController.setIntegratorRange(params.iMin.get(), params.iMax.get());
		pidController.setTolerance(params.tolerance.get(), params.velTolerance.get());
	}

	/**
	 * Returns the accumulated error used in the integral calculation of this
	 * controller.
	 *
	 * @return The accumulated error of this controller.
	 * @see PIDController#getAccumulatedError()
	 */
	public double getAccumulatedError() {
		return pidController.getAccumulatedError();
	}

	/**
	 * Sets the goal for the PIDController.
	 *
	 * @param goal The desired goal.
	 */
	public void setSetpoint(double goal) {
		pidController.setSetpoint(goal);
		updateParams();
	}

	/**
	 * Returns the current goal of the PIDController.
	 *
	 * @return The current goal.
	 * @see PIDController#getSetpoint()
	 */
	public double getSetpoint() {
		return pidController.getSetpoint();
	}

	/**
	 * @return Whether the error is within the acceptable bounds.
	 * @see PIDController#atSetpoint()
	 */
	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	/**
	 * Returns the difference between the goal and the measurement.
	 *
	 * @return The error.
	 * @see PIDController#getError()
	 */
	public double getPositionError() {
		return pidController.getError();
	}

	/**
	 * @return The velocity error.
	 * @see PIDController#getErrorDerivative()
	 */
	public double getVelocityError() {
		return pidController.getErrorDerivative();
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @param goal        The new goal of the controller.
	 * @return The next controller output.
	 *
	 * @see PIDController#calculate(double, double)
	 */
	public double calculate(double measurement, double goal) {
		return pidController.calculate(measurement, goal);
	}

	/**
	 * Returns the next output of the PID controller.
	 *
	 * @param measurement The current measurement of the process variable.
	 * @return The next controller output.
	 *
	 * @see PIDController#calculate(double)
	 */
	public double calculate(double measurement) {
		return pidController.calculate(measurement);
	}

	/**
	 * Resets the previous error and the integral term.
	 *
	 * @see PIDController#reset()
	 */
	public void reset() {
		pidController.reset();
	}
}
