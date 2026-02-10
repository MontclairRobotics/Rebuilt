package frc.robot.util.tunables;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ControlConstants {
	// PID gains
	double kP = 0;
	double kI = 0;
	double kD = 0;
	double tolerance = 0;
	double velTolerance = Double.POSITIVE_INFINITY;
	double iZone = Double.POSITIVE_INFINITY;
	double iMin = Double.NEGATIVE_INFINITY;
	double iMax = Double.POSITIVE_INFINITY;
	double period = 0.02;

	// feedforward gains
	double kV, kA = 0;
	boolean isControllingVelocity = false;

	// physical gains
	double kS, kG = 0;

	// trapezoid profile
	boolean profiled = false;
	double maxVel = 0;
	double maxAcc = 0;

	// continuous control
	boolean isContinuous = false;
	double maxInput;
	double minInput;

	public ControlConstants() {}

	public ControlConstants(ControlConstants constants) {
		this.kP = constants.kP;
		this.kI = constants.kI;
		this.kD = constants.kD;
		this.tolerance = constants.tolerance;
		this.velTolerance = constants.velTolerance;
		this.iZone = constants.iZone;
		this.iMax = constants.iMax;
		this.iMin = constants.iMin;
		this.period = constants.period;
		this.kV = constants.kV;
		this.kA = constants.kA;
		this.isControllingVelocity = constants.isControllingVelocity;
		this.kS = constants.kS;
		this.kG = constants.kG;
		this.maxVel = constants.maxVel;
		this.maxAcc = constants.maxAcc;
		this.isContinuous = constants.isContinuous;
		this.maxInput = constants.maxInput;
		this.minInput = constants.minInput;
	}

	/**
	 * Sets the PID constants.
	 *
	 * @param kP proportional gain (units of output per unit error)
	 * @param kI integral gain (units of output per unit accumulated error)
	 * @param kD derivative gain (units of output per unit error per second)
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withPID(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		return this;
	}

	/**
	 * Sets the feedforward constants.
	 *
	 * @param kV output to give for a velocity of 1 unit per second
	 * @param kA output to give for an acceleration of 1 unit per second squared
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withFeedforward(double kV, double kA) {
		this.kV = kV;
		this.kA = kA;
		return this;
	}

	/**
	 * Sets a controller that measurements are of velocity, and to apply feedforwards control accordingly
	 */
	public ControlConstants withVelocityControl() {
		this.isControllingVelocity = true;
		return this;
	}

	/**
	 * Sets the "physical" feedforward constants.
	 *
	 * @param kS output to overcome static friction
	 * @param kG output to counteract gravity
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withPhysical(double kS, double kG) {
		this.kS = kS;
		this.kG = kG;
		return this;
	}

	/**
	 * Sets the trapezoidal motion profile parameters. Will also set the controller
	 * to be profiled.
	 *
	 * @param maxVel maximum velocity (units per second)
	 * @param maxAcc maximum acceleration (units per second squared)
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withProfile(double maxVel, double maxAcc) {
		this.profiled = true;
		this.maxVel = maxVel;
		this.maxAcc = maxAcc;
		return this;
	}

	/**
	 * Sets whether the controller is set up to be profiled.
	 *
	 * @param profiled whether the controller is profiled. Automatically set to true
	 *                 when {@link #withProfile(double, double)} is called.
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withProfiled(boolean profiled) {
		this.profiled = profiled;
		return this;
	}

	/**
	 * Sets the position tolerance for the controller.
	 *
	 * @param tolerance position tolerance
	 * @return this ControlConstants object for chaining
	 *
	 * @see #withTolerance(double, double)
	 */
	public ControlConstants withTolerance(double tolerance) {
		this.tolerance = tolerance;
		return this;
	}

	/**
	 * Sets the tolerances for the controller.
	 *
	 * @param tolerance    position tolerance
	 * @param velTolerance velocity tolerance
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withTolerance(double tolerance, double velTolerance) {
		this.tolerance = tolerance;
		this.velTolerance = velTolerance;

		return this;
	}

	/**
	 * Sets the I-Zone (where to use integral control) for the controller.
	 *
	 * @param iZone maximum error for which integral control is applied
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withIZone(double iZone) {
		this.iZone = iZone;
		return this;
	}

	/**
	 * Sets the integral range (limiting integral output) for the controller.
	 *
	 * @param iMin minimum accumulated error
	 * @param iMax maximum accumulated error
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withIRange(double iMin, double iMax) {
		this.iMin = iMin;
		this.iMax = iMax;
		return this;
	}

	/**
	 * Sets the period for the controller.
	 *
	 * @param period period in seconds (default 0.02s)
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withPeriod(double period) {
		this.period = period;
		return this;
	}

	/**
	 * Sets the controller to be continuous over the given input range. Primarily
	 * used for angular control. (e.g. 0 and 359 degrees should be considered
	 * adjacent)
	 *
	 * @param minInput minimum input value
	 * @param maxInput maximum input value
	 * @return this ControlConstants object for chaining
	 */
	public ControlConstants withContinuous(double minInput, double maxInput) {
		this.isContinuous = true;
		this.minInput = minInput;
		this.maxInput = maxInput;
		return this;
	}

	/**
	 * Creates a new {@link PIDController} with the configured constants.
	 */
	public PIDController getPIDController() {
		PIDController controller = new PIDController(kP, kI, kD);
		controller.setTolerance(tolerance);
		controller.setIntegratorRange(iMin, iMax);
		controller.setIZone(iZone);

		return controller;
	}

	/**
	 * Creates a new {@link ProfiledPIDController} with the configured constants.
	 */
	public ProfiledPIDController getProfiledPIDController() {
		ProfiledPIDController controller =
				new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAcc));
		controller.setTolerance(tolerance);
		controller.setIntegratorRange(iMin, iMax);
		controller.setIZone(iZone);

		return controller;
	}

	/**
	 * Creates a new {@link ElevatorFeedforward} with the configured constants.
	 */
	public ElevatorFeedforward getElevatorFeedforward() {
		return new ElevatorFeedforward(kS, kG, kV, kA);
	}

	/**
	 * Creates a new {@link SimpleMotorFeedforward} with the configured constants.
	 */
	public SimpleMotorFeedforward getSimpleFeedforward() {
		return new SimpleMotorFeedforward(kS, kV, kA);
	}
}
