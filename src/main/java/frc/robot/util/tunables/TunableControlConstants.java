package frc.robot.util.tunables;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TunableControlConstants {
	// PID gains
	LoggedTunableNumber kP;
	LoggedTunableNumber kI;
	LoggedTunableNumber kD;
	LoggedTunableNumber tolerance;
	LoggedTunableNumber velTolerance;
	LoggedTunableNumber iZone;
	LoggedTunableNumber iMin;
	LoggedTunableNumber iMax;
	double period;

	// feedforward gains
	LoggedTunableNumber kV;
	LoggedTunableNumber kA;
	boolean isControllingVelocity;

	// physical gains
	LoggedTunableNumber kS;
	LoggedTunableNumber kG;

	// trapezoid profile
	boolean profiled;
	LoggedTunableNumber maxVel;
	LoggedTunableNumber maxAcc;

	// continuous control
	boolean isContinuous;
	double maxInput;
	double minInput;

	/**
	 * Creates a new TunableControlConstants object, initializing the tunable
	 * numbers with the values from the given {@link ControlConstants}.
	 *
	 * @param key       base NetworkTables key for the {@link LoggedTunableNumber}s.
	 *                  Should <i>not</i> end with a slash.
	 * @param constants initial ControlConstants to copy values from
	 */
	public TunableControlConstants(String key, ControlConstants constants) {
		this.kP = new LoggedTunableNumber(key + "/kP", constants.kP);
		this.kI = new LoggedTunableNumber(key + "/kI", constants.kI);
		this.kD = new LoggedTunableNumber(key + "/kD", constants.kD);
		this.tolerance = new LoggedTunableNumber(key + "/tolerance", constants.tolerance);
		this.velTolerance = new LoggedTunableNumber(key + "/velTolerance", constants.velTolerance);
		this.iZone = new LoggedTunableNumber(key + "/iZone", constants.iZone);
		this.iMax = new LoggedTunableNumber(key + "/maxIntegral", constants.iMax);
		this.iMin = new LoggedTunableNumber(key + "/minIntegral", constants.iMin);
		this.period = constants.period;
		this.kV = new LoggedTunableNumber(key + "/kV", constants.kV);
		this.kA = new LoggedTunableNumber(key + "/kA", constants.kA);
		this.isControllingVelocity = constants.isControllingVelocity;
		this.kS = new LoggedTunableNumber(key + "/kS", constants.kS);
		this.kG = new LoggedTunableNumber(key + "/kG", constants.kG);
		this.profiled = constants.profiled;
		this.maxVel = new LoggedTunableNumber(key + "/maxVel", constants.maxVel);
		this.maxAcc = new LoggedTunableNumber(key + "/maxAcc", constants.maxAcc);
		this.isContinuous = constants.isContinuous;
		this.maxInput = constants.maxInput;
		this.minInput = constants.minInput;
	}

	/**
	 * Returns an array of all {@link LoggedTunableNumber}s in this
	 * TunableControlConstants.
	 *
	 * @return array of all LoggedTunableNumbers in the order of: kP, kI, kD,
	 *         tolerance,
	 *         velTolerance, iZone, iMin, iMax, kV, kA, kS, kG, maxVel, maxAcc
	 */
	public LoggedTunableNumber[] getAllTunableNumbers() {
		return new LoggedTunableNumber[] {
			kP, kI, kD, tolerance, velTolerance, iZone, iMin, iMax, kV, kA, kS, kG, maxVel, maxAcc
		};
	}

	/**
	 * Creates a new {@link PIDController} (not {@link TunablePIDController}) with
	 * the configured constants.
	 *
	 * @see {@link TunablePIDController} for creating a tunable PID controller.
	 */
	public PIDController getPIDController() {
		PIDController controller = new PIDController(kP.get(), kI.get(), kD.get());
		controller.setTolerance(tolerance.get());
		controller.setIntegratorRange(iMin.get(), iMax.get());
		controller.setIZone(iZone.get());

		return controller;
	}

	/**
	 * Creates a new {@link ProfiledPIDController} (not
	 * {@link TunableProfiledController}) with the configured constants.
	 *
	 * @see {@link TunableProfiledController} for creating a tunable profiled PID
	 *      controller.
	 */
	public ProfiledPIDController getProfiledPIDController() {
		ProfiledPIDController controller = new ProfiledPIDController(
				kP.get(), kI.get(), kD.get(), new TrapezoidProfile.Constraints(maxVel.get(), maxAcc.get()));
		controller.setTolerance(tolerance.get());
		controller.setIntegratorRange(iMin.get(), iMax.get());
		controller.setIZone(iZone.get());

		return controller;
	}

	/**
	 * Creates a new {@link ElevatorFeedforward} with the configured constants.
	 *
	 * <p>
	 * Note: This is not necessary when using a {@link TunableProfiledController}.
	 */
	public ElevatorFeedforward getElevatorFeedforward() {
		return new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
	}

	/**
	 * Creates a new {@link SimpleMotorFeedforward} with the configured constants.
	 *
	 * <p>
	 * Note: This is not necessary when using a {@link TunableProfiledController}.
	 */
	public SimpleMotorFeedforward getSimpleFeedforward() {
		return new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
	}
}
