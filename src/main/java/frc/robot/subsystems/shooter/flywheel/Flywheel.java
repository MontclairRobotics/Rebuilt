package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.constants.FlywheelConstants.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

	private FlywheelIO io;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

	private PIDController pidController;
	private SimpleMotorFeedforward motorFeedforward;

	SysIdRoutine flyWheelRoutine = new SysIdRoutine(
		new SysIdRoutine.Config(
			null,
			Volts.of(4),
			null,
			state -> SignalLogger.writeString("SysIdFlywheel_State", state.toString())
		),
		new SysIdRoutine.Mechanism(output -> io.setVoltage(output.in(Volts)), null, this)
	);

	public Flywheel(FlywheelIO io) {
		this.io = io;

		pidController = new PIDController(kP, kI, kD);
		pidController.setTolerance(TOLERANCE.in(RotationsPerSecond));

		motorFeedforward = new SimpleMotorFeedforward(kS, kV);
	}

	/**
	 * Uses PID and FF Control to ramp up the Flywheel to a target angular velocity
	 * @param targetFlywheelVelocity the target angular velocity of the flywheel
	 */
	public void setVelocityRPS(AngularVelocity targetFlywheelVelocity) {
		double pidOutput = pidController.calculate(io.getFlywheelVelocity().in(RotationsPerSecond), targetFlywheelVelocity.in(RotationsPerSecond));
		double ffOutput = motorFeedforward.calculate(targetFlywheelVelocity.in(RotationsPerSecond));
		double totalOutput = pidOutput + ffOutput;
		io.setVoltage(MathUtil.clamp(totalOutput, 12.0, -12.0));
	}

	/**
	 * Calls {@link #setVelocityRPS} to continuously ramp up the Flywheel to a changing target angular velocity
	 * @param targetFlywheelVelocitySupplier supplier function (lambda) of the target angular velocity of the flywheel
	 */
	public void setVelocityRPS(Supplier<AngularVelocity> targetFlywheelVelocitySupplier) {
		setVelocityRPS(targetFlywheelVelocitySupplier.get());
	}

	/**
	 * @return whether or not the flywheel is within tolerance of its setpoint
	 */
	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Flywheel", inputs);
	}

	public Command holdVelocityCommand(AngularVelocity targetFlywheelVelocity) {
		return Commands.run(() -> {
			setVelocityRPS(targetFlywheelVelocity);
		});
	}

	public Command holdVelocityCommand(Supplier<AngularVelocity> targetFlywheelVelocitySupplier) {
		return Commands.run(() -> {
			setVelocityRPS(targetFlywheelVelocitySupplier);
		});
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return flyWheelRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return flyWheelRoutine.dynamic(direction);
	}
}
