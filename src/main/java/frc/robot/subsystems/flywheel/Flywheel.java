package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

	FlywheelIO io;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

	private PIDController pidController =
		new PIDController(
			FlywheelConstants.SLOT0_CONFIGS.kP,
			FlywheelConstants.SLOT0_CONFIGS.kI,
			FlywheelConstants.SLOT0_CONFIGS.kD
		);
	private SimpleMotorFeedforward motorFeedForward =
		new SimpleMotorFeedforward(
			FlywheelConstants.SLOT0_CONFIGS.kS, FlywheelConstants.SLOT0_CONFIGS.kV
		);

	SysIdRoutine flyWheelRoutine =
		new SysIdRoutine(
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
	}

	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.appliedVoltage = io.getMotorVoltage();
		inputs.tempCelcius = io.getMotorTemp(); // celsius
		inputs.velocity = io.getMotorVelocity(); // RPS
		inputs.velocitySetpoint = pidController.getSetpoint();
	}

	public void setVelocityRPS(DoubleSupplier targetVelocitySupplier) {
		setVelocityRPS(targetVelocitySupplier.getAsDouble());
	}

	public void setVelocityRPS(double targetVelocity) {
		double pidOutput = pidController.calculate(io.getMotorVelocity(), targetVelocity);
		double ffVolts = motorFeedForward.calculate(targetVelocity);
		double totalOutput = pidOutput + ffVolts;
		io.setVoltage(MathUtil.clamp(totalOutput, 12.0, -12.0));
	}

	public boolean atSetPoint() {
		return pidController.atSetpoint();
	}

	@Override
	public void periodic() {
		updateInputs(inputs);
		Logger.processInputs("Flywheel", inputs);
	}

	public Command holdSpeedCommand(double targetVelocity) {
		return Commands.run(() -> {
			setVelocityRPS(targetVelocity);
		});
	}

	public Command holdSpeedCommand(DoubleSupplier targetVelocityRPSSupplier) {
		return Commands.run(() -> {
			setVelocityRPS(targetVelocityRPSSupplier);
		});
	}

	public Command posSysIdQuasistatic(SysIdRoutine.Direction direction) {
		return flyWheelRoutine.quasistatic(direction);
	}

	public Command negSysIdQuasistatic(SysIdRoutine.Direction direction) {
		return flyWheelRoutine.quasistatic(Direction.kReverse);
	}
}
