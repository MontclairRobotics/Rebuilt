package frc.robot.subsystems.shooter.spindexer.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.SerializerConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.tunables.LoggedTunableNumber;


public class Serializer extends SubsystemBase {

	private SerializerIO io;
	private SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();

	private int logCounter;
	private final int loopsPerLog;

	public LoggedTunableNumber serializerSpeedTunable = new LoggedTunableNumber("Spindexer/Serializer Speed RPS", 0);

	public Serializer(SerializerIO io) {
		this.io = io;
		loopsPerLog = RobotContainer.SERIALIZER_DEBUG ? 1 : 5;
	}

	public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

	@Override
	public void periodic() {
		// logCounter++;

		// io.updateInputs(inputs);

		// if(logCounter % loopsPerLog == 0) {
		// 	Logger.processInputs("Serializer", inputs);
		// }
	}

	public void setVelocity(AngularVelocity velocity) {
		io.setVelocity(velocity);
	}

	public void setCurrent(double currentDrawAmps) {
		io.setCurrent(currentDrawAmps);
	}

	public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier) {
		io.setVelocity(targetVelocitySupplier.get());
	}

	public void setVoltage(double voltage) {
		io.setVoltage(voltage);
	}

	public void spinUp() {
		setVelocity(SPIN_VELOCITY);
	}

	public void spinDown() {
		setVelocity(RotationsPerSecond.zero());
	}

	public void applyJoystickInput() {
        double input = -MathUtil.copyDirectionPow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.1), 1.5);
        double voltage = input * RobotController.getBatteryVoltage();
        io.setVoltage(voltage);
    }

    public Command spinDownCommand() {
        return Commands.run(() -> spinDown(), this);
    }

    public Command spinUpCommand() {
		return Commands.run(() -> spinUp(), this);
	}

	public Command setCurrentCommand(DoubleSupplier currentSupplier) {
		return Commands.run(() -> setCurrent(currentSupplier.getAsDouble()), this);
	}

    public Command joystickControlCommand() {
        return Commands.run(() -> applyJoystickInput(), this);
    }
}
