package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.RollersConstants.*;

import java.util.function.Supplier;

public class Rollers extends SubsystemBase {

	private final RollersIO io;
	private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

	public Rollers(RollersIO io) {
		this.io = io;
	}

	public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Rollers", inputs);
	}

	public void setVelocity(AngularVelocity velocity) {
		io.setVelocity(velocity);
	}

	public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier) {
		io.setVelocity(targetVelocitySupplier.get());
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
        return Commands.run(() -> io.setVelocity(RotationsPerSecond.zero()), this);
    }

    public Command spinUpCommand() {
		return Commands.run(() -> setVelocity(SPIN_VELOCITY), this);
	}

	public Command setVoltageCommand(double voltage) {
		return Commands.run(() -> io.setVoltage(voltage), this);
	}

    public Command joystickControlCommand() {
        return Commands.run(() -> applyJoystickInput(), this);
    }
}
