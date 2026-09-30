package frc.robot.subsystems.intake.rollers;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static frc.robot.constants.RollersConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class Rollers extends SubsystemBase {

	private final RollersIO io;
	private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

	private int logCounter;
	private final int loopsPerLog;

	public Rollers(RollersIO io) {
		this.io = io;
		loopsPerLog = RobotContainer.ROLLERS_DEBUG ? 1 : 5;
	}

	public boolean atSetpoint() {
		return inputs.isAtSetpoint;
	}

	@Override
	public void periodic() {
		logCounter++;

		io.updateInputs(inputs);
		Logger.processInputs("Rollers", inputs);

		if(logCounter % loopsPerLog == 0) {
			// any expensive, derived logging here
		}
	}

	public void setVelocity(AngularVelocity velocity) {
		io.setVelocity(velocity);
	}

	public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier) {
		io.setVelocity(targetVelocitySupplier.get());
	}

	public void applyJoystickInput() { //Unused open loop
        double input = -MathUtil.copyDirectionPow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.1), 1.5);
        double voltage = input * 12;

        io.setVoltage(voltage);
    }

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop(), this);
	}
    public Command spinDownCommand() {
        return setVoltageCommand(0);
    }

    public Command spinUpCommand() {
		return setVoltageCommand(SPIN_VOLTAGE);
	}

	public Command setVoltageCommand(DoubleSupplier voltage) {
		return setVoltageCommand(voltage.getAsDouble());
	}

	public Command setVoltageCommand(double voltage) {
		return Commands.run(() -> io.setVoltage(voltage), this);
	}

    public Command joystickControlCommand() { //Unused
        return Commands.run(() -> applyJoystickInput(), this);
    }
}
