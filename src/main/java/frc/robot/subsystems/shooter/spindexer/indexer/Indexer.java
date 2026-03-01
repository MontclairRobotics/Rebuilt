package frc.robot.subsystems.shooter.spindexer.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.IndexerConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

	private IndexerIO io;
	private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

	private int logCounter;
	private final int loopsPerLog;

	public Indexer(IndexerIO io) {
		this.io = io;
		loopsPerLog = RobotContainer.SHOOTER_DEBUG ? 1 : 5;
	}

	public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

	@Override
	public void periodic() {
		logCounter++;

		if(logCounter % loopsPerLog == 0) {
			io.updateInputs(inputs);
			Logger.processInputs("Indexer", inputs);
		}
	}

	public void setVelocity(AngularVelocity velocity) {
		io.setVelocity(velocity);
	}

	public void setVoltage(double voltage) {
		io.setVoltage(voltage);
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

    public Command joystickControlCommand() {
        return Commands.run(() -> applyJoystickInput(), this);
    }
}
