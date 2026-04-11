package frc.robot.subsystems.shooter.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public LoggedTunableNumber tuningFlywheelSpeed;

    private int logCounter;
    private final int loopsPerLog;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        loopsPerLog = RobotContainer.FLYWHEEL_DEBUG ? 1 : 5;
        tuningFlywheelSpeed = new LoggedTunableNumber("Flywheel/TuningFlywheelRPS", 0);
    }

    public boolean atSetpoint() {
        return inputs.isAtSetpoint;
    }

    public void periodic() {
        logCounter++;

        // need to update this every frame
        io.updateInputs(inputs); 
        Logger.processInputs("Flywheel", inputs);

        if(logCounter % loopsPerLog == 0) {
            // any expensive, derived logging here
        }
    }

    public AngularVelocity getVelocity() {
        return inputs.velocity;
    }

    public void setVelocity(AngularVelocity targetVelocity) {
        io.setVelocity(targetVelocity);
    }

    public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier) {
        setVelocity(targetVelocitySupplier.get());
    }

    public void stop() {
        io.stop();
    }

    public void applyJoystickInput() {
        double input = -MathUtil.copyDirectionPow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.1), 1.5);
        double voltage = input * RobotController.getBatteryVoltage();
        io.setVoltage(voltage);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> io.stop(), this);
    }

    public Command setVelocityCommand(AngularVelocity targetVelocity) {
        return Commands.run(() -> io.setVelocity(targetVelocity));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> targetVelocitySupplier) {
        return Commands.run(() -> io.setVelocity(targetVelocitySupplier.get()));
    }

    public Command joystickControlCommand() {
        return Commands.run(() -> applyJoystickInput(), this);
    }

}
