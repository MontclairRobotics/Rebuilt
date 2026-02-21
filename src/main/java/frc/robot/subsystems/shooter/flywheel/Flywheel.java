package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.constants.FlywheelConstants.SLOT0_CONFIGS;

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

    private final LoggedTunableNumber tunableKP = new LoggedTunableNumber("Flywheel/kP", SLOT0_CONFIGS.kP);
    private final LoggedTunableNumber tunableKD = new LoggedTunableNumber("Flywheel/kD", SLOT0_CONFIGS.kD);
    private final LoggedTunableNumber tunableKS = new LoggedTunableNumber("Flywheel/kS", SLOT0_CONFIGS.kS);
    private final LoggedTunableNumber tunableKV = new LoggedTunableNumber("Flywheel/kV", SLOT0_CONFIGS.kV);

    public final LoggedTunableNumber tuningFlywheelSpeed = new LoggedTunableNumber("Flywheel/TuningFlywheelRPM", 0);

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public boolean atGoal() {
        return io.isAtSetpoint();
    }

    private void updateTunables() {
        if(tunableKP.hasChanged(hashCode())
                || tunableKD.hasChanged(hashCode())
                || tunableKS.hasChanged(hashCode())
                || tunableKV.hasChanged(hashCode())) {
            io.setGains(tunableKP.get(), tunableKD.get(), tunableKS.get(), tunableKV.get());
        }
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        updateTunables();
    }

    public void setVelocity(AngularVelocity targetVelocity) {
        io.setVelocity(targetVelocity);
    }

    public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier) {
        io.setVelocity(targetVelocitySupplier.get());
    }

    public void applyJoystickInput() {
        double input = MathUtil.copyDirectionPow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.1), 1.5);
        double voltage = input * RobotController.getBatteryVoltage();
        io.setVoltage(voltage);
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> io.setVoltage(voltage));
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
