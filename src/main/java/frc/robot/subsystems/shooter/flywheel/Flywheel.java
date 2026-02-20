package frc.robot.subsystems.shooter.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Flywheel {

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public boolean atGoal() {
        return io.atGoal();
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    public void setVelocity(AngularVelocity targetVelocity) {
        io.setVelocity(targetVelocity);
    }

    public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier) {
        io.setVelocity(targetVelocitySupplier.get());
    }

    public Command setVelocityCommand(AngularVelocity targetVelocity) {
        return Commands.run(() -> io.setVelocity(targetVelocity));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> targetVelocitySupplier) {
        return Commands.run(() -> io.setVelocity(targetVelocitySupplier.get()));
    }

}
