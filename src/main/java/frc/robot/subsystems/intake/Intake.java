package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Debouncer isStalledDebouncer = new Debouncer(0.05, DebounceType.kRising);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public boolean isStalled() {
    return isStalledDebouncer.calculate(inputs.current > IntakeConstants.INTAKE_STALL_CURRENT);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // --------------------------COMMANDS--------------------------

  // TODO: intake class for SparkMax?

  // stop all commands
  public Command stopCommands() {
    return Commands.runOnce(() -> io.stop(), this);
  }

  // power intake
  public Command intakeCommand() {
    return Commands.run(() -> io.set(IntakeConstants.INTAKE_FUEL_SPEED), this)
        .finallyDo(
            () -> {
              io.stop();
            });
  }

  // used for unjamming
  public Command outtakeCommand() {
    return Commands.run(() -> io.set(IntakeConstants.OUTTAKE_FUEL_SPEED), this);
  }

  // unjam command
  public Command unjamCommand() {
    return Commands.run(() -> io.set(IntakeConstants.INTAKE_FUEL_SPEED), this)
        .until(this::isStalled)
        .withTimeout(1.0)
        .andThen(Commands.run(() -> io.set(-0.1), this).withTimeout(0.1))
        .andThen(intakeCommand());
  }
}
