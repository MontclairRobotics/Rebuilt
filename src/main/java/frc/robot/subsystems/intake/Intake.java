package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Debouncer isStalledDebouncer = new Debouncer(0.05, DebounceType.kRising);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public boolean isStalled() {
    return isStalledDebouncer.calculate(inputs.current > INTAKE_STALL_CURRENT);
  }

  public void intake() {
    io.setVoltage(INTAKE_VOLTAGE);
  }

  public void outtake() {
    io.setVoltage(-INTAKE_VOLTAGE);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // --------------------------COMMANDS--------------------------

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }

  // power intake
  public Command intakeCommand() {
    return Commands.run(this::intake, this).finallyDo(this::stop);
  }

  // used for unjamming
  public Command outtakeCommand() {
    return Commands.run(this::outtake, this);
  }

  // unjam command
  public Command unjamCommand() {
    return Commands.run(this::intake, this)
        .until(this::isStalled)
        .withTimeout(1.0)
        .andThen(Commands.run(this::outtake, this).withTimeout(0.2))
        .andThen(intakeCommand());
  }
}
