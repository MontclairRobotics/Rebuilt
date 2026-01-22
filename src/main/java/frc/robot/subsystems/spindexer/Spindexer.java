package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.SpindexerConstants;

import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", null);
  }

  public Command spinCommand(){
    return Commands.run(
      () -> {
        io.setVoltage(SpindexerConstants.SPIN_SPEED);
      });
  }

  public Command reverseSpinCommand(){
    return Commands.run(
      () -> {
        io.setVoltage(SpindexerConstants.REVERSE_SPIN_SPEED);
      });
  }

  public Command manualControlCommand(){
    return Commands.run(
      () -> {
        io.setVoltage(12 * (Math.pow(RobotContainer.operatorController.getLeftX(), 3)));
      });
  }
}
