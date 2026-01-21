package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  SysIdRoutine spindexerRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              state -> SignalLogger.writeString("SysIdSpindexer_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setVoltage(output.in(Volts)), null, this));

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", null);
  }

  public Command holdSpeedCommand(double targetVelocity) {
    return Commands.run(
        () -> {
          io.setVelocityRPS(targetVelocity);
        });
  }

  public Command holdSpeedCommand(DoubleSupplier targetVelocityRPSSupplier) {
    return Commands.run(
        () -> {
          io.setVelocityRPS(targetVelocityRPSSupplier);
        });
  }

  public Command posSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return spindexerRoutine.quasistatic(direction);
  }

  public Command negSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return spindexerRoutine.quasistatic(Direction.kReverse);
  }
}
