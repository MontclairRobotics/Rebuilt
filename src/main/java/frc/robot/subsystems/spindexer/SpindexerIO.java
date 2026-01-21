package frc.robot.subsystems.spindexer;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public class SpindexerIOInputs {
    public double appliedVoltage = 0.0;
    public double velocity = 0.0; // rps
    public double tempCelcius = 0.0;
    public double velocitySetpoint = 0.0;
  }

  public void updateInputs(SpindexerIOInputs inputs);

  public void setVoltage(double currentVoltage);

  public void stop();

  public void setVelocityRPS(double targetVelocity);

  public void setVelocityRPS(DoubleSupplier targetVelocitySupplier);

  public boolean atSetPoint();
}
