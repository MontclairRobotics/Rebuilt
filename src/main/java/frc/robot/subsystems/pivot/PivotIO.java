package frc.robot.subsystems.pivot;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double appliedVoltage;
    public double current;
    public double tempCelsius;
    public double pivotAngle;
    public boolean encoderConnected;
  }

  public void updateInputs(PivotIOInputs inputs);

  public void setVoltage(double voltage);

  public void stop();

  /**
   * @return the Angle of the pivot
   */
  public Angle getPivotAngle();
}
