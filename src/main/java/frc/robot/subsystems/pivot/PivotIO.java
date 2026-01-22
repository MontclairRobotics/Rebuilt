package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double appliedVoltage;
    public double current;
    public double tempCelsius;

    public Rotation2d pivotAngle;
    public boolean encoderConnected;
    public Rotation2d angle;
  }

  public void updateInputs(PivotIOInputs inputs);

  public void setVoltage(double voltage);

  public double calculateStationaryFeedforward();

  public void stop();

  /**
   * @return the angle of the arm (joint 1) relative to the horizontal
   */
  public Rotation2d getPivotAngle();

  public boolean atSetpoint();

  public double getPercentRotation();

  public void resetPIDController();
}
