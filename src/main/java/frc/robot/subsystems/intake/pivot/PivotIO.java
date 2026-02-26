package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs {
      public double appliedVoltage;
      public double current;
      public double tempCelsius;
      public double angle;
      public boolean encoderConnected;
      public boolean isAtSetpoint;
    }

    public void updateInputs(PivotIOInputs inputs);

    public void setVoltage(double voltage);

    public void stop();

    /**
     * @return the Angle of the pivot
     */
    public Angle getAngle();

    public void setAngle(Angle angle);

    public boolean isAtSetpoint();

    public void resetEncoderPosition();

    public void setGains(double kP, double kD, double kS, double kG);

    public void setMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration, double jerk);
  }
