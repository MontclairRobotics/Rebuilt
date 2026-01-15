package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO{

  @AutoLog

  public class FlywheelIOInputs {
    public double appliedVoltage = 0.0;
    public double velocity = 0.0; //rps
    public double tempCelcius = 0.0;
    public boolean isShooting = false;
    }

  public void updateInputs(FlywheelIOInputs inputs);

  public void setVoltage(double currentVoltage);

  public void stop();
  
  public void setVelocityRPM(double targetRPM);

}


