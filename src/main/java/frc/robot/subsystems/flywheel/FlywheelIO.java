package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO{

@AutoLog

public class FlywheelIOInputs {
   public double appliedVoltageRPM = 0.0;
   public double currentVoltageRPM = 0.0;
   public double currentVelocityRPM = 0.0;
   public double tempCelcius = 0.0;
   public boolean isShooting = false;

  }

public void updateInputs (FlywheelIOInputs inputs);

public void setVoltage(double currentVoltage);

public void runVelocity(double motorRpm);

public void shoot(double shootVoltage);

public void stop();

public boolean isShooting(boolean isShooting);

public void setPID(double kP, double kI, double kD);



}


