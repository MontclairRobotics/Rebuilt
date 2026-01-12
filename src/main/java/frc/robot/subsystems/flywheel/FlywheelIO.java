package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO{

@AutoLog

public class FlywheelIOInputs {
   public double appliedVoltage;
   public double currentVoltage;
   public double currentVelocity;
   public double tempC;
   public boolean isShooting;

  }

public void updateInputs (FlywheelIOInputs inputs);

public void setVoltage(double currentVoltage);

public void shoot(double shootVoltage);

public void stop();

public boolean isShooting();



}


