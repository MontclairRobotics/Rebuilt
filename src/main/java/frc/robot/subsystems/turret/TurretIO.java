package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double velocity;
        public double appliedVoltage;
    }
    
    public void updateInputs(TurretIOInputs inputs);
    public double getRobotRelativeAngle(); //encoders are relative
    public void zeroRelativeEncoder(); 
    public void setRobotRelativeAngle(double angle); 
    public void setRobotRelativeAngle(DoubleSupplier supplier);
    public void setFieldRelativeAngle(double angle);
    public void setFieldRelativeAngle(DoubleSupplier supplier);
    public void setVoltage(double volts); // maybe double?
    public void stop(); // also command w/ binding, especially for testing

}