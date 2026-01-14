package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {
    @AutoLog
    public static class TurretInputs{
        public AngularVelocity currentVelocity;
        public Voltage currentVoltage;
    }
    public void updateInputs(TurretInputs input);
    public void setRobotRelativeAngle(Rotation2d angle); 
    public Rotation2d getRobotRelativeAngle();//encoders are relative
    public void zeroRelativeEncoder(); //make command variant too
    public void setContinuousAngle(DoubleSupplier supplier);
    public void setVoltage(double voltage); // maybe double?
    public void stop(); // also command w/ binding, especially for testing

}