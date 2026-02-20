package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public interface FlywheelIO {

    @AutoLog
    public class FlywheelIOInputs {
        public boolean leftMotorConnected = false;
        public boolean rightMotorConnected = false;

        public AngularVelocity velocity = RotationsPerSecond.zero();
        public AngularAcceleration acceleration = RotationsPerSecondPerSecond.zero();
        public AngularVelocity setpointVelocity = RotationsPerSecond.zero();
        public AngularAcceleration setpointAcceleration = RotationsPerSecondPerSecond.zero();

        public double appliedVoltage = 0.0;
        public double currentDrawAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean atGoal = false;
    }

    public void updateInputs(FlywheelIOInputs inputs);

    public void setVelocity(AngularVelocity targetVelocity);

    public void setVoltage(double voltage);

    public void stop();

    public boolean isAtSetpoint();

    public void setGains(double kP, double kD, double kS, double kV);

}
