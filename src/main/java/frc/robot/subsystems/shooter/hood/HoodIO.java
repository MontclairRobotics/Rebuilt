package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface HoodIO {

    @AutoLog
    public class HoodIOInputs {
        public boolean motorConnected = false;

        public double appliedVoltage = 0.0;
        public double currentDrawAmps = 0.0;
        public double tempCelcius = 0.0;

        public Angle hoodAngle = Rotations.zero();
        public Angle hoodAngleSetpoint = Rotations.zero();
        public AngularVelocity hoodVelocity = RotationsPerSecond.zero();

        public boolean isAtSetpoint = false;
    }

    public void updateInputs(HoodIOInputs inputs);

    public void setAngle(Angle angle);

    public void setVoltage(double voltage);

    public void stop();

    public boolean isAtSetpoint();

    public void resetEncoderPosition();

    public void setGains(double kP, double kD, double kS, double kG);

    public void setMotionMagic(double velocity, double acceleration, double jerk);

    public void setNeutralMode(NeutralModeValue value);

}
