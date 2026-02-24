package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs {
        public boolean motorConnected = false;

        public double appliedVoltage = 0.0;
        public double currentDrawAmps = 0.0;
        public double tempCelcius = 0.0;

        public AngularVelocity velocity = RotationsPerSecond.zero();
        public Angle robotRelativeAngle = Rotations.zero();
        public Angle fieldRelativeAngle = Rotations.zero();
        public Angle robotRelativeAngleSetpoint = Rotations.zero();

        public boolean isAtSetpoint = false;
    }

    public void updateInputs(TurretIOInputs inputs);

    public void setRobotRelativeAngle(Angle angle);

    public void setVoltage(double voltage);

    public void stop();

    public boolean isAtSetpoint();

    public void setGains(double kP, double kD, double kS);

    public void setMotionMagic(double velocity, double acceleration, double jerk);

    public void setNeutralMode(NeutralModeValue value);

}
