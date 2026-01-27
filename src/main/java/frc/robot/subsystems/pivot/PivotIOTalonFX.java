package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.Constants.PivotConstants;
import frc.robot.util.PoseUtils;

public class PivotIOTalonFX implements PivotIO {

  // motor controller
  private TalonFX motor;

  // pid & ff controllers
  private ArmFeedforward PivotFeedforward = new ArmFeedforward(0, 0, 0);
  private PIDController pidController = new PIDController(0, 0, 0);

  // encoders
  private RelativeEncoder relativeEncoder;
  private DutyCycleEncoder PivotEncoder;

  // for degrees
  private Rotation2d angleDeg;
  // insert panicked

  // configs of TalonFX
  TalonFXConfiguration configs = new TalonFXConfiguration();

  public PivotIOTalonFX() {
    motor = new TalonFX(PivotConstants.PIVOT_MOTOR_ID);
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(configs);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.current = motor.getStatorCurrent().getValueAsDouble();
    
    //needed?
    inputs.angle = angleDeg;

    inputs.encoderConnected = PivotEncoder.isConnected();
  }

  //supposed to be zero?
  public double calculateStationaryFeedforward() {
    double voltage = PivotFeedforward.calculate(getPivotAngle().getRadians(), 0);
    return getPivotAngle().getDegrees() > 0 ? voltage : -voltage;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setForearmAngle(Rotation2d angle) {

    double target =
        MathUtil.clamp(
            angleDeg.getRotations(),
            PivotConstants.PIVOT_MIN_ANGLE.getRotations(),
            PivotConstants.PIVOT_MAX_ANGLE.getRotations());

    double voltage = pidController.calculate(getPivotAngle().getRotations(), target) * 5;
    voltage = MathUtil.clamp(voltage, -PivotConstants.VOLTAGE_LIMIT, PivotConstants.VOLTAGE_LIMIT);
    setVoltage(-voltage);
  }

  public void stop() {
    motor.stopMotor();
  }

  public Rotation2d getArmAngle() {
    if (PivotEncoder.isConnected()) {
      return Rotation2d.fromRotations(PivotEncoder.get());
    } else {
      return Rotation2d.fromRotations((relativeEncoder.getPosition() / PivotConstants.TBD) % 1);
    }
  }

  public double getPercentRotation() {
    double distance =
        PoseUtils.getAngleDistance(getPivotAngle(), PivotConstants.PIVOT_MIN_ANGLE).getDegrees();
    double interval =
        PoseUtils.getAngleDistance(PivotConstants.PIVOT_MAX_ANGLE, PivotConstants.PIVOT_MIN_ANGLE)
            .getDegrees();
    return distance / interval;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void resetPIDController() {
    pidController.reset();
  }

  public Rotation2d getPivotAngle() {
    if (PivotEncoder.isConnected()) {
      return Rotation2d.fromRotations(PivotEncoder.get());
    } else {
      return Rotation2d.fromRotations((relativeEncoder.getPosition() / PivotConstants.TBD) % 1);
    }
  }
}
