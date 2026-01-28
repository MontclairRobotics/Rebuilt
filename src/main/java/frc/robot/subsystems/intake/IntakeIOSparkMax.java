package frc.robot.subsystems.intake;

// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {
  private SparkMax motor;
  private SparkMaxConfig config;

  public IntakeIOSparkMax() {
    motor = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit(20).idleMode(IdleMode.kBrake); // TODO: find stall limit

    motor.configure(
        config.inverted(false),
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.current = motor.getOutputCurrent();
    inputs.appliedVoltage = motor.getAppliedOutput();
  }

  @Override
  public void setVoltage(double voltage) {
    setVoltage(voltage);
  }

  @Override
  public void set(double speed) {
    motor.set(speed);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
