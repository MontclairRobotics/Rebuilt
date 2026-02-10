package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.PivotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOTalonFX implements PivotIO {

  private TalonFX motor;

  private DutyCycleEncoder encoder;
  private TalonFXConfiguration configs = new TalonFXConfiguration();

  public PivotIOTalonFX() {
    motor = new TalonFX(CAN_ID);
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(configs);

    encoder = new DutyCycleEncoder(ENCODER_PORT, 1, ENCODER_OFFSET);

    motor.setPosition(
        encoder.get() * GEARING); // aligns the relative encoder to the absolute encoder
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.current = motor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.pivotAngle = getPivotAngle().in(Rotations);
    inputs.encoderConnected = encoder.isConnected();
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void stop() {
    motor.stopMotor();
  }

  public Angle getPivotAngle() {
    if (encoder.isConnected()) {
      return Rotations.of(encoder.get());
    } else {
      return motor.getPosition().getValue().div(GEARING);
    }
  }
}
