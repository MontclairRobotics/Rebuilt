package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.PivotConstants.*;
import static frc.robot.constants.TurretConstants.CAN_BUS;
import static frc.robot.constants.TurretConstants.ENCODER_ID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class PivotIOTalonFX implements PivotIO {

	private TalonFX motor;

	private CANcoder encoder;
	private TalonFXConfiguration configs = new TalonFXConfiguration();

    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);


	public PivotIOTalonFX() {
		motor = new TalonFX(CAN_ID);
		motor.setNeutralMode(NeutralModeValue.Brake);

        encoder = new CANcoder(ENCODER_ID, CAN_BUS);
	}

	public void updateInputs(PivotIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.current = motor.getStatorCurrent().getValueAsDouble();
		inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
		inputs.angle = getAngle().in(Rotations);
		inputs.encoderConnected = encoder.isConnected();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public void setAngle(Angle angle) {
		motor.setControl(request.withPosition(angle));
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

    @Override
    public void setGains(double kP, double kD, double kS, double kG) {
        configs.Slot0.kP = kP;
        configs.Slot0.kD = kD;
        configs.Slot0.kS = kS;
        configs.Slot0.kG = kG;

        motor.getConfigurator().apply(configs.Slot0);
    }

	public Angle getAngle() {
		if (encoder.isConnected()) {
			return encoder.getPosition().getValue();
		} else {
			return motor.getPosition().getValue().div(GEARING);
		}
	}

    @Override
    public void setMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration, double jerk) {
        configs.MotionMagic.MotionMagicCruiseVelocity = velocity.in(RotationsPerSecond);
        configs.MotionMagic.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        configs.MotionMagic.MotionMagicJerk = jerk;

        motor.getConfigurator().apply(configs.MotionMagic);
    }

	@Override
	public boolean isAtSetpoint() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'isAtSetpoint'");
	}
}
