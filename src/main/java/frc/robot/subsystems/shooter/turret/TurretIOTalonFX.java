// package frc.robot.subsystems.shooter.turret;

// import static edu.wpi.first.units.Units.Rotations;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static frc.robot.constants.TurretConstants.*;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import frc.robot.RobotContainer;

// public class TurretIOTalonFX implements TurretIO {

// 	private TalonFX motor;

// 	public TurretIOTalonFX() {
// 		motor = new TalonFX(CAN_ID);
// 	}

// 	@Override
// 	public void updateInputs(TurretIOInputs inputs) {
// 		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
// 		inputs.motorVelocity = getMotorVelocity().in(RotationsPerSecond);
// 		inputs.turretVelocity = getTurretVelocity().in(RotationsPerSecond);
// 		inputs.robotRelativeAngle = getRobotRelativeAngle().in(Rotations);
// 		inputs.fieldRelativeAngle = getFieldRelativeAngle().in(Rotations);
// 	}

// 	@Override
// 	public void setVoltage(double volts) {
// 		motor.setVoltage(volts);
// 	}

// 	@Override
// 	public void stop() {
// 		motor.stopMotor();
// 	}

// 	@Override
// 	public AngularVelocity getMotorVelocity() {
// 		return motor.getVelocity().getValue();
// 	}

// 	@Override
// 	public AngularVelocity getTurretVelocity() {
// 		return getMotorVelocity().div(GEARING);
// 	}

// 	@Override
// 	public Angle getRobotRelativeAngle() {
// 		return motor.getPosition().getValue().div(GEARING);
// 	}

// 	@Override
// 	public Angle getFieldRelativeAngle() {
// 		return RobotContainer.turret.toFieldRelativeAngle(getRobotRelativeAngle());
// 	}

// 	@Override
// 	public void zero() {
// 		motor.setPosition(0);
// 	}
// }
