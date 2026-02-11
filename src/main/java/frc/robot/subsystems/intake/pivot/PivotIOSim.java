package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.PivotConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {

  private SingleJointedArmSim sim;
  private double appliedVoltage;

  public PivotIOSim() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            GEARING,
            10.0,
            0.0,
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            MIN_ANGLE.in(Radians),
            0.0,
            0.0);
  }

  public void updateInputs(PivotIOInputs inputs) {
    // updates the simulator
    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);

    // updates inputs
    inputs.appliedVoltage = appliedVoltage;
    inputs.current = sim.getCurrentDrawAmps();
    inputs.pivotAngle = getPivotAngle().in(Rotations);
    inputs.tempCelsius = 0;
    inputs.encoderConnected = false;
  }

  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
  }

  public void stop() {
    sim.setInputVoltage(0);
  }

  public Angle getPivotAngle() {
    return Radians.of(sim.getAngleRads());
  }
}
