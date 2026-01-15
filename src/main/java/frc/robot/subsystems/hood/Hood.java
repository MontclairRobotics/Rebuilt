package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;

public class Hood {
	private HoodIO hoodIO;
    private HoodIOInputs hoodIOInputs;

    public Hood(HoodIO hoodIO, HoodIOInputs hoodIOInputs) {
        this.hoodIO = hoodIO;
        this.hoodIOInputs = hoodIOInputs;
    }

    public void periodic() {
        hoodIO.updateInputs(hoodIOInputs);
    }

    public void setVoltage(double voltage) {
        hoodIO.setVoltage(voltage);
    }

    /** Gets the angle in Rotations from the relative encoder */
    public double getAngle() {
        return hoodIO.getAngle();
    }

    public void setAngle(double goal) {
        hoodIO.setAngle(goal);
    }

    public void setAngle(DoubleSupplier goalSupplier) {
        hoodIO.setAngle(goalSupplier);
    }

    public void stop() {
        hoodIO.stop();
    }

    public boolean atSetpoint() {
        return hoodIO.atSetpoint();
    }
    
    public void doJoystickControls() {
        double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3);
        voltage = MathUtil.clamp(voltage, -1,1); //TODO: Use real clamp values -1, 1 is temporary
        //Maybe do feedforward?
        this.setVoltage(voltage);
    }

    public void angleToHub() { //Sets the angle to whatever gets us to score in the outpost
        //TODO: Use vision stuffs later to accomplish this task
    }
}