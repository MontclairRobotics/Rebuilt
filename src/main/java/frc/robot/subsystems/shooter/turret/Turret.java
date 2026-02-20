package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.ANGLE_OFFSET;
import static frc.robot.constants.TurretConstants.MAX_ANGLE;
import static frc.robot.constants.TurretConstants.MIN_ANGLE;

import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotContainer;

public class Turret {

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO io) {
        this.io = io;
    }

    /**
	 * Handles turret wrapping through the max and min angles
	 * @param angle the angle to be constrained to a possible turret angle, in rotations
	 * @return the new angle, constrained between our min and max angles
	 */
	public static Angle constrainAngle(Angle angle) {
		if(angle.in(Rotations) > MAX_ANGLE.in(Rotations)) {
			return angle.minus(Rotations.of(1));
		} else if (angle.in(Rotations) < MIN_ANGLE.in(Rotations)) {
			return angle.plus(Rotations.of(1));
		} else {
			return angle;
		}
	}

    /**
	 * @param fieldRelativeAngle target field relative angle of the turret
	 * @return the corresponding target robot relative angle needed to achieve the stated field relative angle
	 */
	public static Angle toRobotRelativeAngle(Angle fieldRelativeAngle) {
		return constrainAngle(fieldRelativeAngle
			.minus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.minus(ANGLE_OFFSET));
	}

	/**
	 * @param robotRelativeAngle target robot relative angle of the turret
	 * @return the corresponding field relative angle the turret would point at the specified robot relative angle
	 */
	public static Angle toFieldRelativeAngle(Angle robotRelativeAngle) {
		return constrainAngle(robotRelativeAngle
			.plus(Rotations.of(RobotContainer.drivetrain.getWrappedHeading().getRotations()))
			.plus(ANGLE_OFFSET));
	}

    // public Angle getRobotRelativeAngle() {
    //     return inputs.robotRelativeAngle;
    // }
}
