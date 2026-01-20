package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.TunerConstants;

public class DriveConstants {

	public static final Distance bumperWidth = Inches.of(0); // TODO: set
	
	public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts; // TODO: set
	public static final LinearAcceleration MAX_ACCELERATION = 
		MetersPerSecondPerSecond.of(12); // TODO: set

	public static final AngularVelocity MAX_ANGULAR_SPEED = 
		RotationsPerSecond.of(4); // TODO: set
  	public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = 
		RotationsPerSecondPerSecond.of(9); // TODO: set

	public static final PathConstraints DEFAULT_CONSTRAINTS =
		new PathConstraints(
			MAX_SPEED, 
			MAX_ACCELERATION, 
			MAX_ANGULAR_SPEED, 
			MAX_ANGULAR_ACCELERATION
		);
}
