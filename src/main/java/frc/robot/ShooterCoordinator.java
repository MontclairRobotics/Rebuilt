package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.constants.Constants.CURRENT_MODE;
import static frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation.FERRY_LEFT;
import static frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation.FERRY_RIGHT;
import static frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation.HUB;

import java.lang.System.Logger.Level;
import java.util.EnumSet;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.shooter.aiming.Aiming;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceManager;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.LeftTrench;
import frc.robot.util.FieldConstants.LinesVertical;
import frc.robot.util.PoseUtils;

public class ShooterCoordinator extends SubsystemBase {
    private CommandSwerveDrivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;

  	private final Distance TRENCH_ZONE_OFFSET = Meters.of(0.8);

  	private int logCounter;
  	private final int loopsPerLog;

  	public enum ShooterIntent{
		INACTIVE,
		ARMED,
		SHOOTING           
	}

	public enum ShooterMode{
		IDLE,
		SCORING,
		FERRYING_RIGHT,
		FERRYING_LEFT
	}

	private ShooterIntent currentShooterIntent = ShooterIntent.INACTIVE;
	private ShooterMode currentShooterMode = ShooterMode.SCORING;

	@Override
	public void periodic() {
		logCounter++;

		if(movingIntoObstacle()) {
		updateTrenchZonesVeloBased();
		} else {
		resetTrenchZones();
		}
		updateShooterIntent();
		updateShooterMode();

		switch(currentShooterIntent){
		case INACTIVE:
			break;
		case ARMED:
			break;
		case SHOOTING:
			switch(currentShooterMode){
			case IDLE:
				break;
			case SCORING:
				shooter.setParameters(
							() -> Aiming.calculateShot(HUB, shooter.withConstantVelocity, shooter.whileMoving)
						);
				break;
			case FERRYING_LEFT:
				shooter.setParameters(
							() -> Aiming.calculateShot(FERRY_LEFT, shooter.withConstantVelocity, shooter.whileMoving)
						);
				break;
			case FERRYING_RIGHT:
				shooter.setParameters(
							() -> Aiming.calculateShot(FERRY_RIGHT, shooter.withConstantVelocity, shooter.whileMoving)
						);
				break;
			}
			break;
		}
	};

		public ShooterCoordinator(CommandSwerveDrivetrain drivetrain, Intake intake, Shooter shooter, Vision vision) {
		loopsPerLog = RobotContainer.SUPERSTRUCTURE_DEBUG ? 1 : 10;

		this.drivetrain = drivetrain;
		this.intake = intake;
		this.vision = vision;
		this.shooter = shooter;
	}

	public ShooterIntent updateShooterIntent(){
		if(RobotContainer.operatorController.getCircleButtonPressed()){
		if(currentShooterIntent == ShooterIntent.INACTIVE){
			currentShooterIntent = ShooterIntent.ARMED;}
		else {
			currentShooterIntent = ShooterIntent.INACTIVE;
		}
		};
		if(RobotContainer.operatorController.getSquareButtonPressed()){
		if(currentShooterIntent==ShooterIntent.ARMED){
			currentShooterIntent = ShooterIntent.SHOOTING;}
		else {
			currentShooterIntent= ShooterIntent.INACTIVE;
		}
		};
		return currentShooterIntent;
	}

	public ShooterMode updateShooterMode(){
		if(inTrenchDangerZone()){
		currentShooterMode = ShooterMode.IDLE;
		}
		else if(shouldBeScoring()){
		currentShooterMode = ShooterMode.SCORING;
		}
		else if(shouldFerryLeft()){
		currentShooterMode = ShooterMode.FERRYING_LEFT;
		}
		else if(shouldFerryRight()){
		currentShooterMode = ShooterMode.FERRYING_RIGHT;
		}
		return currentShooterMode;
	}

	public void resetTrenchZones() {
		FieldConstants.Zones.TRENCH_DANGER_ZONES = new Translation2d[][]{
		// near right trench
		new Translation2d[] {
			new Translation2d(LinesVertical.HUB_CENTER.minus(TRENCH_ZONE_OFFSET), Meters.zero()),
			new Translation2d(LinesVertical.HUB_CENTER.plus(TRENCH_ZONE_OFFSET), FieldConstants.RightTrench.WIDTH)
		},

		// near left trench
		new Translation2d[] {
			new Translation2d(LinesVertical.HUB_CENTER.minus(TRENCH_ZONE_OFFSET), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
			new Translation2d(LinesVertical.HUB_CENTER.plus(TRENCH_ZONE_OFFSET), FieldConstants.FIELD_WIDTH)
		},

		// far right trench
		new Translation2d[] {
			new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(TRENCH_ZONE_OFFSET), Meters.zero()),
			new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(TRENCH_ZONE_OFFSET), FieldConstants.RightTrench.WIDTH)
		},

		// far left trench
		new Translation2d[] {
			new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(TRENCH_ZONE_OFFSET), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
			new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(TRENCH_ZONE_OFFSET), FieldConstants.FIELD_WIDTH)
		}
		};
	}

	public void updateTrenchZonesVeloBased() {
		//Updates width of zone based on robot velocity
		Distance dynamicTrenchDangerZoneWidth = Meters.of(TRENCH_ZONE_OFFSET.in(Meters) + Math.abs(drivetrain.getFieldRelativeVelocity().getX())* HoodConstants.HOOD_LOWER_TIME);
		FieldConstants.Zones.TRENCH_DANGER_ZONES = new Translation2d[][]{
		// near right trench
		new Translation2d[] {
			new Translation2d(LinesVertical.HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), Meters.zero()),
			new Translation2d(LinesVertical.HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.RightTrench.WIDTH)
		},

		// near left trench
		new Translation2d[] {
			new Translation2d(LinesVertical.HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
			new Translation2d(LinesVertical.HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH)
		},

		// far right trench
		new Translation2d[] {
			new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), Meters.zero()),
			new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.RightTrench.WIDTH)
		},

		// far left trench
		new Translation2d[] {
			new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH.minus(LeftTrench.WIDTH)),
			new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(dynamicTrenchDangerZoneWidth), FieldConstants.FIELD_WIDTH)
		}
		};
	}

	public boolean isInScoringZone() {
		Translation2d pos = shooter.turret.getFieldRelativePosition();
		return
		(AllianceManager.isRed() ?
		pos.getX() >= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.HUB_CENTER.in(Meters), 0)).getX()
			:
		pos.getX() <= FieldConstants.LinesVertical.HUB_CENTER.in(Meters)
		);
	}

	public boolean shouldBeScoring() {
			if(!AllianceManager.isAllianceKnown()) return false;
			return !inTrenchDangerZone() && (isInScoringZone() || RobotContainer.shouldShootAuto);
		}

	public boolean shouldFerryLeft() {
	if(!AllianceManager.isAllianceKnown()) return false;
		Translation2d pos = shooter.turret.getFieldRelativePosition();

		return
		!inTrenchDangerZone() &&
		(
		AllianceManager.isRed() ?
		(
			pos.getY() <= PoseUtils.flipTranslationAlliance(new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
			&& pos.getX() <= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters), 0)).getX()
		)
		:
		(
			pos.getY() >= FieldConstants.LinesHorizontal.CENTER.in(Meters)
			&& pos.getX() >= FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters)
		)
		|| RobotContainer.shouldShootAuto);
	}

	public boolean shouldFerryRight() {
	if(!AllianceManager.isAllianceKnown()) return false;
		Translation2d pos = shooter.turret.getFieldRelativePosition();

		return
		!inTrenchDangerZone() &&
		(AllianceManager.isRed() ?
			(
			pos.getY() >= PoseUtils.flipTranslationAlliance(new Translation2d(0, FieldConstants.LinesHorizontal.CENTER.in(Meters))).getY()
				&& pos.getX() <= PoseUtils.flipTranslationAlliance(new Translation2d(FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters), 0)).getX()
		)
			:
		(
			pos.getY() <= FieldConstants.LinesHorizontal.CENTER.in(Meters)
				&& pos.getX() >= FieldConstants.LinesVertical.NEUTRAL_ZONE_NEAR.in(Meters)
		)
		|| RobotContainer.shouldShootAuto);
	}

	public boolean isRedAlliance() {
			return AllianceManager.isRed();
	}
	
		// whether we are in the zone to apply trench lock
		public boolean inTrenchZone() {
			Pose2d robotPose = drivetrain.getRobotPose();
			for (Translation2d[] zone : FieldConstants.Zones.TRENCH_ZONES) {
				if (robotPose.getX() >= zone[0].getX()
						&& robotPose.getX() <= zone[1].getX()
						&& robotPose.getY() >= zone[0].getY()
						&& robotPose.getY() <= zone[1].getY()) {
					return true;
				}
			}
			return false;
		}

		public boolean inTrenchDangerZone() {
			Translation2d turretPose = shooter.turret.getFieldRelativePosition();
			for (Translation2d[] zone : FieldConstants.Zones.TRENCH_DANGER_ZONES) {
				if (
						turretPose.getX() >= zone[0].getX()
						&& turretPose.getX() <= zone[1].getX()
						&& turretPose.getY() >= zone[0].getY()
						&& turretPose.getY() <= zone[1].getY()) {
					return true;
				}
			}
			return false;
		}

		public double turretToTrenchDistance() {
			return Math.abs(FieldConstants.LinesVertical.HUB_CENTER.in(Meters) - shooter.getFieldRelativePosition().getX());
		}

		// whether we are in the zone to apply bump lock
		public boolean inBumpZone() {
			Pose2d robotPose = drivetrain.getRobotPose();
			for (Translation2d[] zone : FieldConstants.Zones.BUMP_ZONES) {
				if (robotPose.getX() >= zone[0].getX()
						&& robotPose.getX() <= zone[1].getX()
						&& robotPose.getY() >= zone[0].getY()
						&& robotPose.getY() <= zone[1].getY()) {
					return true;
				}
			}
			return false;
		}
	
	// are we moving INTO the trench?
	public boolean movingIntoObstacle() {
		Pose2d robotPose = drivetrain.getRobotPose();
		boolean movingIntoObstacleOnBlue =
		(robotPose.getX() < FieldConstants.LinesVertical.ALLIANCE_ZONE.plus(FieldConstants.Hub.WIDTH.div(2)).plus(Constants.BUMPER_WIDTH).in(Meters)
		&& drivetrain.getForwardVelocityFromController() > 0 )

		|| (robotPose.getX() > FieldConstants.LinesVertical.ALLIANCE_ZONE.in(Meters)
		&& robotPose.getX() < FieldConstants.LinesVertical.CENTER.in(Meters)
		&& drivetrain.getForwardVelocityFromController() < 0)

		|| (robotPose.getX() < FieldConstants.LinesVertical.OPP_ALLIANCE_ZONE.in(Meters)
		&& robotPose.getX() > FieldConstants.LinesVertical.CENTER.in(Meters)
		&& drivetrain.getForwardVelocityFromController() > 0)

		|| (robotPose.getX() > FieldConstants.LinesVertical.OPP_ALLIANCE_ZONE.minus(FieldConstants.Hub.WIDTH.div(2)).minus(Constants.BUMPER_WIDTH).in(Meters)
		&& drivetrain.getForwardVelocityFromController() < 0);

		if(AllianceManager.isRed()) return !movingIntoObstacleOnBlue;
		return movingIntoObstacleOnBlue;
	}
	
}
