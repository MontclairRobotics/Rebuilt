package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.subsystems.shooter.aiming.Aiming;

public class ShooterCoordinator extends SubsystemBase{
    public static ShooterGoal shooterGoal;
		private Shooter shooter;

		@Override
		public void periodic(){
			shooterGoal = calculateGoal(
				Superstructure.isInScoringZone(),
				Superstructure.isInLeftFerryZone(),
				Superstructure.isInRightFerryZone(),
				Superstructure.isInTrenchZone(),
				operatorWantsScoring(),
				operatorWantsFerrying(),
				operatorWantsFiring()
			);
		}

    public enum ShooterIntent{

		INACTIVE(
			false, 
			false, 
			false, 
			false
		),
		ARMED(
			true, 
			true, 
			true, 
			false
		),
		FIRING(
			true, 
			true, 
			true, 
			true
		);

		private boolean allowFlywheel;
		boolean allowTurret;
		private boolean allowHood;
		private boolean allowSpindexer;
		
		ShooterIntent(boolean allowFlywheel, boolean allowTurret, boolean allowHood, boolean allowSpindexer) {
			this.allowFlywheel = allowFlywheel;
			this.allowTurret = allowTurret;
			this.allowHood = allowHood;
			this.allowSpindexer = allowSpindexer;
		}

		public boolean isFlywheelAllowed() {return allowFlywheel;}
		public boolean isTurretAllowed() {return allowTurret;}
		public boolean isHoodAllowed() {return allowHood;}
		public boolean isSpindexerAllowed() {return allowSpindexer;}

	}

	public enum ShooterMode{

		IDLE(Translation2d.kZero),
		SCORING(Aiming.TargetLocation.HUB.getLocation()),
		FERRYING_RIGHT(Aiming.TargetLocation.FERRY_RIGHT.getLocation()),
		FERRYING_LEFT(Aiming.TargetLocation.FERRY_LEFT.getLocation());

		private Translation2d pointOfInterest;

		ShooterMode(Translation2d pointOfInterest) {
			this.pointOfInterest = pointOfInterest;
		}

		public Translation2d pointOfInterest() {
            return pointOfInterest;
        }
	}

    public record ShooterGoal(ShooterMode mode, ShooterIntent intent) {
        public ShooterGoal(ShooterMode mode, ShooterIntent intent) {
            this.mode = mode;
            this.intent = intent;
        }
    }

    public ShooterCoordinator(Shooter shooter) {
        this.shooter = shooter;
				shooterGoal = new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
    }

	public boolean operatorWantsFiring(){
		return RobotContainer.operatorController.circle().getAsBoolean();
	}
	
	public boolean operatorWantsScoring(){
		return RobotContainer.operatorController.square().getAsBoolean();
	}

	public boolean operatorWantsFerrying(){
		return RobotContainer.operatorController.triangle().getAsBoolean();
	}

    public ShooterGoal calculateGoal(
		boolean isInScoringZone, 
		boolean isInLeftFerryZone, 
		boolean isInRightFerryZone,
		boolean isInTrenchZone,
		boolean operatorWantsScoring,
		boolean operatorWantsFerrying,
		boolean operatorWantsFiring
	) {
		if(isInTrenchZone) {
			return new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
		}

		if(RobotContainer.shouldShootAuto){
			return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.FIRING);
		}

		if(operatorWantsScoring) {
			if(operatorWantsFiring) {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.FIRING);
			}
		}

		if(operatorWantsFerrying){
			if(operatorWantsFiring){
				if(isInLeftFerryZone){
					return new ShooterGoal(ShooterMode.FERRYING_LEFT, ShooterIntent.FIRING);
				}
				else if (isInRightFerryZone){
					return new ShooterGoal(ShooterMode.FERRYING_RIGHT, ShooterIntent.FIRING);
				}
				else{
					return new ShooterGoal(ShooterMode.FERRYING_LEFT, ShooterIntent.FIRING);
				}
			}
			return new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
			}
		return new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);		
	}
}
