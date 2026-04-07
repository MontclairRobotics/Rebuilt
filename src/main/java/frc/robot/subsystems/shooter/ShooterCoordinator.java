package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.shooter.aiming.Aiming;

public class ShooterCoordinator {
    
    private Shooter shooter;

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
		SHOOTING(
			true, 
			true, 
			true, 
			true
		);

		private boolean allowFlywheel;
		private boolean allowTurret;
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
    }
 
    public ShooterGoal calculateGoal() {
        // to be replaced with actual logic
        return new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE); 
    }

}
