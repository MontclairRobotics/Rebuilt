package frc.robot;
import com.ctre.phoenix6.Orchestra;
public class Music  {

Orchestra m_orchestra = new Orchestra();
m_orchestra.addInstrument(RobotContainer.shooter.flywheel.right_motor);
m_orchestra.addInstrument(RobotContainer.shooter.flywheel.left_motor);
var status =m_orchestra.loadMusic("somethong");


}
