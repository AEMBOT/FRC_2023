package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class GoToPosition extends CommandBase{
    
 private final ArmSubsystem m_armSubsystem;
 private final double m_targetExtend;

 public GoToPosition(ArmSubsystem armSubsystem, double targetExtend){
      m_armSubsystem = armSubsystem;
      m_targetExtend = targetExtend;
 }

@Override
public void initialize(){
  m_armSubsystem.setExtendMeter(m_targetExtend);
}

 //@Override
 //public void execute() {
  /*if(m_armSubsystem.extendEncoder.getPosition() < m_targetExtend){
    m_armSubsystem.retractArm();
  } else if(m_armSubsystem.extendEncoder.getPosition() > m_targetExtend){
    m_armSubsystem.extendArm();
  } else{
    m_armSubsystem.stopExtend();
  }
  */
  /*if (m_armSubsystem.extendEncoder.getPosition() < -40) {
    m_armSubsystem.retractArm();
  }
  else if (m_armSubsystem.extendEncoder.getPosition() > -35) {
    m_armSubsystem.extendArm();
  }
  else {
    m_armSubsystem.stopExtend();
  }
  */
 //}


 @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopExtend();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_armSubsystem.extendEncoder.getPosition()- m_targetExtend) < 0.005;
    //return m_armSubsystem.extendEncoder.getPosition() >= -40 && m_armSubsystem.extendEncoder.getPosition() <= -35;
  }
}