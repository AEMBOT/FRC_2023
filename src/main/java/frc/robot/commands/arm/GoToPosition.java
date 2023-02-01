package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class GoToPosition extends CommandBase{
    
 private final ArmSubsystem m_armSubsystem;

 public GoToPosition(ArmSubsystem armSubsystem){
        m_armSubsystem = armSubsystem;
 }

 @Override
 public void execute() {
    m_armSubsystem.extendArm();
    m_armSubsystem.angleDown();
 }

 @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopExtend();
    m_armSubsystem.stopAngle();
  }

  @Override
  public boolean isFinished() {
    return m_armSubsystem.goToPosition();
  }
}
