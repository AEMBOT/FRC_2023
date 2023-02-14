package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class GoToPosition extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    public GoToPosition(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        if (m_armSubsystem.extendEncoder.getPosition() < -40) {
            m_armSubsystem.retractArm();
        } else if (m_armSubsystem.extendEncoder.getPosition() > -35) {
            m_armSubsystem.extendArm();
        } else {
            m_armSubsystem.stopExtend();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopExtend();
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.extendEncoder.getPosition() >= -40 && m_armSubsystem.extendEncoder.getPosition() <= -35;
    }
}