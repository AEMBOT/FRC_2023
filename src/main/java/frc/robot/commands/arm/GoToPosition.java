package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class GoToPosition extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_armSubsystem;
    private final double m_targetExtend;
    private final double m_targetAngle;

    public GoToPosition(ArmSubsystem armSubsystem, double targetExtend, double targetAngle) {
        m_armSubsystem = armSubsystem;
        m_targetExtend = targetExtend;
        m_targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        m_armSubsystem.setExtendPIDState(true);
        m_armSubsystem.setArmPosition(m_targetExtend, m_targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopExtend();
        m_armSubsystem.stopAngle();
        m_armSubsystem.setExtendPIDState(false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_armSubsystem.extendEncoder.getPosition() - m_targetExtend) < 0.005 && Math.abs(m_armSubsystem.getAnglePosition() - m_targetAngle) < 0.005;

    }
}