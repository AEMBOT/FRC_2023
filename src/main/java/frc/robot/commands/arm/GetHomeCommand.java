package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class GetHomeCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    public GetHomeCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_armSubsystem.retractArm();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopExtend();
        m_armSubsystem.resetExtendEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_armSubsystem.isCurrentLimited();
    }
}
