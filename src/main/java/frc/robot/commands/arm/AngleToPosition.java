// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AngleToPosition extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSubsystem m_elevator;
    private final double m_targetAngle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AngleToPosition(ArmSubsystem subsystem, double targetAngle) {
        m_elevator = subsystem;
        m_targetAngle = targetAngle;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        m_elevator.getAnglePosition();
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        double currentAngle = m_elevator.getAnglePosition();

        if (currentAngle < m_targetAngle) {
            m_elevator.angleDown();
        } else if (currentAngle > m_targetAngle) {
            m_elevator.angleUp();
        } else {
            m_elevator.stopAngle();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elevator.stopAngle();
    }

    // Returns true when the command should end.


    @Override
    public boolean isFinished() {
        return Math.abs(m_elevator.getAnglePosition() - m_targetAngle) < .005;
    }
}

