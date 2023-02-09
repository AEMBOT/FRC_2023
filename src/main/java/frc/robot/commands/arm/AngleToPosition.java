// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentAngle = m_elevator.getAnglePosition();
    double sigA = Math.signum(currentAngle-m_targetAngle);
    
    if(sigA == 1.0){
      m_elevator.angleDown();
    } else if (sigA == -1.0){
      m_elevator.angleUp();
    } else{
      m_elevator.stopAngle();
    }
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.getAnglePosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stopAngle();
  }

  // Returns true when the command should end.
  
  
  @Override
  public boolean isFinished() {
    //no way to check if extend meets target extend?
    return Math.abs(m_elevator.getAnglePosition() - m_targetAngle) < 5;
  }
}

