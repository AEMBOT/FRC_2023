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
  private final double m_targetPos; //Get targetPosition from controller??

  //private final double m_angleMotorRotation;
  //private final double m_angle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AngleToPosition(ArmSubsystem subsystem, /*double angleMotorRotation, double angle, */ double targetPosition) {
    m_elevator = subsystem;
    m_targetPos = targetPosition;
    //m_angleMotorRotation = angleMotorRotation;
    //m_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPos = m_elevator.getAnglePosition();
    double diff = currentPos - m_targetPos;
    double sig = Math.signum(diff);
    if(sig == 1){
      m_elevator.angleDown();
    } else {
      m_elevator.angleUp();
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
    return Math.abs(m_elevator.getAnglePosition() - m_targetPos) < .05;
  }
}

