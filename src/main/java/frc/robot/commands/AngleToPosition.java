// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmToPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevator;
  private final double m_angleMotorRotation;
  private final double m_angle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmToPosition(ElevatorSubsystem subsystem, double angleMotorRotation) {
    m_elevator = subsystem;
    m_angleMotorRotation = angleMotorRotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double diff = angleEncoder.getPosition() - targetPosition;
    double sig = Math.signum(diff);
    if(sig == 1){
      m_elevator.angleDown();
    } else {
      m_elevator.angleUp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
        if(Math.abs(angleEncoder.getPosition()- targetPosition) < .05){
          return Math.abs(angleEncoder.getPosition()-targetPosition) < .05;
        }
  }
}
