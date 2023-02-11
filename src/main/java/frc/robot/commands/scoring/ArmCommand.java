// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.AngleToPosition;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private final AutoScoring.TARGET_POSITION_Y target_position_y;
  private AngleToPosition angleToPosition;
  private GoToPosition goToPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem, AutoScoring.TARGET_POSITION_Y y) {
    m_subsystem = subsystem;
    target_position_y = y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (target_position_y){
      case TOP:
        angleToPosition = new AngleToPosition(m_subsystem, 0.0);
        goToPosition = new GoToPosition(m_subsystem);
        break;
      case CENTER:
        angleToPosition = new AngleToPosition(m_subsystem, 0.0);
        goToPosition = new GoToPosition(m_subsystem);
        break;
      case BOTTOM:
        angleToPosition = new AngleToPosition(m_subsystem, 0.0);
        goToPosition = new GoToPosition(m_subsystem);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new Trigger((() -> true)).whileTrue(angleToPosition);
    new Trigger((() -> true)).whileTrue(goToPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
