// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Field3d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.ExampleSubsystem;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.InputDevices.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.InputDevices.SECONDARY_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  @Log
  private final DrivebaseS drivebaseS = new DrivebaseS();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ClampSubsystem m_clampSubsystem = new ClampSubsystem(Constants.PNEUMATIC_CLAMP_EXTEND_PORT);

  // Controllers
  private final CommandXboxController m_primaryController = new CommandXboxController(PRIMARY_CONTROLLER_PORT);
  private final CommandXboxController m_secondaryController = new CommandXboxController(SECONDARY_CONTROLLER_PORT);

  @Log
  private final Field2d field = new Field2d();
  @Log
  private final Field3d field3d = new Field3d();
  private final FieldObject2d target = field.getObject("target");

  @Log
  SendableChooser<Command> autoSelector = new SendableChooser<Command>();

  PathPlannerTrajectory pathPlannerTrajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    target.setPose(new Pose2d(0, 0, new Rotation2d()));


    drivebaseS.setDefaultCommand(
            new OperatorControlC(
                    m_primaryController::getLeftY,
                    m_primaryController::getLeftX,
                    m_primaryController::getRightX,
                    drivebaseS
            )
    );
    // Configure the button bindings
    configureBindings();

    autoSelector.setDefaultOption("pathPlanner", new InstantCommand());
    autoSelector.addOption("1 Meter Forward",
            drivebaseS.pathPlannerCommand(
                    DrivebaseS.generateTrajectoryToPose(
                            drivebaseS.getPose(),
                            drivebaseS.getPose().plus(new Transform2d(new Translation2d(1.0, 0.0), drivebaseS.getPoseHeading())),
                            drivebaseS.getFieldRelativeLinearSpeedsMPS()
                    )
            )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Primary Controller
    new Trigger(RobotController::getUserButton).onTrue(runOnce(()->drivebaseS.resetPose(new Pose2d())));
    m_primaryController.povCenter().onFalse(
            runOnce(
                    ()->drivebaseS.setRotationState(
                            Units.degreesToRadians(m_primaryController.getHID().getPOV()))
            )
    );
    m_primaryController.a().toggleOnTrue(drivebaseS.chasePoseC(target::getPose));


    // Secondary Controller
    m_secondaryController.a().toggleOnTrue(new StartEndCommand(
      // Extends the clamp
      () -> m_clampSubsystem.extend(),
      // Retracts the clamp
      () -> m_clampSubsystem.retract(),
      // Requires the clamp subsystem
      m_clampSubsystem
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoSelector.getSelected(); // Autos.exampleAuto(m_exampleSubsystem);
  }

  public void periodic() {
    drivebaseS.drawRobotOnField(field);
    field3d.setRobotPose(new Pose3d(drivebaseS.getPose()));
  }

  public void onEnabled() {
    drivebaseS.resetRelativeRotationEncoders();
  }
}
