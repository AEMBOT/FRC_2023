// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.AutoConstants.ALLIANCE;
import static frc.robot.Constants.InputDevices.PRIMARY_CONTROLLER_PORT;
import static frc.robot.Constants.InputDevices.SECONDARY_CONTROLLER_PORT;
import static frc.robot.Constants.ArmConstants.angleToFloor;
import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.GetHomeCommand;
import frc.robot.commands.docking.Docking;
import frc.robot.commands.docking.DockingForceBalance;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.commands.arm.AngleToPosition;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.ExampleSubsystem;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(new Limelight[]{new Limelight("limelight")});
  private final Limelight m_limelight = new Limelight();
  @Log
  private final DrivebaseS drivebaseS = new DrivebaseS(m_limelight);

  //Commands
  private Docking m_docking = new Docking(drivebaseS, m_limelight);
  private DockingForceBalance m_dockingForceBalance = new DockingForceBalance(drivebaseS);
  private GetHomeCommand m_GetHomeCommand = new GetHomeCommand(m_armSubsystem);
  private GoToPosition m_GoToPosition = new GoToPosition(m_armSubsystem);
  private AngleToPosition m_AngleToPosition = new AngleToPosition(m_armSubsystem, angleToFloor);
  //private AngleToPosition m_AngleToPositionFloor = new AngleToPosition(m_armSubsystem, angleToSubstation);

  // Controllers
  private final CommandXboxController m_primaryController = new CommandXboxController(PRIMARY_CONTROLLER_PORT);
  private final CommandXboxController m_secondaryController = new CommandXboxController(SECONDARY_CONTROLLER_PORT);

  @Log
  private final Field2d field = new Field2d();
  @Log
  private final Field3d field3d = new Field3d();
  private final FieldObject2d target = field.getObject("target");
  private final AprilTagFieldLayout APRILTAG_LAYOUT;

  @Log
  SendableChooser<Command> autoSelector = new SendableChooser<Command>();

  PathPlannerTrajectory pathPlannerTrajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (ALLIANCE == DriverStation.Alliance.Red) {
      APRILTAG_LAYOUT = TAG_FIELD_LAYOUT;
      APRILTAG_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    } else {
      APRILTAG_LAYOUT = TAG_FIELD_LAYOUT;
    }
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
    pathPlannerTrajectory = PathPlanner.loadPath("twopiece", 1.0, 0.5);

    autoSelector.addOption("twopiece",
            new SequentialCommandGroup(
                    new InstantCommand(() -> drivebaseS.resetPose(pathPlannerTrajectory.getInitialHolonomicPose())),
                    drivebaseS.pathPlannerCommand(pathPlannerTrajectory)
            )
    );
//    autoSelector.addOption("apriltag",
//            drivebaseS.pathPlannerCommand(
//                    DrivebaseS.generateTrajectoryToPose(
//                            drivebaseS.getPose(),
//                            TAG_FIELD_LAYOUT.getTagPose(3).get().toPose2d().exp(new Twist2d(-1.0, 0.0, 0.0)),
//                            drivebaseS.getFieldRelativeLinearSpeedsMPS(),
//                            1,
//                            0.5
//                    )
//            )
//    );
    autoSelector.addOption("apriltag",
            drivebaseS.chasePoseC(() -> APRILTAG_LAYOUT.getTagPose(3).get().toPose2d().exp(new Twist2d(-Units.inchesToMeters(20.25), 0.0, 0.0))  ));
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
            ));

    //m_primaryController.a().toggleOnTrue(drivebaseS.chasePoseC(target::getPose));


    // Secondary Controller
    // Clamp
    m_secondaryController.a().toggleOnTrue(new StartEndCommand(
      // Extends the clamp
            m_armSubsystem::extendClamp,
      // Retracts the clamp
            m_armSubsystem::retractClamp,
      // Requires the Arm subsystem
      m_armSubsystem
    ));

    // Elevator
    //
    // Angle Motor
    m_secondaryController.rightBumper().onFalse(new RunCommand(
            m_armSubsystem::stopAngle,
      m_armSubsystem));

    m_secondaryController.leftBumper().onFalse(new RunCommand(
            m_armSubsystem::stopAngle,
      m_armSubsystem));

    m_secondaryController.rightBumper().whileTrue(new RunCommand(
            m_armSubsystem::angleUp,
      m_armSubsystem));

    m_secondaryController.leftBumper().whileTrue(new RunCommand(
            m_armSubsystem::angleDown,
      m_armSubsystem));
    //
    // Extend Motor
    m_secondaryController.rightTrigger().onFalse(new RunCommand(
            m_armSubsystem::stopExtend,
      m_armSubsystem));
    
    m_secondaryController.leftTrigger().onFalse(new RunCommand(
            m_armSubsystem::stopExtend,
      m_armSubsystem));

    m_secondaryController.rightTrigger().whileTrue(new RunCommand(
            m_armSubsystem::extendArm,
      m_armSubsystem));

    m_secondaryController.leftTrigger().whileTrue(new RunCommand(
            m_armSubsystem::retractArm,
      m_armSubsystem));

    // Elevator go to Position
    m_secondaryController.y().whileTrue(m_GoToPosition);
      //Fix this to incorporate different precise angle positions, only has one inaccurate angle at the moment
    m_secondaryController.a().onTrue(m_AngleToPosition);
    //m_secondaryController.a().WhileTrue(m_AngleToPositionFloor);

    //Docking
    m_secondaryController.b().whileTrue(m_docking);

    m_primaryController.a().whileTrue(m_dockingForceBalance);

    m_secondaryController.x().whileTrue(new RunCommand(visionSubsystem.limelights[0]::test, visionSubsystem.limelights[0]));

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
    SmartDashboard.putString("Alliance", ALLIANCE.toString());
    drivebaseS.drawRobotOnField(field);
    field3d.setRobotPose(new Pose3d(drivebaseS.getPose()));
  }

  public void onEnabled() {
    CommandScheduler.getInstance().schedule(m_GetHomeCommand);
    m_armSubsystem.stopAngle();
    m_armSubsystem.stopExtend();
  }

  public void onInit() {
    drivebaseS.resetRelativeRotationEncoders();
  }
}
