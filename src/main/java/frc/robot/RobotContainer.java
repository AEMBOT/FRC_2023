// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.InputDevices.*;
import static frc.robot.Constants.VisionConstants.*;

import java.sql.Driver;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.GetHomeCommand;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.commands.docking.AutoPathDocking;
import frc.robot.commands.docking.Docking;
import frc.robot.commands.docking.DockingForceBalance;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.commands.scoring.DriverAssist;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystem.TargetGrid;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.LedConstants.*;
import static frc.robot.Constants.VisionConstants.APRILTAG_LAYOUT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    @Log(methodName="getTotalCurrent")
    private PowerDistribution power = new PowerDistribution();

    // Subsystems
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(new Limelight[]{new Limelight("limelight")});
    private final Limelight m_limelight = new Limelight();
    private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
    @Log
    private final DrivebaseS drivebaseS = new DrivebaseS(m_limelight);

    //Commands
    private final Docking m_docking = new Docking(drivebaseS, m_limelight);
    private final AutoPathDocking m_newDocking = new AutoPathDocking(drivebaseS, m_limelight);
    private final DockingForceBalance m_dockingForceBalance = new DockingForceBalance(drivebaseS);
    private final GetHomeCommand m_GetHomeCommand = new GetHomeCommand(m_armSubsystem);
    private final GoToPosition m_GoToPositionPickUp = new GoToPosition(m_armSubsystem, 0, angleToSubstation);
    private final GoToPosition m_GoToPositionMid = new GoToPosition(m_armSubsystem, extendToMid, angleToDelivery);
    private final GoToPosition m_GoToPositionHigh = new GoToPosition(m_armSubsystem, extendToHigh, angleToDelivery);
    private final GoToPosition m_GoToPositionTest = new GoToPosition(m_armSubsystem, 1, 0);
    private DriverAssist m_driverAssist = new DriverAssist(m_limelight, drivebaseS, m_armSubsystem,TargetGrid.NONE);


    // Controllers
    private final CommandXboxController m_primaryController = new CommandXboxController(PRIMARY_CONTROLLER_PORT);
    private final CommandXboxController m_secondaryController = new CommandXboxController(SECONDARY_CONTROLLER_PORT);
    private final CommandGenericHID m_numpad = new CommandGenericHID(NUMPAD_CONTROLLER_PORT);

    // Path Planner Trajectories
    private final PathPlannerTrajectory twoPiecePath = PathPlanner.loadPath("twopiece", 1.0, 0.5);

    // Path Planner Built Autos
    private final SwerveAutoBuilder autoBuilder = drivebaseS.getSwerveAutoBuilder();
    private final Command redLeft_blueRight = autoBuilder.fullAuto(
            PathPlanner.loadPath("redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
    );
    private final Command redRight_blueLeft = autoBuilder.fullAuto(
            PathPlanner.loadPath("redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
    );
    private final Command leave_redLeft_blueRight = autoBuilder.fullAuto(
            PathPlanner.loadPath("leave-redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
    );
    private final Command leave_redRight_blueLeft = autoBuilder.fullAuto(
            PathPlanner.loadPath("leave-redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
    );
    private final Command twopiece_redLeft_blueRight = autoBuilder.fullAuto(
            PathPlanner.loadPath("twopiece-redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
    );
    private final Command twopiece_redRight_blueLeft = autoBuilder.fullAuto(
            PathPlanner.loadPath("twopiece-redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
    );

    @Log
    private final Field2d field = new Field2d();
    @Log
    private final Field3d field3d = new Field3d();
    private final FieldObject2d target = field.getObject("target");

    @Log
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();
    GenericEntry pathDelay = Shuffleboard.getTab("Auto").add("Path Delay Time", 2.0).getEntry();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        target.setPose(new Pose2d(0, 0, new Rotation2d()));

        // Subsystem Default Commands
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

        // Build Auto Event Map
        eventMap.put("placeConeHigh",
                new SequentialCommandGroup(
                        m_GoToPositionHigh,
                        new InstantCommand(m_armSubsystem::extendClamp)
                )
        );
        eventMap.put("floorPickup",
                new SequentialCommandGroup(
                        new InstantCommand(m_armSubsystem::extendClamp),
                        m_GoToPositionPickUp,
                        new InstantCommand(m_armSubsystem::retractClamp)
                )
        );
        eventMap.put("autoDock", m_newDocking);

        // Build Autos
        autoSelector.setDefaultOption("No-op", new InstantCommand());
        autoSelector.addOption("Leave Immediately",
                new ProxyCommand(
                        () -> drivebaseS.pathPlannerCommand(
                                DrivebaseS.generateTrajectoryToPose(
                                        drivebaseS.getPose(),
                                        drivebaseS.getPose().plus(ONE_METER_BACK.times(5.0)),
                                        drivebaseS.getFieldRelativeLinearSpeedsMPS()
                                )
                        )
                )
        );

        autoSelector.addOption("redLeft-blueRight", redLeft_blueRight);
        autoSelector.addOption("redRight-blueLeft", redRight_blueLeft);
        autoSelector.addOption("leave-redLeft-blueRight", leave_redLeft_blueRight);
        autoSelector.addOption("leave-redRight-blueLeft", leave_redRight_blueLeft);
        autoSelector.addOption("twopiece-redLeft-blueRight", twopiece_redLeft_blueRight);
        autoSelector.addOption("twopiece-redRight-blueLeft", twopiece_redRight_blueLeft);

        autoSelector.addOption("twopiece",
                new SequentialCommandGroup(
//                        new InstantCommand(m_armSubsystem::extendClamp),
//                        m_GoToPositionHigh,
//                        new InstantCommand(m_armSubsystem::retractClamp),
                        new ParallelCommandGroup(
//                                new GoToPosition(m_armSubsystem, 0, -0.5),
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> drivebaseS.resetPose(twoPiecePath.getInitialHolonomicPose())),
                                        drivebaseS.pathPlannerCommand(twoPiecePath)
                                        //m_newDocking
                                )
                        )
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
                drivebaseS.chasePoseC(
                        () -> APRILTAG_LAYOUT.getTagPose(3).get().toPose2d().plus(new Transform2d(new Translation2d(2.77, 2.5), new Rotation2d(Math.PI)))
                )
        );
        field.getObject("target").setPose(APRILTAG_LAYOUT.getTagPose(3).get().toPose2d().plus(new Transform2d(new Translation2d(2.77, 2.5), new Rotation2d(Math.PI))));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        // Primary Controller
        new Trigger(RobotController::getUserButton).onTrue(runOnce(() -> drivebaseS.resetPose(new Pose2d())));
        m_primaryController.povCenter().onFalse(
                runOnce(
                        () -> drivebaseS.setRotationState(
                                Units.degreesToRadians(m_primaryController.getHID().getPOV()))
                ));

        m_primaryController.back().whileTrue(drivebaseS.chasePoseC(
                () -> DOUBLE_SUBSTATION.plus(DOUBLE_SUBSTATION_OFFSET_LEFT)
        ));

        m_primaryController.start().whileTrue(drivebaseS.chasePoseC(
                () -> DOUBLE_SUBSTATION.plus(DOUBLE_SUBSTATION_OFFSET_RIGHT)
        ));

        m_numpad.button(-1).onTrue(
                //m_driverAssist.setTargetGrid(TargetGrid.INNER)
                Commands.runOnce(() -> m_armSubsystem.setTargetGrid(TargetGrid.INNER), m_armSubsystem)
        );
        m_numpad.button(-2).onTrue(
                Commands.runOnce(() -> m_armSubsystem.setTargetGrid(TargetGrid.COOP), m_armSubsystem)
        );
        m_numpad.button(-3).onTrue(
                Commands.runOnce(() -> m_armSubsystem.setTargetGrid(TargetGrid.OUTER), m_armSubsystem)
        );
        
        

        m_numpad.button(1).whileTrue(
                new SequentialCommandGroup(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(CONE_OFFSET_LEFT).plus(ONE_METER_BACK.times(0.5))),
                m_GoToPositionPickUp
                ), Commands.runOnce(() -> m_armSubsystem.setTargetGrid(TargetGrid.NONE), m_armSubsystem)
        ));
        m_numpad.button(2).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(ONE_METER_BACK.times(0.5))),
                new GoToPosition(m_armSubsystem, 0, 0)
                )
        );

        m_numpad.button(3).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(CONE_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5))
                ), m_GoToPositionPickUp
        ));

        m_numpad.button(4).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(CONE_OFFSET_LEFT).plus(ONE_METER_BACK.times(0.5))),
                m_GoToPositionMid
                
         ));

        m_numpad.button(5).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(ONE_METER_BACK.times(0.5))),
                m_GoToPositionMid
                )
        );

        m_numpad.button(6).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(CONE_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5))
                ), m_GoToPositionMid
        ));

        m_numpad.button(7).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(CONE_OFFSET_LEFT).plus(ONE_METER_BACK.times(0.5))),
                m_GoToPositionHigh
                )
        );

        m_numpad.button(8).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(ONE_METER_BACK.times(0.5))),
                m_GoToPositionHigh
                )
        );

        m_numpad.button(9).whileTrue(
                new ParallelCommandGroup(
                drivebaseS.chasePoseC(
                        () -> m_driverAssist.goToStation(m_armSubsystem.getTargetGrid()).plus(CONE_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5))
                ), m_GoToPositionHigh
        ));

        m_numpad.button(10).whileTrue(
                new SequentialCommandGroup(
                new ParallelCommandGroup(
                        m_GoToPositionHigh, new InstantCommand(m_armSubsystem::toggleClamp, m_armSubsystem)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(m_armSubsystem::toggleClamp, m_armSubsystem), 
                        new GoToPosition(m_armSubsystem, 0,0)) //higher than go to position high, same extension
                ));
        

        //m_primaryController.a().toggleOnTrue(drivebaseS.chasePoseC(target::getPose));


        // Secondary Controller
        // Clamp

        m_secondaryController.a().toggleOnTrue(new InstantCommand(
                // Toggles the clamp
                m_armSubsystem::toggleClamp,
                // Requires the Arm subsystem
                m_armSubsystem
        ));

        // Elevator
        // Angle Motor
        m_secondaryController.leftBumper().whileTrue(
                new RunCommand(m_armSubsystem::angleDown).finallyDo((interrupted) -> m_armSubsystem.stopAngle())
        );

        m_secondaryController.rightBumper().whileTrue(
                new RunCommand(m_armSubsystem::angleUp).finallyDo((interrupted) -> m_armSubsystem.stopAngle())
        );

        // Extend Motor
        m_secondaryController.leftTrigger().whileTrue(
                new RunCommand(m_armSubsystem::retractArm).finallyDo((interrupted) -> m_armSubsystem.stopExtend())
        );

        m_secondaryController.rightTrigger().whileTrue(
                new RunCommand(m_armSubsystem::extendArm).finallyDo((interrupted) -> m_armSubsystem.stopExtend())
        );

        // Elevator go to Position\
        //y will be replaced with numpad buttons 
        m_secondaryController.y().whileTrue(m_GoToPositionTest.andThen(new InstantCommand(m_armSubsystem::extendClamp)));
        //Docking
        m_secondaryController.b().whileTrue(m_newDocking);

        m_primaryController.a().whileTrue(m_dockingForceBalance);

        m_secondaryController.x().whileTrue(new RunCommand(visionSubsystem.limelights[0]::test, visionSubsystem.limelights[0]));

        m_secondaryController.start().onTrue(runOnce(() -> m_LedSubsystem.setColor(colorYellow), m_LedSubsystem));
        m_secondaryController.back().onTrue(runOnce(() -> m_LedSubsystem.setColor(colorPurple), m_LedSubsystem));
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

    public void ClawStateMachine() {
        /*
         * Be able to have the driver override Ultrasonic stuck on or off
         * DPad Up - Disengage auto claw actuation, close claw
         * DPad Down - Disengage auto claw actuation, open claw
         * DPad Left/Right - Engage auto claw actuation.
         *
         * int getMedian(arr) { //Assuming an array of 5 datapoints
         *      Arrays.sort(arr);
         *      return arr[2];
         * }
         *
         *
         *
         */
        // Needs to
    }

  public void periodic() {
    SmartDashboard.putString("Alliance", ALLIANCE.toString());
    drivebaseS.drawRobotOnField(field);
    field3d.setRobotPose(new Pose3d(drivebaseS.getPose()));
    //Put LED stuff here

  }

    public void onEnabled() {
        CommandScheduler.getInstance().schedule(new GetHomeCommand(m_armSubsystem));
        ALLIANCE = DriverStation.getAlliance();
    }

    public void onInit() {
        drivebaseS.resetRelativeRotationEncoders();
    }
}
