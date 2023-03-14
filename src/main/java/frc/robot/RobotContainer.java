// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.InputDevices.*;
import static frc.robot.Constants.VisionConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.GetHomeCommand;
import frc.robot.commands.arm.GoToPosition;
import frc.robot.commands.docking.AutoPathDocking;
import frc.robot.commands.docking.DockingForceBalance;
import frc.robot.commands.drivetrain.OperatorControlC;
import frc.robot.commands.drivetrain.OperatorControlHoldingC;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.VisionConstants.APRILTAG_LAYOUT;
import static frc.robot.commands.arm.ArmCommands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    @Log(methodName = "getTotalCurrent")
    private PowerDistribution power = new PowerDistribution();
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private SerialPort serial;

    // Subsystems
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final Limelight m_limelight = new Limelight();
    @Log
    private final DrivebaseS drivebaseS = new DrivebaseS(m_limelight);

    //Commands
    private final AutoPathDocking m_newDocking = new AutoPathDocking(drivebaseS);
    private final DockingForceBalance m_teleopDocking = new DockingForceBalance(drivebaseS);
    private final GetHomeCommand m_GetHomeCommand = new GetHomeCommand(m_armSubsystem);
    private final GoToPosition m_GoToPositionTest = new GoToPosition(m_armSubsystem, 1, 0);


    // Controllers
    private final CommandXboxController m_primaryController = new CommandXboxController(PRIMARY_CONTROLLER_PORT);
    private final CommandXboxController m_secondaryController = new CommandXboxController(SECONDARY_CONTROLLER_PORT);
    private final CommandGenericHID m_numpad = new CommandGenericHID(NUMPAD_CONTROLLER_PORT);

    // Path Planner Trajectories
    private final PathPlannerTrajectory twoPiecePath = PathPlanner.loadPath("twopiece", 1.0, 0.5);

    @Log
    private final Field2d field = new Field2d();
    @Log
    private final Field3d field3d = new Field3d();
    private final FieldObject2d target = field.getObject("target");

    @Log
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();
    GenericEntry pathDelay = Shuffleboard.getTab("Auto").add("Path Delay Time", 2.0).getEntry();

    // Driver Controls
    @Log
    private int lastPressedNumpad = -1;
    @Log(methodName = "toString")
    private TargetPosition targetPosition = TargetPosition.NONE;

    // Arbitrary Subsystem Triggers
    private Trigger limitSwitchTrigger = new Trigger(m_intakeSubsystem::getLimitSwitchState);
    private Trigger endgameLEDTrigger = new Trigger(DriverStation::isTeleopEnabled).and(new Trigger(() -> DriverStation.getMatchTime() < 30));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        target.setPose(new Pose2d(0, 0, new Rotation2d()));
        compressor.enableDigital();
        serial = new SerialPort(115200, SerialPort.Port.kUSB);

        // Subsystem Default Commands
        drivebaseS.setDefaultCommand(
                new OperatorControlC(
                        m_primaryController::getLeftY,
                        m_primaryController::getLeftX,
                        m_primaryController::getRightX,
                        false,
                        drivebaseS
                )
        );
        // Configure the button bindings
        configureBindings();

        // Build Auto Event Map
        eventMap.put("placeConeHigh",
                new SequentialCommandGroup(
                        m_armSubsystem.getGoToPositionCommand(extendToHigh, angleToHigh).withTimeout(3),
                        new InstantCommand(m_intakeSubsystem::openClamp)
                )
        );
        eventMap.put("floorPickup",
                new SequentialCommandGroup(
                        new InstantCommand(m_intakeSubsystem::openClamp),
                        m_armSubsystem.getGoToPositionCommand(extendToGroundPickup, angletoFloorPickUp).withTimeout(3),
                        new InstantCommand(m_intakeSubsystem::closeClamp)
                )
        );
        eventMap.put("openIntake", new InstantCommand(m_intakeSubsystem::openClamp, m_intakeSubsystem));
        eventMap.put("closeIntake", new InstantCommand(m_intakeSubsystem::closeClamp, m_intakeSubsystem));
        eventMap.put("goToFloorPickup", m_armSubsystem.getGoToPositionCommand(extendToGroundPickup, angletoFloorPickUp));
        eventMap.put("goToHighAngle", m_armSubsystem.getGoToPositionCommand(minExtendHardStop, angleToHigh));
        eventMap.put("goToHighPlace", m_armSubsystem.getGoToPositionCommand(extendToHigh, angleToHigh));
        eventMap.put("autoDock", m_newDocking);
        eventMap.put("stowArm", m_armSubsystem.getGoToPositionCommand(minExtendHardStop, maxAngleHardStop));

        // Construct Autos
        // Path Planner Built Autos
        SwerveAutoBuilder autoBuilder = drivebaseS.getSwerveAutoBuilder();
/* 
        Command redLeft_blueRight = new SequentialCommandGroup(
                autoBuilder.fullAuto(
                        PathPlanner.loadPath("redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
                ).withTimeout(9.75),
                new AutoPathDocking(drivebaseS)
        );

        Command redRight_blueLeft = new SequentialCommandGroup(
                autoBuilder.fullAuto(
                        PathPlanner.loadPath("redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
                ).withTimeout(10.0),
                new AutoPathDocking(drivebaseS)
        );*/

        Command redLeft_blueRight = autoBuilder.fullAuto(
                PathPlanner.loadPath("redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command redRight_blueLeft = autoBuilder.fullAuto(
                PathPlanner.loadPath("redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command leave_redLeft_blueRight = autoBuilder.fullAuto(
                PathPlanner.loadPath("leave-redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command leave_redRight_blueLeft = autoBuilder.fullAuto(
                PathPlanner.loadPath("leave-redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command twopiece_redLeft_blueRight = autoBuilder.fullAuto(
                PathPlanner.loadPath("twopiece-redLeft-blueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command twopiece_redRight_blueLeft = autoBuilder.fullAuto(
                PathPlanner.loadPath("twopiece-redRight-blueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command basicRedLeft_blueRight = autoBuilder.fullAuto(
                PathPlanner.loadPath("BasicRedLeft-BlueRight", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        Command basicRedRight_blueLeft = autoBuilder.fullAuto(
                PathPlanner.loadPath("BasicRedRight-BlueLeft", maxVelMetersPerSec, maxAccelMetersPerSecondSq)
        );

        // Build Autos
        autoSelector.setDefaultOption("No-op", new InstantCommand());
        autoSelector.addOption("Leave Immediately",
                new ProxyCommand(
                        () -> drivebaseS.pathPlannerCommand(
                                DrivebaseS.generateTrajectoryToPose(
                                        drivebaseS.getPose(),
                                        drivebaseS.getPose().plus(ONE_METER_BACK.times(5.0)),
                                        drivebaseS.getFieldRelativeLinearSpeedsMPS(),
                                        1, 0.3
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
        autoSelector.addOption("BasicRedLeft-BlueRight", basicRedLeft_blueRight);
        autoSelector.addOption("BasicRedRight-BlueLeft", basicRedRight_blueLeft);

        autoSelector.addOption("twopiece",
                new SequentialCommandGroup(
//                        new InstantCommand(m_armSubsystem::openClamp),
//                        m_GoToPositionHigh,
//                        new InstantCommand(m_armSubsystem::closeClamp),
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
        field.getObject("target").setPose(GRID_LEFT.plus(CONE_OFFSET_RIGHT).plus(ONE_METER_BACK.times(0.5)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        // Subsystem State Machines
        limitSwitchTrigger.onTrue(new InstantCommand(() -> serial.writeString("g")).ignoringDisable(true));
        endgameLEDTrigger.onTrue(new InstantCommand(() -> serial.writeString("2")));
        // Primary Controller
        new Trigger(RobotController::getUserButton).onTrue(runOnce(() -> drivebaseS.resetPose(new Pose2d())));

        m_primaryController.rightBumper().whileTrue(
                new OperatorControlHoldingC(
                        m_primaryController::getLeftY,
                        m_primaryController::getLeftX,
                        m_primaryController::getRightX,
                        drivebaseS
                )
        );

        m_primaryController.back().whileTrue(drivebaseS.chasePoseC(
                () -> DOUBLE_SUBSTATION.plus(DOUBLE_SUBSTATION_OFFSET_LEFT)
        ));

        m_primaryController.start().whileTrue(drivebaseS.chasePoseC(
                () -> DOUBLE_SUBSTATION.plus(DOUBLE_SUBSTATION_OFFSET_RIGHT)
        ));
/* 
        m_numpad.button(20).whileTrue(new ProxyCommand(
                () -> HighPiecePickUpCommand(drivebaseS, m_armSubsystem, lastPressedNumpad)
        ));*/

        m_primaryController.a().whileTrue(
                new ProxyCommand(
                        () -> {
                            if (lastPressedNumpad == 10 || lastPressedNumpad == 11) {
                                return getHighPiecePickUpCommand(drivebaseS, m_armSubsystem, m_intakeSubsystem, targetPosition, lastPressedNumpad);
                            } else {
                                return getPlaceGamePieceCommand(drivebaseS, m_armSubsystem, targetPosition, lastPressedNumpad);
                            }
                        }
                )
        );

        m_numpad.button(15).onTrue(new InstantCommand(
                // Toggles the clamp
                m_intakeSubsystem::toggleClamp,
                // Requires the Arm subsystem
                m_intakeSubsystem
        ));

        m_numpad.button(10).onTrue(
                Commands.runOnce(() -> {
                    targetPosition = TargetPosition.DOUBLE_SUBSTATION;
                    lastPressedNumpad = 10;
                })
        );

        m_numpad.button(11).onTrue(
                Commands.runOnce(() -> {
                    targetPosition = TargetPosition.DOUBLE_SUBSTATION;
                    lastPressedNumpad = 11;
                })
        );

        //single substation
        /* 
        m_numpad.button(21).onTrue(
                Commands.runOnce(() -> {
                        lastPressedNumpad = 21;
                })
        );*/

        m_numpad.button(12).onTrue(
                //m_driverAssist.setTargetGrid(TargetGrid.INNER)
                Commands.runOnce(() -> targetPosition = TargetPosition.LEFT_GRID)
        );
        m_numpad.button(13).onTrue(
                Commands.runOnce(() -> targetPosition = TargetPosition.COOP_GRID)
        );
        m_numpad.button(14).onTrue(
                Commands.runOnce(() -> targetPosition = TargetPosition.RIGHT_GRID)
        );

        m_numpad.button(16).whileTrue(
                m_armSubsystem.getGoToPositionCommand(minExtendHardStop, maxAngleHardStop)
        );

        m_numpad.button(17).whileTrue(
//                m_armSubsystem.getGoToPositionCommand(minExtendHardStop, startingConfigurationAngle)
                new ProxyCommand(
                        () -> getArmExtensionCommand(m_armSubsystem, lastPressedNumpad)
                )
        );

        m_numpad.button(18).whileTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> lastPressedNumpad = 18),
                        new ProxyCommand(
                                () -> getPickUpPieceFromGround(drivebaseS, m_armSubsystem, m_intakeSubsystem, lastPressedNumpad)
                        ))
        );
        m_numpad.button(1).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 1)
        );

        m_numpad.button(2).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 2)
        );

        m_numpad.button(3).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 3)
        );

        m_numpad.button(4).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 4)
        );

        m_numpad.button(5).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 5)
        );

        m_numpad.button(6).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 6)
        );

        m_numpad.button(7).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 7)
        );

        m_numpad.button(8).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 8)
        );

        m_numpad.button(9).whileTrue(
                new InstantCommand(() -> lastPressedNumpad = 9)
        );

        m_numpad.button(19).whileTrue(
                new InstantCommand(() -> serial.writeString("y")).ignoringDisable(true)
        );

        m_numpad.button(20).whileTrue(
                new InstantCommand(() -> serial.writeString("p")).ignoringDisable(true)
        );
        // Secondary Controller
        // Clamp

        m_secondaryController.a().toggleOnTrue(new InstantCommand(
                // Toggles the clamp
                m_intakeSubsystem::toggleClamp,
                // Requires the Arm subsystem
                m_intakeSubsystem
        ));

        // Elevator
        // Angle Motor
        m_secondaryController.rightBumper().whileTrue(
                new RunCommand(m_armSubsystem::angleDown).finallyDo((interrupted) -> m_armSubsystem.stopAngle())
        );

        m_secondaryController.leftBumper().whileTrue(
                new RunCommand(m_armSubsystem::angleUp).finallyDo((interrupted) -> m_armSubsystem.stopAngle())
        );

        // Extend Motor
//        m_secondaryController.leftTrigger(TRIGGER_DEADBAND).whileTrue(
//                new RunCommand(() -> m_armSubsystem.retractArm(
//                        applyDeadband(m_secondaryController.getLeftTriggerAxis(), TRIGGER_DEADBAND)
//                ))
//        );
//
//        m_secondaryController.rightTrigger(TRIGGER_DEADBAND).whileTrue(
//                new RunCommand(() -> m_armSubsystem.extendArm(
//                        applyDeadband(m_secondaryController.getRightTriggerAxis(), TRIGGER_DEADBAND)
//                ))
//        );
//
//        m_secondaryController.leftTrigger(TRIGGER_DEADBAND).or(m_secondaryController.rightTrigger(TRIGGER_DEADBAND)).whileFalse(
//                new RunCommand(m_armSubsystem::stopExtend)
//        );

        m_secondaryController.leftTrigger().whileTrue(
                new RunCommand(m_armSubsystem::retractArm).finallyDo((interrupted) -> m_armSubsystem.stopExtend())
        );

        m_secondaryController.rightTrigger().whileTrue(
                new RunCommand(m_armSubsystem::extendArm).finallyDo((interrupted) -> m_armSubsystem.stopExtend())
        );

        // Elevator go to Position\
        //y will be replaced with numpad buttons 
        m_secondaryController.y().whileTrue(m_GoToPositionTest.andThen(new InstantCommand(m_intakeSubsystem::openClamp)));
        //Docking
        m_primaryController.b().whileTrue(m_teleopDocking);

        // slow mode for driver
        m_primaryController.leftBumper().whileTrue(
                new OperatorControlC(
                        m_primaryController::getLeftY,
                        m_primaryController::getLeftX,
                        m_primaryController::getRightX,
                        true,
                        drivebaseS
                )
        );
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

        SmartDashboard.putNumber("Pitch", drivebaseS.getPitch());
        SmartDashboard.putNumber("Roll", drivebaseS.getRoll());
        SmartDashboard.putNumber("Yaw", drivebaseS.getHeading().getRadians());
    }

    public void onDisabled() {
    }

    public void onEnabled() {
        CommandScheduler.getInstance().schedule(new InstantCommand(m_armSubsystem::resetExtendEncoder));
        CommandScheduler.getInstance().schedule(new GetHomeCommand(m_armSubsystem));
        ALLIANCE = DriverStation.getAlliance();
    }

    public void onInit() {
        drivebaseS.resetRelativeRotationEncoders();
        if (ALLIANCE == Alliance.Red) {
                serial.writeString("r");
        } else if (ALLIANCE == Alliance.Blue) {
                serial.writeString("b");
        } else {
                serial.writeString("o");
        }
    }
}
