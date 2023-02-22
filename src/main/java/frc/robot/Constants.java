// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseS;

import java.util.HashMap;
import java.util.List;

import static frc.robot.Constants.AutoConstants.ALLIANCE;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double LIMELIGHT_HEIGHT = 0.0; // Height of limelight above ground (in m)

    public static final class InputDevices {

        public static final int PRIMARY_CONTROLLER_PORT = 0;
        public static final int SECONDARY_CONTROLLER_PORT = 1;

    }

    public static final class DriveConstants {

        static public final double WHEEL_BASE_WIDTH_M = Units.inchesToMeters(25);
        static public final double WHEEL_RADIUS_M = 0.0508; //Units.inchesToMeters(4.0/2.0); //four inch (diameter) wheels
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(20.0);
        static public final double ROBOT_MOI_KGM2 = 1.0 / 12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_WIDTH_M * 1.1), 2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(12);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Math.PI * 8;
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS / 0.125; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25; //0-full time of 0.25 second

        // HELPER ORGANIZATION CONSTANTS
        static public final int FL = 0; // Front Left Module Index
        static public final int FR = 1; // Front Right Module Index
        static public final int BL = 2; // Back Left Module Index
        static public final int BR = 3; // Back Right Module Index
        static public final int NUM_MODULES = 4;

        // Internal objects used to track where the modules are at relative to
        // the center of the robot, and all the implications that spacing has.
        static private final double HW = WHEEL_BASE_WIDTH_M / 2.0;

        public enum ModuleConstants {
            FL("FL", 9, 2, 12, 2.351588, HW, HW),
            FR("FR", 3, 4, 10, 2.109219, HW, -HW),
            BL("BL", 5, 6, 13, 0.971008, -HW, HW),
            BR("BR", 7, 8, 11, 1.366774, -HW, -HW);

            public final String name;
            public final int driveMotorID;
            public final int rotationMotorID;
            public final int magEncoderID;
            /**
             * absolute encoder offsets for the wheels
             * 180 degrees added to offset values to invert one side of the robot so that it doesn't spin in place
             */
            public final double magEncoderOffset;
            public final Translation2d centerOffset;

            ModuleConstants(String name, int driveMotorID, int rotationMotorID, int magEncoderID, double magEncoderOffset, double xOffset, double yOffset) {
                this.name = name;
                this.driveMotorID = driveMotorID;
                this.rotationMotorID = rotationMotorID;
                this.magEncoderID = magEncoderID;
                this.magEncoderOffset = magEncoderOffset;
                centerOffset = new Translation2d(xOffset, yOffset);

            }
        }

        static public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                ModuleConstants.FL.centerOffset,
                ModuleConstants.FR.centerOffset,
                ModuleConstants.BL.centerOffset,
                ModuleConstants.BR.centerOffset
        );


        public static final double WHEEL_REVS_PER_ENC_REV = 1.0 / 6.75;
        public static final double AZMTH_REVS_PER_ENC_REV = 1.0 / (150.0 / 7.0);

        public static final double rotationMotorMaxSpeedRadPerSec = 1.0;
        public static final double rotationMotorMaxAccelRadPerSecSq = 1.0;

        //kv: (12 volts * 60 s/min * 1/5.14 WRevs/MRevs * wheel rad * 2pi  / (6000 MRPM *
        /**
         * ks, kv, ka
         */
        //public static final double[] DRIVE_FF = {0.11452, 1.9844, 0.31123};
        public static final double[] DRIVE_FF = {0.055, 2.6826, 0.1188};

        public static final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(DRIVE_FF[0], DRIVE_FF[1], DRIVE_FF[2]);

        public static final double rotationkP = 3;
        //public static final double rotationkD = 0.05 / 2.5;
        public static final double rotationkD = 0;

        //public static final double drivekP = 4.6; // 0.06 w/measurement delay?
        public static final double drivekP = 3;


        public static final double MAX_MODULE_SPEED_FPS = Units.feetToMeters(12);
        public static final double MAX_TELEOP_TURN_RATE = Math.PI * 4; //Rate the robot will spin with full rotation command

        public static final int ENC_PULSE_PER_REV = 1;
        public static final double WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV / WHEEL_REVS_PER_ENC_REV;  //Assume 1-1 gearing for now
        public static final double AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV / AZMTH_REVS_PER_ENC_REV; //Assume 1-1 gearing for now
        public static final double WHEEL_ENC_WHEEL_REVS_PER_COUNT = 1.0 / WHEEL_ENC_COUNTS_PER_WHEEL_REV;
        public static final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0 / AZMTH_ENC_COUNTS_PER_MODULE_REV;

        public static final TrapezoidProfile.Constraints X_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
        public static final TrapezoidProfile.Constraints Y_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);

        public static final TrapezoidProfile.Constraints NO_CONSTRAINTS = new TrapezoidProfile.Constraints(Integer.MAX_VALUE, Integer.MAX_VALUE);
        public static final TrapezoidProfile.Constraints THETA_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(4 * Math.PI, 16 * Math.PI);


    }

    public static final class AutoConstants {
        public static DriverStation.Alliance ALLIANCE = DriverStation.getAlliance();

        public static final double maxVelMetersPerSec = 2;
        public static final double maxAccelMetersPerSecondSq = 1;

        public static HashMap<String, Command> eventMap = new HashMap<String, Command>();
    }

    public static final class LedConstants {
        public static final int[] colorRed = {64, 3, 3}; //red
        public static final int[] colorBlue = {0,0,64}; //blue
        public static final int[] colorYellow = {64,32,0}; //yellow
        public static final int[] colorPurple = {64,0,64}; //purple
        //These show up as rainbow... 
        public static final int[] color = {64,36,0}; //orange
        public static final int[] colorGreen = {0,64,0}; //green    
    }
    
    
    public static final class ArmConstants {

        public static final int angleMotorCanID = 10;
        public static final int angleEncoderPort = 0;
        public static final int angleEncoderOffset = 0;
        public static final int extendMotorCanID = 11;
        public static final int clampSolenoidID = 0;
        public static final int movingAverage = 5;
        public static final int extendMotorCurrentLimit = 35;
        public static final int angleMotorCurrentLimit = 40;

        public static final int ultrasonicPingPort = 4;
        public static final int ultrasonicEchoPort = 3;

        //Angle and Extend arm Constants
        public static final double angleToDelivery = -.48;
        public static final double angleToFloor = -.36;
        public static final double angleToSubstation = -.2;

        public static final double maxAngleHardStop = -0.78;
        public static final double minAngleSoftStop = 0.25;

        public static final double minExtendHardStop = 0.00;
        public static final double maxExtendSoftStop = 1.25;

        //All of these need testing...
        //meter value * tick conversion

        public static final double extendTickToMeter = 0.01285875;

        public static final double extendOffset = 2.5;
        public static final double extendToFloor = 0.257175;
        public static final double extendToMid = 0.3857625;
        public static final double extendToHigh = 0.6429375;
        public static final double extendToSubstation = 0.6429375;
        // Arm Constants that need measuring
        public static final double pivotPointHeight = 0; // Height of pivot point of arm above point where belt separates from belt wheel
        public static final double pivotPointDistance = 0; // Distance of pivot point of arm behind point where belt separates from belt wheel
        public static final double pivotRadius = 0; // Distance from belt connection to pivot point of arm
    }

    public static final class VisionConstants {
        public static AprilTagFieldLayout getFieldLayout() {
            AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(
                    List.of(
                            new AprilTag(
                                    1,
                                    new Pose3d(
                                            15.513558, 1.071626, 0.462788, new Rotation3d(0, 0, Math.PI))
                            ),
                            new AprilTag(
                                    2,
                                    new Pose3d(
                                            15.513558, 2.748026, 0.462788, new Rotation3d(0, 0, Math.PI))
                            ),
                            new AprilTag(
                                    3,
                                    new Pose3d(
                                            15.513558, 4.424426, 0.462788, new Rotation3d(0, 0, Math.PI))
                            ),
                            new AprilTag(
                                    4,
                                    new Pose3d(
                                            16.178784, 6.749796, 0.695452, new Rotation3d(0, 0, Math.PI))
                            ),
                            new AprilTag(
                                    5,
                                    new Pose3d(
                                            0.36195, 6.749796, 0.695452, new Rotation3d(0, 0, 0))
                            ),
                            new AprilTag(
                                    6,
                                    new Pose3d(
                                            1.02743, 4.424426, 0.462788, new Rotation3d(0, 0, 0))
                            ),
                            new AprilTag(
                                    7,
                                    new Pose3d(
                                            1.02743, 2.748026, 0.462788, new Rotation3d(0, 0, 0))
                            ),
                            new AprilTag(
                                    8,
                                    new Pose3d(
                                            1.02743, 1.071626, 0.462788, new Rotation3d(0, 0, 0))
                            )
                    ),
                    16.54175,
                    8.0137
            );

            if (ALLIANCE == DriverStation.Alliance.Red) {
                fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            }
            return fieldLayout;
        }

        public static AprilTagFieldLayout APRILTAG_LAYOUT = getFieldLayout();
        public static final double FIELD_LENGTH = 16.54175;
        public static final double FIELD_WIDTH = 8.0137;

        public static final Pose2d GRID_OUTER =
                ALLIANCE == DriverStation.Alliance.Red ?
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25),
                                FIELD_WIDTH - Units.feetToMeters(6.25 / 2.0),
                                new Rotation2d(Math.PI)
                        ) :
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25),
                                Units.feetToMeters(6.25 / 2.0),
                                new Rotation2d(Math.PI)
                        );

        public static final Pose2d GRID_COOP =
                ALLIANCE == DriverStation.Alliance.Red ?
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25),
                                FIELD_WIDTH - (Units.feetToMeters(6.25) + Units.feetToMeters(5.5 / 2.0)),
                                new Rotation2d(Math.PI)
                        ) :
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25),
                                Units.feetToMeters(6.25) + Units.feetToMeters(5.5 / 2.0),
                                new Rotation2d(Math.PI)
                        );

        public static final Pose2d GRID_INNER =
                ALLIANCE == DriverStation.Alliance.Red ?
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25),
                                FIELD_WIDTH - (Units.feetToMeters(6.25) + Units.feetToMeters(5.5) + Units.feetToMeters(6.25 / 2.0)),
                                new Rotation2d(Math.PI)
                        ) :
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25),
                                Units.feetToMeters(6.25) + Units.feetToMeters(5.5) + Units.feetToMeters(6.25 / 2.0),
                                new Rotation2d(Math.PI)
                        );

        public static final Transform2d CONE_OFFSET_LEFT = new Transform2d(
                new Translation2d(
                        0,
                        Units.feetToMeters(1) + Units.inchesToMeters(6.5)
                ),
                new Rotation2d()
        );

        public static final Transform2d CONE_OFFSET_RIGHT = new Transform2d(
                new Translation2d(
                        0,
                        -(Units.feetToMeters(1) + Units.inchesToMeters(6.5))
                ),
                new Rotation2d()
        );

        public static final Pose2d CHARGE_STATION_CENTER =
                ALLIANCE == DriverStation.Alliance.Red ?
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25) + Units.inchesToMeters(96.75),
                                Units.feetToMeters(6.25) + Units.feetToMeters(5.5 / 2),
                                new Rotation2d(0)
                        ) :
                        new Pose2d(
                                Units.feetToMeters(4) + Units.inchesToMeters(8.25) + Units.inchesToMeters(96.75),
                                FIELD_WIDTH - (Units.feetToMeters(6.25) + Units.feetToMeters(5.5 / 2)),
                                new Rotation2d(Math.PI)
                        );

        public static final Pose2d DOUBLE_SUBSTATION =
                ALLIANCE == DriverStation.Alliance.Red ?
                        APRILTAG_LAYOUT.getTagPose(5).get().toPose2d() :
                        APRILTAG_LAYOUT.getTagPose(4).get().toPose2d();

        public static final Transform2d DOUBLE_SUBSTATION_OFFSET_LEFT = new Transform2d(
                new Translation2d(
                        1.0,
                        -(Units.inchesToMeters(46.25) - Units.inchesToMeters(32.25 / 2.0))
                ),
                new Rotation2d(Math.PI)
        );

        public static final Transform2d DOUBLE_SUBSTATION_OFFSET_RIGHT = new Transform2d(
                new Translation2d(
                        1.0,
                        Units.inchesToMeters(46.25) - Units.inchesToMeters(32.25 / 2.0)
                ),
                new Rotation2d(Math.PI)
        );

        /**
         * Convenience Pose to be used with Pose2d.plus()
         **/
        public static final Transform2d ONE_METER_BACK = new Transform2d(
                new Translation2d(
                        -1,
                        0
                ),
                new Rotation2d()
        );
    }
}
