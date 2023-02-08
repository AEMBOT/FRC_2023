// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;

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
        static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((WHEEL_BASE_WIDTH_M*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Math.PI * 4;
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.125; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25; //0-full time of 0.25 second

        // HELPER ORGANIZATION CONSTANTS
        static public final int FL = 0; // Front Left Module Index
        static public final int FR = 1; // Front Right Module Index
        static public final int BL = 2; // Back Left Module Index
        static public final int BR = 3; // Back Right Module Index
        static public final int NUM_MODULES = 4;

        // Internal objects used to track where the modules are at relative to
        // the center of the robot, and all the implications that spacing has.
        static private double HW = WHEEL_BASE_WIDTH_M/2.0;

        public enum ModuleConstants {
            FL("FL", 9, 2, 12, 2.351588, HW, HW),
            FR("FR", 3, 4, 10, 2.109219, HW, -HW),
            BL("BL", 5, 6, 13, 0.971008, -HW, HW),
            BR("BR", 7, 8, 11, 1.366774 , -HW, -HW);

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
            private ModuleConstants(String name, int driveMotorID, int rotationMotorID, int magEncoderID, double magEncoderOffset, double xOffset, double yOffset) {
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
        /** ks, kv, ka */
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
        public static final double WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV/ WHEEL_REVS_PER_ENC_REV;  //Assume 1-1 gearing for now
        public static final double AZMTH_ENC_COUNTS_PER_MODULE_REV = ENC_PULSE_PER_REV / AZMTH_REVS_PER_ENC_REV; //Assume 1-1 gearing for now
        public static final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        public static final double AZMTH_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(AZMTH_ENC_COUNTS_PER_MODULE_REV));

        public static final TrapezoidProfile.Constraints X_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
        public static final TrapezoidProfile.Constraints Y_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);

        public static final TrapezoidProfile.Constraints NO_CONSTRAINTS = new TrapezoidProfile.Constraints(Integer.MAX_VALUE, Integer.MAX_VALUE);
        public static final TrapezoidProfile.Constraints THETA_DEFAULT_CONSTRAINTS = new TrapezoidProfile.Constraints(4*Math.PI, 16*Math.PI);


    }

    public static final class AutoConstants {
        public static final DriverStation.Alliance ALLIANCE = DriverStation.getAlliance();

        public static final double maxVelMetersPerSec = 2;
        public static final double maxAccelMetersPerSecondSq = 1;

    }

    public static final class ArmConstants {

        public static final int angleMotorCanID = 15;
        public static final int extendMotorCanID = 14;
        public static final int clampSolenoidID = 0;
        public static final int movingAverage = 5;
        public static final int extendMotorCurrentLimit = 35;
        public static final int angleMotorCurrentLimit = 35;
        //Arm stuff below this is untested, needs accuracy
        public static final double angleToSubstation = 3;
        public static final double angleToFloor = 2;
    }

    public static final class VisionConstants {
        public static AprilTagFieldLayout TAG_FIELD_LAYOUT = new AprilTagFieldLayout(
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
        public static final double FIELD_LENGTH = 16.54175;
        public static final double FIELD_WIDTH = 8.0137;
    }
}
