package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.NomadMathUtil;
import frc.robot.util.sim.SimGyroSensorModel;
import frc.robot.util.sim.wpiClasses.QuadSwerveSim;
import frc.robot.util.sim.wpiClasses.SwerveModuleSim;
import frc.robot.util.trajectory.PPChasePoseCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static frc.robot.Constants.AutoConstants.eventMap;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import static java.lang.Math.abs;

public class DrivebaseS extends SubsystemBase implements Loggable {

    /**
     * Subsystem that controls the drivetrain of the robot
     * Handles all the odometry and base movement for the chassis
     */


    private final AHRS navx = new AHRS(Port.kMXP);
    private final SimGyroSensorModel simNavx = new SimGyroSensorModel();
    private final Limelight limelight;

    
    public final PIDController xController = new PIDController(chassisTranslationalkP, 0, chassisTranslationalkD);
    
    public final PIDController yController = new PIDController(chassisTranslationalkP, 0, chassisTranslationalkD);
    @Log
    public final PIDController thetaController = new PIDController(chassisThetakP, 0, chassisThetakD);
    public final PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(xController, yController, thetaController);

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    private final SwerveDrivePoseEstimator odometry;

    private final List<SwerveModuleSim> moduleSims = List.of(
            DrivebaseS.swerveSimModuleFactory(),
            DrivebaseS.swerveSimModuleFactory(),
            DrivebaseS.swerveSimModuleFactory(),
            DrivebaseS.swerveSimModuleFactory()
    );

    private final QuadSwerveSim quadSwerveSim =
            new QuadSwerveSim(
                    WHEEL_BASE_WIDTH_M,
                    WHEEL_BASE_WIDTH_M,
                    ROBOT_MASS_kg,
                    ROBOT_MOI_KGM2,
                    moduleSims
            );

    @Log
    private final SwerveModule fl = new SwerveModule(ModuleConstants.FL);
    @Log
    private final SwerveModule fr = new SwerveModule(ModuleConstants.FR);
    @Log
    private final SwerveModule bl = new SwerveModule(ModuleConstants.BL);
    @Log
    private final SwerveModule br = new SwerveModule(ModuleConstants.BR);
    @Log.Exclude
    private final List<SwerveModule> modules = List.of(
            fl, fr, bl, br
    );

    private final LinearFilter visionPoseAverageX = LinearFilter.movingAverage(VISION_AVERAGING_TIME);
    private final LinearFilter visionPoseAverageY = LinearFilter.movingAverage(VISION_AVERAGING_TIME);
    private final LinearFilter visionPoseAverageT = LinearFilter.movingAverage(VISION_AVERAGING_TIME);

    private Pose2d targetPose = new Pose2d();

    public DrivebaseS(Limelight m_limelight) {
        holonomicDriveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(0.5)));
        navx.reset();

        odometry =
                new SwerveDrivePoseEstimator(
                        m_kinematics,
                        new Rotation2d(getHeading().getRadians()),
                        getModulePositions(),
                        new Pose2d()
                );
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(VISION_MEASUREMENT_STD_DEV, VISION_MEASUREMENT_STD_DEV, VISION_MEASUREMENT_STD_DEV));
        resetPose(new Pose2d());
        limelight = m_limelight;
    }

    @Override
    public void periodic() {
        // update the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());
        if (!limelight.getDataAccessedBefore() &&
                limelight.visionTargetsFound() &&
                abs(Timer.getFPGATimestamp() - (limelight.getLastTimestamp() / 1000.0)) < 1.0
        ) {
            Pose2d limelightPose = limelight.getPosition().toPose2d();
            if (
                    abs(limelightPose.getX()) < 0.1 &&
                            abs(limelightPose.getY()) < 0.1 &&
                            abs(limelightPose.getRotation().getRadians()) < 0.1
            ) {
                return;
            }
            double poseAvgX = visionPoseAverageX.calculate(limelightPose.getX());
            double poseAvgY = visionPoseAverageY.calculate(limelightPose.getY());
            double poseAvgT = visionPoseAverageT.calculate(limelightPose.getRotation().getRadians());
            if (
                    abs(poseAvgX - limelightPose.getX()) < VISION_TRANSLATIONAL_RANGE &&
                            abs(poseAvgY - limelightPose.getY()) < VISION_TRANSLATIONAL_RANGE &&
                            abs(poseAvgT - limelightPose.getRotation().getRadians()) < VISION_ROTATIONAL_RANGE
                // limelightPose.minus(odometry.getEstimatedPosition()).getTranslation().getNorm() < 1.0
            ) {
                odometry.addVisionMeasurement(limelightPose, limelight.getLastTimestamp() / 1000.0);
            }
        }
    }

    public void drive(ChassisSpeeds speeds) {
        // use kinematics (wheel placements) to convert overall robot state to array of individual module states
        SwerveModuleState[] states;

        // For now, *don't* set the modules to whatever they were last at. Let them all go to 0

        // If we are stopped (no wheel velocity commanded) then any number of wheel angles could be valid.
        // By default it would point all modules forward when stopped. Here, we override this.
        if (abs(speeds.vxMetersPerSecond) < 0.01
                && abs(speeds.vyMetersPerSecond) < 0.01
                && abs(speeds.omegaRadiansPerSecond) < 0.01) {
            states = getStoppedStates();
        } else {
            // make sure the wheels don't try to spin faster than the maximum speed possible
            states = m_kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    states,
                    speeds,
                    DriveConstants.MAX_MODULE_SPEED_FPS,
                    DriveConstants.MAX_FWD_REV_SPEED_MPS,
                    DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC
            );
        } 
        
        /* 
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(
                0,
                new Rotation2d(0));
        }*/
        setModuleStates(states);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPoseHeading()));
    }

    public void driveFieldRelativeHeading(ChassisSpeeds speeds) {
        double omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
        double currentTargetRadians = thetaController.getSetpoint();

        double newTargetRadians = currentTargetRadians + (omegaRadiansPerSecond / 50);


        double commandRadiansPerSecond =
                thetaController.calculate(getPoseHeading().getRadians(), newTargetRadians);

        speeds.omegaRadiansPerSecond = commandRadiansPerSecond;
        driveFieldRelative(speeds);
    }


    /**
     * method for driving the robot
     * Parameters:
     * forward linear value
     * sideways linear value
     * rotation value
     * if the control is field relative or robot relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        /**
         * ChassisSpeeds object to represent the overall state of the robot
         * ChassisSpeeds takes a forward and sideways linear value and a rotational value
         *
         * speeds is set to field relative or default (robot relative) based on parameter
         */
        ChassisSpeeds speeds =
                isFieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        forward, strafe, rotation, getPoseHeading())
                        : new ChassisSpeeds(forward, strafe, rotation);

        drive(speeds);

    }

    /**
     * Return the desired states of the modules when the robot is stopped. This can be an x-shape to hold against defense,
     * or all modules forward. Here we have it stopping all modules but leaving the angles at their current positions.
     *
     * @return
     */
    private SwerveModuleState[] getStoppedStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = new SwerveModuleState(
                0,
                new Rotation2d(MathUtil.angleModulus(modules.get(i).getCanEncoderAngle().getRadians())));
        }
        return states;
//        SwerveModuleState[] states = new SwerveModuleState[4];
//        for (int i = 0; i < 4; i++) {
//            states[i] = new SwerveModuleState(
//                    0,
//                    new Rotation2d(0));
//        }
//        return states;
    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {
        for (int i = 0; i < NUM_MODULES; i++) {
            modules.get(i).setDesiredStateClosedLoop(moduleStates[i]);
        }
    }

    /*
     *  returns an array of SwerveModuleStates.
     *  Front(left, right), Rear(left, right)
     *  This order is important to remain consistent across the codebase, or commands can get swapped around.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = new SwerveModuleState(
                    modules.get(i).getCurrentVelocityMetersPerSecond(),
                    modules.get(i).getCanEncoderAngle());
        }
        return states;
    }

    /**
     * Return the module positions for odometry.
     *
     * @return an array of 4 SwerveModulePosition objects
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < NUM_MODULES; i++) {
            states[i] = new SwerveModulePosition(
                    modules.get(i).getDriveDistanceMeters(),
                    modules.get(i).getCanEncoderAngle());
        }
        return states;
    }

    /**
     * Return the current position of the robot on field
     * Based on drive encoder and gyro reading
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Return the simulated estimate of the robot's pose.
     * NOTE: on a real robot this will return a new Pose2d, (0, 0, 0)
     *
     * @return
     */
    public Pose2d getSimPose() {
        if (RobotBase.isSimulation()) {
            return quadSwerveSim.getCurPose();
        } else {
            return new Pose2d();
        }
    }

    /**
     * Reset the pose of odometry and sim to the given pose.
     *
     * @param pose The Pose2d to reset to.
     */
    public void resetPose(Pose2d pose) {
        quadSwerveSim.modelReset(pose);
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    // reset the measured distance driven for each module
    public void resetDriveDistances() {
        modules.forEach((module) -> module.resetDistance());
    }


    /**
     * @return the current navX heading (which will not match odometry after drift or reset)
     */
    @Log(methodName = "getRadians")
    public Rotation2d getHeading() {
        if (RobotBase.isSimulation()) {
            return simNavx.getRotation2d();
        }
        return navx.getRotation2d();
    }

    @Log(methodName = "getRadians")
    /**
     * Gets the current heading based on odometry. (this value will reflect odometry resets)
     * @return the current odometry heading.
     */
    public Rotation2d getPoseHeading() {
        return getPose().getRotation();
    }

    /*
     * Resets the navX to 0 position;
     */
    public void resetImu() {
        navx.reset();
        simNavx.resetToPose(new Pose2d());
    }

    public float getPitch() {
        return navx.getPitch();
    }

    public float getRoll() {
        return navx.getRoll();
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d target) {
        targetPose = target;
    }

    public boolean atTargetPose() {
        Transform2d error = getPose().minus(getTargetPose());
        Transform2d tolerance = new Transform2d(
                new Translation2d(0.02, 0.02),
                Rotation2d.fromDegrees(0.5)
        );
        return abs(error.getRotation().getRadians()) < tolerance.getRotation().getRadians() &&
                abs(error.getX()) < tolerance.getX() &&
                abs(error.getY()) < tolerance.getY();
    }

    public void setRotationState(double radians) {
        thetaController.setSetpoint(radians);
    }

    /**
     * Returns a Translation2d representing the linear robot speed in field coordinates.
     */
    public Translation2d getFieldRelativeLinearSpeedsMPS() {
        // Get robot relative speeds from module states
        ChassisSpeeds robotRelativeSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
        // Get field relative speeds by undoing the field-robot conversion (which was just a rotation by the heading)
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                robotRelativeSpeeds.vxMetersPerSecond,
                robotRelativeSpeeds.vyMetersPerSecond,
                robotRelativeSpeeds.omegaRadiansPerSecond,
                getPoseHeading().unaryMinus()
        );
        // Convert to translation
        Translation2d translation = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
        // to avoid angle issues near 0, if the distance is 0.01 or less just return (0, 0)
        if (translation.getNorm() < 0.01) {
            return new Translation2d();
        } else {
            return translation;
        }
    }

    @Override
    public void simulationPeriodic() {

        // set inputs. Set 0 if the robot is disabled.
        if (!DriverStation.isEnabled()) {
            for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
                moduleSims.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
                double azmthVolts = modules.get(idx).getAppliedRotationVoltage();
                double wheelVolts = modules.get(idx).getAppliedDriveVoltage() * 1.44;
                moduleSims.get(idx).setInputVoltages(wheelVolts, azmthVolts);
            }
        }

        Pose2d prevRobotPose = quadSwerveSim.getCurPose();

        // Update model (several small steps)
        for (int i = 0; i < 20; i++) {
            quadSwerveSim.update(0.001);
        }


        //Set the state of the sim'd hardware
        for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
            double azmthPos = moduleSims.get(idx).getAzimuthEncoderPositionRev();
            azmthPos = azmthPos / AZMTH_ENC_COUNTS_PER_MODULE_REV * 2 * Math.PI;
            double wheelPos = moduleSims.get(idx).getWheelEncoderPositionRev();
            wheelPos = wheelPos / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;

            double wheelVel = moduleSims.get(idx).getWheelEncoderVelocityRevPerSec();
            wheelVel = wheelVel / WHEEL_ENC_COUNTS_PER_WHEEL_REV * 2 * Math.PI * WHEEL_RADIUS_M;
            modules.get(idx).setSimState(azmthPos, wheelPos, wheelVel);

        }
        // Set the gyro based on the difference between the previous pose and this pose.
        simNavx.update(quadSwerveSim.getCurPose(), prevRobotPose);
    }

    /**
     * A convenience method to draw the robot pose and 4 poses representing the wheels onto the field2d.
     *
     * @param field
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        field.getObject("modules").setPoses(List.of(
                getPose().transformBy(new Transform2d(ModuleConstants.FL.centerOffset, getModuleStates()[FL].angle)),
                getPose().transformBy(new Transform2d(ModuleConstants.FR.centerOffset, getModuleStates()[FR].angle)),
                getPose().transformBy(new Transform2d(ModuleConstants.BL.centerOffset, getModuleStates()[BL].angle)),
                getPose().transformBy(new Transform2d(ModuleConstants.BR.centerOffset, getModuleStates()[BR].angle))
        ));
    }

    static SwerveModuleSim swerveSimModuleFactory() {
        return new SwerveModuleSim(DCMotor.getNEO(1),
                DCMotor.getNEO(1),
                WHEEL_RADIUS_M,
                1.0 / AZMTH_REVS_PER_ENC_REV, // steering motor rotations per wheel steer rotation
                1.0 / WHEEL_REVS_PER_ENC_REV,
                1.0 / AZMTH_REVS_PER_ENC_REV, // same as motor rotations because NEO encoder is on motor shaft
                1.0 / WHEEL_REVS_PER_ENC_REV,
                1.5,
                2,
                ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES,
                0.01
        );
    }


    public void resetRelativeRotationEncoders() {
        /* Note that we use the class name not a variable name.
         * This way we pass a method of the general SwerveModule class to be called
         * as each module.
         * So this expands to:
         *  modules.get(0).initRotationOffset();
         *  modules.get(1).initRotationOffset();
         *  ...
         */
        modules.forEach(SwerveModule::initRotationOffset);
    }

    public void resetPID() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    /****COMMANDS */
    public Command pathPlannerCommand(Supplier<PathPlannerTrajectory> path) {
        PathPlannerTrajectory retrievePath = path.get();
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
                retrievePath,
                this::getPose,
                m_kinematics,
                xController,
                yController,
                thetaController,
                this::setModuleStates,
                true,
                this
        );
        return command;
    }

    public Command pathPlannerCommand(PathPlannerTrajectory path) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
                path,
                this::getPose,
                m_kinematics,
                xController,
                yController,
                thetaController,
                this::setModuleStates,
                true,
                this
        );
        return command;
    }

    public SwerveAutoBuilder getSwerveAutoBuilder() {
        return new SwerveAutoBuilder(
                this::getPose,
                this::resetPose,
                m_kinematics,
                new PIDConstants(chassisTranslationalkP, 0.0, chassisTranslationalkD),
                new PIDConstants(chassisThetakP, 0.0, chassisThetakD),
                this::setModuleStates,
                eventMap,
                true,
                this
        );
    }


    /**
     * For use with PPChasePoseCommand
     * Generates a PathPlannerTrajectory on the fly to drive to the target pose.
     * Takes into account the current speed of the robot for the start point.
     * The returned PathPlannerTrajectory will go straight towards the target from the robot pose.
     * The component of the current velocity that points toward the target will be used as the initial
     * velocity of the trajectory.
     *
     * @param robotPose             the current robot pose
     * @param target                the target pose
     * @param currentSpeedVectorMPS a Translation2d where x and y are the robot's x and y field-relative speeds in m/s.
     * @return a PathPlannerTrajectory to the target pose.
     */
    public static PathPlannerTrajectory generateTrajectoryToPose(Pose2d robotPose, Pose2d target, Translation2d currentSpeedVectorMPS, double maxVelocity, double maxAccel) {
        // Robot velocity calculated from module states.
        Rotation2d fieldRelativeTravelDirection = NomadMathUtil.getDirection(currentSpeedVectorMPS);
        double travelSpeed = currentSpeedVectorMPS.getNorm();


        Translation2d robotToTargetTranslation = target.getTranslation().minus(robotPose.getTranslation());
        // Initial velocity override is the component of robot velocity along the robot-to-target vector.
        // If the robot velocity is pointing away from the target, start at 0 velocity.
        Rotation2d travelOffsetFromTarget = NomadMathUtil.getDirection(robotToTargetTranslation).minus(fieldRelativeTravelDirection);
        travelSpeed = Math.max(0, travelSpeed * travelOffsetFromTarget.getCos());
        // We only want to regenerate if the target is far enough away from the robot.
        // PathPlanner has issues with near-zero-length paths and we need a particular tolerance for success anyway.
        if (
                robotToTargetTranslation.getNorm() > 0.1
        ) {
            PathPlannerTrajectory pathPlannerTrajectory = PathPlanner.generatePath(
                    new PathConstraints(maxVelocity, maxAccel),
                    //Start point. At the position of the robot, initial travel direction toward the target,
                    // robot rotation as the holonomic rotation, and putting in the (possibly 0) velocity override.
                    new PathPoint(
                            robotPose.getTranslation(),
                            NomadMathUtil.getDirection(robotToTargetTranslation),
                            robotPose.getRotation(),
                            travelSpeed), // position, heading
                    // position, heading
                    new PathPoint(
                            target.getTranslation(),
                            NomadMathUtil.getDirection(robotToTargetTranslation),
                            target.getRotation()) // position, heading
            );
            return pathPlannerTrajectory;
        }

        return new PathPlannerTrajectory();
    }

    public static PathPlannerTrajectory generateTrajectoryToPose(Pose2d robotPose, Pose2d target, Translation2d currentSpeedVectorMPS) {
        return generateTrajectoryToPose(robotPose, target, currentSpeedVectorMPS, 4, 4);
    }


    /**
     * For use with PathPlannerCommand
     * Generates a PathPlannerTrajectory on the fly to drive through the supplied waypoints.
     * Takes into account the current speed of the robot for the start point.
     * The returned PathPlannerTrajectory will go straight towards the target from the robot pose.
     * The component of the current velocity that points toward the first waypoint will be used as the initial
     * velocity of the trajectory.
     * <p>
     * This function assumes that your trajectory has no waypoints spaced less than 0.1 meters apart.
     * Do not use this function if your waypoints are spaced closer than this, as the generator will have undefined behavior.
     *
     * @param pathWaypoints Waypoint List, minimum length 2.
     * @param currentSpeedVectorMPS a Translation2d where x and y are the robot's x and y field-relative speeds in m/s.
     * @param maxVelocity Maximum Velocity in Meters per Second.
     * @param maxAcceleration Maximum Acceleration in Meters per Second Squared.
     * @return a PathPlannerTrajectory through the supplied waypoints.
     */
    public static PathPlannerTrajectory generateTrajectoryToPose(ArrayList<Pose2d> pathWaypoints, Translation2d currentSpeedVectorMPS, double maxVelocity, double maxAcceleration) {
        if (pathWaypoints.size() < 2) {
            return new PathPlannerTrajectory();
        }

        ArrayList<PathPoint> pathPoints = new ArrayList<>();

        // Robot velocity calculated from module states.
        Rotation2d fieldRelativeTravelDirection = NomadMathUtil.getDirection(currentSpeedVectorMPS);
        double travelSpeed = currentSpeedVectorMPS.getNorm();

        Translation2d robotToTargetTranslation = pathWaypoints.get(1).getTranslation().minus(pathWaypoints.get(0).getTranslation());
        // Initial velocity override is the component of robot velocity along the robot-to-target vector.
        // If the robot velocity is pointing away from the target, start at 0 velocity.
        Rotation2d travelOffsetFromTarget = NomadMathUtil.getDirection(robotToTargetTranslation).minus(fieldRelativeTravelDirection);
        travelSpeed = Math.max(0, travelSpeed * travelOffsetFromTarget.getCos());

        pathPoints.add(
                new PathPoint(
                        pathWaypoints.get(0).getTranslation(),
                        NomadMathUtil.getDirection(robotToTargetTranslation),
                        pathWaypoints.get(0).getRotation(),
                        travelSpeed
                )
        );

        for (int i = 1; i < pathWaypoints.size() - 1; i += 1) {
            robotToTargetTranslation = pathWaypoints.get(i + 1).getTranslation().minus(pathWaypoints.get(i).getTranslation());
            pathPoints.add(
                    new PathPoint(
                            pathWaypoints.get(i).getTranslation(),
                            NomadMathUtil.getDirection(robotToTargetTranslation),
                            pathWaypoints.get(i).getRotation()
                    )
            );
        }

        pathPoints.add(
                new PathPoint(
                        pathWaypoints.get(pathWaypoints.size() - 1).getTranslation(),
                        NomadMathUtil.getDirection(robotToTargetTranslation),
                        pathWaypoints.get(pathWaypoints.size() - 1).getRotation()
                )
        );

        try {
            return PathPlanner.generatePath(
                    new PathConstraints(maxVelocity, maxAcceleration),
                    pathPoints
            );
        } catch (Exception e) {
            return new PathPlannerTrajectory();
        }
    }

    /**
     * Creates a new pose-chase command.
     * This command generates and follows the target pose supplied by targetSupplier.
     * If the target has moved since the last generation, regen the trajectory.
     * If the trajectory is finished, switch to direct x-y-theta PID to hold the pose.
     *
     * @param targetSupplier the Supplier for the target Pose2d.
     * @return the PPChasePoseCommand
     */
    public Command chasePoseC(Supplier<Pose2d> targetSupplier) {
        return new PPChasePoseCommand(
                targetSupplier,
                this::getPose,
                holonomicDriveController,
                this::drive,
                (PathPlannerTrajectory traj) -> {
                }, // empty output for current trajectory.
                (startPose, endPose) -> DrivebaseS.generateTrajectoryToPose(startPose, endPose, getFieldRelativeLinearSpeedsMPS(), 1, 0.3),
                this);
    }

    public Command chasePoseC(Supplier<Pose2d> targetSupplier, double maxVelocity, double maxAcceleration) {
        return new PPChasePoseCommand(
                targetSupplier,
                this::getPose,
                holonomicDriveController,
                this::drive,
                (PathPlannerTrajectory traj) -> {
                }, // empty output for current trajectory.
                (startPose, endPose) -> DrivebaseS.generateTrajectoryToPose(startPose, endPose, getFieldRelativeLinearSpeedsMPS(), maxVelocity, maxAcceleration),
                this);
    }

}