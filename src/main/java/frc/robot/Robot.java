// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.docking.Docking;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Pipeline;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private Limelight m_Limelight;
    private DrivebaseS m_drivebase;
    AHRS navx = new AHRS(Port.kMXP);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        LiveWindow.disableAllTelemetry();
        Logger.configureLoggingAndConfig(m_robotContainer, false);
        SmartDashboard.putNumber("vel", new SwerveModuleState().speedMetersPerSecond);
        m_robotContainer.onInit();

        // Remove before Comp
        PathPlannerServer.startServer(5811);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        NetworkTableInstance.getDefault().flush();
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
        Logger.updateEntries();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        CommandScheduler.getInstance().cancelAll();

        SmartDashboard.putNumber("Pitch", navx.getPitch());
        SmartDashboard.putNumber("Roll", navx.getRoll());
        SmartDashboard.putNumber("Yaw", navx.getYaw());
        SmartDashboard.putNumber("Tilt", Docking.tilt(navx.getRoll(), navx.getPitch()));
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_robotContainer.onEnabled();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        m_robotContainer.onEnabled();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Pitch", navx.getPitch());
        SmartDashboard.putNumber("Roll", navx.getRoll());
        SmartDashboard.putNumber("Yaw", navx.getYaw());
        SmartDashboard.putNumber("Tilt", Docking.tilt(navx.getRoll(), navx.getPitch()));
        SmartDashboard.putNumber("RawGyroY", navx.getRawGyroY());
        SmartDashboard.putNumber("VelocityY ", navx.getVelocityY());
    }


    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        teleopInit();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        teleopPeriodic();
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
