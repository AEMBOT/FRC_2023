package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.IntakeConstants.clampSolenoidID;
import static frc.robot.Constants.IntakeConstants.limitSwitchID;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
    @Log(methodName = "get")
    private final Solenoid m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clampSolenoidID);
    private final DigitalInput limitSwitch = new DigitalInput(limitSwitchID);
    private final Debouncer debouncer = new Debouncer(0.1);

    public IntakeSubsystem() {
        closeClamp();
    }

    public Command getIntakeAutoClampCommand() {
        return startEnd(this::openClamp, this::closeClamp).until(this::getLimitSwitchState);
    }

    @Override
    public void periodic() {

    }

    @Log
    public boolean getLimitSwitchState() {
        return debouncer.calculate(!limitSwitch.get()); // The limit switch is active-low, but code is easier to understand active-high
    }

    // Extends the clamp
    public void openClamp() {
        m_clampSolenoid.set(true);
    }

    // Retracts the clamp
    public void closeClamp() {
        m_clampSolenoid.set(false);
    }

    // Toggles the clamp
    public void toggleClamp() {
        m_clampSolenoid.set(!m_clampSolenoid.get());
    }
}
