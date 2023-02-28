package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Numpad extends GenericHID{

    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public Numpad(int port){
        super(port);
    }
    
    enum Key{
        k1(1),
        k2(2),
        k3(3),
        k4(4),
        k5(5),
        k6(6),
        k7(7),
        k8(8),
        k9(9),
        kTab(10),
        kEquals(11),
        kForwardSlash(12),
        kAsterisk(13),
        kBackspace(14),
        kEnter(15),
        k0(16),
        kEscape(17),
        kPeriod(18),
        kMinus(19),
        kPlus(20);
        public final int value;
        Key(int i){
            this.value = i;
        }
        public String toString() {
            var name = this.name().substring(1); // Remove leading `k`
            if (name.endsWith("Bumper")) {
                return name;
            }
            return name + "Button";
        }
    }
    public boolean getKey(Key key){
        return getRawButton(key.value);
    }
    public Trigger button(Key key, EventLoop loop) {
        return new Trigger(loop, () -> this.getKey(key));
    }
    public Trigger button(Key key){
        return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> this.getKey(key));
    }
    public Trigger button(int key){
        return button(Key.values()[key]);
    }
}
