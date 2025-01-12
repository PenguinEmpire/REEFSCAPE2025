@ -0,0 +1,35 @@
package frc;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ControlInput {
    Joystick accessoryJoystick;  
    Joystick rightJoystick;  
    Joystick leftJoystick;
    XboxController xboxController;
    public ControlInput() {
        accessoryJoystick = new Joystick(0);
        rightJoystick = new Joystick(1);
        leftJoystick = new Joystick(2);
        xboxController = new XboxController(3);
    }

    public Joystick getLeftJoystick() {
        return leftJoystick;
    }
    public Joystick getRightJoystick() {
        return rightJoystick;
    }
    public Joystick getAccessoryJoystick() {
        return accessoryJoystick;
    }
    public XboxController getGameController() {
        return xboxController;
    }

   
}