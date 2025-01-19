package frc;

import edu.wpi.first.wpilibj.PS4Controller;

public class ControlInput {
    // Main controller
    private final PS4Controller ps5Controller;

    // PS5 controller on port 0
    public ControlInput() {
        ps5Controller = new PS4Controller(0); 
    }

    // Accessors for PS4 controller inputs
    public PS4Controller getController() {
        return ps5Controller;
    }

    // Joystick axes for driving
    public double getForward() {
        return -ps5Controller.getLeftY(); // Forward/backward movement
    }

    public double getStrafe() {
        return -ps5Controller.getLeftX(); // Strafing left/right
    }

    public double getRotation() {
        return ps5Controller.getRightX(); // Rotation (turning)
    }
}
