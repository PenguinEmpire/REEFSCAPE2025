package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;

public class ControlInput {
    private final PS5Controller ps5Controller;

    public ControlInput(int port) {
        this.ps5Controller = new PS5Controller(port);
    }

    public double getForward() {
        return ps5Controller.getLeftY(); 
    }

    public double getStrafe() {
        return ps5Controller.getLeftX();
    }

    public double getRotation() {
        return -ps5Controller.getRightX(); 
    }
}
