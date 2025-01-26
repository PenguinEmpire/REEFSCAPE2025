package frc;

import edu.wpi.first.wpilibj.PS4Controller;

public class ControlInput {
    private final PS4Controller ps5Controller;

    public ControlInput() {
        ps5Controller = new PS4Controller(0);
    }

    public double getForward() {
        return -ps5Controller.getLeftY();
    }

    public double getStrafe() {
        return -ps5Controller.getLeftX();
    }

    public double getRotation() {
        return ps5Controller.getRightX();
    }
}
