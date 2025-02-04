package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;

public interface ControlInput {
    double getForward();
    double getStrafe();
    double getRotation();

    static ControlInput forPS5Controller(PS5Controller controller) {
        return new ControlInput() {
            public double getForward() { return -controller.getLeftY(); }
            public double getStrafe() { return controller.getLeftX(); }
            public double getRotation() { return controller.getRightX(); }
        };
    }
}
