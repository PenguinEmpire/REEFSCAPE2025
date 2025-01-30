package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;


public interface ControlInput {
    double getForward();  // Forward/backward input
    double getStrafe();   // Left/right input
    double getRotation(); // Rotational input

   
    static ControlInput forPS4Controller(PS4Controller controller) {
        return new ControlInput() {
            @Override
            public double getForward() {
                return -controller.getLeftY(); // Forward/backward input
            }

            @Override
            public double getStrafe() {
                return -controller.getLeftX(); // Left/right input
            }

            @Override
            public double getRotation() {
                return controller.getRightX(); // Rotational input
            }
        };
    }
}
