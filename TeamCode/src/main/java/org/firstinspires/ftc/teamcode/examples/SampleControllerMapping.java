package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.internal.ControllerMapping;
import org.firstinspires.ftc.internal.Experimental;
import org.firstinspires.ftc.internal.OptimizedController;

import java.util.HashMap;

@Experimental
public class SampleControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, OptimizedController.Key> initializeMapping(HashMap<String, OptimizedController.Key> mapping) {

        // Adding controls
        mapping.put("ArmSnap", OptimizedController.Key.RIGHT_STICK_BUTTON);
        mapping.put("ArmSmooth", OptimizedController.Key.RIGHT_STICK_Y);
        mapping.put("ArmControl", OptimizedController.Key.X);
        mapping.put("Odometry", OptimizedController.Key.LEFT_STICK_Y);
        mapping.put("Claw", OptimizedController.Key.A);
        mapping.put("Duck", OptimizedController.Key.B);

        return mapping;
    }
}
