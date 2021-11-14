package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.internal.ControllerMapping;
import org.firstinspires.ftc.internal.Experimental;
import org.firstinspires.ftc.internal.OptimizedController;

import java.util.HashMap;

@Experimental
public class SampleControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, OptimizedController.Key> initializeMapping(HashMap<String, OptimizedController.Key> mapping) {


        /*
        RS: Snap arm up down on current side
        RB: Change high position to mid for lower placements
        LB: Swap arm left right at current height
        RSy: Move arm up down when arm is in manual control mode
        X: Change between automatic and manual arm control mode
        BACK: set current arm position as all the way down on the front
        LSy: move odometry up and down
        A: Open and Close Claw
        B: Move duck Spinner
        Y: Move duck in reverse direction
        RT: Move duck spinner (float)
        LT: Move duck spinner in reverse direction (float)
         */
        mapping.put("ArmSnap", OptimizedController.Key.RIGHT_STICK_BUTTON);
        mapping.put("ArmMid", OptimizedController.Key.RIGHT_BUMPER);
        mapping.put("ArmSwap", OptimizedController.Key.LEFT_BUMPER);
        mapping.put("ArmSmooth", OptimizedController.Key.RIGHT_STICK_Y);
        mapping.put("ArmControl", OptimizedController.Key.X);
        mapping.put("ArmReset", OptimizedController.Key.BACK);
        mapping.put("Odometry", OptimizedController.Key.LEFT_STICK_Y);
        mapping.put("Claw", OptimizedController.Key.A);
        mapping.put("Duck", OptimizedController.Key.B);
        mapping.put("ReverseDuck", OptimizedController.Key.Y);
        mapping.put("DuckSpeed", OptimizedController.Key.RIGHT_TRIGGER);
        mapping.put("DuckSpeedReverse", OptimizedController.Key.LEFT_TRIGGER);

        return mapping;
    }
}
