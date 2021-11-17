package org.firstinspires.ftc.teamcode.finals;

import org.firstinspires.ftc.internal.ControllerMapping;
import org.firstinspires.ftc.internal.Experimental;
import org.firstinspires.ftc.internal.OptimizedController;

import java.util.HashMap;

@Experimental
public class FreightFrenzyControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, ControlInput> initializeMapping(HashMap<String, ControlInput> mapping) {


        /*
        RS: Snap arm up down on current side
        RB: Change high position to mid for lower placements
        LB: Swap arm left right at current height
        RSy: Move arm up down when arm is in manual control mode
        X: Change between automatic and manual arm control mode
        LSy: move odometry up and down
        A: Open and Close Claw
        B: Move duck Spinner
        Y: Move duck in reverse direction
        RT: Move duck spinner (float)
        LT: Move duck spinner in reverse direction (float)
         */
        mapping.put("ArmSnap", new ControlInput(OptimizedController.Key.RIGHT_STICK_BUTTON, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("ArmMid", new ControlInput(OptimizedController.Key.RIGHT_STICK_BUTTON, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("ArmSwap", new ControlInput(OptimizedController.Key.LEFT_BUMPER, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("ArmSmooth", new ControlInput(OptimizedController.Key.RIGHT_STICK_Y, Controller.CONTROLLER2, Type.FLOAT));
        mapping.put("ArmControl", new ControlInput(OptimizedController.Key.X, Controller.CONTROLLER2, Type.TOGGLE));
        mapping.put("Odometry", new ControlInput(OptimizedController.Key.LEFT_STICK_Y, Controller.CONTROLLER2, Type.FLOAT));
        mapping.put("Claw", new ControlInput(OptimizedController.Key.A, Controller.CONTROLLER1, Type.TOGGLE));
        mapping.put("Duck", new ControlInput(OptimizedController.Key.B, Controller.CONTROLLER2, Type.BOOL));
        mapping.put("ReverseDuck", new ControlInput(OptimizedController.Key.Y, Controller.CONTROLLER2, Type.BOOL));
        mapping.put("DuckSpeed", new ControlInput(OptimizedController.Key.RIGHT_TRIGGER, Controller.CONTROLLER2, Type.FLOAT));
        mapping.put("DuckSpeedReverse", new ControlInput(OptimizedController.Key.LEFT_TRIGGER, Controller.CONTROLLER2, Type.FLOAT));

        return mapping;
    }
}
