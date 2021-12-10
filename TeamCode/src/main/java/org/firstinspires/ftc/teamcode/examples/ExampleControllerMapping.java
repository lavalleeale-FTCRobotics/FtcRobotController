package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.internal.ControllerMapping;
import org.firstinspires.ftc.internal.Experimental;
import org.firstinspires.ftc.internal.OptimizedController;

import java.util.HashMap;


@Experimental
public class ExampleControllerMapping implements ControllerMapping {
    @Override
    public HashMap<String, ControlInput> initializeMapping(HashMap<String, ControlInput> mapping) {

        mapping.put("Motor", new ControlInput(OptimizedController.Key.A, Controller.CONTROLLER2, Type.TOGGLE));

        return mapping;
    }
}
