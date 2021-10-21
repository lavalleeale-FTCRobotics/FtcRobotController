package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.internal.HardwareAliasMapping;
import org.firstinspires.ftc.internal.Experimental;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Experimental
public class SampleHardwareAliasMapping implements HardwareAliasMapping {

    @Override
    public HashMap<String, List<String>> initializeMapping(HashMap<String, List<String>> mapping) {


        // For each hardware component actual map name -- add aliases you might want to use
        mapping.put("xEncoder", Arrays.asList("outtakeMotor1", "outtake1"));
        mapping.put("yEncoder", Arrays.asList("wobbleMotor"));

        return mapping;
    }
}
