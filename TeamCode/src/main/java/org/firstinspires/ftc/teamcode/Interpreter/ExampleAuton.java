package org.firstinspires.ftc.teamcode.Interpreter;

import org.firstinspires.ftc.teamcode.dependencies.Enums;

/**
 * Created by gbhat on 3/9/2019.
 */

public class ExampleAuton {
    MethodData[] methods = new MethodData[] {
            new MethodData("Robot","translate", Enums.Direction.BACK, 800, 0.2),
            //new MethodData("getGoldPosition", null);
            new MethodData("Robot","rotate", "ccw", 80, 0.2)
    };
}
