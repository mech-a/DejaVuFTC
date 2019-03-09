package org.firstinspires.ftc.teamcode;

/**
 * Created by gbhat on 3/9/2019.
 */

public class MethodData {
    public String functionName;
    public Object[] functionParams;

    MethodData(String functionName, Object... args) {
        this.functionName = functionName;
        functionParams = args;
    }
}
