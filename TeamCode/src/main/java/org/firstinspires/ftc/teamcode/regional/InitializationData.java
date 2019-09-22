package org.firstinspires.ftc.teamcode.regional;

public class InitializationData {
    private Object type;
    private String configurationName;

    InitializationData() {
        type = null;
        configurationName = null;
    }
    InitializationData(Object type, String configurationName) {
        this.type = type;
        this.configurationName = configurationName;
    }


}
