package org.firstinspires.ftc.teamcode.regional;

public class Assembly {
    InitializationData initData;
    public InitializationData getInitializationData() {
        return initData;
    }

    public Assembly() {
        initData = null;
    }

    public Assembly(Object object, String configurationName) {
        initData = new InitializationData(object, configurationName);
    }
}
