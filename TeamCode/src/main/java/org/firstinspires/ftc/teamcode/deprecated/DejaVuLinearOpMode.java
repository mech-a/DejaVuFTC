package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.deprecated.called.HWRobot;

/**
 * Created by gbhat on 2/27/2018.
 */

@SuppressWarnings("unused")
public abstract class DejaVuLinearOpMode extends LinearOpMode {
    HWRobot r = new HWRobot();


    //type addition of
    //i.e. TYPE.FWD, TYPE.MTRS
    public void initHW(String hw) {
        r.getOpModeData(telemetry,hardwareMap);
        r.init(hw);


    }

    public synchronized void translate(int counts, double speed, String direction) {


    }


}
