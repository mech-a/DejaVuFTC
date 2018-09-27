package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by gbhat on 3/6/2018.
 */


@Autonomous(name= "Testing for Deja Vu OpMode",group = "Deja Vu Opmode Testing")
@Disabled
public class TestTranslateDejaVuOpMode extends DejaVuLinearOpMode {

    @Override
    public void runOpMode() {
        initHW("mtr");


        waitForStart();

    }
}
