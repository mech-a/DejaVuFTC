package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DejaVuLinearOpMode;

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
