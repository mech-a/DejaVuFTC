package org.firstinspires.ftc.teamcode.deprecated.called;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by aadhidhya on 9/9/18.
 */


@Disabled
public class TestExtension extends LinearOpMode {
    @Override
    public void runOpMode() {
        //to be overriden when extended for actual op mode
    }
    public void translate() {


    }
    public void inits() {

    }
    public void rotate() {

    }
    //cv functions

    public void printSomething() {
        telemetry.addData("Status", "x");
        telemetry.update();
    }
}

