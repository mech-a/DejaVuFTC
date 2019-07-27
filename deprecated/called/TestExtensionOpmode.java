package org.firstinspires.ftc.teamcode.deprecated.called;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by gbhat on 9/16/2018.
 */

@TeleOp(name="Testing Extension", group = "Internal")
@Disabled
public class TestExtensionOpmode extends org.firstinspires.ftc.teamcode.deprecated.called.TestExtension {


    @Override
    public void runOpMode() {
        telemetry.addData("xd","xd");
        telemetry.update();

        waitForStart();

        printSomething();

    }
}

