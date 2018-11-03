package org.firstinspires.ftc.teamcode.functionality;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dependencies.Robot;

@TeleOp(name="TestTranslate", group="Internal")
//@Disabled
public class TestTranslate extends LinearOpMode {

    // Declare OpMode members.
    Robot r = new Robot(this);

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        r.start(hardwareMap, telemetry);
        r.init();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        telemetry.addData("Translating:", "12inches fwd @ 0.1");
        telemetry.update();
        sleep(750);
        r.translate(12, 0.1);

        telemetry.addData("Translating:", "16inches back @ 0.25");
        telemetry.update();
        sleep(750);
        r.translate(16, 0.25);


    }
}
