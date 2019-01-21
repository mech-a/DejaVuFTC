package org.firstinspires.ftc.teamcode.preregional.DogeCVTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.preregional.dependencies.Enums;
import org.firstinspires.ftc.teamcode.preregional.dependencies.Robot;

@Autonomous(name="tensorflow test", group = "Sensor")
public class TensorFlowTesting extends LinearOpMode {
    Robot r = new Robot(this);

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();
        r.cvInit();

        // Wait until we're told to go
        //telemetry.addData("Hz", r.getGyroHertz().toString());
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            //r.getGoldPosition();
            if (r.getGoldPosition() == Enums.GoldPosition.RIGHT) {
                telemetry.addData("side:", "right");
            } else if (r.getGoldPosition() == Enums.GoldPosition.MIDDLE) {
                telemetry.addData("side:", "middle");
            } else if (r.getGoldPosition() == Enums.GoldPosition.LEFT) {
                telemetry.addData("side:", "left");
            }
            sleep(300);
            telemetry.update();
        }
    }
}
