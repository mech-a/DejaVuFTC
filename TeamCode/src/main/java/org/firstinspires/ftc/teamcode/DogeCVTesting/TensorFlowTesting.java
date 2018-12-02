package org.firstinspires.ftc.teamcode.DogeCVTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.HD_COUNTS_PER_INCH;

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
