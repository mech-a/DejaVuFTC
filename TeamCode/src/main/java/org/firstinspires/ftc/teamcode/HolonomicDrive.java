package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by gbhat on 8/20/2017.
 */

@TeleOp(name="Holonomic Drive", group="DriveOPs")
//@Disabled
public class HolonomicDrive extends LinearOpMode{
    //HWRobotOld robot = new HWRobotOld();
    HWRobot robot = new HWRobot();
    private double ch1,ch2,ch3,ch4;



    @Override
    public void runOpMode() {
        //robot.init(hardwareMap);
        robot.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.mtrFL = hardwareMap.dcMotor.get("fl_drive");
//        robot.mtrFR = hardwareMap.dcMotor.get("fr_drive");
//        robot.mtrBL = hardwareMap.dcMotor.get("bl_drive");
//        robot.mtrBR = hardwareMap.dcMotor.get("br_drive");
//
//        robot.mtrFL.setDirection(DcMotor.Direction.REVERSE);
//        robot.mtrFR.setDirection(DcMotor.Direction.FORWARD);
//        robot.mtrBL.setDirection(DcMotor.Direction.REVERSE);
//        robot.mtrBR.setDirection(DcMotor.Direction.FORWARD);
//
//        robot.mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        prompt(telemetry, "Init", "HW initialized");

        waitForStart();

        while(opModeIsActive()) {
           /*
           powFL = robot.joystick[0][1] + robot.joystick[0][0] + robot.joystick[0][2];
           powFR = robot.joystick[0][1] - robot.joystick[0][0] - robot.joystick[0][2];
           powBL = robot.joystick[0][1] + robot.joystick[0][0] - robot.joystick[0][2];
           powBR = robot.joystick[0][1] - robot.joystick[0][0] + robot.joystick[0][2];
*/
            /*
            ch1 = gamepad1.right_stick_x;
            ch3 = -gamepad1.left_stick_y;
            ch4 = gamepad1.left_stick_x;

            powFL = ch3 + ch1 + ch4;
            powFR = ch3 - ch1 - ch4;
            powBL = ch3 + ch1 - ch4;
            powBR = ch3 - ch1 + ch4;
            */

            ch1 = gamepad1.left_stick_x;
            ch2 = -gamepad1.left_stick_y;
            ch3 = gamepad1.right_stick_x;
            ch4 = -gamepad1.left_stick_y;

            robot.powFL = ch2 + ch1 + ch3;
            robot.powFR = ch2 - ch1 - ch3;
            robot.powBL = ch2 - ch1 + ch3;
            robot.powBR = ch2 + ch1 - ch3;


//            mtrFL.setPower(robot.powFL);
//            mtrFR.setPower(robot.powFR);
//            mtrBL.setPower(robot.powBL);
//            mtrBR.setPower(robot.powBR);


        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
}

