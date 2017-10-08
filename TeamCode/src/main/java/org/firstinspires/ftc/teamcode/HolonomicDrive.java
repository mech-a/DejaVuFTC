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
    //HWRobot robot = new HWRobot();
    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
    private double ch1,ch2,ch3,ch4;
    private DcMotor mtrFL, mtrFR, mtrBL, mtrBR;


    @Override
    public void runOpMode() {
        //robot.init(hardwareMap);
        mtrFL = hardwareMap.dcMotor.get("fl_drive");
        mtrFR = hardwareMap.dcMotor.get("fr_drive");
        mtrBL = hardwareMap.dcMotor.get("bl_drive");
        mtrBR = hardwareMap.dcMotor.get("br_drive");

        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBR.setDirection(DcMotor.Direction.REVERSE);

        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            powFL = ch2 + ch1 + ch3;
            powFR = ch2 - ch1 - ch3;
            powBL = ch2 - ch1 + ch3;
            powBR = ch2 + ch1 - ch3;


            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);


        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
}

