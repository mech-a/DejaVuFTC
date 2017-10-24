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
    HWRobot robot = new HWRobot();
    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
    private double ch1,ch2,ch3,ch4;
    private double g2ch1, g2ch2, g2ch3, g2ch4;
    double spdLinearUp = 0.4, spdLinearDown = -spdLinearUp;

    @Override
    public void runOpMode() {
        /*//robot.init(hardwareMap);
        mtrFL = hardwareMap.dcMotor.get("fl_drive");
        mtrFR = hardwareMap.dcMotor.get("fr_drive");
        mtrBL = hardwareMap.dcMotor.get("bl_drive");
        mtrBR = hardwareMap.dcMotor.get("br_drive");

        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/

        robot.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER, telemetry);
        prompt(telemetry, "Init", "HW initialized");
        waitForStart();

        while(opModeIsActive()) {
            ch1 = gamepad1.left_stick_x;
            ch2 = -gamepad1.left_stick_y;
            ch3 = gamepad1.right_stick_x;
            ch4 = -gamepad1.left_stick_y;

            g2ch1 = gamepad2.left_stick_x;
            g2ch2 = -gamepad1.left_stick_y;
            g2ch3 = gamepad1.right_stick_x;
            g2ch4 = -gamepad1.left_stick_y;

            powFL = ch2 + ch1 + ch3;
            powFR = ch2 - ch1 - ch3;
            powBL = ch2 - ch1 + ch3;
            powBR = ch2 + ch1 - ch3;

            robot.mtrFL.setPower(powFL);
            robot.mtrFR.setPower(powFR);
            robot.mtrBL.setPower(powBL);
            robot.mtrBR.setPower(powBR);

            if(g2ch2 > 0.1) {
                robot.mtrFL.setPower(spdLinearUp);
            }
            else if(g2ch2 < 0.1 && g2ch2 > -0.1) {
                robot.mtrFL.setPower(0);
            }
            else if (g2ch2 < - 0.1) {
                robot.mtrFL.setPower(spdLinearDown);
            }

            if(g2ch3 > 0.1) {
                //robot..setPower(spdLinearUp);
            }
            else if(g2ch3 < 0.1 && g2ch3 > -0.1) {
                robot.mtrFL.setPower(0);
            }
            else if (g2ch3 < - 0.1) {
                robot.mtrFL.setPower(spdLinearDown);
            }




        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
}

