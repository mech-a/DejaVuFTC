package org.firstinspires.ftc.teamcode.deprecated.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by gbhat on 8/20/2017.
 */

@TeleOp(name="Holonomic Drive 4 Motors", group="DriveOPs")
//@Disabled
public class HolonomicDrive4Motors extends LinearOpMode{
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR;
    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
    private double ch1,ch2,ch3,ch4;
    private double g2ch1, g2ch2, g2ch3, g2ch4;
    double spdLinearUp = 1, spdLinearDown = -spdLinearUp;
    double clawMid = 0.5;
    double clawIncrement = 0.01;
    double clawPos;
    private boolean runSlow = false;
    private boolean runFast = false;
    private boolean linearRun = false;
    private boolean linUp = false;
    private boolean linDown = false;
    private double slowLimit = 0.25;
    private double fastLimit = 0.75;
    private double modifierValueDefault = 0.5;
    private double modifierValue = modifierValueDefault;

    @Override
    public void runOpMode() {

        mtrFL = hardwareMap.dcMotor.get("fl_drive");
        mtrFR = hardwareMap.dcMotor.get("fr_drive");
        mtrBL = hardwareMap.dcMotor.get("bl_drive");
        mtrBR = hardwareMap.dcMotor.get("br_drive");


        // Set directions for motors.
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);


        //zero power behavior
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set power for all motors.
        mtrFL.setPower(powFL);
        mtrFR.setPower(powFR);
        mtrBL.setPower(powBL);
        mtrBR.setPower(powBR);


        // Set all motors to run with given mode
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {


            setChannels();
            setPowers();

            //TODO make better solution for speed switching
            speedSwitch();

            powFL *= modifierValue;
            powFR *= modifierValue;
            powBL *= modifierValue;
            powBR *= modifierValue;


            telemetry.addData("modifier value", modifierValue);
            //telemetry.addData("RunSlow", runSlow);

            telemetry.update();

            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);
            sleep(125);
        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
    private void setChannels() {
        ch1 = gamepad1.left_stick_x;
        ch2 = -gamepad1.left_stick_y;
        ch3 = gamepad1.right_stick_x;
        ch4 = -gamepad1.left_stick_y;

        g2ch1 = gamepad2.left_stick_x;
        g2ch2 = -gamepad2.left_stick_y;
        g2ch3 = gamepad2.right_stick_x;
        g2ch4 = -gamepad2.left_stick_y;
    }

    private void setPowers() {
        powFL = ch2 + ch1 + ch3;
        powFR = ch2 - ch1 - ch3;
        powBL = ch2 - ch1 + ch3;
        powBR = ch2 + ch1 - ch3;

    }

    private void speedSwitch() {
        if(gamepad1.left_bumper) {
            runSlow = true;
        }
        else if(gamepad1.right_bumper) {
            runFast = true;
        }

        if(runSlow) {
            runFast = false;
            runSlow = false;
            if(modifierValue == modifierValueDefault) {
                modifierValue = slowLimit;
            }
            else if(modifierValue == slowLimit || modifierValue == fastLimit) {
                modifierValue = modifierValueDefault;
            }
        }

        if(runFast) {
            runFast = false;
            runSlow = false;
            if(modifierValue == modifierValueDefault) {
                modifierValue = fastLimit;
            }
            else if(modifierValue == slowLimit || modifierValue == fastLimit) {
                modifierValue = modifierValueDefault;
            }
        }
    }

}

