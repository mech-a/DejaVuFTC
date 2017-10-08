package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HWRobotNew
{
    /* Public OpMode members. */
    private DcMotor mtrFL, mtrFR, mtrBL, mtrBR;

    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    // Initialize standard Hardware interfaces.
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map.
        hwMap = ahwMap;

        // Define and initialize motors.
        mtrFL = ahwMap.dcMotor.get("fl_drive");
        mtrFR = ahwMap.dcMotor.get("fr_drive");
        mtrBL = ahwMap.dcMotor.get("bl_drive");
        mtrBR = ahwMap.dcMotor.get("br_drive");

        // Set directions for motors.
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        // Set power for all motors.
        mtrFL.setPower(powFL);
        mtrFR.setPower(powFR);
        mtrBL.setPower(powBL);
        mtrBR.setPower(powBR);

        // Set all motors to run without encoders.
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

