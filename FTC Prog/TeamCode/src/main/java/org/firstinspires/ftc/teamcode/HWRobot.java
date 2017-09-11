package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gbhat on 8/20/2017.
 */

public class HWRobot {
    public DcMotor motorFL = null;
    public DcMotor motorBL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBR = null;

    public static final double DEADZONE = 0.1;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HWRobot(){}

    public void init(HardwareMap thwMap) {
        hwMap = thwMap;

        motorFL = hwMap.dcMotor.get("fl_drive");
        motorBL = hwMap.dcMotor.get("bl_drive");
        motorFR = hwMap.dcMotor.get("fr_drive");
        motorBR = hwMap.dcMotor.get("br_drive");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }
}
