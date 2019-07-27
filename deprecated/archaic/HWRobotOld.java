package org.firstinspires.ftc.teamcode.deprecated.archaic;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gbhat on 8/20/2017.
 */

@TeleOp(name="HWMAP", group="Invis")
@Disabled
public class HWRobotOld extends LinearOpMode{
    public DcMotor motorFL = null;
    public DcMotor motorBL = null;
    public DcMotor motorFR = null;
    public DcMotor motorBR = null;
    public double[][] joystick = {
            {
                    gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y
            },
            {
                    gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x, -gamepad2.right_stick_y
            }
    };

    public boolean[][] buttons = {
            {
                    gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y,
            },
            {
                    gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y
            }
    };


    public static final double DEADZONE = 0.1;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HWRobotOld(){}

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

    @Override
    public void runOpMode(){
        //useless
    }
}
