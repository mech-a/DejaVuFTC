package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
public class STEMCodeTrainingPOV extends LinearOpMode {

    public DcMotor mtr0;
    public DcMotor mtr1;
    public DcMotor mtr2;
    public DcMotor mtr3;
    public double leftPower;
    public double rightPower;
    public double drive;
    public double turn;



    public void runOpMode(){

        mtr0 = hardwareMap.dcMotor.get("motor_0");
        mtr0.setDirection(DcMotor.Direction.FORWARD);
        mtr0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtr1 = hardwareMap.dcMotor.get("motor_1");
        mtr1.setDirection(DcMotor.Direction.REVERSE);
        mtr1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtr2 = hardwareMap.dcMotor.get("motor_2");
        mtr2.setDirection(DcMotor.Direction.REVERSE);
        mtr2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mtr3 = hardwareMap.dcMotor.get("motor_3");
        mtr3.setDirection(DcMotor.Direction.FORWARD);
        mtr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            leftPower = drive + turn;
            rightPower = drive - turn;

            mtr0.setPower(leftPower);
            mtr3.setPower(leftPower);
            mtr1.setPower(rightPower);
            mtr2.setPower(rightPower);

            sleep(50);

        }
    }

}
