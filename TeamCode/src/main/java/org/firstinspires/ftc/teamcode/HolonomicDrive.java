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
<<<<<<< HEAD
    //HWRobotOld robot = new HWRobotOld();
    HWRobot robot = new HWRobot();
=======
    HWRobot robot = new HWRobot();
    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
>>>>>>> 18bcdc468f7a255fda503bc18ad0e336e41bdad0
    private double ch1,ch2,ch3,ch4;



    @Override
    public void runOpMode() {
<<<<<<< HEAD
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
=======
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
>>>>>>> 18bcdc468f7a255fda503bc18ad0e336e41bdad0

        robot.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER, telemetry);
        prompt(telemetry, "Init", "HW initialized");
        waitForStart();

        while(opModeIsActive()) {
            ch1 = gamepad1.left_stick_x;
            ch2 = -gamepad1.left_stick_y;
            ch3 = gamepad1.right_stick_x;
            ch4 = -gamepad1.left_stick_y;

            robot.powFL = ch2 + ch1 + ch3;
            robot.powFR = ch2 - ch1 - ch3;
            robot.powBL = ch2 - ch1 + ch3;
            robot.powBR = ch2 + ch1 - ch3;

<<<<<<< HEAD

//            mtrFL.setPower(robot.powFL);
//            mtrFR.setPower(robot.powFR);
//            mtrBL.setPower(robot.powBL);
//            mtrBR.setPower(robot.powBR);


=======
            robot.mtrFL.setPower(powFL);
            robot.mtrFR.setPower(powFR);
            robot.mtrBL.setPower(powBL);
            robot.mtrBR.setPower(powBR);
>>>>>>> 18bcdc468f7a255fda503bc18ad0e336e41bdad0
        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
}

