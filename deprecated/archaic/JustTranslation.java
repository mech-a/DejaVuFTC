package org.firstinspires.ftc.teamcode.deprecated.archaic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@Autonomous(name="NewAut", group="Comp")
//@Disabled
public class JustTranslation extends LinearOpMode{
    // Declare OpMode members.
    int countsPerInch = 45;
    int inToSZ = 20;
    DcMotor mtrFL;
    DcMotor mtrBL;
    DcMotor mtrFR;
    DcMotor mtrBR;


    @Override
    public void runOpMode() {
        // getOpModeData(telemetry,hardwareMap); init("motors");
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
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        
         mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         mtrFL.setTargetPosition(10000);
         mtrFR.setTargetPosition(10000);
         mtrBL.setTargetPosition(10000);
         mtrBR.setTargetPosition(10000);


         mtrFL.setPower(0.25);
         mtrFR.setPower(0.25);
         mtrBL.setPower(0.25);
         mtrBR.setPower(0.25);
        while( mtrFL.isBusy() &&  mtrFR.isBusy() &&  mtrBL.isBusy() &&  mtrBR.isBusy() && opModeIsActive()) {
            telemetry.addData("MTRFL",  mtrFL.getCurrentPosition());
            telemetry.addData("MTRFR",  mtrFR.getCurrentPosition());
            telemetry.addData("MTRBL",  mtrBL.getCurrentPosition());
            telemetry.addData("MTRBR",  mtrBR.getCurrentPosition());
            telemetry.update();
        }

         mtrFL.setPower(0);
         mtrFR.setPower(0);
         mtrBL.setPower(0);
         mtrBR.setPower(0);

    }
}
