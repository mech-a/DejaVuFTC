package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.called.HWRobot;

/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@Autonomous(name="Back", group="Comp")
//@Disabled
public class JustTranslation extends LinearOpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();
    int countsPerInch = 252;
    int inToSZ = 20;


    @Override
    public void runOpMode() {
        robot.getOpModeData(telemetry,hardwareMap);robot.init("motors");

        //Translate forward at a speed of 0.6 1120 *COUNTS* while a
        //robot.translate("fwd", 0.6, 1120, a);

        //Translate forward at a speed of 0.6 12 *INCHES* while a
        //robot.translate("left", 0.4, 12.0, a);

        //Translate (rotate) clockwise using counts at a speed of 0.2 ninety degrees while a
        //DO NOT USE THIS THERE IS A BUG
        //robot.translate("cw", 0.2, robot.ninetyDegreesInInches, a);

        //Rotate clockwise using gyro 90 degrees at a speed of 0.2 while a
        //robot.rotate("cw", 0.2, 90, a);

        //Rotate counter clockwise 135 degrees at a speed of 0.3 while a
        //robot.rotate("ccw", 0.3, 135, a);
        boolean a = opModeIsActive();
        waitForStart();


        //Code used in competition
        //robot.translate("fwd", 0.2, countsPerInch * inToSZ, a);
        //sleep(250);




//        robot.translate("fwd", 0.2, 2 * 1120, a);
//        sleep(250);
//        robot.translate("cw", 0.2, robot.ninetyDegreesInInches, a);
//        sleep(250);
//        robot.translate("ccw", 0.2, robot.turnAroundInInches, a);
//        sleep(250);

        // kalie u did rotate(cw, 0.3, and countsPerInch *InToSZ <- ?? u did rotate) - gaurav
        //robot.translate("fwd", 0.3, countsPerInch * inToSZ, a);
        //sleep(250);
        robot.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.mtrFL.setTargetPosition(1120);
        robot.mtrFR.setTargetPosition(1120);
        robot.mtrBL.setTargetPosition(1120);
        robot.mtrBR.setTargetPosition(1120);


        robot.mtrFL.setPower(0.2);
        robot.mtrFR.setPower(0.2);
        robot.mtrBL.setPower(0.2);
        robot.mtrBR.setPower(0.2);
        while(robot.mtrFL.isBusy() && robot.mtrFR.isBusy() && robot.mtrBL.isBusy() && robot.mtrBR.isBusy()) {
            telemetry.addData("MTRFL", robot.mtrFL.getCurrentPosition());
            telemetry.addData("MTRFR", robot.mtrFR.getCurrentPosition());
            telemetry.addData("MTRBL", robot.mtrBL.getCurrentPosition());
            telemetry.addData("MTRBR", robot.mtrBR.getCurrentPosition());
            telemetry.update();
        }

        robot.mtrSetSpeed(0);

    }
}
