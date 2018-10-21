/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * Copy Me Linear
 */

@TeleOp(name="POVDrive4Motorstest", group="DriveOPs")
//@Disabled
public class POVDrive4Motorstest extends LinearOpMode {

    // Declare OpMode members.
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR;
    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
    private double ch2,ch3;
    private double epsilon = 0.1;
    private double count = 0;
    private double oldJoystickCh2;
    private double oldJoystickCh3;
    private double power = 0;
    private double joystick;
    private double numSteps = 20;



    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

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
        //mtrFL.setPower(powFL);
        //mtrFR.setPower(powFR);
        //mtrBL.setPower(powBL);
        //mtrBR.setPower(powBR);


        // Set all motors to run with given mode
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ch2 = -gamepad1.left_stick_y/2;
            ch3 = gamepad1.right_stick_x/2;

            if(Math.abs(ch2-power)>epsilon)
                power += (ch2-power)/numSteps;

            powFL = Range.clip(power + ch3, -1, 1);
            powFR = Range.clip(power - ch3, -1, 1);
            powBL = Range.clip(power + ch3, -1, 1);
            powBR = Range.clip(power - ch3, -1, 1);

            if(count % 5 == 0)
                oldJoystickCh2 = ch2;

            if(Math.abs(oldJoystickCh2) < epsilon && Math.abs(ch2) < epsilon)
                power = 0;



            telemetry.addData("Joystick", joystick);
            telemetry.addData("power", power);

            count++;

            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);

            telemetry.update();
            sleep(50);
        }
    }


    public void setChannels() {
        ch2 = -gamepad1.left_stick_y ;
        ch3 = Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x ;
    }

    public void setPowers() {

        powFL = Range.clip(ch2 + ch3, -1, 1);
        powFR = Range.clip(ch2 - ch3, -1, 1);
        powBL = Range.clip(ch2 + ch3, -1, 1);
        powBR = Range.clip(ch2 - ch3, -1, 1);
    }
}