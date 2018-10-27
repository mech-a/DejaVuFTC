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

@TeleOp(name="POVDrive4Motorstest2", group="DriveOPs")
//@Disabled
public class POVDrive4Motorstest2 extends LinearOpMode {

    // Declare OpMode members.
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR, mtrV,mtrI;
    public DcMotor mtrFL, mtrFR, mtrBL, mtrBR, mtrV,mtrI, mtrA,mtrH;
    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
    private double g1ch2,g1ch3, g2ch2, g2ch3, g2left, g2right;
    private double epsilon = 0.1;
    private double count = 0;
    private double oldJoystickCh2 = 0;
    private double oldJoystickCh3;
    private double power = 0;
    private double joystick;
    private double numSteps = 10;
    private double numStepsHigh = numSteps;
    private double numStepsLow = numSteps/2;
    private int modifier = 2;
    private double change;
    private double limitedch2 = 0.5;
    private double limit = 0.1;
    private double increase= 0.1;
    private boolean g2a;
    private int mtrVticks;
    private double mtrVpower;
    private int buttoncount;



    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        mtrFL = hardwareMap.dcMotor.get("fl_drive");
        mtrFR = hardwareMap.dcMotor.get("fr_drive");
        mtrBL = hardwareMap.dcMotor.get("bl_drive");
        mtrBR = hardwareMap.dcMotor.get("br_drive");
        mtrV= hardwareMap.dcMotor.get("raise_motor");
        mtrI= hardwareMap.dcMotor.get("intake_motor");
        mtrA =  hardwareMap.dcMotor.get("arm_motor");
        mtrH=  hardwareMap.dcMotor.get("telescoping_motor");


        // Set directions for motors.
        mtrFL.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);
        mtrV.setDirection(DcMotor.Direction.FORWARD);
        mtrI.setDirection(DcMotor.Direction.FORWARD);



        //zero power behavior
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set power for all motors.
        //mtrFL.setPower(powFL);
        //mtrFR.setPower(powFR);
        //mtrBL.setPower(powBL);
        //mtrBR.setPower(powBR);
       mtrV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Set all motors to run with given mode
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrI.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrV.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //moving the robot
            g1ch2 = -gamepad1.left_stick_y/modifier;
            g1ch3 = gamepad1.right_stick_x/modifier;
            //
            g2ch2 = -gamepad2.left_stick_y/modifier;
            g2ch3 = gamepad2.right_stick_x/modifier;
            // trigger for intake
            g2left = gamepad2.left_trigger;
            g2right = gamepad2.right_trigger;
            g2a = gamepad2.a;
            /*

            change = g1ch2 - limitedch2;
            if(change > limit){
                change = limit;

            }
            else if(change < limit){
                change = -limit;
            }
            limitedch2 += change;

            if(count % 2 == 0){
                oldJoystickCh2 = ch2;
            }
            increase = (oldJoystickCh2 + ch2)/2
            power += increase

            or it could be

            power += a(previousjoystick)- (1-a)currenjoystick
            */










            powFL = Range.clip(g1ch2 + g1ch3, -1, 1);
            powFR = Range.clip(g1ch2 - g1ch3, -1, 1);
            powBL = Range.clip(g1ch2 + g1ch3, -1, 1);
            powBR = Range.clip(g1ch2 - g1ch3, -1, 1);

            count++;
            if(g2right > 0){
                mtrI.setPower(g2right);
            }
            else{
                mtrI.setPower(0.0);
            }
            if (g2left > 0){
                mtrI.setPower(-g2left);
            }
            else{
                mtrI.setPower(0.0);
            }
            if(gamepad2.dpad_up){
                mtrV.setTargetPosition(100);
                mtrV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrV.setPower(0.10);
            }
            else if(gamepad2.dpad_down){
                mtrV.setTargetPosition(-100);
                mtrV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mtrV.setPower(0.10);
            }

            if (gamepad2.y)
                mtrA.setPower(0.1);
            else if (gamepad2.a)
                mtrA.setPower(-0.1);
            else
                mtrA.setPower(0.0);

            mtrH.setPower(g2ch2);














            mtrFL.setPower(powFL);
            mtrFR.setPower(powFR);
            mtrBL.setPower(powBL);
            mtrBR.setPower(powBR);


            sleep(50);
        }
    }


    public void setChannels() {
        g1ch2 = -gamepad1.left_stick_y ;
        g1ch3 = Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x ;
    }

    public void setPowers() {

        powFL = Range.clip(g1ch2 + g1ch3, -1, 1);
        powFR = Range.clip(g1ch2 - g1ch3, -1, 1);
        powBL = Range.clip(g1ch2 + g1ch3, -1, 1);
        powBR = Range.clip(g1ch2 - g1ch3, -1, 1);
    }
}
