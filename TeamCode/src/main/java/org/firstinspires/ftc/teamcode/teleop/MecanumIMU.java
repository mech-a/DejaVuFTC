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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Enums.*;
import org.firstinspires.ftc.teamcode.dependencies.Robot;



/**
 * MecanumIMU
 * Read more about the code on https://www.chiefdelphi.com/media/papers/2390
 */

@TeleOp(name="MecanumIMU", group="Teleops")
@Disabled
public class MecanumIMU extends LinearOpMode {

    // Declare OpMode members.
    Robot r = new Robot(this, OpModeType.TELEOP);

    double[] g1 = new double[4];

    double modifier = 0.125;

    double powFL, powFR, powBR, powBL;

    double fwd, strafe, rotate;


    public enum DriveMode {
        FIELD, ROBOT
    }

    DriveMode driveMode = DriveMode.FIELD;

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        r.start(hardwareMap,telemetry);
        r.init();


        waitForStart();

        // run until the end of the match (driver presses STOP)
        //TODO create teleop parent class that extends linearopmode w gamepad f(x)s

        while (opModeIsActive()) {
            //basic mecanum driving that is respective to the robot
            setGamepads();

            mecanum();

            if(gamepad1.y) {
                driveMode = DriveMode.ROBOT;
            }
            else if (gamepad1.a || driveMode == DriveMode.FIELD) {
                driveMode = DriveMode.FIELD;
            }

            telemetry.addData("Mode", driveMode.toString());

            setPowers();

            telemetry.update();

            sleep(50);
        }
    }

    private void mecanum() {
        if(driveMode == DriveMode.FIELD) {

            double heading = Math.toRadians(r.getHeading());
            if(heading > 0) {
                //ccw
                fwd = g1[1] * Math.cos(heading) - g1[0] * Math.sin(heading);
                strafe = g1[1] * Math.cos(heading) + g1[0] * Math.sin(heading);
            }
            else {
                //cw
                fwd = g1[1] * Math.cos(heading) + g1[0] * Math.sin(heading);
                strafe = -g1[1] * Math.cos(heading) + g1[0] * Math.sin(heading);
            }

            rotate = g1[2];

            powFL = fwd + rotate + strafe;
            powFR = fwd - rotate - strafe;
            powBL = fwd + rotate - strafe;
            powBR = fwd - rotate + strafe;

        }

        else {

            powFL = g1[1] + g1[2] + g1[0];
            powFR = g1[1] - g1[2] - g1[0];
            powBL = g1[1] + g1[2] - g1[0];
            powBR = g1[1] - g1[2] + g1[0];

        }
    }

    private void setGamepads() {
        //left joystick x, y, right joystick x, y
        g1[0] = gamepad1.left_stick_x * modifier;
        g1[1] = -gamepad1.left_stick_y * modifier;
        g1[2] = gamepad1.right_stick_x * modifier;
        g1[3] = -gamepad1.right_stick_y * modifier;
    }

    private void setPowers() {
        r.driveMotors[0].setPower(powFL);
        r.driveMotors[1].setPower(powFR);
        r.driveMotors[2].setPower(powBR);
        r.driveMotors[3].setPower(powBL);
    }



}