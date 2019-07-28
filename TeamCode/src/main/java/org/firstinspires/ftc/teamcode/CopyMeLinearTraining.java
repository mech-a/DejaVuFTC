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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Copy Me Linear
 */

@TeleOp(name="CopyMe", group="Internal")
@Disabled
public class CopyMeLinearTraining extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftBottomDrive;
    private DcMotor rightBottomDrive;
    private DcMotor leftTopDrive;
    private DcMotor rightTopDrive;


    @Override
    public void runOpMode() {

        leftBottomDrive  = hardwareMap.get(DcMotor.class, "left_bottom_drive");
        rightBottomDrive = hardwareMap.get(DcMotor.class, "right_bottom_drive");
        leftTopDrive  = hardwareMap.get(DcMotor.class, "left_top_drive");
        rightTopDrive = hardwareMap.get(DcMotor.class, "right_top_drive");

        leftBottomDrive.setDirection(DcMotor.Direction.FORWARD);
        leftTopDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBottomDrive.setDirection(DcMotor.Direction.REVERSE);
        rightTopDrive.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            double slowSpeedRatio = 0.25, defaultSpeedRatio = 0.5, fastSpeedRatio = 0.75;

            double leftDrive = -gamepad1.left_stick_y;
            double rightDrive = -gamepad1.right_stick_y;
            boolean fastButton = gamepad1.a;
            boolean slowButton = gamepad1.b;

            boolean fastToggled = false;
            boolean slowToggled = false;

            double modifier;

            if (fastButton==true && slowButton==true){

            }

            else if (fastButton==true) {
                
                if (slowToggled==false) {
                    fastToggled = !fastToggled;
                }

                else {
                    slowToggled = !slowToggled;
                }

            }

            else if (slowButton==true) {

                if (fastToggled==false) {
                    slowToggled = !slowToggled;
                }

                else {
                    fastToggled = !fastToggled;
                }
            }

            if (fastToggled) {
                modifier = fastSpeedRatio;
            }

            else if (slowToggled) {
                modifier = slowSpeedRatio;
            }

            else {
                modifier = defaultSpeedRatio;
            }

            leftBottomDrive.setPower(leftDrive*modifier);
            leftTopDrive.setPower(leftDrive*modifier);
            rightBottomDrive.setPower(rightDrive*modifier);
            rightTopDrive.setPower(rightDrive*modifier);
        }
    }
}
