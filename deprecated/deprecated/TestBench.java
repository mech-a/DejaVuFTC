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

package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Copy Me Linear
 */

@TeleOp(name="claw stuff", group="Internal")
//@Disabled
public class TestBench extends LinearOpMode {

    // Declare OpMode members.
    DcMotor clawL = null;
    DcMotor clawR = null;
    DcMotor lift = null;
    double powClawL = 0;
    double powClawR = 0;
    double powLift = 0;
    double clawSpeed = 0.5;
    double liftSpeed = 1;


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        clawL = hardwareMap.dcMotor.get("left_claw");
        clawR = hardwareMap.dcMotor.get("right_claw");
        lift = hardwareMap.dcMotor.get("lift_motor");

        clawL.setDirection(DcMotor.Direction.REVERSE);
        clawR.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        clawL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawL.setPower(powClawL);
        clawR.setPower(powClawR);
        lift.setPower(powLift);

        clawL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a && gamepad1.left_bumper) {
                powClawL = -clawSpeed;
                powClawR = -clawSpeed;
            }
            else if (gamepad1.a) {
                powClawL = clawSpeed;
                powClawR = clawSpeed;
            }
            else {
                powClawL = 0;
                powClawR = 0;
            }

            if(gamepad1.x) {
                powLift = -liftSpeed;
            }
            else if (gamepad1.y) {
                powLift = liftSpeed;
            }
            else {
                powLift = 0;
            }



            telemetry.addData("Pow ClawL:", powClawL);
            telemetry.addData("Pow ClawR:", powClawR);
            telemetry.addData("Pow Lift:", powLift);
            clawL.setPower(powClawL);
            clawR.setPower(powClawR);
            lift.setPower(powLift);
            telemetry.update();
            sleep(375);
        }
    }
}
