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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.deprecated.called.HWRobot;


/**
 * Copy Me Linear
 */

@TeleOp(name="MultiTester", group="Internal")
//@Disabled
public class MultiTest extends LinearOpMode {

    // Declare OpMode members.
    HWRobot r = new HWRobot();
    double powClaws = 0;
    double powFL,powFR,powBL,powBR,powLin;
    double epsilon = 0.01;
    boolean a;


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        r.getOpModeData(telemetry,hardwareMap);
        r.init("all");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            a = opModeIsActive();

            if(gamepad1.left_bumper && gamepad1.right_stick_button) {
                powLin-=epsilon;
                powLin = Range.clip(powLin, -1, 1);
            }
            else if(gamepad1.left_bumper){}


            else if(gamepad1.right_bumper) {
                r.rotate("cw", 0.2, 90,a);
            }


            telemetryBlock();





        }
    }
    private void telemetryBlock() {
        telemetry.addData("Arm Position", r.srvJewelArm.getPosition());
        telemetry.addData("Hitter Position", r.srvJewelHitter.getPosition());
        telemetry.addLine()
                .addData("Hue", r.hsv[0])
                .addData("Saturation", r.hsv[1])
                .addData("Value", r.hsv[2]);
        telemetry.addData("Vumark", r.vuf);
        telemetryMotors();
    }

    private void telemetryMotors() {
        telemetry.addLine()
                .addData("mtrFL: power", powFL)
                .addData("counts: ", r.mtrFL.getCurrentPosition());
        telemetry.addLine()
                .addData("mtrBL: power", powBL)
                .addData("counts: ", r.mtrBL.getCurrentPosition());
        telemetry.addLine()
                .addData("mtrFR: power", powFR)
                .addData("counts: ", r.mtrFR.getCurrentPosition());
        telemetry.addLine()
                .addData("mtrBR: power", powBR)
                .addData("counts: ", r.mtrBR.getCurrentPosition());
        telemetry.addLine()
                .addData("mtrLinear: power", powLin)
                .addData("counts: ", r.mtrLinear.getCurrentPosition());
        telemetry.addData("claw powers", powClaws);
    }

}
