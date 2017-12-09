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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.called.HWRobot;


/**
 * Copy Me Linear
 */

@TeleOp(name="Values", group="Internal")
//@Disabled
public class ValueTester extends LinearOpMode {

    // Declare OpMode members.
    //HWRobot r = new HWRobot();
    double currentPosArm = 0;
    double currentPosHitter = 0;

    public Servo srvArmLeftJewel, srvHitterLeftJewel;

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        //r.getOpModeData(telemetry,hardwareMap);r.init("servos");
        srvArmLeftJewel = hardwareMap.servo.get("arm_left_jewel");
        srvHitterLeftJewel = hardwareMap.servo.get("hitter_left_jewel");
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_bumper && gamepad1.a) {
                currentPosArm+=0.01;
            }
            else if(gamepad1.left_bumper) {
                currentPosArm-=0.01;
            }

            if (gamepad1.right_bumper && gamepad1.a) {
                currentPosHitter+=0.01;
            }
            else if(gamepad1.right_bumper) {
                currentPosHitter-=0.01;
            }
            currentPosArm = Range.clip(currentPosArm, -1, 1);
            currentPosHitter = Range.clip(currentPosHitter, -1, 1);

            srvArmLeftJewel.setPosition((currentPosArm));
            srvHitterLeftJewel.setPosition(currentPosHitter);

            //r.srvIntakeL.setPower(currentPowerL);
            //r.srvIntakeR.setPower(-currentPowerR);

            telemetry.addData("srvArm pos", currentPosArm);
            telemetry.addData("srvHitter pos", currentPosHitter);
            telemetry.update();
            sleep(50);
        }
    }
}