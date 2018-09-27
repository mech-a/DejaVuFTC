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

package org.firstinspires.ftc.teamcode.deprecated.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.deprecated.called.AutonHandler;
import org.firstinspires.ftc.teamcode.deprecated.called.HWRobot;

import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.ARM_JEWEL_DOWN;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.ARM_JEWEL_UP;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.EXTRUDE_CLAW_POWER;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.HITTER_JEWEL_MIDDLE;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.HITTER_JEWEL_NORTH;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SPEED_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SPEED_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SPEED_TO_TURN;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SPEED_TO_VUFORIA;


/**
 * back red
 */

@Autonomous(name="BackRedbackup", group="Blue")
//@Disabled
public class BackBlueBackup extends LinearOpMode {

    // Declare OpMode members.
    AutonHandler at = new AutonHandler();
    HWRobot r = new HWRobot();
    String team = "red";
    String area = "back";
    String vuf;

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        boolean a = opModeIsActive();
        r.getOpModeData(telemetry,hardwareMap);
        r.init("all");
        waitForStart();

        // run until the end of the match (driver presses STOP)

        a = opModeIsActive();







        jewel(team, a);


        //r.translate("fwd",0.2,1120,a);
        r.translate("fwd", SPEED_TO_VUFORIA, 10.0, a);
        sleep(500);
        //vuf = r.getVuMark(a);
        sleep(500);
        r.translate("fwd", SPEED_TO_CRYPTO, 26.0, a);
        sleep(500);
        r.rotate("cw", SPEED_TO_TURN, 90, a);
        sleep(500);
        r.moveForCrypto(vuf, a);
        sleep(500);
        r.translate("fwd", SPEED_TO_PLACE_GLYPH, 9,a);
        sleep(500);
        extrudeGlyph();
    }

    private void jewel(String teamColor, boolean active) {
        r.srvJewelArm.setPosition(ARM_JEWEL_DOWN/2);
        r.srvJewelHitter.setPosition(HITTER_JEWEL_MIDDLE);
        sleep(250);
        r.srvJewelArm.setPosition(ARM_JEWEL_DOWN);
        sleep(500);
        r.knockOffJewel(teamColor, active);
        sleep(500);
        r.srvJewelHitter.setPosition(HITTER_JEWEL_MIDDLE);
        sleep(250);
        r.srvJewelArm.setPosition(ARM_JEWEL_UP/2);
        r.srvJewelHitter.setPosition(HITTER_JEWEL_NORTH);
        sleep(250);
        r.srvJewelArm.setPosition(ARM_JEWEL_UP);
    }

    private void extrudeGlyph() {
        r.mtrClawL.setPower(EXTRUDE_CLAW_POWER);
        r.mtrClawR.setPower(EXTRUDE_CLAW_POWER);
        sleep(1000);
        r.mtrClawL.setPower(0);
        r.mtrClawR.setPower(0);
    }
}
