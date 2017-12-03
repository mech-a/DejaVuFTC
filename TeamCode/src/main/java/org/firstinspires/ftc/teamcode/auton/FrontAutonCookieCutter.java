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

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.called.HWRobot;

import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA_FRONT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.JEWEL_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.JEWEL_SERVO_UP;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_PLACE_GLYPH;


/**
 * Copy Me Linear
 */

@TeleOp(name="Front Auton Cookie Cutter", group="testing")
@Disabled
public class FrontAutonCookieCutter extends LinearOpMode {

    // Declare OpMode members.
    HWRobot r = new HWRobot();
    boolean a;
    double heading = 0;
    String vuf = null;
    static String TEAM = "";


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        r.getOpModeData(telemetry,hardwareMap);r.init("all");
        a = opModeIsActive();
        getHeading();

        waitForStart();
        //TODO finish front auton
        // run until the end of the match (driver presses STOP)
        //Release servo latch and read the color sensor
        r.jewelServoFlip(JEWEL_SERVO_DOWN);
        r.refreshHSV();
        //knock off jewel
        r.knockOffJewel(TEAM,a);
        r.jewelServoFlip(JEWEL_SERVO_UP);

        r.translate("fwd", 0.2, COUNTS_TO_VUFORIA_FRONT, a);
        sleep(500);
        vuf = r.getVuMark(a);

        r.moveForCrypto(vuf, a);

        r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
        //r.releaseClaw();
    }

    private void getHeading() {
        heading = AngleUnit.DEGREES.fromUnit(r.angles.angleUnit, r.angles.firstAngle);
    }
}
