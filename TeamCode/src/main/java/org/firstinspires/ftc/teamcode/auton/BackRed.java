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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.called.AutonHandler;
import org.firstinspires.ftc.teamcode.called.HWRobot;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_CRYPTO_FRONT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_GET_TO_EDGE_OF_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNT_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.DEGREES_TO_TURN_FOR_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.EXTRUDE_CLAW_POWER;
import static org.firstinspires.ftc.teamcode.called.RobotValues.NEW_COUNTS_TO_CRYPTO_FRONT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_TURN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_VUFORIA;


/**
 * back red
 */

@Autonomous(name="BackRed", group="Red")
//@Disabled
public class BackRed extends LinearOpMode {

    // Declare OpMode members.
    //AutonHandler a = new AutonHandler();
    HWRobot r = new HWRobot();
    String team = "red";
    String area = "eff-back";
    String generalDirection;
    String vuf = "";

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    boolean blueTeam = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)

        //a.autonInit(telemetry,hardwareMap,this);
        r.getOpModeData(telemetry,hardwareMap,this);
        r.init("all");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                cameraMonitorViewId
        );

        parameters.vuforiaLicenseKey = r.vuforiaKey;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        if(team.toLowerCase().equals("red")) {
            blueTeam = false;
            generalDirection = "fwd";
        }
        else if(team.toLowerCase().equals("blue")) {
            blueTeam = true;
            generalDirection = "bk";
        }

        telemetry.addData("done:", "init");
        telemetry.update();
        waitForStart();

        boolean a = opModeIsActive();
        // run until the end of the match (driver presses STOP)
        //a.auton(team,area,telemetry,hardwareMap,active);
        //efficient auton
        if(area.toLowerCase().equals("back")) {
            //teamString = (blueTeam) ? "blue" : "red";
            //jewel(team, a);
            //vuf = r.getVuMark(a);
            if(a) {
                relicTrackables.activate();
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                for(int i = 1; i < 5; i++) {
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    sleep(200);
                }
                if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                    vuf = "UNKNOWN";
                }
                else if(vuMark == RelicRecoveryVuMark.LEFT) {
                    vuf = "LEFT";
                }
                else if(vuMark == RelicRecoveryVuMark.CENTER) {
                    vuf = "CENTER";
                }
                else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                    vuf = "RIGHT";
                }
            }

            r.translate(generalDirection, SPEED_TO_VUFORIA, COUNTS_TO_VUFORIA, a);
            sleep(500);
            //sleep(500);
            r.translate(generalDirection, SPEED_TO_CRYPTO, COUNT_TO_CRYPTO, a);
            sleep(500);
            r.rotate("cw", SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);
            sleep(500);
            r.moveForCrypto(vuf, a);
            sleep(500);
            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);
            sleep(500);
            extrudeGlyphStart();
            sleep(600);
            r.translate("back", 0.2, 4.0, a);
            extrudeGlyphStop();
        }
        else if (area.toLowerCase().equals("backtest")) {
            if(a) {
                r.getVuMark(a);
                r.translate(generalDirection, SPEED_TO_VUFORIA, COUNTS_TO_VUFORIA, a);
                sleep(500);
                //sleep(500);
                r.translate(generalDirection, SPEED_TO_CRYPTO, COUNT_TO_CRYPTO, a);
                sleep(500);
                r.rotate("cw", SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);
                sleep(500);
                r.moveForCrypto(a);
                sleep(500);
                r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);
                sleep(500);
                extrudeGlyphStart();
                sleep(600);
                r.translate("back", 0.2, 4.0, a);
                extrudeGlyphStop();
            }
        }

        else if (area.toLowerCase().equals("eff-back")) {
            //more efficient test code; doesnt translate left or right to complete going to crypto
            //sleep(5000);
            getVuMarkDataNotFromHWROBOT();

            telemetry.addData("passed:", "vuforia");
            telemetry.update();

            sleep(500);
            //jewel(team, a);
            //sleep(500);
            r.translate(generalDirection,SPEED_TO_CRYPTO,COUNTS_TO_GET_TO_EDGE_OF_CRYPTO, a);
            sleep(500);
            r.adjustPosForCrypto(vuf,generalDirection,team,false,a);
            sleep(500);
            //r.rotate("cw", SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);
            r.rotate("cw", SPEED_TO_TURN, 90, a);
            sleep(500);
            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);
            sleep(500);
            extrudeGlyphStart();
            sleep(600);
            r.translate("back", SPEED_TO_PLACE_GLYPH, 6.0, a);
            extrudeGlyphStop();
        }

        else if (area.toLowerCase().equals("front")) {
            //jewel(team, a);
            //vuf = r.getVuMark(a);
            if(a) {
                relicTrackables.activate();
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                for(int i = 1; i < 5; i++) {
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                }
                if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
                    vuf = "UNKNOWN";
                }
                else if(vuMark == RelicRecoveryVuMark.LEFT) {
                    vuf = "LEFT";
                }
                else if(vuMark == RelicRecoveryVuMark.CENTER) {
                    vuf = "CENTER";
                }
                else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                    vuf = "RIGHT";
                }
            }

            r.translate(generalDirection, 0.2, COUNTS_TO_VUFORIA, a);
            sleep(500);
            //sleep(500);
            r.translate(generalDirection, 0.2, COUNTS_TO_CRYPTO_FRONT, a);
            sleep(500);
            r.rotate("ccw",0.2,90,a);
            sleep(500);
            r.translate("fwd", 0.2, 12*COUNTS_PER_INCH, a);

            //TODO not sure if it'll move 90degrees more or not move
            if(blueTeam) {
                r.rotate("ccw",0.2,170,a);
                r.init("imu");
                sleep(2000);
                r.rotate("ccw",0.2,10,a);
            }
            else {
                r.rotate("cw",0.2,0.001,a);
            }


            //if perhaps we use strafing, use this code (insert after counts to crypto front
            /*
            r.translate("left", 0.2, 12 * COUNTS_PER_INCH, a);
            if (blueTeam) {
                r.rotate("ccw", 0.2, 179, a);
            }*/


            //String quickdir = (blueTeam) ? "ccw" : "cw";

            sleep(500);


            r.moveForCrypto(vuf, a);

            sleep(700);

            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
            sleep(500);
            extrudeGlyphStart();
            sleep(600);
            r.translate("back", 0.2, 4.0, a);
            extrudeGlyphStop();
        }
        else if (area.toLowerCase().equals("fronttest")) {
            if(a) {
                r.getVuMark(a);
                r.translate(generalDirection, 0.2, COUNTS_TO_VUFORIA, a);
                sleep(500);
                //sleep(500);
                r.translate(generalDirection, 0.2, COUNTS_TO_CRYPTO_FRONT, a);
                sleep(500);
                r.rotate("ccw",0.2,90,a);
                sleep(500);
                r.translate("fwd", 0.2, 12*COUNTS_PER_INCH, a);

                //TODO not sure if it'll move 90degrees more or not move
                if(blueTeam) {
                    r.rotate("ccw",0.2,170,a);
                    r.init("imu");
                    sleep(2000);
                    r.rotate("ccw",0.2,10,a);
                }
                else {
                    r.rotate("cw",0.2,0.001,a);
                }
                sleep(500);
                r.moveForCrypto(a);

                sleep(700);

                r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
                sleep(500);
                extrudeGlyphStart();
                sleep(600);
                r.translate("back", 0.2, 3.0, a);
                extrudeGlyphStop();

            }
        }

        else if (area.toLowerCase().equals("eff-front")) {
            //efficient auton
            //sleep(5000);
            getVuMarkDataNotFromHWROBOT();

            telemetry.addData("passed:", "vuforia");
            telemetry.update();

            //sleep(500);
            //jewel(team, a);
            sleep(500);
            r.translate(generalDirection,SPEED_TO_VUFORIA,NEW_COUNTS_TO_CRYPTO_FRONT, a);
            sleep(500);
            r.rotate("ccw",SPEED_TO_TURN, 90, a);
            sleep(500);
            r.adjustPosForCrypto(vuf,generalDirection,team,true,a);
            sleep(500);
            if(blueTeam) {
                r.init("imu");
                r.rotate("ccw", SPEED_TO_TURN, 90, a);
            }
            else if (!blueTeam) {
                r.init("imu");
                r.rotate("cw", SPEED_TO_TURN, 90, a);
            }
            sleep(500);
            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
            sleep(500);
            extrudeGlyphStart();
            sleep(500);
            r.translate("back", SPEED_TO_PLACE_GLYPH, 6.0, a);
            sleep(500);
            extrudeGlyphStop();
        }

        else if (area.toLowerCase().equals("jewel")) {
            //jewel(team, a);
        }
        else if (area.toLowerCase().equals("vuf strafe")) {
            r.moveForCrypto("LEFT", a);
            r.moveForCrypto("RIGHT", a);
        }
    }

    private void extrudeGlyphStart() {
        r.mtrClawL.setPower(EXTRUDE_CLAW_POWER);
        r.mtrClawR.setPower(EXTRUDE_CLAW_POWER);
        sleep(1000);
    }

    private void extrudeGlyphStop() {
        r.mtrClawL.setPower(0);
        r.mtrClawR.setPower(0);
    }

    private void getVuMarkDataNotFromHWROBOT() {
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        for(int i = 1; i < 10; i++) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(200);
        }
        if(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            vuf = "UNKNOWN";
        }
        else if(vuMark == RelicRecoveryVuMark.LEFT) {
            vuf = "LEFT";
        }
        else if(vuMark == RelicRecoveryVuMark.CENTER) {
            vuf = "CENTER";
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT) {
            vuf = "RIGHT";
        }
    }
}
