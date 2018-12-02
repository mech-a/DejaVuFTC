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

package org.firstinspires.ftc;

import android.os.WorkSource;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


/**
 * CraterSideAuton
 * The CraterSideAuton program is the code and instructions for
 * the robot based on starting on the Crater side of the lander.
 *
 * @author Gaurav
 * @version 1.0
 * @since 2018-10-29
 */

@Autonomous(name="Crater Side Auton", group="Internal")
//@Disabled
public class CraterSideAuton extends LinearOpMode {
    /**
     * This class extends the "Robot" class, a dependencies
     * class that holds motor and rotational control
     */
    Robot r = new Robot(this);

    boolean left = false;
    boolean middle = false;
    boolean right = false;
    //TODO better names
    String directionForMineralFirst = "";
    String directionForMineralSecond = "";
    double distanceForLineUp = 0;

    double baseDistance = 30;

    double inchesToDepot = 60;
    double inchesToCrater = 70;

    String samplePos = "center";

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        r.start(hardwareMap, telemetry);
        r.init();
        r.cvInit();

        //r.detectorInit();
        //for center
        //24 inches backward
        //6 inches forward
        //rotate 90 ccw
        //drive backwards 43 in
        //rotate 135 ccw
        //39 inches backward
        //60 +/-3 inches forward
        //drop arm

        //for left
        //rotate 26 ccw
        //25.5 inches backwards
        //rotate 105 ccw
        //33 inches backwards
        //rotate 135 ccw
        //37 inches backwards
        //61 inches forward

        //for right
        //rotate 26 cw
        //25.5 inches backwards
        //6 inches forward
        //rotate 90 ccw
        //50 inches backwards
        //rotate 135 ccw
        //40 inches backwards
        //59 inches forward

        waitForStart();
        r.armMotors[0].setPower(-0.25);

        r.positionDrive(2, -40, 0.2);


        sleep(350);
        r.servoMotors[1].setPosition(0.35);
        sleep(350);


        //drop from lander

        r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 840) {}
        r.armMotors[0].setPower(0);

        //position to read

        r.strafe(4, 0.1);
        sleep(100);
        r.translate(-5, 0.15);
        sleep(100);
        r.strafe(-6, 0.1);
        sleep(100);

        //read samples

        if (r.getGoldPosition() == Enums.GoldPosition.RIGHT) {
            samplePos = "right";
        } else if (r.getGoldPosition() == Enums.GoldPosition.MIDDLE) {
            samplePos = "center";
        } else if (r.getGoldPosition() == Enums.GoldPosition.LEFT) {
            samplePos = "left";
        }

        r.translate(-6, 0.15);
        sleep(500);
        // movement for knocking off jewel
        if(samplePos == "right") {
            r.strafe(-12, 0.1);
            sleep(100);
            r.translate(-10, 0.1);
            sleep(100);
            r.translate(10, 0.1);
            sleep(100);
            r.rotate("ccw", 0.1, 90);
            sleep(100);
            r.translate(-48, 0.1);
            sleep(100);
        } else if (samplePos == "center") {
            //r.strafe(4, 0.4);
            sleep(100);
            r.translate(-10, 0.1);
            sleep(100);
            r.translate(10, 0.1);
            sleep(100);
            r.rotate("ccw", 0.1, 90);
            sleep(100);
            r.translate(-36, 0.1);
            sleep(100);
        } else {
            r.strafe(16, 0.1);
            sleep(100);
            r.translate(-10, 0.1);
            sleep(100);
            r.translate(10, 0.1);
            sleep(100);
            r.rotate("ccw", 0.1, 90);
            sleep(100);
            r.translate(-20, 0.1);
            sleep(100);
        }
        // move to align with wall
        r.rotate("ccw", 0.1, 45.5);
        sleep(100);

        // strafe into wall to straighten out robot
        r.strafe(-11, 0.1);
        sleep(100);
        r.strafe(3, 0.1);
        sleep(100);

        // move to depot
        r.translate(-60,0.2);
        sleep(100);

        // dump team marker
        r.servoMotors[1].setPosition(0);
        sleep(100);

        // park on crater
        r.translate(70, 0.1);

        telemetry.update();


//        if (r.GoldinCenter()) {
//            r.translate(24, -0.05);
//            r.translate(6, 0.05);
//            r.rotate("ccw", 90, 0.05);
//            r.translate(43, -0.05);
//
//
//        }
//        else {
//            r.translate(4,-0.1);
//            r.rotate("ccw", 0.05, 26);
//            telemetry.addData("xpos",detector.getScreenPosition().x);
//            double currentXPos = detector.getScreenPosition().x;
//            telemetry.update();
//            sleep(2000);
//            if (currentXPos < 400 && currentXPos > 200) {
//                r.translate(25.5, -0.05);
//                r.rotate("ccw", 0.05, 105);
//                r.translate(33, -0.05);
//
//            }
//            else{
//                r.rotate("cw", 0.05, 26);
//                r.translate(25.5, -0.05);
//                r.translate(6, 0.05);
//                r.rotate("ccw", 0.05, 90);
//                r.translate(50, -0.05);
//
//
//            }
//
//        }
        /*

        if(LEFT) {
            r.rotate("ccw", 0.05, 26);
            r.translate(25.5, -0.05);
            r.rotate("ccw", 0.05, 105);
            r.translate(33, -0.05);
        }
        else if(MID) {
            r.translate(24, -0.05);
            r.translate(6, 0.05);
            r.rotate("ccw", 90, 0.05);
            r.translate(43, -0.05);
        }
        else if(RIGHT) {
            r.rotate("cw", 0.05, 26);
            r.translate(25.5, -0.05);
            r.translate(6, 0.05);
            r.rotate("ccw", 0.05, 90);
            r.translate(50, -0.05);
        }
        */

//        r.rotate("ccw", 0.05, 135);
//        sleep(500);
//        r.translate(40, -0.05);
//        sleep(500);
//        r.servoMotors[0].setPosition(0);
//        sleep(500);
//        r.translate(60, 0.05);
//        sleep(500);
//        DOES NOT Work
//                putting arm in the crater
//        r.positionDrive(2,measure the counts, 0.05);
//        r.positionDrive(1.measure the counts,0.05);


















        // run until the end of the match (driver presses STOP)
        //sleep(3000);

//        if(r.goldLocation() == Robot.GoldPosition.LEFT){
//            directionForMineralFirst = "ccw";
//            directionForMineralSecond = "cw";
//            distanceForLineUp = baseDistance;
//        }
//        else if (r.goldLocation() == Robot.GoldPosition.MIDDLE) {
//            distanceForLineUp = baseDistance + 12*Math.sqrt(2);
//
//        }
//        else if (r.goldLocation() == Robot.GoldPosition.RIGHT) {
//            directionForMineralFirst = "cw";
//            directionForMineralSecond = "ccw";
//            distanceForLineUp = baseDistance + 2*12*Math.sqrt(2);
//        }
        /*

        r.translate(4, -0.05);
        sleep(1000);

        if(!directionForMineralFirst.equals("")) {
            r.rotate(directionForMineralFirst, 0.05, 44.5);
            r.translate(22.45, -0.05);
            r.rotate(directionForMineralSecond, 0.05, 0);
        }
        else {
            r.translate(15,-0.05);
        }

        /**
         * The constants for motion around the field after the
         * sampling portion of the Autonomous period.
         */
//        r.translate(4, -0.05);
//        sleep(1000);
//        r.translate(4, 0.05);
//        r.rotate("ccw", 0.05, 90);
//        r.translate(distanceForLineUp, -0.05);
//        r.rotate("ccw", 0.05, 135);
//        r.translate(inchesToDepot, -0.05);
//        //Deploy marker
//        sleep(1000);
//        r.translate(inchesToCrater, 0.05);

    }
}
