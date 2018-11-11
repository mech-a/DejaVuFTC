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

import org.firstinspires.ftc.teamcode.dependencies.Robot;


/**
 * CraterSide
 */

@Autonomous(name="Crater Side Auton", group="Internal")
//@Disabled
public class CraterSideAuton extends LinearOpMode {

    // Declare OpMode members.
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



    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        r.start(hardwareMap, telemetry);
        r.init();
        r.detectorInit();
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

        r.rotate("ccw", 0.05, 135);
        sleep(500);
        r.translate(40, -0.05);
        sleep(500);
        r.translate(60, 0.05);
        sleep(500);

















        // run until the end of the match (driver presses STOP)
        sleep(3000);

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


        r.translate(4, -0.05);
        sleep(1000);
        r.translate(4, 0.05);
        r.rotate("ccw", 0.05, 90);
        r.translate(distanceForLineUp, -0.05);
        r.rotate("ccw", 0.05, 135);
        r.translate(inchesToDepot, -0.05);
        //Deploy marker
        sleep(1000);
        r.translate(inchesToCrater, 0.05);
    }
}
