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



    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        r.start(hardwareMap, telemetry);
        r.init();
        r.detectorInit();

        waitForStart();

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

        /**
         * The constants for motion around the field after the
         * sampling portion of the Autonomous period.
         */
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
