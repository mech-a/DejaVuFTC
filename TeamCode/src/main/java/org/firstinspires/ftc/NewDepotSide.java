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
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.CustomGoldDetector;
import org.firstinspires.ftc.teamcode.dependencies.Robot;


@Autonomous(name = "New Depot Side", group = "Auton")
public class NewDepotSide extends LinearOpMode {

    Robot r = new Robot(this);


    //constants that will probably be moved to the Constants class
    private CustomGoldDetector detector;
    public static final double DETECTOR_CENTER = 200;
    public static final double DETECTOR_CENTER_THRESHOLD = 15;
    public static final double SAMPLE_ANGLE = 25;
    public static final double SAMPLE_DISTANCE_CENTER = 35;
    public static final double SAMPLE_DISTANCE_OTHER = 42;
    public static final double TRANSLATE_SPEED = 0.25;
    public static final double ROTATE_SPEED = 0.25;

    double samplex = 0;
    String samplePos = "center";

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();
        //r.driveMotors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //r.driveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        detector = new CustomGoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.enable();



        telemetry.addData("Stat:" ,"Initialized");
        telemetry.update();

        waitForStart();






        r.armMotors[0].setPower(-0.25);

        sleep(100);
        r.servoMotors[1].setPosition(0.35);
        sleep(100);

        r.positionDrive(2, -40, 0.2);
        sleep(100);


        //r.positionDrive(0,840,0.3 );

        r.armMotors[0].setPower(0.5);
        while(!isStopRequested() && r.armMotors[0].getCurrentPosition() <= 820) {}
        r.armMotors[0].setPower(0);


        sleep(100);
        telemetry.addData("robot:", "lowered");
        telemetry.update();

        //28 inches to go forward to center
        //26.5 inches to center mineral for angle measuring
        //31.5/2 inches from mineral
        //30.7 degrees to mineral
        //36 inches from mineral to depot
        //for left mineral turn 30.7 degrees ccw and move 30.8 inches then 66.4 degrees cw and move 39.3 inches
        //for right mineral turn 30.7 degreescw and move 30.8 inches then 66.4 degrees ccw and move 39.3 inches
        //for center mineral go straight 62.5 inches
        // and then move 45 degrees ccw and move 72 inches for crater


        //for left mineral
//        r.rotate("ccw",45, 0.05);
//        r.positionDrive(0,-800,0.5);
//        r.rotate("cw",0.05,0);
        //18-19 or 11.6 degrees cw to rotate out of the hook
        //drive 2 inches forward
        //8 or 10 inches forward
        //rotate 31 ccw degrees or 41
        //28.5 inches forward
        //rotate 21 degrees cw
        //34 inches forward
        r.rotate("cw",0.1,18);
        //TODO translate 4
        r.translate(4,-0.15);
        r.rotate("ccw",0.1,0);
//        r.rotate("ccw",0.05,31);
//        r.translate(28.5,-0.05);
//        r.rotate("cw",0.05,20);

//

//        r.translate(4,-0.1);
//        r.rotate("ccw", 0.1, 27);
//        r.translate(30.8, -0.1);
//        r.rotate("cw",0.05,35);
//        r.translate(39.3,-0.05);
//        sleep(1000);
//        r.rotate("ccw",0.1,40);
//        r.translate(65,0.1);
//        r.translate(10,0.05);


//        r.rotate("ccw", 0.1, 35);
//        r.translate(30.8, -0.1);
//        r.rotate("cw",0.05,35);
//        r.translate(39.3,-0.05);
//        r.servoMotors[0].setPosition(0);


        //TODO check if last motor hits pos or first motor hits position



        /*
        // for right mineral
        r.translate(4,-0.1);
        r.rotate("cw", 0.1, 35 );
        r.translate(30.8, -0.1);
        r.rotate("ccw",0.05,28);
        r.translate(39.3,-0.1);
        sleep(1000);
        r.rotate("ccw",0.05,45);
        r.translate(65,0.1);
        r.translate(10,0.05);
        */




        if ((detector.getScreenPosition().x < 400 && detector.getScreenPosition().x > 150) && detector.getScreenPosition().y >= 200) {
            telemetry.addData("Position:", "Center");
            telemetry.update();
            //make this less before running

            //TODO yesterday it was 56, i changed to 54, need to test
            r.translate(54, -0.15);



        }
        else {
            r.translate(4,-0.15);
            r.rotate("ccw", 0.15, 35);
            telemetry.addData("xpos", detector.getScreenPosition().x);
            double currentXPos = detector.getScreenPosition().x;
            double currentYPos = detector.getScreenPosition().y;
            telemetry.update();
            sleep(1000);
            if ((currentXPos < 450 && currentXPos > 200) && currentYPos >= 200){
                r.translate(30.8, -0.15);
                r.rotate("cw",0.1,33);
                r.translate(33.3,-0.15);

            }
            else{
                r.rotate("cw", 0.1, 35);
                r.translate(30.8, -0.15);
                r.rotate("ccw",0.05,32);
                r.translate(33.3,-0.15);


            }

        }




        //placeholder for dropping the team marker
        sleep(100);
        r.positionDrive(2, -220, 0.2);
        sleep(500);
        r.positionDrive(2, 220, 0.2);
        sleep(100);
        r.rotate("cw",0.1,45.5);
        r.translate(75,0.2);


        //.addData("xpos",detector.getScreenPosition().x);



        telemetry.update();


        detector.disable();





    }
}