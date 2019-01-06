package org.firstinspires.ftc;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.CustomGoldDetector;


@TeleOp(name="Gold Detector Test", group="DogeCV")

public class GoldDetectorTest extends OpMode
{
    private CustomGoldDetector detector;


    @Override
    public void init() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        detector = new CustomGoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();


    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    @Override
    public void loop() {
        telemetry.addData("X Pos" , detector.getScreenPosition().x);
        telemetry.addData("Y Pos" , detector.getScreenPosition().y);// Gold X pos.

        telemetry.addData("alignment", detector.isAligned());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.disable();
    }

}