package org.firstinspires.ftc.teamcode.archaic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.called.HWRobot;

/**
 * Autonomous made to test out translation and other functions related to movement.
 * Made By Gaurav
 */

@Autonomous(name="CookieCutter Red Back", group="Testing")
//@Disabled
public class CookieCutterAutonomous extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();
    private String pictograph;
    private int cryptoboxAdditionalCounts = 0;
    int countsPerInch = 252;
    VuforiaLocalizer vuforia;
    String vuf = "";
    int inToVuf = 24;
    int vufToCrypto = 15;
    int moveToColumn = 7;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.getOpModeData(telemetry,hardwareMap);robot.init("all");
        boolean a = opModeIsActive();
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = robot.vuforiaKey;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

        //Finds Vumark
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
//        pictograph = robot.getVuMark(a);
//        telemetry.addData("Pictograph", pictograph);
//        telemetry.update();
//        sleep(1000);

        robot.translate("fwd", 0.2, countsPerInch * inToVuf, a);

        relicTrackables.activate();
        while(opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
                if(vuMark == RelicRecoveryVuMark.LEFT) {
                    vuf = "LEFT";
                }
                else if(vuMark == RelicRecoveryVuMark.CENTER) {
                    vuf = "CENTER";
                }
                else if(vuMark == RelicRecoveryVuMark.RIGHT) {
                    vuf ="RIGHT";
                }
                break;
            }
            else{
                //telemetry.addData("VuMark", "not visible");
                vuf = "UNKNOWN";
            }
            telemetry.update();
            break;
        }
        robot.translate("fwd", 0.2, countsPerInch * vufToCrypto, a);
        robot.rotate("cw", 0.1, 90, a);
        if(vuf.equals("LEFT")) {
            robot.translate("left", 0.2, countsPerInch * moveToColumn, a);
            //robot.translate("forward", 0.1, countsPerInch * );
        }
        else if(vuf.equals("CENTER")) {
            //robot.translate("fwd", 0.1, countsPerInch * moveToColumn, a);
        }
        else if(vuf.equals("RIGHT")) {
            robot.translate("right", 0.2, countsPerInch * moveToColumn, a);
        }
        else if(vuf.equals("UNKNOWN")) {
            telemetry.addData("unkn read", "");
            telemetry.update();
        }

        robot.translate("fwd", 0.15, countsPerInch * 9, a);
        robot.srvL.setPosition(0);
        robot.srvR.setPosition(1);


        /*


        //Translates into position for jewel
        robot.translate("right", 0.2, 5000, a);
        robot.srvJewel.setPosition(0.5);
        robot.refreshHSV();

        //if the ball is blue
        if(robot.hsv[0] > 215 && robot.hsv[0] < 270) {
            robot.translate("fwd", 0.2, 350, a);
            robot.srvJewel.setPosition(0);
            robot.translate("bk", 0.2, 350, a);
        }
        else {
            robot.translate("bk", 0.2, 350, a);
            robot.srvJewel.setPosition(0);
            robot.translate("fwd", 0.2, 350, a);
        }

        //TODO implement while loop checking for color of tape underneath robot to stop, then make additional counts a value * 1,2,3 for better movement

        //Decide how far to go for cryptobox
        if(pictograph.equals("RIGHT")) {
            robot.translate("fwd", 0.2, 500, a);
        }
        else if(pictograph.equals("CENTER")) {
            robot.translate("fwd", 0.2,  2 * 500, a);
        }
        else if(pictograph.equals("LEFT")) {
            robot.translate("fwd", 0.2,  3 * 500, a);
        }

        //Rotate and release jewel
        robot.rotate("cw", 0.2, 90, a);
        robot.translate("fwd", 0.1, 1120, a);
        robot.srvL.setPosition(0.75);
        robot.srvR.setPosition(0.25);

*/
    }

}
