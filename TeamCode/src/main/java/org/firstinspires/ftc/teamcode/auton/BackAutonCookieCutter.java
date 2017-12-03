package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.called.HWRobot;

import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNT_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.DEGREES_TO_TURN_FOR_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.JEWEL_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.JEWEL_SERVO_UP;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_TURN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_VUFORIA;

@TeleOp(name="back auton", group="testing")
@Disabled
public class BackAutonCookieCutter extends LinearOpMode {
    //TODO implement a handler class for all autons ex auton(String team, String direction) that can be called instead big block

    // Declare OpMode members.
    HWRobot r = new HWRobot();
    //RobotValues v = new RobotValues();
    boolean a = false;
    String vuf = null;
    static String TEAM = "";




    //String directionToRotateForCryptobox = null;

    double heading = 0;

    @Override
    public void runOpMode() {
        /*r.getOpModeData(telemetry,hardwareMap);r.init("all");
        a = opModeIsActive();
        getHeading();
        //telemetry.addData("current heading:", heading);
        //telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Release servo latch and read the color sensor
        r.jewelServoFlip(JEWEL_SERVO_DOWN);
        sleep(2000);
        r.refreshHSV();
        //knock off jewel
        sleep(500);
        r.knockOffJewel(TEAM,a);
        r.jewelServoFlip(JEWEL_SERVO_UP);*/


        //
        r.translate("fwd", SPEED_TO_VUFORIA, COUNTS_TO_VUFORIA, a);
        vuf = r.getVuMark(a);

        r.translate("fwd", SPEED_TO_CRYPTO, COUNT_TO_CRYPTO, a);
        r.rotate("cw", SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);

        r.moveForCrypto(vuf, a);
        //TODO fully implement a function that allows the robot to go in a direction and stay at an angle
        /*ex
        while(x){
        if(heading > desired angle) {
        counteract;
        }
        etc
        }
         */

        r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);

        //r.releaseClaw();
    }

    private void getHeading() {
        heading = AngleUnit.DEGREES.fromUnit(r.angles.angleUnit, r.angles.firstAngle);
    }
}