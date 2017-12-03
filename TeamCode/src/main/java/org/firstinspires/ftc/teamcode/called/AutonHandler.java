package org.firstinspires.ftc.teamcode.called;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA_FRONT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNT_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.DEGREES_TO_TURN_FOR_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.JEWEL_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.JEWEL_SERVO_UP;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_TURN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_VUFORIA;

/**
 * Created by gbhat on 11/9/2017.
 */

public class AutonHandler {
    HWRobot r = new HWRobot();

    boolean blueTeam = false;
    String vuf;
    String rotation;

    private double heading;
    boolean b = true;

    public void auton(String team, String area, Telemetry telemetry, HardwareMap hwMap, boolean a) {
        r.getOpModeData(telemetry, hwMap); r.init("all");
        r.driveMotorPolarity("reverse");


        if(team.toLowerCase().equals("red")) {
            rotation = "ccw";
            blueTeam = false;
        }
        else if(team.toLowerCase().equals("blue")) {
            rotation = "cw";
            blueTeam = true;
        }


        if(area.toLowerCase().equals("back")) {

            //Release servo latch and read the color sensor
            r.jewelServoFlip(JEWEL_SERVO_DOWN);

            rsleep(2000);

            r.refreshHSV();
            //knock off jewel

            r.knockOffJewel(team ,a);
            rsleep(250);
            r.jewelServoFlip(JEWEL_SERVO_UP);



            r.translate("back", SPEED_TO_VUFORIA, COUNTS_TO_VUFORIA, a);
            vuf = r.getVuMark(a);

            r.translate("back", SPEED_TO_CRYPTO, COUNT_TO_CRYPTO, a);
            r.rotate(rotation, SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);

            r.moveForCrypto(vuf, a);

            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);

            fullExtrudeGlyph();
        }

        else if (area.toLowerCase().equals("front")) {

            r.jewelServoFlip(JEWEL_SERVO_DOWN);
            r.refreshHSV();
            //knock off jewel
            r.knockOffJewel(team,a);
            r.jewelServoFlip(JEWEL_SERVO_UP);

            r.translate("back", 0.2, COUNTS_TO_VUFORIA_FRONT, a);

            rsleep(500);

            vuf = r.getVuMark(a);

            if(blueTeam) {
                r.translate("left", 0.2, 12 * COUNTS_PER_INCH, a);
            }
            else {
                r.translate("right", 0.2, 12* COUNTS_PER_INCH, a);
            }

            rsleep(500);


            r.moveForCrypto(vuf, a);

            rsleep(700);

            r.translate("back", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
            rsleep(500);
            fullExtrudeGlyph();
        }
    }

    private void fullExtrudeGlyph() {
        r.moveSlide("up");
        r.extrudeGlyphs("top");
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        r.extrudeGlyphs("stop");
    }

    private void rsleep(long milliseconds) {
        try {
            sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
