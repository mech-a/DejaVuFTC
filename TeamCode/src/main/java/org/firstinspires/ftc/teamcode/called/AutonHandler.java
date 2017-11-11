package org.firstinspires.ftc.teamcode.called;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;
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

    boolean blueTeam;
    String vuf;
    String direction;
    String rtn;
    private double heading;
    boolean b = true;

    public void auton(String team, String area, Telemetry telemetry, HardwareMap hwMap, boolean a) {
        r.getOpModeData(telemetry, hwMap); r.init("all");


        if(team.toLowerCase().equals("red")) {
            direction = "fwd";
            blueTeam = false;
        }
        else if(team.toLowerCase().equals("blue")) {
            direction = "backwards";
            blueTeam = true;
        }


        if(area.equals("back")) {

            /*//Release servo latch and read the color sensor
            r.jewelServoFlip(JEWEL_SERVO_DOWN);

            try {
                sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            r.refreshHSV();
            //knock off jewel
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            r.knockOffJewel(team ,a);
            r.jewelServoFlip(JEWEL_SERVO_UP);
*/


            r.translate(direction, SPEED_TO_VUFORIA, COUNTS_TO_VUFORIA, a);
            vuf = r.getVuMark(a);

            r.translate(direction, SPEED_TO_CRYPTO, COUNT_TO_CRYPTO, a);
            r.rotate("cw", SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);

            r.moveForCrypto(vuf, a);

            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);

            r.releaseClaw();
        }

        else if (area.toLowerCase().equals("front")) {
            /*
            r.jewelServoFlip(JEWEL_SERVO_DOWN);
            r.refreshHSV();
            //knock off jewel
            r.knockOffJewel(team,a);
            r.jewelServoFlip(JEWEL_SERVO_UP);

            r.translate(direction, 0.2, COUNTS_TO_VUFORIA_FRONT, a);

            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            */

            vuf = r.getVuMark(a);
            if(blueTeam) {
                r.rotate("ccw", 0.2, 179, a);
            }

            r.moveForCrypto(vuf, a);

            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
            r.releaseClaw();
        }
    }

}
