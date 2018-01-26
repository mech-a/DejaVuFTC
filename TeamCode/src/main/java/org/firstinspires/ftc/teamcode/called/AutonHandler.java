package org.firstinspires.ftc.teamcode.called;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.called.RobotValues.ARM_JEWEL_DOWN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.ARM_JEWEL_UP;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_PER_INCH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_CRYPTO_FRONT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA;
//import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNTS_TO_VUFORIA_FRONT;
import static org.firstinspires.ftc.teamcode.called.RobotValues.COUNT_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.DEGREES_TO_TURN_FOR_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.EXTRUDE_CLAW_POWER;
import static org.firstinspires.ftc.teamcode.called.RobotValues.HITTER_JEWEL_MIDDLE;
import static org.firstinspires.ftc.teamcode.called.RobotValues.HITTER_JEWEL_NORTH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_CRYPTO;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_PLACE_GLYPH;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_TURN;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SPEED_TO_VUFORIA;

/**
 * Created by gbhat on 11/9/2017.
 */

//TODO make ramp speed functions; reduces slippage
public class AutonHandler {
    HWRobot r = new HWRobot();

    boolean blueTeam = false;
    String vuf;
    String generalDirection;
    String teamString;


    public void autonInit(Telemetry atele, HardwareMap hwmap) {
        r.getOpModeData(atele, hwmap);
        r.init("all");
    }

    public void auton(String team, String area, Telemetry telemetry, HardwareMap hwMap, boolean a) {
        //TODO error due to this? might need seperate initialization of getOpModeData
        //r.getOpModeData(telemetry, hwMap); r.init("all");

        if(team.toLowerCase().equals("red")) {
            blueTeam = false;
            generalDirection = "fwd";
        }
        else if(team.toLowerCase().equals("blue")) {
            blueTeam = true;
            generalDirection = "bk";
        }


        if(area.toLowerCase().equals("back")) {
            teamString = (blueTeam) ? "blue" : "red";
            jewel(teamString, a);


            r.translate(generalDirection, SPEED_TO_VUFORIA, COUNTS_TO_VUFORIA, a);
            rsleep(500);
            vuf = r.getVuMark(a);
            rsleep(500);
            r.translate(generalDirection, SPEED_TO_CRYPTO, COUNT_TO_CRYPTO, a);
            rsleep(500);
            r.rotate("cw", SPEED_TO_TURN, DEGREES_TO_TURN_FOR_CRYPTO, a);
            rsleep(500);
            r.moveForCrypto(vuf, a);
            rsleep(500);
            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH,a);
            rsleep(500);
            extrudeGlyph();
            rsleep(600);
            r.translate("back");
        }

        else if (area.toLowerCase().equals("front")) {
            teamString = (blueTeam) ? "blue" : "red";
            jewel(teamString, a);

            r.translate(generalDirection, 0.2, COUNTS_TO_VUFORIA, a);
            rsleep(500);
            vuf = r.getVuMark(a);
            rsleep(500);
            r.translate(generalDirection, 0.2, COUNTS_TO_CRYPTO_FRONT, a);
            rsleep(500);
            r.rotate("ccw",0.2,90,a);
            rsleep(500);
            r.translate("fwd", 0.2, 12*COUNTS_PER_INCH, a);

            //TODO not sure if it'll move 90degrees more or not move
            if(blueTeam) {
                r.rotate("ccw",0.2,90,a);
            }
            else {
                r.rotate("cw",0.2,90,a);
            }
            //String quickdir = (blueTeam) ? "ccw" : "cw";

            rsleep(500);


            r.moveForCrypto(vuf, a);

            rsleep(700);

            r.translate("fwd", SPEED_TO_PLACE_GLYPH, COUNTS_TO_PLACE_GLYPH, a);
            rsleep(500);
            extrudeGlyph();
        }
    }

    private void extrudeGlyph() {
        r.mtrClawL.setPower(EXTRUDE_CLAW_POWER);
        r.mtrClawR.setPower(EXTRUDE_CLAW_POWER);
        rsleep(1000);
        r.mtrClawL.setPower(0);
        r.mtrClawR.setPower(0);
    }

    private void rsleep(long milliseconds) {
        try {
            sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void jewel(String teamColor, boolean active) {
        r.srvJewelArm.setPosition(ARM_JEWEL_DOWN/2);
        r.srvJewelHitter.setPosition(HITTER_JEWEL_MIDDLE);
        rsleep(250);
        r.srvJewelArm.setPosition(ARM_JEWEL_DOWN);
        rsleep(500);
        r.knockOffJewel(teamColor, active);
        rsleep(500);
        r.srvJewelHitter.setPosition(HITTER_JEWEL_MIDDLE);
        rsleep(250);
        r.srvJewelArm.setPosition(ARM_JEWEL_UP/2);
        r.srvJewelHitter.setPosition(HITTER_JEWEL_NORTH);
        rsleep(250);
        r.srvJewelArm.setPosition(ARM_JEWEL_UP);
    }

}
