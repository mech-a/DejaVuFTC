package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by gbhat on 3/7/2019.
 */

public abstract class DejaVuLinearOpMode extends LinearOpMode {
    private FTPHandler ftp;
    //TODO set homepath
    private String homePath;
    protected void download(String fname) {
        try {
            ftp = new FTPHandler();
            ftp.downloadFile(fname, homePath+fname);
            //TODO make log.d
            System.out.println("FTP File downloaded successfully");
            ftp.disconnect();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
