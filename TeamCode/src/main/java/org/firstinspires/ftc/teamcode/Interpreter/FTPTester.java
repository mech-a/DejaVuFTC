package org.firstinspires.ftc.teamcode.Interpreter;

/**
 * Created by gbhat on 3/7/2019.
 */

public class FTPTester {
    public static void download() {
        FTPHandler ftp;
        try {
            ftp = new FTPHandler();
            ftp.downloadFile("WORKBOOK_1.pdf", "/Users/gbhat/Desktop/workbook_1_ftp.pdf");
            System.out.println("FTP File downloaded successfully");
            ftp.disconnect();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
