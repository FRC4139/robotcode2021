package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CSVPath {
    File f;
    BufferedReader br;
    FileReader fr;
    File f1;
    BufferedReader br1;
    FileReader fr1;
    int line = 0; 
    public CSVPath(String fileLocation) {

        try {
            f = new File(fileLocation);
            fr = new FileReader(f);
            br = new BufferedReader(fr);
            f1 = new File("/media/sda1/hello.txt");
            fr1 = new FileReader(f1);
            br1 = new BufferedReader(fr1);
            SmartDashboard.putString("test", br1.readLine());
            br.readLine();
        } catch (FileNotFoundException e) {
            System.out.println("File not found...");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("Out of lines???");
            e.printStackTrace();
        } 
        
    }

    public int Update(Controller c) throws IOException {
        String s = br.readLine();
        line++;
        if (s == null) {
            SmartDashboard.putString("reader", "something was null");
            return -5;
        }
        if (s == "END") {
            c.setDriveSpeed(0, 0);
            return -1; 
        }

        String[] sp = s.split(",");
        double[] speeds = new double[5];
        for(int i =0; i < 5; i++) speeds[i] = Double.parseDouble(sp[i]);
        
        c.setRawDriveSpeeds(new double[] {speeds[0], speeds[1], speeds[2], speeds[3]});
        c.setIntakeSpeed(speeds[4]);
        
        return line; 
    }
}
