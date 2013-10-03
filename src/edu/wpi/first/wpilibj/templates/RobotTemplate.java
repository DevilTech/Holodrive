/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

public class RobotTemplate extends IterativeRobot {

    private CANJaguar jaglf;
    private CANJaguar jagrf;
    private CANJaguar jaglb;
    private CANJaguar jagrb;
    private Joystick joy;
    private I2C cread;
    private I2C cwrite;
    private I2C gread;
    private I2C gwrite;
    private ADXL345_I2C ac;
    private int bl;
    private int bg;
    private int ba;
    private byte buff[];
    private byte buffg[];
    private byte buffa[];
    public double longestwheel = 3.83 * 37 / 100;
    public double shortwheel = 6.5 * Math.PI / 100;
    public double secondlongwheel = 31 * Math.PI / 100;
    public double thirdlongwheel = 22.5 * 4.71 / 100;
    int gSampleRateDivider = 0;
    
    double p = 0;
    double d = 0;
    int startHeading;
    
    Timer time;

    public void robotInit() {
        //gyro 0x3h
        
        bl = 6;
        bg = 2;
        ba = 1;
        
        buff = new byte[bl];
        buffg =  new byte[bg];
        buffa = new byte[ba];
        
        gread = new I2C(DigitalModule.getInstance(1), 0xD0);
        gwrite = new I2C(DigitalModule.getInstance(1), 0xD1);
        
        cwrite = new I2C(DigitalModule.getInstance(1), 0x3C);
        cread = new I2C(DigitalModule.getInstance(1), 0x3D);
        
        cwrite.write(2, 0);
        cwrite.write(1, 0xC0);
        gwrite.write(21, gSampleRateDivider);
        gwrite.write(22, 0x1A);
        
        time = new Timer(); 
        time.start();

        try {
            jaglf = new CANJaguar(2);
            jagrf = new CANJaguar(3);
            jaglb = new CANJaguar(12);
            jagrb = new CANJaguar(13);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }

        joy = new Joystick(1);
        gread.read(33, bg, buffg);
        startHeading = byteCombo(buffg[0], buffg[1]);
    }

    public void autonomousPeriodic() {
    }

    public void teleopPeriodic() {
        
        cread.read(3, bl, buff);
        
        gread.read(33, bg, buffg);
       // System.out.println("Gyro: " + byteCombo(buffg[0], buffg[1]));
        System.out.println("Compass: " + byteCombo(buff[2], buff[3]));
        //System.out.println("Time Change: " + time.get());
        System.out.println(byteCombo((byte)0xFF, (byte)0));
        time.reset();

        
       
    }

    public void testPeriodic() {
    }
    
    public void holoSensor(){
        gread.read(33, bg, buffg);
        int theta = byteCombo(buffg[0], buffg[1]) - startHeading;
        
    }
    
    public void holoNoSensor(){
        double max;
         while (joy.getRawButton(5)) {
            turnLeft();
        }
        while (joy.getRawButton(6)) {
            turnRight();
        }
        
        try {
            double right = joy.getX() * Math.abs(joy.getX());
            double forward = -joy.getY() * Math.abs(joy.getY());
            double clockwise = joy.getRawAxis(4) * Math.abs(joy.getRawAxis(4));

            double lf = forward + clockwise + right;
            double rf = forward - clockwise - right;
            double lb = forward + clockwise - right;
            double rb = forward - clockwise + right;

            max = Math.abs(lf);
            
            if (Math.abs(rf) > max) {
                max = Math.abs(rf);
            }
            if (Math.abs(lb) > max) {
                max = Math.abs(lb);
            }
            if (Math.abs(rb) > max) {
                max = Math.abs(rb);
            }
            if (max > 1) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }

            jaglf.setX(lf);
            jagrf.setX(rf);
            jaglb.setX(lb);
            jagrb.setX(rb);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    
    

    private void turnLeft() {
        try {
            jaglf.setX(-shortwheel / longestwheel);
            jagrf.setX(-thirdlongwheel / longestwheel);
            jaglb.setX(secondlongwheel / longestwheel);
            jagrb.setX(-longestwheel);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }
    

    private void turnRight() {
        try {
            jagrf.setX(-shortwheel / longestwheel);
            jaglf.setX(-thirdlongwheel / longestwheel);
            jagrb.setX(secondlongwheel / longestwheel);
            jaglb.setX(-longestwheel);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    private int byteCombo(byte num1, byte num2) {
        return (int) ((num1 & 0xff | num2 & 0xff));
    }
}
