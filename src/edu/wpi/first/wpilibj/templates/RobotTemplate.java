/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
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
    private I2C awrite;
    private I2C aread;
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
    private int minX = 2000;
    private int minY = 2000;
    private int maxX = -2000;
    private int maxY = -2000;
    double yDist = 0;
    double xDist = 0;
    Encoder enX = new Encoder(1,2);
    Encoder enY = new Encoder(3,4);
    private double initialHeading;
    
    double p = 0;
    double d = 0;
    int startHeading;
    
    Timer time;

    public void robotInit() {
        
        bl = 6;
        bg = 2;
        ba = 6;
        
        buff = new byte[bl];
        buffg =  new byte[bg];
        buffa = new byte[ba];
        
        setupCompass();
        setupGyro();
        setupAccel();
        setupPots();
        
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
        initialHeading = getCAngle();
    }

    public void autonomousPeriodic() {
        System.out.println("AUTO");
    }

    public void teleopPeriodic() {
        cread.read(3, bl, buff);
        gread.read(33, bg, buffg);
        aread.read(50, ba, buffa);
        for(int i = 0; i < 6; i += 2) {
            if(byteCombo(buff[i], buff[i+1]) == -4096) {
                System.out.println("OVERFLOW  ");
            } else {
                switch(i) {
                    case 0: System.out.print("x: " + getCX() + " "); break;
                    case 1: System.out.print("  "); break;
                    case 4: System.out.print("y: " + getCY() + " "); break;
                    case 5: System.out.print("  "); break;
                }
            }
        }
        System.out.print("atan: " + getCAngle());
        System.out.print("Encoder: " + enX.getDistance());
        System.out.println("");
        holoSensor();
    }

    public void testPeriodic() {
    }
    
    public void holoSensor(){
        double max;
         while (joy.getRawButton(5)) {
            turnLeft();
        }
        while (joy.getRawButton(6)) {
            turnRight();
        }
        
        try {
            double theta = getCAngle() - initialHeading;
            double temp;
            double right = joy.getX() * Math.abs(joy.getX());
            double forward = -joy.getY() * Math.abs(joy.getY());
            double clockwise = joy.getRawAxis(4) * Math.abs(joy.getRawAxis(4));
            
            temp  = forward*Math.cos(theta) + right*Math.sin(theta);
            right = -forward*Math.sin(theta) + right*Math.cos(theta);
            forward = temp;

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
        int x = num1;
        x = x << 8;
        int y = num2;
        x |= y & 0x000000ff;
        return x;
    }
    
    public double atan2 (double y, double x) {
        double pi=Math.PI;
        double pi2=Math.PI/2;
        if (x>=0) { /* right half-plane */
            if (y>=0) { /* 1st quadrant */
                if (y<=x) {
                    if (x==0) return 0;  /* x & y both zero */
                    else  return f(y/x);
                } else  return pi2 - f(x/y);
            } else {  /* 4th quadrant */
                if (-y<=x) return -f(-y/x);
                else    return -pi2 + f(-x/y);
            }
        } else {  /* left half-plane */
            if (y>=0) {  /* 2nd quadrant */
                if (y>=-x)  return pi2 + f(-x/y);
                else    return pi - f(-y/x);
            } else {  /* 3rd quadrant */
                if (y>=x)   return -pi + f(y/x);
                else    return -pi2 - f(x/y);
            }
        }
    }
    
    double getAX() {
        return byteCombo(buff[0], buff[1]);
    }
    
    double getAY() {
        return byteCombo(buff[2], buff[3]);
    }
    
    double getCX() {
        return byteCombo(buff[0], buff[1]) - 458;
    }
    
    double getCY() {
        return byteCombo(buff[4], buff[5]) - 93;
    }
    
    double getCAngle() {
        return Math.toDegrees(atan2(getCY(), getCX()));
    }

    double f(double t) {
    /* This provides 1/10 of a degree accuracy: */
        return -0.001096995+t*(1.041963708+t*(-0.196333807+t*(-0.060821409)));
    }
    
    int minVal(int newVal, int oldMin) {
        if(newVal < oldMin) {
            return newVal;
        } else return oldMin;
    }
    
    int maxVal(int newVal, int oldMax) {
        if(newVal > oldMax) {
            return newVal;
        } else return oldMax;
    }
    
    double[] getDisp() {
        double[] displacement = {enX.getDistance(), enY.getDistance()};
        return displacement;
    }
    
    void setupPots() {
        enX.start();
        enX.setDistancePerPulse(8.64360/90);
        enY.start();
        enY.setDistancePerPulse(8.64360/90);
    }
    
    void setupCompass() {
        cwrite = new I2C(DigitalModule.getInstance(1), 0x3C);
        cread = new I2C(DigitalModule.getInstance(1), 0x3D);
        
        cwrite.write(0, 0x58);
        cwrite.write(1, 0);
        cwrite.write(2, 0);
    }
    
    void setupGyro() {
        gwrite = new I2C(DigitalModule.getInstance(1), 0xD1);
        gread = new I2C(DigitalModule.getInstance(1), 0xD0);

        gwrite.write(21, gSampleRateDivider);
        gwrite.write(22, 0x1A);
    }
    
    void setupAccel() {
        awrite = new I2C(DigitalModule.getInstance(1), 0xA6);
        aread = new I2C(DigitalModule.getInstance(1), 0xA7);
        
        awrite.write(44, 0x0C);
        awrite.write(45, 0x08);
        awrite.write(49, 0);
    }
    
    void turnToRight(){ 
        double right = initialHeading -90;
        double theta = getCAngle() - right;
        double pRate = p * theta;
        
        double lf = pRate;
        double rf = -pRate;
        double lb = pRate;
        double rb = -pRate;

            double max = Math.abs(lf);
            
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
        
    }
}
