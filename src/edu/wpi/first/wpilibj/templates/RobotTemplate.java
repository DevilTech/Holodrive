/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotTemplate extends IterativeRobot {
    CANJaguar jaglf, jagrf, jaglb, jagrb;

    Joystick joy;
    ADXL345_I2C ad;
    I2C cread, cwrite;
    I2C gread, gwrite;
    I2C aread, awrite;

    byte buffc[], buffg[], buffa[];

    int bc, bg, ba;
    int gSampleRateDivider = 0;
    int minX = 2000;
    int minY = 2000;
    int maxX = -2000;
    int maxY = -2000;
    int maxVoltage = 12;
    int maxRampRate = 22;
    
    double tempVeloHistoryY = 0;
    double currentVeloY = 0;
    double tempVeloHistoryX = 0;
    double currentVeloX = 0;

    Encoder enX = new Encoder(1, 2);
    Encoder enY = new Encoder(3, 4);
    
    double longestwheel = 3.83 * 37 / 100;
    double shortwheel = 6.5 * Math.PI / 100;
    double secondlongwheel = 31 * Math.PI / 100;
    double thirdlongwheel = 22.5 * 4.71 / 100;
    double initialHeading, heading;
    double theta, theta90, theta180, theta270;
    double forward, clockwise, right;
    double temp;
    double PY = .0;
    double PX = .0;
    double PR = .25;
    double DY = 0;
    double DX = 0;
    double p = 1;
    double d = 0;
    double diff = 0;
    double gScale = Math.PI / (180 * 14.375);
    double gScaleForSensor = 1;
    double dt = 0.02;
    double KpR = 14.39 * dt;
    double KiR = 38.68 * dt;
    double KpY = 0.0987 * dt;	// Kp constant from 30deg toe-in drivetrain model in in/s
    double KdY = 0;		// Kd constant from 30deg toe-in drivetrain model in in/s^2
    double KpX = 0.0127 * dt;	// Kp constant from 60deg toe-in drivetrain model in in/s
    double KdX = 0;  // Kd constant from 60deg toe-in drivetrain model in in/s^2
    //double KdY = 0.04 * dt;
    //double KdX = 0.01 * dt;
    double maxXY = 132;         //max expected forward velocity in IPS (138 = 11.5ft/s)
    double GZ = 0;
    double aScale = 32.174 * 12 * .004;	// g force to in/s^2 conversion, scale for LSB per g
    double errorH = 0;
    double joyX, joyY, joyZ;

    boolean FCMode = true;
    boolean openX, openY, openC = false;
    boolean iSetting = false;
    boolean open = false;

    Timer time;
    
    public void robotInit() {
        try {           
            SmartDashboard.putNumber("Heading", heading);
            
            bc = 6;
            bg = 6;
            ba = 6;
            heading = 0;
            
            ad = new ADXL345_I2C(1, ADXL345_I2C.DataFormat_Range.k2G);
            
            buffc = new byte[bc];
            buffg = new byte[bg];
            buffa = new byte[ba];

            setupCompass();
            setupGyro();
            setupAccel();
            setupPots();
            readAll();
            time = new Timer();
            time.start();

            jaglf = new CANJaguar(2);
            jagrf = new CANJaguar(3);
            jaglb = new CANJaguar(12);
            jagrb = new CANJaguar(13);

            joy = new Joystick(1);
            gread.read(33, bg, buffg);
            initialHeading = getCRAngle();
            
            jaglf.setVoltageRampRate(maxRampRate);
            jagrf.setVoltageRampRate(maxRampRate);
            jaglb.setVoltageRampRate(maxRampRate);
            jagrb.setVoltageRampRate(maxRampRate);
            jaglf.configMaxOutputVoltage(maxVoltage);
            jagrf.configMaxOutputVoltage(maxVoltage);
            jaglb.configMaxOutputVoltage(maxVoltage);
            jagrb.configMaxOutputVoltage(maxVoltage);

        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    public void autonomousPeriodic() { System.out.println("AUTO"); }

    public void teleopPeriodic() {
        PID_Drive();
        //sensorPrint();
        smartPull();
        smartPush();
        //holoSensor();
    }

    public void testPeriodic() {
        
    }

    public void teleopInit() {
        smartInit();
        time.reset();
    }
    
    public void disabledInit(){
       smartInit();
       forward = 0;
       clockwise = 0;
       right = 0;
       openC = false;
       openX = true;
       openY = false;
    }
    
    public void disabledPeriodic(){
       smartPush();
       smartPull();
       forward = 0;
       clockwise = 0;
       right = 0;
    }

    public void sensorPrint() {
        readAll();
       
    }

    public void sensorTest() {
        if (time.get() < 3) {
            try {
                jaglf.setX(1, (byte) 1);
                jagrf.setX(1, (byte) 1);
                jaglb.setX(1, (byte) 1);
                jagrb.setX(1, (byte) 1);
                CANJaguar.updateSyncGroup((byte) 1);
                readAll();
                System.out.println(time.get() + "," + getGZ());
            } catch (CANTimeoutException ex) { ex.printStackTrace(); }
            /*  }else if (time.get() < 7){
             try 
             {
             //for x
             jaglf.setX(-1,(byte)1);
             jagrf.setX(1,(byte)1);
             jaglb.setX(-1,(byte)1);
             jagrb.setX(1,(byte)1);
             CANJaguar.updateSyncGroup((byte)1);
             readAll();
             //label for things
             System.out.println(time.get() + "," + getGZ() + "," + getCAngle()+ "," + enX.getRate());
             } catch (CANTimeoutException ex) 
             {
             ex.printStackTrace();
             }  */
        } else {
            try {
                jaglf.setX(0);
                jagrf.setX(0);
                jaglb.setX(0);
                jagrb.setX(0);
            } catch (CANTimeoutException ex) { ex.printStackTrace(); }
        }
    }

    public void chassisSetup() {
        theta90 = (initialHeading > Math.PI / 2) ? (initialHeading - 3 * Math.PI / 2) : initialHeading + Math.PI / 2;
        theta180 = (initialHeading > 0) ? (initialHeading - Math.PI) : (initialHeading + Math.PI);
        theta270 = (initialHeading < Math.PI / 2) ? (initialHeading - Math.PI) : (initialHeading + Math.PI);
    }

    public void getInput() {
        readAll();
        FCMode = true;
        theta = getCHRAngle();
        joyY = -joy.getY() * Math.abs(joy.getY());
        joyX = joy.getX() * Math.abs(joy.getX());

        if (Math.abs(joy.getZ()) > .01) {
            joyZ = joy.getZ() * Math.abs(joy.getZ()) * Math.abs(joy.getZ());
            heading = theta;
            errorH = 0;
        } else { errorH = radRap(heading - theta) *KiR; }

        if (FCMode) {
            //takes values from joysticks and changes the values to the correct 
            //vector based on compass input
            temp = forward * Math.cos(theta) + right * Math.sin(theta);
            right = -forward * Math.sin(theta) + right * Math.cos(theta);
            forward = temp;
        }
        SmartDashboard.putNumber("heading_put", heading);
    }
    
    public void PID_Drive() {
        double oForward = 0, oRight = 0, oClock = 0;

        getInput();
        GZ = getGZ() * gScale;

        double VY = enY.getRate();
        double AY = ad.getAcceleration(ADXL345_I2C.Axes.kY) / aScale;// expected V range +/- maxXY
        double VX = enX.getRate();
        double AX = ad.getAcceleration(ADXL345_I2C.Axes.kX) / aScale;	// expected A range +/- 28.6

        //adds to forward the amount in which we want to move in y direction in in/s
        //max velocity times amount requested (-1, 1), minus current speed
        //then, the derivative of the speed (acceleration) is added to to the value of forward 
        forward = clamp(forward);
        forward = forward + KpY * (maxXY * joyY - VY);//PD expected range +/- 1.0
        System.out.println((maxXY * joyY) + ", " + (maxXY * joyY - VY));
        right = clamp(right);
        right = right + KpX * (maxXY * joyX - VX) ;	//PD expected range +/- 0.577
        clockwise = clamp(clockwise);
        clockwise = clockwise + KpR * (6.28 * joyZ + GZ) + errorH; //replace 0 with KpR

        if (Math.abs(forward) > 10 && !openY) {
            System.out.println("ERROR: PID-Y open (1.5 < " + forward + "): " + joy.getY() + ", " + VY + ", " + AY);
            oForward = -joy.getY();
            openY = true;
        }
        if (openY) { oForward = -joy.getY(); }

        if (Math.abs(right) > 10 && !openX) {
            System.out.println("ERROR: PID-X open (1 < " + right + "): " + joy.getX() + ", " + VX + ", " + AX);
            oRight = joy.getX() * 0.577;
            openX = true;
        }
        if (openX) { oRight = joy.getX() * 0.577; }

        if (Math.abs(clockwise) > 10 && !openC) {
            System.out.println("ERROR: PID-Z open (1 < " + clockwise + "): " + joy.getZ());
            oClock = joy.getZ();
            openC = true;
        }
        if (openC) { oClock = joy.getZ(); }
        /* max right =  cos(60) / cos(30)    
        double PID_Y = forward - PY*(forward-enY.getRate()) - DY * getAY(); 
        double PID_X = (right - PX*(right-enX.getRate())* .577 - DX * getAX());
        double PID_R = (clockwise - PR*(clockwise-(getGZ()/.935))); */

        double lf, rf, lb, rb;
        double ff, fc, fr;

        fc = openC ? oClock : clockwise;
        ff = openY ? oForward : forward - KdY * AY;
        fr = openX ? oRight : right - KdX * AX ;

        lf = ff + fc + fr;
        rf = ff - fc - fr;
        lb = ff + fc - fr;
        rb = ff - fc + fr;

        double max = Math.abs(lf);

        if (Math.abs(rf) > max) { max = Math.abs(rf); }
        if (Math.abs(lb) > max) { max = Math.abs(lb); }
        if (Math.abs(rb) > max) { max = Math.abs(rb); }

        if (max > 1) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        try {
            jaglf.setX(lf);
            jagrf.setX(rf);
            jaglb.setX(lb);
            jagrb.setX(rb);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    public void smartInit() {
        SmartDashboard.putNumber("CW", clockwise);
        SmartDashboard.putNumber("GZ", GZ);
        SmartDashboard.putNumber("enX", enX.getRate());
        SmartDashboard.putNumber("enY", enY.getRate());
        SmartDashboard.putNumber("errorH" , errorH);
        SmartDashboard.putNumber("C", clockwise);
        SmartDashboard.putNumber("R", right);
        SmartDashboard.putNumber("F", forward); //here
        SmartDashboard.putNumber("heading", heading);
        SmartDashboard.putNumber("theta", theta);
        
        SmartDashboard.putNumber("kpR", KpR);
        SmartDashboard.putNumber("kpR", KpR);
        SmartDashboard.putNumber("kpX", KpX);
        SmartDashboard.putNumber("kpY", KpY);
        SmartDashboard.putNumber("kdX", KdX);
        SmartDashboard.putNumber("kdY", KdY);
        SmartDashboard.putNumber("KiR", KiR);
        
        SmartDashboard.putBoolean("openC", openC);
        SmartDashboard.putBoolean("openX", openX);
        SmartDashboard.putBoolean("openY", openY);
        SmartDashboard.putBoolean("FullOpen", open); 
        
       
    }
    public void smartPush(){
        SmartDashboard.putNumber("CW", clockwise);
        SmartDashboard.putNumber("GZ", GZ);
        SmartDashboard.putNumber("enX", enX.getRate());
        SmartDashboard.putNumber("enY", enY.getRate());
        SmartDashboard.putNumber("errorH" , errorH);
        SmartDashboard.putNumber("C", clockwise);
        SmartDashboard.putNumber("R", right);
        SmartDashboard.putNumber("F", forward); //here
        SmartDashboard.putNumber("heading", heading);
        SmartDashboard.putNumber("theta", theta);
    }
    public void smartPull(){
        
        KpR = SmartDashboard.getNumber("kpR");
        KpX = SmartDashboard.getNumber("kpX");
        KpY = SmartDashboard.getNumber("kpY");
        KdX = SmartDashboard.getNumber("kdX");
        KdY = SmartDashboard.getNumber("kdY");
        KiR = SmartDashboard.getNumber("KiR");
         
        if(SmartDashboard.getBoolean("FullOpen") ){
            openC = true;
            openX = true;
            openY = true;
        }
        SmartDashboard.getBoolean("openC", openC);
        SmartDashboard.getBoolean("openX", openX);
        SmartDashboard.getBoolean("openY", openY);
        
    }

    public void holoSensor() {
        readAll();
        double max;

        while (joy.getRawButton(5)) { turnLeft(); }
        while (joy.getRawButton(6)) { turnRight(); }

        try {
            double theta = getCRAngle() - initialHeading;
            double right = joy.getX() * Math.abs(joy.getX());
            double forward = -joy.getY() * Math.abs(joy.getY());
            double clockwise = joy.getZ() * Math.abs(joy.getZ());
            double temp;

            temp = forward * Math.cos(theta) + right * Math.sin(theta);
            right = -forward * Math.sin(theta) + right * Math.cos(theta);
            forward = temp;

            System.out.println(joy.getX() + ", " + joy.getY());

            double lf = forward + clockwise + right;
            double rf = forward - clockwise - right;
            double lb = forward + clockwise - right;
            double rb = forward - clockwise + right;

            System.out.println((Math.floor(lf * 1000) / 1000) + ", " + (Math.floor(rf * 1000) / 1000) + ", " + (Math.floor(lb * 1000) / 1000) + "," + (Math.floor(rb * 1000) / 1000));

            max = Math.abs(lf);

            if (Math.abs(rf) > max) { max = Math.abs(rf); }
            if (Math.abs(lb) > max) { max = Math.abs(lb); }
            if (Math.abs(rb) > max) { max = Math.abs(rb); }
            
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

    void turnLeft() {
        try {
            jaglf.setX(-shortwheel / longestwheel);
            jagrf.setX(-thirdlongwheel / longestwheel);
            jaglb.setX(secondlongwheel / longestwheel);
            jagrb.setX(-longestwheel);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    void turnRight() {
        try {
            jagrf.setX(-shortwheel / longestwheel);
            jaglf.setX(-thirdlongwheel / longestwheel);
            jagrb.setX(secondlongwheel / longestwheel);
            jaglb.setX(-longestwheel);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    public int byteCombo(byte num1, byte num2) { return ((num1 << 8) | num2 & 0x000000ff); }
    public int byteComboTenBit(byte num1, byte num2) { return (((num1 << 8) & 0xF) | (num2 << 8)); }

    public double atan2(double y, double x) {
        double pi = Math.PI;
        double pi2 = Math.PI / 2;
        if (x >= 0) { /* right half-plane */
            if (y >= 0) { /* 1st quadrant */
                if (y <= x) {
                    if (x == 0) {
                        return 0;  /* x & y both zero */
                    } else {
                        return f(y / x);
                    }
                } else {
                    return pi2 - f(x / y);
                }
            } else {  /* 4th quadrant */
                if (-y <= x) {
                    return -f(-y / x);
                } else {
                    return -pi2 + f(-x / y);
                }
            }
        } else {  /* left half-plane */
            if (y >= 0) {  /* 2nd quadrant */
                if (y >= -x) {
                    return pi2 + f(-x / y);
                } else {
                    return pi - f(-y / x);
                }
            } else {  /* 3rd quadrant */
                if (y >= x) {
                    return -pi + f(y / x);
                } else {
                    return -pi2 - f(x / y);
                }
            }
        }
    }

    double getAX() { return byteCombo(buffa[1], buffa[0]); }
    double getAY() { return byteCombo(buffa[3], buffa[2]); }

    double getGX() { return byteCombo(buffg[0], buffg[1]); }
    double getGY() { return byteCombo(buffg[2], buffg[3]); }
    double getGZ() { return (byteCombo(buffg[4], buffg[5])) / gScaleForSensor; }

    double getCX() { return byteCombo(buffc[0], buffc[1]); } // - 458
    double getCY() { return byteCombo(buffc[4], buffc[5]); } //  - 93

    double getGAngle() { return Math.toDegrees(atan2(getGY(), getGX())); }
    double getGRAngle() { return atan2(getGY(), getGX()); }

    double getCAngle() { return Math.toDegrees(atan2(getCY(), getCX())); }
    double getCRAngle() { return atan2(getCY(), getCX()); }
    double getCHRAngle() { return radRap(atan2(getCY(), getCX()) - initialHeading); }
    
    double getEnYD(){
        currentVeloY = enY.getRate();
        double s = currentVeloY-tempVeloHistoryY;
        tempVeloHistoryY = currentVeloY;
        return s;
    }
    double getEnXD(){
        currentVeloX = enX.getRate();
        double s = currentVeloX-tempVeloHistoryX;
        tempVeloHistoryX = currentVeloX;
        return s;
    }
    
    public double radRap(double d){ return (d> Math.PI) ? d-2*Math.PI : (d< -Math.PI) ? d+ 2*Math.PI : d; }
    double f(double t) {
        /* This provides 1/10 of a degree accuracy: */
        return -0.001096995 + t * (1.041963708 + t * (-0.196333807 + t * (-0.060821409)));
    }
    
    int minVal(int newVal, int oldMin) { return (newVal < oldMin) ? newVal : oldMin; }
    int maxVal(int newVal, int oldMax) { return (newVal > oldMax) ? newVal : oldMax; }

    double[] getDisp() { return new double[] {enX.getDistance(), enY.getDistance()}; }

    void setupPots() {
        enX.start();
        enY.start();
        enX.setDistancePerPulse(8.64360 / 90);
        enY.setDistancePerPulse(8.64360 / 90);
    }

    void setupCompass() {
        cwrite = new I2C(DigitalModule.getInstance(1), 0x3C);
        cread = new I2C(DigitalModule.getInstance(1), 0x3D);

        cwrite.write(0, 88);
        cwrite.write(1, 64);
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

        awrite.write(44, 0x0A);
        awrite.write(45, 0x08);
        awrite.write(49, 0x08);
    }

    void turnToRight() {
        double right = initialHeading - 90;
        if (getCAngle() != right) {
            double theta = getCAngle() - right;
            double pRate = p * theta;

            double lf = pRate;
            double rf = -pRate;
            double lb = pRate;
            double rb = -pRate;

            double max = Math.abs(lf);

            if (Math.abs(rf) > max) { max = Math.abs(rf); }
            if (Math.abs(lb) > max) { max = Math.abs(lb); }
            if (Math.abs(rb) > max) { max = Math.abs(rb); }

            if (max > 1) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }
            try {
                jaglf.setX(lf);
                jagrf.setX(rf);
                jaglb.setX(lb);
                jagrb.setX(rb);
            } catch (CANTimeoutException ex) {
                ex.printStackTrace();
            }
            diff -= theta;
        }
    }

    public void readAll() {
        cread.read(3, bc, buffc);
        gread.read(29, bg, buffg);
        aread.read(50, ba, buffa);
    }
    
    public double clamp(double value){
         return (value > 1) ? 1 : (value < -1) ? -1 : value;
    }

    public double dropNum(double num){ return num-Math.floor(num); }
}