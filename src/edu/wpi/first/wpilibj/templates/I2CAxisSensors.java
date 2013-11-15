
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.I2C;

public class I2CAxisSensors {
    
    private I2C aread;
    private I2C awrite;
    private I2C cread;
    private I2C cwrite;
    private I2C gread;
    private I2C gwrite;
    private int bc;
    private int bg;
    private int ba;
    private byte buffc[];
    private byte buffg[];
    private byte buffa[];
    int gSampleRateDivider = 0;
    
    public I2CAxisSensors() {
        bc = 6;
        bg = 6;
        ba = 6;
        
        buffc = new byte[bc];
        buffg = new byte[bg];
        buffa = new byte[ba];
        
        setupCompass();
        setupGyro();
        setupAccel();
    }
    
    public double getAccelX() {
        return 1;
    }
    
    public double getAccelY() {
        return 1;
    }
    
    public double getCompX() {
        return 1;
    }
    
    public double getCompY() {
        return 1;
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
}
