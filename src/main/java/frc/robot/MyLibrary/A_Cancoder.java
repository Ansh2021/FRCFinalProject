package frc.robot.MyLibrary;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class A_Cancoder extends CANcoder {
    
    public boolean inverted = false;

    public A_Cancoder (int id, String canbus) {
        super(id, canbus);
        init(); 
    }

    public A_Cancoder(int id) {
        super(id, "rio");
        init();
    }

    public void init() {
        super.getConfigurator().apply(new CANcoderConfiguration());
        super.clearStickyFaults();
    }

    public void setPosDeg(double pos) {
        super.setPosition(pos / (2.0 * Math.PI));
    }

    public double givePosDeg() {
        if (!inverted) {
            return super.getAbsolutePosition().getValueAsDouble() * 360.0;
        } else {
            return -super.getAbsolutePosition().getValueAsDouble() * 360.0;
        }
    }

    public void resetPos() {
        super.setPosition(0.0);
    }
}