package frc.robot.MyLibrary;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class A_Motor extends TalonFX {
    public A_Motor (int id, String canBus) {
        super(id, canBus);
        init();
    }

    public A_Motor (int id) {
        super(id, "rio");
        init();
    }

    public void init() {
        setEncoderDegrees(0.0);
    }

    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    public void setBrake(boolean brakd) {
        super.setNeutralMode(brakd ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setRampRate(double rampRate) {
        OpenLoopRampsConfigs openlooprampsconfigs = new OpenLoopRampsConfigs();
        openlooprampsconfigs.VoltageOpenLoopRampPeriod = rampRate;
        super.getConfigurator().apply(openlooprampsconfigs);

        ClosedLoopRampsConfigs closedlooprampsconfigs = new ClosedLoopRampsConfigs();
        closedlooprampsconfigs.VoltageClosedLoopRampPeriod = rampRate;
        super.getConfigurator().apply(closedlooprampsconfigs);
    }

    public void setEncoderDegrees(double pos) {
        super.getConfigurator().setPosition(pos / 360.0);
    }

    public double getRPM() {
        return (super.getVelocity().getValueAsDouble() * 60.0);
    }

    public double getPosDeg() {
        return super.getRotorPosition().getValueAsDouble() * 360.0;
    }
}
