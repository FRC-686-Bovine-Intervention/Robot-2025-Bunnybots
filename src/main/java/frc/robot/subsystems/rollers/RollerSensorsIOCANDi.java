package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;

import frc.robot.constants.HardwareDevices;

public class RollerSensorsIOCANDi implements RollerSensorsIO {
    private final CANdi candi = HardwareDevices.candiID.candi();

    private final StatusSignal<S1StateValue> s1Signal;
    private final StatusSignal<S2StateValue> s2Signal;

    private final BaseStatusSignal[] refreshSignals;

    public RollerSensorsIOCANDi() {
        var config = new CANdiConfiguration();

        // config.DigitalInputs
        //     .withS1FloatState(S1FloatStateValue.)
        // ;

        this.candi.getConfigurator().apply(config);

        this.s1Signal = this.candi.getS1State();
        this.s2Signal = this.candi.getS2State();

        this.refreshSignals = new BaseStatusSignal[] {
            this.s1Signal,
            this.s2Signal,
        };
    }

    @Override
    public void updateInputs(RollerSensorsIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.refreshSignals);

        inputs.kickerSensor = this.s2Signal.getValue() == S2StateValue.High;
    }
}
