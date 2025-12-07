package frc.util;


import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.TimeUnit;
import edu.wpi.first.units.measure.measure.Current;
import edu.wpi.first.wpilibj.Timer;

public class CurrentSpikeDetector {
    private final Timer timer = new Timer();
    private final Supplier<CurrentUnit> threshold;
    private final Supplier<TimeUnit> time;

    public CurrentSpikeDetector(Supplier<CurrentUnit> threshold, Supplier<TimeUnit> time) {
        this.threshold = threshold;
        this.time = time;
    }

    public void update(Current current) {
        if (current.gte(threshold.get())) {
            timer.start();
        } else {
            timer.stop();
            timer.reset();
        }
    }

    public boolean hasSpike() {
        return timer.hasElapsed(time.get().in(Seconds));
    }
}
