package frc.robot.subsystems;

import frc.robot.subsystems.VisionModule;

public class VisionPipelineRunnable implements Runnable {

    VisionModule visionModule;

    public VisionPipelineRunnable(VisionModule visionModule) {
        this.visionModule = visionModule;
    }

    @Override
    public void run() {

        while(true) {
            visionModule.trackPipelineResults();
            try {
                Thread.sleep(10); // we don't expect to ever get here, actually
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

    }
}
