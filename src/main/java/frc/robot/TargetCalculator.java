package frc.robot;

import frc.vision.VisionServer;

public class TargetCalculator {

    private final VisionServer mVisionServer = VisionServer.getInstance(); 

    public TargetCalculator() {

    }

    public double[] getFirstTarget() {
        var targetObj = mVisionServer.getTarget();
        double[] arr = null;
        if (targetObj != null && targetObj.isValid()){
            var targetList=targetObj.getTargets();
            if (targetList.size()>0){
                var y=targetList.get(0).getY();
                var z=targetList.get(0).getZ();
                arr = new double[]{y,z};
            }
        }
        return arr;
    }
}
