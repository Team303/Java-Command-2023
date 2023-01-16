package frc.robot.team303.lib.kinematics;

import java.util.HashMap;

import au.edu.federation.caliko.FabrikBone2D;
import au.edu.federation.caliko.FabrikChain2D;
import au.edu.federation.caliko.FabrikStructure2D;
import au.edu.federation.utils.Vec2f;


public class IKWrapper {
    FabrikStructure2D structure = new FabrikStructure2D();
    FabrikChain2D chain = new FabrikChain2D(); // Create a new 2D chain
    float armLength;
    float s1Ratio, s2Ratio, s3Ratio;
    HashMap<Integer, Float> segmentRatio = new HashMap<>();
    HashMap<Integer, Float> segmentLength = new HashMap<>();
    HashMap<Integer, Tuple<a,b>> segmentAngleConstraint = new HashMap<>();

   
    public void setArmLength(float lengthInches) {
        float armLength = lengthInches;
    }
    public float getArmLength() {
        return armLength;
    }
    public void setSegmentLengthRatio(int segmentIndex, float ratio) {
        segmentRatio.put(segmentIndex, ratio);
    }
    public float[] getSegmentLengthRatios() {
        float[] output;
        int x=0;
        for (float i : segmentRatio.values()) {
            output[x] = i;
            x++;
        }
    }
    public float getSegmentLengthRatio(int segmentIndex) {
        return segmentRatio.get(segmentIndex);
    }
    public void setSegmentLengths() {
        float[] output;
        int x=0;
        for (float i : segmentRatio.values()) {
            segmentLength.put(x,i*armLength);
            x++;
        }
    }
    public float[] getSegmentLengths() {
        float[] output;
        int x=0;
        for (float i : segmentLength.values()) {
            output[x] = i;
            x++;
        }
    }
    public float getSegmentLength(int segmentIndex) {
        return segmentLength.get(segmentIndex);
    }
    public void setAngleConstraint(int segmentIndex, float clockwiseConstraint, float counterclockwiseConstraint) {
        Float[] angleConstraints;
        angleConstraints[0] = clockwiseConstraint;
        angleConstraints[1] = counterclockwiseConstraint;
        segmentAngleConstraint.put(segmentIndex,angleConstraints);
    }
    public float[]

            
   }

      float s2AngleConstraint = 135.0f;
      float s3AngleConstraint = 135.0f;
      float s1Length = armLength * s1Ratio, s2Length = armLength * s2Ratio, s3Length = armLength * s3Ratio;
      final Vec2f UP = new Vec2f(0.0f, 1.0f);
      final Vec2f RIGHT = new Vec2f(1.0f, 0.0f);
      FabrikBone2D base = new FabrikBone2D(new Vec2f(), new Vec2f(0.0f, s1Length));
      base.setClockwiseConstraintDegs(45.0f);
      base.setAnticlockwiseConstraintDegs(45.0f);
      chain.addBone(base);
      chain.setBaseboneConstraintType(BaseboneConstraintType2D.GLOBAL_ABSOLUTE);
      chain.setBaseboneConstraintUV(UP);
      chain.addConsecutiveConstrainedBone(RIGHT, s2Length, s2AngleConstraint, s2AngleConstraint);
      chain.addConsecutiveConstrainedBone(new Vec2f(1.0f, -1.0f), s3Length, s3AngleConstraint, s3AngleConstraint);
      structure.addChain(chain);
      Vec2f screenMousePos = new Vec2f();
      Vec2f worldMousePos = new Vec2f();
      chain.solveForTarget(worldMousePos);



}
