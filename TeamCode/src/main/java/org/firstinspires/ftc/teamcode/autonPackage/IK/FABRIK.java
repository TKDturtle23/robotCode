package org.firstinspires.ftc.teamcode.autonPackage.IK;
import java.util.List;
public class FABRIK {
    double angleBTP(Vector2 p1, Vector2 p2) {

        // tan-1 opposite over adjacent (by-ay)/(bx-ax

        return Math.atan((p2.e1 - p1.e1) / (p2.e0 - p1.e0));
    }
    public FABRIK(List<Joint> iJoints, List<Double> iLengths) {

        LJoints = iJoints;

        Ldistances = iLengths;
    }

    List<Joint> LJoints;
    List<Double> Ldistances;

    Vector2 findDistance(Vector2 v1, Vector2 v2) {
        return new Vector2(v2.e0 - v1.e0, v2.e1 - v1.e1);
        //double distance =  Math.sqrt(Math.pow(vDistance.e0, 2) + Math.pow(vDistance.e1, 2));
    }
    void setJListFromArr(List<Joint> list, Joint[] objects) {
        for (int i = 0; i < objects.length; i++) {
            list.set(i, objects[i]);
        }
    }
    double pTheorem(Vector2 v) {
        return Math.sqrt(Math.pow(v.e0, 2) + Math.pow(v.e1, 2));
    }
    List<Joint> IK(Vector2 startLocation, Vector2 endLocation, long maxIterations, double tolerance) {
        Joint[] Joints = LJoints.toArray(new Joint[LJoints.size()]);
        Double[] distances = Ldistances.toArray(new Double[Ldistances.size()]);

        Vector2 iDistance = findDistance(startLocation, endLocation);
        Vector2 distance = new Vector2(Math.abs(iDistance.e0), Math.abs(iDistance.e1));
        //distances.clear();

        if (distances.length != LJoints.size() - 1) {
            return null;
        }

        if (!isReachable(pTheorem(distance))) {


            for (int i = 0; i < Joints.length - 1; i++){
                double r = pTheorem(findDistance(endLocation, Joints[i].location));
                double k = distances[i] / r;

                Joints[i + 1].location =  Joints[i].location.multiply((1 - k)).add(endLocation.multiply(k));
            }
            for (int i = 0; i < Joints.length; i++) {
                LJoints.set(i, Joints[i]);

            }

            setJListFromArr(LJoints, Joints);
            return LJoints;
        }
        // if it is reachable

        Vector2 b = Joints[0].location;
        double dif = pTheorem(findDistance(Joints[Joints.length - 1].location, endLocation));
        int iterations = 0;
        while (dif > tolerance && iterations < maxIterations) {
            // Stage 1 forward reaching
            Joints[Joints.length - 1].location = endLocation;
            for (int i = Joints.length - 2; i >= 0; i--) {


                double r = pTheorem(findDistance(Joints[i + 1].location, Joints[i].location));
                double k = distances[i] / r;

                Joints[i].location =  Joints[i + 1].location.multiply((1 - k)).add(Joints[i].location.multiply(k));
            }
            // stage 2 backward reaching
            Joints[0].location = b;

            for (int i = 0; i < Joints.length - 1; i++) {


                double r = pTheorem( findDistance(Joints[i + 1].location, Joints[i].location) );
                double k = distances[i] / r;

                Joints[i + 1].location =  Joints[i].location.multiply( ( 1 - k ) ).add( Joints[i + 1].location.multiply(k) );
            }

            dif = pTheorem(findDistance(Joints[Joints.length - 1].location, endLocation));
            iterations++;
        }




        setJListFromArr(LJoints, Joints);
        return LJoints;
    }

    boolean isReachable(double distance) {
        double length = 0;
        for (Double aDouble : Ldistances) {
            length += aDouble;
        }
        return !(length < distance);
    }
}
