package org.firstinspires.ftc.teamcode.autonPackage.IK;

public class Vector2 {

    public Vector2(double x, double y) {
        e0 = x;
        e1 = y;
    }
    public double e0;
    public double e1;

    public Vector2 subtract(Vector2 v) {
        return new Vector2(this.e0  - v.e0, this.e1 - v.e1);
    }
    Vector2 add(Vector2 v) {
        return new Vector2(this.e0  + v.e0, this.e1 + v.e1);
    }
    Vector2 divide(Vector2 v) {
        return new Vector2(this.e0  / v.e0, this.e1 / v.e1);
    }
    public Vector2 multiply(Double d) {
        return new Vector2(this.e0 * d, this.e1 * d);
    }
    public Vector2 multiply(Vector2 v) {
        return new Vector2(e0 * v.e0, e1 * v.e1);
    }
    public double magnitude() {
        return Math.sqrt((e0 * e0)+(e1 * e1));
    }
    Vector2 add(Double d) {
        return new Vector2(this.e0 + d, this.e1 + d);
    }


    void println() {
        System.out.println("x: " + e0 + " y: " + e1);
    }
}
