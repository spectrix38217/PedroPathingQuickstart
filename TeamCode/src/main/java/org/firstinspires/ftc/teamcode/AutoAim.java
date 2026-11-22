package org.firstinspires.ftc.teamcode;

public class AutoAim {

    public static class Point3D {
        public double x, y, headingDeg;
        public Point3D(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }
    }

    public static class Triangle {
        public Point3D p1, p2, p3;
        public Triangle(Point3D p1, Point3D p2, Point3D p3) {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
        }

        public boolean contains(double x, double y) {
            double denom = (p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y);
            if (Math.abs(denom) < 1e-9) return false;
            double l1 = ((p2.y - p3.y) * (x - p3.x) + (p3.x - p2.x) * (y - p3.y)) / denom;
            double l2 = ((p3.y - p1.y) * (x - p3.x) + (p1.x - p3.x) * (y - p3.y)) / denom;
            double l3 = 1.0 - l1 - l2;
            return l1 >= -1e-5 && l2 >= -1e-5 && l3 >= -1e-5;
        }

        public double interpolateHeading(double x, double y) {
            double denom = (p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y);
            double l1 = ((p2.y - p3.y) * (x - p3.x) + (p3.x - p2.x) * (y - p3.y)) / denom;
            double l2 = ((p3.y - p1.y) * (x - p3.x) + (p1.x - p3.x) * (y - p3.y)) / denom;
            double l3 = 1.0 - l1 - l2;
            return l1 * p1.headingDeg + l2 * p2.headingDeg + l3 * p3.headingDeg;
        }
    }

    // Define Area 1 points as provided by the user:
    // x: 27.83 y: 1.97, heading 55.19
    // x: 35.08 y: 17.24 heading: 58.17
    // x: 53.50 y: -2.32 heading: 90.87
    // x: 54.09 y: 14.29 heading: 88.28
    // x: 82.24 y: 1.57 heading: 118.26
    // x: 75.12 y: 17.56 heading: 122.2
    private static final Point3D A1 = new Point3D(27.83, 1.97, 55.19);
    private static final Point3D A2 = new Point3D(35.08, 17.24, 58.17);
    private static final Point3D A3 = new Point3D(53.50, -2.32, 90.87);
    private static final Point3D A4 = new Point3D(54.09, 14.29, 88.28);
    private static final Point3D A5 = new Point3D(82.24, 1.57, 118.26);
    private static final Point3D A6 = new Point3D(75.12, 17.56, 122.2);

    // Triangles for Area 1
    private static final Triangle[] AREA_1_TRIANGLES = new Triangle[] {
        new Triangle(A1, A2, A3),
        new Triangle(A2, A3, A4),
        new Triangle(A2, A4, A6),
        new Triangle(A3, A4, A5),
        new Triangle(A4, A5, A6)
    };

    // Define Area 2 points as provided by the user:
    // x: 28.11 y: 125.16 heading: 299.82
    // x: 35.67 y: 111.34 heading: 300.24
    // x: 56.06 y: 129.41 heading: 270.27
    // x: 54.72 y: 110.75 heading: 267.95
    // x: 78.31 y: 110.51 heading: 233.61
    // x: 80.87 y: 125.94 heading: 241.95
    private static final Point3D B1 = new Point3D(28.11, 125.16, 299.82);
    private static final Point3D B2 = new Point3D(35.67, 111.34, 300.24);
    private static final Point3D B3 = new Point3D(56.06, 129.41, 270.27);
    private static final Point3D B4 = new Point3D(54.72, 110.75, 267.95);
    private static final Point3D B5 = new Point3D(80.87, 125.94, 241.95);
    private static final Point3D B6 = new Point3D(78.31, 110.51, 233.61);

    // Triangles for Area 2
    private static final Triangle[] AREA_2_TRIANGLES = new Triangle[] {
        new Triangle(B1, B2, B3),
        new Triangle(B2, B3, B4),
        new Triangle(B2, B4, B6),
        new Triangle(B3, B4, B5),
        new Triangle(B4, B5, B6)
    };

    /**
     * Checks if the given (x, y) position is inside any defined auto-aim area.
     */
    public static boolean isInArea(double x, double y) {
        for (Triangle t : AREA_1_TRIANGLES) {
            if (t.contains(x, y)) {
                return true;
            }
        }
        for (Triangle t : AREA_2_TRIANGLES) {
            if (t.contains(x, y)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Calculates the interpolated heading in radians for the given (x, y) position
     * when inside a valid auto-aim area.
     */
    public static double getInterpolatedHeadingRadians(double x, double y) {
        for (Triangle t : AREA_1_TRIANGLES) {
            if (t.contains(x, y)) {
                double headingDeg = t.interpolateHeading(x, y);
                return Math.toRadians(headingDeg);
            }
        }
        for (Triangle t : AREA_2_TRIANGLES) {
            if (t.contains(x, y)) {
                double headingDeg = t.interpolateHeading(x, y);
                return Math.toRadians(headingDeg);
            }
        }
        return 0.0;
    }
}
