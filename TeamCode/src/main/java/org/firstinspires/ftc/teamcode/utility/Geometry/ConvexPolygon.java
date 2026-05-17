package org.firstinspires.ftc.teamcode.utility.Geometry;

import com.acmerobotics.roadrunner.Pose2d;
import java.util.ArrayList;
import java.util.Arrays;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

public class ConvexPolygon {
    private Vector2D[] vertices;
    private int n;

    public ConvexPolygon(Vector2D[] vertices) {
        if (vertices == null || vertices.length < 3) {
            throw new IllegalArgumentException("ConvexPolygon requires at least 3 vertices");
        }
        this.n = vertices.length;
        this.vertices = sortVerticesCCW(vertices.clone());
        if (!isConvex()) {
            throw new IllegalArgumentException("Given vertices do not form a convex polygon");
        }
    }

    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3) {
        this(new Vector2D[]{p1, p2, p3});
    }

    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        this(new Vector2D[]{p1, p2, p3, p4});
    }

    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4, Vector2D p5) {
        this(new Vector2D[]{p1, p2, p3, p4, p5});
    }

    public ConvexPolygon(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4, Vector2D p5, Vector2D p6) {
        this(new Vector2D[]{p1, p2, p3, p4, p5, p6});
    }

    private Vector2D[] sortVerticesCCW(Vector2D[] verts) {
        double centerX = 0, centerY = 0;
        for (Vector2D p : verts) {
            centerX += p.getX();
            centerY += p.getY();
        }
        centerX /= verts.length;
        centerY /= verts.length;

        final double cx = centerX;
        final double cy = centerY;

        Arrays.sort(verts, (a, b) -> {
            double angleA = Math.atan2(a.getY() - cy, a.getX() - cx);
            double angleB = Math.atan2(b.getY() - cy, b.getX() - cx);
            return Double.compare(angleA, angleB);
        });

        if (signedArea(verts) < 0) {
            reverseArray(verts);
        }

        return verts;
    }

    private double signedArea(Vector2D[] verts) {
        double area = 0;
        int m = verts.length;
        for (int i = 0; i < m; i++) {
            Vector2D p1 = verts[i];
            Vector2D p2 = verts[(i + 1) % m];
            area += p1.getX() * p2.getY() - p2.getX() * p1.getY();
        }
        return area / 2;
    }

    private void reverseArray(Vector2D[] verts) {
        int i = 0, j = verts.length - 1;
        while (i < j) {
            Vector2D tmp = verts[i];
            verts[i] = verts[j];
            verts[j] = tmp;
            i++;
            j--;
        }
    }

    public boolean isConvex() {
        if (n < 3) return false;

        boolean hasPositive = false;
        boolean hasNegative = false;
        boolean isAllCollinear = true;

        for (int i = 0; i < n; i++) {
            Vector2D p1 = vertices[i];
            Vector2D p2 = vertices[(i + 1) % n];
            Vector2D p3 = vertices[(i + 2) % n];

            double cross = crossProduct(p1, p2, p3);
            if (Math.abs(cross) > 1e-10) {
                isAllCollinear = false;
                if (cross > 0) hasPositive = true;
                if (cross < 0) hasNegative = true;
            }

            if (hasPositive && hasNegative) return false;
        }
        if (isAllCollinear) return false;
        return true;
    }

    private double crossProduct(Vector2D p1, Vector2D p2, Vector2D p3) {
        double vx1 = p2.getX() - p1.getX();
        double vy1 = p2.getY() - p1.getY();
        double vx2 = p3.getX() - p2.getX();
        double vy2 = p3.getY() - p2.getY();
        return vx1 * vy2 - vy1 * vx2;
    }

    public ConvexPolygon inRelative(double x, double y, double theta) {
        Vector2D[] transformed = new Vector2D[n];
        for (int i = 0; i < n; i++) {
            transformed[i] = Vector2D.rotate(
                new Vector2D(vertices[i].getX() - x, vertices[i].getY() - y),
                theta
            );
        }
        return new ConvexPolygon(transformed);
    }

    public ConvexPolygon inRelative(Pose2d p) {
        return inRelative(p.position.x, p.position.y, p.heading.toDouble());
    }

    public ConvexPolygon inAbsolute(double x, double y, double theta) {
        Vector2D[] transformed = new Vector2D[n];
        for (int i = 0; i < n; i++) {
            Vector2D rotated = Vector2D.rotate(vertices[i], -theta);
            transformed[i] = new Vector2D(rotated.getX() + x, rotated.getY() + y);
        }
        return new ConvexPolygon(transformed);
    }

    public ConvexPolygon inAbsolute(Pose2d p) {
        return inAbsolute(p.position.x, p.position.y, p.heading.toDouble());
    }

    public Vector2D NearestVectorFrom(double x, double y) {
        return NearestVectorFrom(new Vector2D(x, y));
    }

    public Vector2D NearestVectorFrom(Vector2D p) {
        if (Contains(p)) {
            return new Vector2D(0, 0);
        }

        Vector2D nearest = null;
        double minDistSq = Double.MAX_VALUE;

        for (int i = 0; i < n; i++) {
            Vector2D v1 = vertices[i];
            Vector2D v2 = vertices[(i + 1) % n];

            Vector2D closest = closestPointOnSegment(p, v1, v2);
            double distSq = (closest.getX() - p.getX()) * (closest.getX() - p.getX()) +
                            (closest.getY() - p.getY()) * (closest.getY() - p.getY());

            if (distSq < minDistSq) {
                minDistSq = distSq;
                nearest = closest;
            }
        }

        return new Vector2D(nearest.getX() - p.getX(), nearest.getY() - p.getY());
    }

    private Vector2D closestPointOnSegment(Vector2D p, Vector2D v1, Vector2D v2) {
        double dx = v2.getX() - v1.getX();
        double dy = v2.getY() - v1.getY();
        double lenSq = dx * dx + dy * dy;

        if (lenSq < 1e-10) return new Vector2D(v1.getX(), v1.getY());

        double t = Math.max(0, Math.min(1, ((p.getX() - v1.getX()) * dx + (p.getY() - v1.getY()) * dy) / lenSq));

        return new Vector2D(v1.getX() + t * dx, v1.getY() + t * dy);
    }

    public boolean Contains(double x, double y) {
        return Contains(new Vector2D(x, y));
    }

    public boolean Contains(Vector2D p) {
        for (int i = 0; i < n; i++) {
            Vector2D v1 = vertices[i];
            Vector2D v2 = vertices[(i + 1) % n];

            if (crossProduct(v1, v2, p) < -1e-10) {
                return false;
            }
        }
        return true;
    }

    public boolean Contains(ConvexPolygon other) {
        for (int i = 0; i < other.n; i++) {
            if (!Contains(other.vertices[i])) {
                return false;
            }
        }
        return true;
    }

    public boolean IsIntersected(ConvexPolygon other) {
        for (int i = 0; i < n; i++) {
            if (other.Contains(vertices[i])) {
                return true;
            }
        }

        for (int i = 0; i < other.n; i++) {
            if (Contains(other.vertices[i])) {
                return true;
            }
        }

        for (int i = 0; i < n; i++) {
            Vector2D a1 = vertices[i];
            Vector2D a2 = vertices[(i + 1) % n];

            for (int j = 0; j < other.n; j++) {
                Vector2D b1 = other.vertices[j];
                Vector2D b2 = other.vertices[(j + 1) % other.n];

                if (segmentsIntersect(a1, a2, b1, b2)) {
                    return true;
                }
            }
        }

        return false;
    }

    private boolean segmentsIntersect(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        double d1 = isLeft(p3, p4, p1);
        double d2 = isLeft(p3, p4, p2);
        double d3 = isLeft(p1, p2, p3);
        double d4 = isLeft(p1, p2, p4);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }

        if (Math.abs(d1) < 1e-10 && onSegment(p3, p4, p1)) return true;
        if (Math.abs(d2) < 1e-10 && onSegment(p3, p4, p2)) return true;
        if (Math.abs(d3) < 1e-10 && onSegment(p1, p2, p3)) return true;
        if (Math.abs(d4) < 1e-10 && onSegment(p1, p2, p4)) return true;

        return false;
    }

    private boolean onSegment(Vector2D p1, Vector2D p2, Vector2D q) {
        return Math.min(p1.getX(), p2.getX()) - 1e-10 <= q.getX() &&
               q.getX() <= Math.max(p1.getX(), p2.getX()) + 1e-10 &&
               Math.min(p1.getY(), p2.getY()) - 1e-10 <= q.getY() &&
               q.getY() <= Math.max(p1.getY(), p2.getY()) + 1e-10;
    }

    public ConvexPolygon IntersectWith(ConvexPolygon clip) {
        if (!this.IsIntersected(clip)) {
            throw new IllegalStateException("Polygons do not intersect");
        }
        ArrayList<Vector2D> subject = new ArrayList<>();
        for (Vector2D p : this.vertices) subject.add(p);

        for (int i = 0; i < clip.n; i++) {
            if (subject.size() < 3) break;

            Vector2D clipEdgeStart = clip.vertices[i];
            Vector2D clipEdgeEnd = clip.vertices[(i + 1) % clip.n];

            ArrayList<Vector2D> output = new ArrayList<>();

            for (int j = 0; j < subject.size(); j++) {
                Vector2D current = subject.get(j);
                Vector2D next = subject.get((j + 1) % subject.size());

                boolean currentInside = isLeft(clipEdgeStart, clipEdgeEnd, current) >= -1e-10;
                boolean nextInside = isLeft(clipEdgeStart, clipEdgeEnd, next) >= -1e-10;

                if (currentInside) {
                    if (nextInside) {
                        output.add(next);
                    } else {
                        Vector2D intersection = lineIntersection(
                            current, next, clipEdgeStart, clipEdgeEnd
                        );
                        output.add(intersection);
                    }
                } else {
                    if (nextInside) {
                        Vector2D intersection = lineIntersection(
                            current, next, clipEdgeStart, clipEdgeEnd
                        );
                        output.add(intersection);
                        output.add(next);
                    }
                }
            }

            subject = output;
        }

        if (subject.size() < 3) {
            throw new IllegalStateException("Intersection is empty or degenerate (less than 3 vertices)");
        }

        Vector2D[] result = removeCollinearPoints(subject.toArray(new Vector2D[0]));
        if (result.length < 3) {
            throw new IllegalStateException("Intersection is empty or degenerate (less than 3 vertices)");
        }
        return new ConvexPolygon(result);
    }

    private double isLeft(Vector2D v1, Vector2D v2, Vector2D p) {
        return (v2.getX() - v1.getX()) * (p.getY() - v1.getY()) -
               (v2.getY() - v1.getY()) * (p.getX() - v1.getX());
    }

    private Vector2D lineIntersection(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4) {
        double x1 = p1.getX(), y1 = p1.getY();
        double x2 = p2.getX(), y2 = p2.getY();
        double x3 = p3.getX(), y3 = p3.getY();
        double x4 = p4.getX(), y4 = p4.getY();

        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denom) < 1e-10) {
            return new Vector2D(x1, y1);
        }

        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;

        return new Vector2D(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
    }

    private Vector2D[] removeCollinearPoints(Vector2D[] pts) {
        int n = pts.length;
        if (n <= 3) return pts;

        ArrayList<Vector2D> result = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            Vector2D prev = pts[(i - 1 + n) % n];
            Vector2D curr = pts[i];
            Vector2D next = pts[(i + 1) % n];

            if (Math.abs(crossProduct(prev, curr, next)) > 1e-10) {
                result.add(curr);
            }
        }

        return result.toArray(new Vector2D[0]);
    }

    public int getVertexCount() {
        return n;
    }

    public Vector2D[] getVertices() {
        return vertices.clone();
    }

    public Vector2D getVertex(int index) {
        if (index < 0 || index >= n) {
            throw new IndexOutOfBoundsException("Vertex index out of bounds");
        }
        return vertices[index];
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("ConvexPolygon[");
        for (int i = 0; i < n; i++) {
            sb.append(vertices[i].toString());
            if (i < n - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
}