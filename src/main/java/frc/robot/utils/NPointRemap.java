package frc.robot.utils;

import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Scanner;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.Filesystem;

public class NPointRemap {
    private ArrayList<Point> points;
    private boolean isSorted;
    private String filepath;

    private class PointComparator implements Comparator<Point> {
        @Override
        public int compare(Point p1, Point p2) {
            return (int) Math.signum(p2.x - p1.x); // Descending Order
        }
    }

    public NPointRemap() {
        points = new ArrayList<Point>();
        isSorted = false;
    }

    public NPointRemap(String filePath) {
        points = new ArrayList<Point>();
        isSorted = false;
        this.filepath = filePath;
        readFromFile();
    }

    public NPointRemap compile() {
        Collections.sort(points, new PointComparator());
        isSorted = true;
        return this;
    }

    public NPointRemap addPoint(double x, double y) {
        isSorted = false;
        points.add(new Point(x, y));
        return this;
    }

    public double calcY(double x) {
        if (points.size() >= 2) {
            if (!isSorted) {
                compile();
            }
            for (int i = 0; i < points.size() - 1; i++) {
                if (x <= points.get(i + 1).x) {
                    return MathUtils.remap(points.get(i + 1).x, points.get(i).x, x, points.get(i + 1).y,
                            points.get(i).y);
                }
            }
            int size = points.size();
            return MathUtils.remap(points.get(size - 1).x, points.get(size - 2).x, x, points.get(size - 1).y,
                    points.get(size - 2).y);
        }
        return 0;
    }

    public void readFromFile() {
        String path = Filesystem.getDeployDirectory().toPath().resolve(filepath).toString();
        File file = new File(path);
        if (file.exists()) {
            try {
                Scanner sc = new Scanner(file);
                // System.out.println(sc.nextLine());
                while (true) {
                    if (sc.hasNext()) {
                        double x = sc.nextDouble();
                        double y = sc.nextDouble();
                        addPoint(x, y);
                        // System.out.println(x + " " + y);
                    } else {
                        break;
                    }
                }
                isSorted = true;
                sc.close();
                return;
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else {
            System.out.println(path + " doesn't exist.");
        }
    }
}