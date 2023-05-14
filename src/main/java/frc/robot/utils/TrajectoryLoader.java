// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.stream.DoubleStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class TrajectoryLoader {

    private static Pose2d offset = new Pose2d(0.427, 0.302, Rotation2d.fromDegrees(0));

    private TrajectoryLoader() {
    }

    private static Trajectory createTrajectoryFromElements(double[] elements) {
        // Make sure that the elements have the correct length.

        // Create a list of states from the elements.
        List<Trajectory.State> states = new ArrayList<Trajectory.State>();
        for (int i = 0; i < elements.length; i += 7) {
            states.add(
                    new Trajectory.State(
                            elements[i],
                            elements[i + 1],
                            elements[i + 2],
                            new Pose2d(elements[i + 3], elements[i + 4], new Rotation2d(elements[i + 5])),
                            elements[i + 6]));
        }
        return new Trajectory(states);
    }

    public static double[] loadAutoTrajectory(String name) throws Exception {
        String fullPath = "Paths/" + name;
        String trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fullPath).toString();
        Scanner scan = new Scanner(new File(trajectoryPath));
        System.out.println("Found " + trajectoryPath);
        scan.nextLine(); // Skip line
        ArrayList<Double> points = new ArrayList<>();
        while (scan.hasNextLine()) {
            String[] frags = scan.nextLine().split(",");
            double tanx = Double.parseDouble(frags[2]); // tangent x
            double tany = Double.parseDouble(frags[3]); // tangent y

            points.add(Double.parseDouble(frags[0])); // x
            points.add(Double.parseDouble(frags[1]) + 8.2296); // y (Pathweaver saves with a weird offset so we
                                                               // circumvent this)
            points.add(Math.toDegrees(Math.atan2(tany, tanx))); // theta
            points.add(Math.sqrt(tanx * tanx + tany * tany)); // weight
        }
        double[] primitive = new double[points.size()];
        for (int i = 0; i < primitive.length; ++i) {
            primitive[i] = points.get(i);
        }
        return primitive;
    }
}
