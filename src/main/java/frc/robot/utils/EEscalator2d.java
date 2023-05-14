// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.config.Constants;

/** Add your docs here. */
public class EEscalator2d {

        Mechanism2d mech;
        MechanismLigament2d esc_t, lnx_t, wrs_t;
        MechanismLigament2d esc_m, lnx_m, wrs_m;

        public EEscalator2d() {
                mech = new Mechanism2d(2, 1.5, new Color8Bit(0, 0, 0));
                MechanismRoot2d root = mech.getRoot("SS", 0.8, 0);
                MechanismLigament2d bumpers = root.append(
                                new MechanismLigament2d("bumpers", 0.839, 180, 120, new Color8Bit(0, 255, 0)));
                MechanismLigament2d poleDist1 = root.append(
                                new MechanismLigament2d("pole dist 1", 0.58, 0, 1, new Color8Bit(80, 80, 80)));
                MechanismLigament2d poleDist2 = root.append(
                                new MechanismLigament2d("pole dist 2", 1.01, 0, 1, new Color8Bit(80, 80, 80)));
                MechanismLigament2d probe = root.append(
                                new MechanismLigament2d("probe", 1.3, 0, 20, new Color8Bit(80, 80, 80)));

                MechanismLigament2d shelf1h = poleDist1.append(
                                new MechanismLigament2d("shelf 1 h", 0.60, 90, 40, new Color8Bit(80, 80, 80)));
                MechanismLigament2d pole1 = poleDist1.append(
                                new MechanismLigament2d("pole 1", 0.87, 90, 20, new Color8Bit(80, 80, 80)));

                MechanismLigament2d shelf2h = poleDist2.append(
                                new MechanismLigament2d("shelf 2 h", 0.90, 90, 40, new Color8Bit(80, 80, 80)));
                MechanismLigament2d pole2 = poleDist2.append(
                                new MechanismLigament2d("pole 2", 1.17, 90, 20, new Color8Bit(80, 80, 80)));

                MechanismLigament2d shelf1 = shelf1h.append(
                                new MechanismLigament2d("shelf 1", 0.215, 90, 1, new Color8Bit(80, 80, 80)));
                MechanismLigament2d shelf2 = shelf2h.append(
                                new MechanismLigament2d("shelf 2", 0.215, 90, 1, new Color8Bit(80, 80, 80)));
                MechanismLigament2d shelf1f = shelf1.append(
                                new MechanismLigament2d("shelf 1 f", 0.60, 90, 1, new Color8Bit(80, 80, 80)));
                MechanismLigament2d shelf2f = shelf2.append(
                                new MechanismLigament2d("shelf 2 f", 0.90, 90, 1, new Color8Bit(80, 80, 80)));

                esc_m = bumpers
                                .append(new MechanismLigament2d("escalator measured",
                                                Constants.escalatorStowed_m * Constants.escalator_angle_scaling,
                                                52 - 180, 25,
                                                new Color8Bit(255, 0, 0)));
                lnx_m = esc_m
                                .append(new MechanismLigament2d("linex measured", Constants.linxStowed_m, -52, 8,
                                                new Color8Bit(255, 0, 0)));
                wrs_m = lnx_m
                                .append(new MechanismLigament2d("wrist measured", 0.299, Constants.wristStowed_deg, 8,
                                                new Color8Bit(255, 0, 0)));
                wrs_m.append(new MechanismLigament2d("intake measured 1", 0.197, 207.72, 6,
                                new Color8Bit(255, 0, 0)));
                wrs_m.append(new MechanismLigament2d("intake measured 2", 0.082, 54, 6,
                                new Color8Bit(255, 0, 0)));

                esc_t = bumpers
                                .append(new MechanismLigament2d("escalator target",
                                                Constants.escalatorStowed_m * Constants.escalator_angle_scaling,
                                                52 - 180, 30,
                                                new Color8Bit(255, 255, 255)));
                lnx_t = esc_t
                                .append(new MechanismLigament2d("linex target", Constants.linxStowed_m, -52, 10,
                                                new Color8Bit(255, 255, 255)));
                wrs_t = lnx_t
                                .append(new MechanismLigament2d("wrist target", 0.299, Constants.wristStowed_deg, 10,
                                                new Color8Bit(255, 255, 255)));
                wrs_t.append(new MechanismLigament2d("intake target 1", 0.197, 207.72, 8,
                                new Color8Bit(255, 255, 255)));
                wrs_t.append(new MechanismLigament2d("intake target 2", 0.082, 54, 8,
                                new Color8Bit(255, 255, 255)));

                SmartDashboard.putData("Mech2d", mech);
        }

        public void drawTarget(double escP, double lnxP, double wrsP) {
                esc_t.setLength(escP * Constants.escalator_angle_scaling);
                lnx_t.setLength(lnxP);
                wrs_t.setAngle(wrsP);
        }

        public void drawMeasured(double escP, double lnxP, double wrsP) {
                esc_m.setLength(escP * Constants.escalator_angle_scaling);
                lnx_m.setLength(lnxP);
                wrs_m.setAngle(wrsP);
        }

}
