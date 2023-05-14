// Copyright (c) Matey and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Based on https://github.com/marcbone/s_curve

package frc.robot.utils;

/** Arrrrrrrr */
public class Scurvy {

    public static class Constraints {
        public double vel, acc, jrk;

        public Constraints(double vel, double acc, double jrk) {
            this.vel = vel;
            this.acc = acc;
            this.jrk = jrk;
        }

        @Override
        public String toString() {
            return "vel: " + vel + ", acc: " + acc + ", jrk: " + jrk;
        }

        public Constraints copy() {
            return new Constraints(vel, acc, jrk);
        }
    }

    public static class TimeIntervals {
        public double tJa, tJd, tA, tV, tD;

        public TimeIntervals(double tJa, double tJd, double tA, double tV, double tD) {
            this.tJa = tJa;
            this.tJd = tJd;
            this.tA = tA;
            this.tV = tV;
            this.tD = tD;
        }

        public double totalTime_s() {
            return tA + tV + tD;
        }

        public boolean isMaxAccelNotReached() {
            return tA < (2 * tJa) || tD < (2 * tJd);
        }
    }

    public static class State {
        public final double p, v, a;

        public State(double p, double v, double a) {
            this.p = p;
            this.v = v;
            this.a = a;
        }

        public State(double p, double v) {
            this(p, v, 0);
        }

        public State(double p) {
            this(p, 0, 0);
        }
    }

    public static class Params {
        public final TimeIntervals intervals;
        public final Constraints cons;
        public final double jMax, jMin, aLimA, aLimD, vLim;
        public final boolean forwards;
        public final State start, end;

        public Params(Constraints cons, State start, State end) {
            this.start = start;
            this.end = end;
            forwards = end.p >= start.p;
            this.cons = cons;
            this.intervals = calcIntervals();
            this.aLimA = cons.jrk * intervals.tJa;
            this.aLimD = -cons.jrk * intervals.tJd;
            this.vLim = v0() + (intervals.tA - intervals.tJa) * aLimA;
            this.jMax = cons.jrk;
            this.jMin = -cons.jrk;
        }

        private TimeIntervals calcIntervals() {
            if (start.p == end.p)
                return new TimeIntervals(0, 0, 0, 0, 0);
            return calcIntervals_case1();
        }

        private boolean isAccMaxNotReached() {
            return (cons.vel - v0()) * cons.jrk < MathUtils.sq(cons.acc);
        }

        private boolean isAccMinNotReached() {
            return (cons.vel - v1()) * cons.jrk < MathUtils.sq(cons.acc);
        }

        private TimeIntervals calcIntervals_case1() {
            TimeIntervals times = new TimeIntervals(0, 0, 0, 0, 0);
            Constraints newCons = cons.copy();

            if (isAccMaxNotReached()) {
                times.tJa = Math.sqrt((newCons.vel - v0()) / newCons.jrk);
                times.tA = 2 * times.tJa;
            } else {
                times.tJa = newCons.acc / newCons.jrk;
                times.tA = times.tJa + (newCons.vel - v0()) / newCons.acc;
            }

            if (isAccMinNotReached()) {
                times.tJd = Math.sqrt((newCons.vel - v1()) / newCons.jrk);
                times.tD = 2. * times.tJd;
            } else {
                times.tJd = newCons.acc / newCons.jrk;
                times.tD = times.tJd + (newCons.vel - v1()) / newCons.acc;
            }

            times.tV = h() / newCons.vel - times.tA / 2.0 * (1.0 + v0() / newCons.vel)
                    - times.tD / 2.0 * (1. + v1() / newCons.vel);
            if (times.tV <= 0) {
                return calcIntervals_case2(cons, 0);
            }
            if (times.isMaxAccelNotReached()) {
                newCons.acc *= 0.5;
                if (newCons.acc > 0.01) {
                    return calcIntervals_case2(newCons, 0);
                }
                newCons.acc = 0;
            }
            times = handleNegativeAccTime(times, cons);
            // System.out.printf("%.2f %.2f %.2f\n", times.tA, times.tV, times.tD);

            return times;
        }

        private TimeIntervals handleNegativeAccTime(TimeIntervals times, Constraints newCons) {

            if (times.tA < 0) {
                times.tJa = 0;
                times.tA = 0;
                times.tD = 2 * h() / (v0() + v1());
                times.tJd = (newCons.jrk * h() - Math.sqrt(
                        newCons.jrk * (newCons.jrk * MathUtils.sq(h()) + MathUtils.sq(v0() + v1()) * (v1() - v0()))))
                        / (newCons.jrk * (v1() + v0()));
            }
            if (times.tD < 0) {
                times.tJa = 0;
                times.tD = 0;
                times.tA = 2 * h() / (v0() + v1());
                times.tJd = (newCons.jrk * h() - Math.sqrt(
                        newCons.jrk * (newCons.jrk * MathUtils.sq(h()) - MathUtils.sq(v0() + v1()) * (v1() - v0()))))
                        / (newCons.jrk * (v1() + v0()));
            }

            return times;
        }

        private TimeIntervals calcIntervals_case2(Constraints cons, int recursion_depth) {
            recursion_depth += 1;
            TimeIntervals times = getTimes_case2(cons);
            Constraints newCons = cons.copy();
            if (times.isMaxAccelNotReached()) {
                newCons.acc *= 0.5;
                if (newCons.acc > 0.01) {
                    return calcIntervals_case2(newCons, recursion_depth);
                }
                newCons.acc = 0;
            }
            times = handleNegativeAccTime(times, newCons);
            if (recursion_depth != 1) {
                newCons.acc *= 2;
            }
            return calcIntervals_case2_precise(newCons, recursion_depth);
        }

        private TimeIntervals getTimes_case2(Constraints newCons) {
            double tJa = newCons.acc / newCons.jrk;
            double tJd = newCons.acc / newCons.jrk;
            double delta = MathUtils.qu(newCons.acc) / MathUtils.sq(newCons.jrk)
                    + 2 * (MathUtils.sq(v0()) + MathUtils.sq(v1()))
                    + newCons.acc * (4 * h() - 2 * newCons.acc / newCons.jrk * (v0() + v1()));
            double tA = (MathUtils.sq(newCons.acc) / newCons.jrk - 2 * v0() + Math.sqrt(delta)) / (2 * newCons.acc);
            double tD = (MathUtils.sq(newCons.acc) / newCons.jrk - 2 * v1() + Math.sqrt(delta)) / (2 * newCons.acc);
            double tV = 0.0;
            return new TimeIntervals(tJa, tJd, tA, tV, tD);
        }

        private TimeIntervals calcIntervals_case2_precise(Constraints cons, int _recursion_depth) {
            _recursion_depth += 1;
            TimeIntervals times = getTimes_case2(cons);
            Constraints newCons = cons.copy();
            if (times.isMaxAccelNotReached()) {
                newCons.acc *= 0.99;
                if (newCons.acc > 0.01) {
                    return calcIntervals_case2_precise(newCons, _recursion_depth);
                }
                newCons.acc = 0;
            }
            return handleNegativeAccTime(times, newCons);
        }

        public double h() {
            return Math.abs(start.p - end.p);
        }

        public double p0() {
            return forwards ? start.p : end.p;
        }

        public double p1() {
            return forwards ? end.p : start.p;
        }

        public double v0() {
            return forwards ? start.v : -end.v;
        }

        public double v1() {
            return forwards ? end.v : -start.v;
        }

        public double a0() {
            return forwards ? start.a : -end.a;
        }

        public double a1() {
            return forwards ? end.a : -start.a;
        }
    }

    public Params params;

    public Scurvy(Constraints cons, State start, State end) {
        params = new Params(cons, start, end);
    }

    public double getTotalTime() {
        return params.intervals.totalTime_s();
    }

    public State calculate(double time) {
        TimeIntervals times = params.intervals;
        double tt = times.totalTime_s();
        double t = params.forwards ? time : tt - time;
        double p, v, a;
        double vsign = params.forwards ? 1.0 : -1.0;
        if (t < 0.0) {
            p = params.p0();
            v = params.v0();
            a = 0.0;
        } else if (t <= times.tJa) {
            p = params.p0() + params.v0() * t + params.jMax * t * t * t / 6.0;
            v = params.v0() + params.jMax * t * t / 2.0;
            a = params.jMax * t;
        } else if (t <= times.tA - times.tJa) {
            p = params.p0() + params.v0() * t
                    + params.aLimA / 6.0 * (3. * t * t - 3. * times.tJa * t + MathUtils.sq(times.tJa));
            v = params.v0() + params.aLimA * (t - times.tJa / 2.0);
            a = params.aLimA;
        } else if (t <= times.tA) {
            p = params.p0() + (params.vLim + params.v0()) * times.tA / 2.0 - params.vLim * (times.tA - t)
                    - params.jMin * MathUtils.cube(times.tA - t) / 6.0;
            v = params.vLim + params.jMin * MathUtils.sq(times.tA - t) / 2.0;
            a = params.jMax * (times.tA - t);
        } else if (t <= times.tA + times.tV) {
            p = params.p0() + (params.vLim + params.v0()) * times.tA / 2.0 + params.vLim * (t - times.tA);
            v = params.vLim;
            a = 0.0;
        } else if (t <= tt - times.tD + times.tJd) {
            p = params.p1() - (params.vLim + params.v1()) * times.tD / 2.
                    + params.vLim * (t - tt + times.tD)
                    - params.jMax * MathUtils.cube(t - tt + times.tD) / 6.0;
            v = params.vLim - params.jMax * MathUtils.sq(t - tt + times.tD) / 2.0;
            a = params.jMin * (t - tt + times.tD);
        } else if (t <= tt - times.tJd) {
            p = params.p1() - (params.vLim + params.v1()) * times.tD / 2. + params.vLim * (t - tt + times.tD)
                    + params.aLimD / 6.0 * (3.0 * MathUtils.sq(t - tt + times.tD)
                            - 3.0 * times.tJd * (t - tt + times.tD) + MathUtils.sq(times.tJd));
            v = params.vLim + params.aLimD * (t - tt + times.tD - times.tJd / 2.0);
            a = params.aLimD;
        } else if (t <= tt) {
            p = params.p1() - params.v1() * (tt - t) - params.jMax * MathUtils.cube(tt - t) / 6.0;
            v = params.v1() + params.jMax * MathUtils.sq(tt - t) / 2.0;
            a = params.jMin * (tt - t);
        } else {
            p = params.p1();
            v = params.v1();
            a = 0.0;
        }

        return new State(p, v * vsign, a);
    }

}
