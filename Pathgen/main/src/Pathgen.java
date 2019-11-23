    package org.usfirst.frc.team449.pathgen;

    import jaci.pathfinder.Pathfinder;
    import jaci.pathfinder.Trajectory;
    import jaci.pathfinder.Waypoint;
    import jaci.pathfinder.modifiers.TankModifier;

    import java.io.FileWriter;
    import java.io.IOException;
    import java.util.HashMap;
    import java.util.Map;

    /**
     * Generates a motion profile that hits any number of waypoints.
     */
    public class Pathgen {

        public static void main(String[] args) throws IOException {

            //Calculated by driving each wheel n inches in opposite directions, then taking the angle moved, θ, and finding
            // the circumference of a circle moved by the robot via C = 360 * n / θ
            //You then find the diameter via C / π.

            final double naviWheelbase = 2.34;

            final double LENGTH = 39.5 / 12.;
            final double WIDTH = 34.5 / 12.;
            final double CUBE_LENGTH = 13. / 12.;
            final double DIAGONAL = Math.sqrt(WIDTH * WIDTH + LENGTH * LENGTH);
            final double INT_ANGLE = Math.atan2(WIDTH, LENGTH);

            Map<String, Waypoint[]> profiles = new HashMap<>();
            Waypoint[] zigzag = new Waypoint[] {
                    new Waypoint(0, 0, 0),
                    new Waypoint(17.5, 12.25, 37.889188),
                    new Waypoint(13, -12, -122.333)
            };
            Waypoint[] uTurn = new Waypoint[]{
                    new Waypoint(13, -12, -122.333),
                    new Waypoint(20, -18, 58),
                    new Waypoint(13, -12, 58)
            };
            Waypoint[] forward = new Waypoint[]{
                    new Waypoint(0,0,0),
                    new Waypoint(100, 0, 0)
            };
            Waypoint[] backward = new Waypoint[]{
                     new Waypoint(0,0,0),
                    new Waypoint(-100, 0, 0)
            };

    //        profiles.put("", );

            final String ROBOT_NAME = "navi";

            Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
                    0.05, 5., 5., 15.); //Units are seconds, feet/second, feet/(second^2), and feet/(second^3)

            for (String profile : profiles.keySet()) {
                Trajectory trajectory = Pathfinder.generate(profiles.get(profile), config);

                TankModifier tm = new TankModifier(trajectory).modify(naviWheelbase); //Units are feet

                FileWriter lfw = new FileWriter(ROBOT_NAME + "Left" + profile + "Profile.csv", false);
                FileWriter rfw = new FileWriter(ROBOT_NAME + "Right" + profile + "Profile.csv", false);


                lfw.write(tm.getLeftTrajectory().length() + "\n");
                for (int i = 0; i < tm.getLeftTrajectory().length(); i++) {
                    lfw.write(tm.getLeftTrajectory().get(i).position + ",\t" + tm.getLeftTrajectory().get(i).velocity + ",\t"
                            + tm.getLeftTrajectory().get(i).acceleration + ",\t" + tm.getLeftTrajectory().get(i).dt);
                    lfw.write("\n");
                }

                rfw.write(tm.getRightTrajectory().length() + "\n");
                for (int i = 0; i < tm.getRightTrajectory().length(); i++) {
                    rfw.write(tm.getRightTrajectory().get(i).position + ",\t" + tm.getRightTrajectory().get(i).velocity +
                            ",\t" + tm.getLeftTrajectory().get(i).acceleration + ",\t" + tm.getRightTrajectory().get(i).dt);
                    rfw.write("\n");
                }

                lfw.flush();
                lfw.close();
                rfw.flush();
                rfw.close();
            }
        }
    }