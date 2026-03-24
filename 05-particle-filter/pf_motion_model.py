from manimlib import *
import numpy as np


class PFMotionModel(Scene):
    """
    Particle Filter Motion Model Animation
    Shows how Odometry/IMU data flows into nonlinear motion equations
    With detailed sensor visualization
    """

    def construct(self):
        # Title
        title = Text("Particle Filter: Motion Model (Predict Step)", font_size=36, color=YELLOW)
        title.to_edge(UP)
        self.play(Write(title))
        self.wait(0.3)

        # ========== Part 1: Odometry Visualization ==========
        odom_title = Text("1. Odometry (Wheel Encoder)", font_size=28, color=BLUE)
        odom_title.move_to(UP * 2.5 + LEFT * 3)
        self.play(FadeIn(odom_title))

        # Draw a wheel
        wheel_center = LEFT * 4.5 + UP * 0.5
        wheel = Circle(radius=1, color=WHITE, stroke_width=3)
        wheel.move_to(wheel_center)

        # Spokes on the wheel
        spokes = VGroup()
        for i in range(8):
            angle = i * PI / 4
            spoke = Line(
                wheel_center,
                wheel_center + np.array([np.cos(angle), np.sin(angle), 0]),
                color=GREY
            )
            spokes.add(spoke)

        # Encoder ticks around the wheel (like slots)
        encoder_ticks = VGroup()
        n_ticks = 16
        for i in range(n_ticks):
            angle = i * 2 * PI / n_ticks
            tick = Line(
                wheel_center + 0.85 * np.array([np.cos(angle), np.sin(angle), 0]),
                wheel_center + 1.0 * np.array([np.cos(angle), np.sin(angle), 0]),
                color=YELLOW,
                stroke_width=2
            )
            encoder_ticks.add(tick)

        wheel_group = VGroup(wheel, spokes, encoder_ticks)

        # Diameter annotation
        diameter_line = Line(wheel_center + LEFT, wheel_center + RIGHT, color=GREEN)
        d_label = Text("D", font_size=20, color=GREEN)
        d_label.next_to(diameter_line, DOWN, buff=0.1)

        self.play(ShowCreation(wheel_group))
        self.play(ShowCreation(diameter_line), FadeIn(d_label))

        # Encoder sensor (detector)
        sensor = Rectangle(width=0.3, height=0.5, color=RED, fill_opacity=0.8)
        sensor.move_to(wheel_center + RIGHT * 1.15)
        sensor_label = Text("sensor", font_size=14, color=RED)
        sensor_label.next_to(sensor, RIGHT, buff=0.1)
        self.play(FadeIn(sensor), FadeIn(sensor_label))

        # Animate wheel rotation and pulse counting
        pulse_counter = Text("ticks: 0", font_size=20, color=YELLOW)
        pulse_counter.move_to(wheel_center + DOWN * 1.8)
        self.play(FadeIn(pulse_counter))

        # Rotate wheel and count pulses
        tick_count = 0
        for rotation in range(2):  # 2 partial rotations
            for step in range(4):
                # Rotate wheel
                self.play(
                    Rotate(wheel_group, angle=PI/4, about_point=wheel_center),
                    run_time=0.3
                )
                tick_count += 2  # 2 ticks per PI/4 rotation (16 ticks per full rotation)

                # Flash sensor
                self.play(
                    sensor.animate.set_color(WHITE),
                    run_time=0.1
                )
                self.play(
                    sensor.animate.set_color(RED),
                    run_time=0.1
                )

                # Update counter
                new_counter = Text(f"ticks: {tick_count}", font_size=20, color=YELLOW)
                new_counter.move_to(wheel_center + DOWN * 1.8)
                self.remove(pulse_counter)
                self.add(new_counter)
                pulse_counter = new_counter

        # Show formula derivation
        formula_group = VGroup()

        # Arrow from wheel to formula
        arrow1 = Arrow(wheel_center + RIGHT * 1.5, LEFT * 1.5 + UP * 0.5, buff=0.1, color=WHITE)
        self.play(ShowCreation(arrow1))

        # Formula box
        formula_box = VGroup(
            Rectangle(width=4, height=2.5, color=BLUE),
            Text("Δs = (ticks / N) × π × D", font_size=20).shift(UP * 0.7),
            Text("N = ticks per revolution", font_size=16, color=GREY).shift(UP * 0.1),
            Text("D = wheel diameter", font_size=16, color=GREEN).shift(DOWN * 0.4),
            Text("→ Linear displacement!", font_size=18, color=YELLOW).shift(DOWN * 0.9),
        )
        formula_box.move_to(RIGHT * 1 + UP * 0.5)
        self.play(ShowCreation(formula_box))
        self.wait(0.5)

        # Differential drive for Δθ_odom - with turning animation
        diff_title = Text("Differential Drive → Δθ_odom", font_size=20, color=BLUE)
        diff_title.move_to(RIGHT * 2 + UP * 1)
        self.play(FadeIn(diff_title))

        # Robot body (top view) - initial position
        robot_center = RIGHT * 0.5 + DOWN * 1.5
        robot_body = Rectangle(width=1.6, height=1, color=WHITE, fill_opacity=0.2)
        robot_body.move_to(robot_center)

        # Two wheels on sides
        left_wheel_pos = robot_center + LEFT * 0.8
        right_wheel_pos = robot_center + RIGHT * 0.8
        left_wheel = Rectangle(width=0.15, height=0.4, color=BLUE, fill_opacity=0.8).move_to(left_wheel_pos)
        right_wheel = Rectangle(width=0.15, height=0.4, color=BLUE, fill_opacity=0.8).move_to(right_wheel_pos)

        # Wheelbase line
        wb_line = Line(left_wheel_pos, right_wheel_pos, color=PURPLE)
        wb_label = Text("L", font_size=16, color=PURPLE).next_to(wb_line, DOWN, buff=0.1)

        # Direction arrow
        direction_arrow = Arrow(robot_center, robot_center + UP * 0.6, buff=0, color=YELLOW, stroke_width=4)

        robot_group = VGroup(robot_body, left_wheel, right_wheel, direction_arrow)

        self.play(
            ShowCreation(robot_body),
            ShowCreation(left_wheel), ShowCreation(right_wheel),
            ShowCreation(wb_line), FadeIn(wb_label),
            ShowCreation(direction_arrow),
        )
        self.wait(0.3)

        # Show different wheel distances
        # Left wheel moves less, right wheel moves more -> robot turns left
        left_arc_length = 0.3
        right_arc_length = 0.8

        # Draw arc paths for each wheel
        # The robot rotates around a point (ICC - Instantaneous Center of Curvature)
        # When Δs_R > Δs_L, robot turns left (counterclockwise)

        left_path = Arrow(left_wheel_pos, left_wheel_pos + UP * left_arc_length, buff=0, color=ORANGE, stroke_width=3)
        right_path = Arrow(right_wheel_pos, right_wheel_pos + UP * right_arc_length, buff=0, color=ORANGE, stroke_width=3)

        left_dist_label = Text(f"Δs_L", font_size=14, color=ORANGE).next_to(left_path, LEFT, buff=0.1)
        right_dist_label = Text(f"Δs_R", font_size=14, color=ORANGE).next_to(right_path, RIGHT, buff=0.1)

        self.play(
            ShowCreation(left_path), ShowCreation(right_path),
            FadeIn(left_dist_label), FadeIn(right_dist_label),
        )

        # Show that right > left
        comparison = Text("Δs_R > Δs_L → Turn Left!", font_size=16, color=YELLOW)
        comparison.move_to(RIGHT * 0.5 + DOWN * 2.5)
        self.play(FadeIn(comparison))
        self.wait(0.5)

        # Animate robot turning
        # Calculate rotation angle (simplified)
        delta_theta = (right_arc_length - left_arc_length) / 1.6  # L = 1.6 (robot width)

        # Create rotated robot
        robot_group_copy = robot_group.copy()

        self.play(
            Rotate(robot_group, angle=delta_theta, about_point=robot_center),
            run_time=1.5
        )

        # Show the angle
        angle_arc = Arc(
            radius=0.4,
            start_angle=PI/2,
            angle=delta_theta,
            color=GREEN,
            stroke_width=3
        )
        angle_arc.move_to(robot_center + UP * 0.2 + LEFT * 0.1)
        theta_label = Text("Δθ", font_size=16, color=GREEN).next_to(angle_arc, UP + RIGHT, buff=0.05)

        self.play(ShowCreation(angle_arc), FadeIn(theta_label))

        # Show the geometric relationship
        # Arc length = radius × angle  =>  angle = arc_length / radius
        # For differential drive: Δθ ≈ (Δs_R - Δs_L) / L

        formula_box2 = VGroup(
            Rectangle(width=5, height=2.2, color=BLUE),
            Text("Arc length formula:", font_size=16).shift(UP * 0.7),
            Text("s = r × θ  →  θ = s / r", font_size=16, color=GREY).shift(UP * 0.2),
            Text("Δθ = (Δs_R - Δs_L) / L", font_size=20, color=YELLOW).shift(DOWN * 0.4),
        )
        formula_box2.move_to(RIGHT * 4.5 + DOWN * 1.5)

        self.play(ShowCreation(formula_box2))

        # Visual explanation
        explain = Text("Arc diff / Wheelbase = Turn angle", font_size=16, color=GREEN)
        explain.next_to(formula_box2, DOWN, buff=0.2)
        self.play(FadeIn(explain))

        self.wait(1.5)

        # Clean up for next section
        self.play(
            FadeOut(diff_title),
            FadeOut(robot_group), FadeOut(wb_line), FadeOut(wb_label),
            FadeOut(left_path), FadeOut(right_path),
            FadeOut(left_dist_label), FadeOut(right_dist_label),
            FadeOut(comparison), FadeOut(angle_arc), FadeOut(theta_label),
            FadeOut(formula_box2), FadeOut(explain),
        )

        # ========== Clear and show IMU ==========
        self.play(
            FadeOut(odom_title), FadeOut(wheel_group), FadeOut(diameter_line), FadeOut(d_label),
            FadeOut(sensor), FadeOut(sensor_label), FadeOut(pulse_counter),
            FadeOut(arrow1), FadeOut(formula_box),
        )

        # ========== Part 2: IMU Visualization ==========
        imu_title = Text("2. IMU (Gyroscope)", font_size=28, color=GREEN)
        imu_title.move_to(UP * 2.5)
        self.play(FadeIn(imu_title))

        # Show robot top view with IMU
        robot_imu_center = LEFT * 4 + DOWN * 0.5

        # Ground plane (top view)
        ground_label = Text("Top View (Bird's Eye)", font_size=16, color=GREY)
        ground_label.move_to(LEFT * 4 + UP * 1.8)
        self.play(FadeIn(ground_label))

        # Robot body (rectangle, top view)
        robot_body_imu = Rectangle(width=1.5, height=1, color=WHITE, fill_opacity=0.2)
        robot_body_imu.move_to(robot_imu_center)

        # IMU chip on robot
        imu_chip = VGroup(
            Rectangle(width=0.4, height=0.4, color=GREEN, fill_opacity=0.8),
            Text("IMU", font_size=10, color=WHITE),
        )
        imu_chip.move_to(robot_imu_center)

        # Direction arrow (robot facing direction)
        robot_dir = Arrow(robot_imu_center, robot_imu_center + UP * 0.8, buff=0, color=YELLOW, stroke_width=4)
        dir_label = Text("Front", font_size=12, color=YELLOW).next_to(robot_dir, UP, buff=0.05)

        self.play(
            ShowCreation(robot_body_imu),
            ShowCreation(imu_chip),
            ShowCreation(robot_dir),
            FadeIn(dir_label),
        )

        # World coordinate axes
        world_origin = LEFT * 4 + DOWN * 2.3
        world_x = Arrow(world_origin, world_origin + RIGHT * 1, buff=0, color=RED, stroke_width=2)
        world_y = Arrow(world_origin, world_origin + UP * 1, buff=0, color=GREEN, stroke_width=2)
        world_x_label = Text("X", font_size=12, color=RED).next_to(world_x, RIGHT, buff=0.05)
        world_y_label = Text("Y", font_size=12, color=GREEN).next_to(world_y, UP, buff=0.05)
        z_up_label = Text("Z points UP (out of screen)", font_size=14, color=BLUE)
        z_up_label.move_to(LEFT * 4 + DOWN * 3)

        self.play(
            ShowCreation(world_x), ShowCreation(world_y),
            FadeIn(world_x_label), FadeIn(world_y_label),
            FadeIn(z_up_label),
        )
        self.wait(0.3)

        # Show rotation around Z axis (yaw)
        rotation_label = Text("Robot rotates around Z axis (Yaw)", font_size=16, color=YELLOW)
        rotation_label.move_to(LEFT * 4 + UP * 1.2)
        self.play(FadeIn(rotation_label))

        # Rotation arc
        rotation_arc = Arc(
            radius=1.2,
            start_angle=PI/2,
            angle=PI/3,
            color=YELLOW,
            stroke_width=4
        )
        rotation_arc.move_to(robot_imu_center)

        omega_label = Text("ωz = angular velocity", font_size=14, color=YELLOW)
        omega_label.next_to(rotation_arc, RIGHT, buff=0.2)

        self.play(ShowCreation(rotation_arc), FadeIn(omega_label))

        # Animate robot rotating
        robot_group_imu = VGroup(robot_body_imu, imu_chip, robot_dir, dir_label)
        self.play(
            Rotate(robot_group_imu, angle=PI/3, about_point=robot_imu_center),
            run_time=1.5
        )

        # Show angle change
        angle_arc_imu = Arc(
            radius=0.5,
            start_angle=PI/2,
            angle=PI/3,
            color=GREEN,
            stroke_width=3
        )
        angle_arc_imu.move_to(robot_imu_center + UP * 0.2)
        dtheta_label_imu = Text("Δθ", font_size=16, color=GREEN)
        dtheta_label_imu.next_to(angle_arc_imu, UP + RIGHT, buff=0.1)

        self.play(ShowCreation(angle_arc_imu), FadeIn(dtheta_label_imu))

        # Formula explanation (right side)
        formula_imu = VGroup(
            Rectangle(width=5, height=3, color=GREEN),
            Text("Gyroscope measures:", font_size=16).shift(UP * 1),
            Text("ωz = angular velocity (rad/s)", font_size=16, color=YELLOW).shift(UP * 0.4),
            Text("around Z axis (vertical)", font_size=14, color=GREY).shift(DOWN * 0.1),
            Text("Δθ_imu = ωz × Δt", font_size=20, color=YELLOW).shift(DOWN * 0.7),
            Text("(integrate over time)", font_size=14, color=GREY).shift(DOWN * 1.1),
        )
        formula_imu.move_to(RIGHT * 2.5 + DOWN * 0.3)

        self.play(ShowCreation(formula_imu))
        self.wait(1.5)

        # ========== Clear and show Fusion ==========
        self.play(
            FadeOut(imu_title), FadeOut(ground_label),
            FadeOut(robot_group_imu), FadeOut(rotation_arc), FadeOut(omega_label),
            FadeOut(rotation_label), FadeOut(angle_arc_imu), FadeOut(dtheta_label_imu),
            FadeOut(world_x), FadeOut(world_y),
            FadeOut(world_x_label), FadeOut(world_y_label), FadeOut(z_up_label),
            FadeOut(formula_imu),
        )

        # ========== Part 3: Fusion ==========
        fusion_title = Text("3. Sensor Fusion", font_size=28, color=ORANGE)
        fusion_title.move_to(UP * 2.5)
        self.play(FadeIn(fusion_title))

        # Two input boxes
        odom_input = VGroup(
            Rectangle(width=3, height=1.5, color=BLUE),
            Text("Odometry", font_size=18, color=BLUE).shift(UP * 0.3),
            Text("Δs, Δθ_odom", font_size=16).shift(DOWN * 0.2),
        )
        odom_input.move_to(LEFT * 4 + UP * 0.8)

        imu_input = VGroup(
            Rectangle(width=3, height=1.5, color=GREEN),
            Text("IMU", font_size=18, color=GREEN).shift(UP * 0.3),
            Text("Δθ_imu", font_size=16).shift(DOWN * 0.2),
        )
        imu_input.move_to(LEFT * 4 + DOWN * 1)

        self.play(ShowCreation(odom_input), ShowCreation(imu_input))

        # Fusion box
        fusion_box = VGroup(
            Rectangle(width=4.5, height=2.5, color=ORANGE),
            Text("Complementary Filter", font_size=18, color=ORANGE).shift(UP * 0.8),
            Text("Δs (from odometry)", font_size=16, color=BLUE).shift(UP * 0.2),
            Text("Δθ = α·Δθ_odom + (1-α)·Δθ_imu", font_size=16).shift(DOWN * 0.3),
            Text("α ∈ [0,1] (tuning param)", font_size=14, color=GREY).shift(DOWN * 0.8),
        )
        fusion_box.move_to(RIGHT * 0.5 + DOWN * 0.1)

        arrow_odom = Arrow(odom_input.get_right(), fusion_box.get_left() + UP * 0.3, buff=0.1, color=BLUE)
        arrow_imu = Arrow(imu_input.get_right(), fusion_box.get_left() + DOWN * 0.3, buff=0.1, color=GREEN)

        self.play(
            ShowCreation(arrow_odom), ShowCreation(arrow_imu),
            ShowCreation(fusion_box),
        )
        self.wait(0.5)

        # Output
        output_box = VGroup(
            Rectangle(width=3, height=1.5, color=YELLOW),
            Text("Output", font_size=18, color=YELLOW).shift(UP * 0.3),
            Text("Δs, Δθ", font_size=20).shift(DOWN * 0.2),
        )
        output_box.move_to(RIGHT * 5 + DOWN * 0.1)

        arrow_out = Arrow(fusion_box.get_right(), output_box.get_left(), buff=0.1, color=ORANGE)
        self.play(ShowCreation(arrow_out), ShowCreation(output_box))
        self.wait(1)

        # ========== Clear and show Motion Model ==========
        self.play(
            FadeOut(fusion_title), FadeOut(odom_input), FadeOut(imu_input),
            FadeOut(fusion_box), FadeOut(arrow_odom), FadeOut(arrow_imu),
            FadeOut(arrow_out), FadeOut(output_box),
        )

        # ========== Part 4: Nonlinear Motion Model ==========
        motion_title = Text("4. Nonlinear Motion Model", font_size=28, color=RED)
        motion_title.move_to(UP * 2.5)
        self.play(FadeIn(motion_title))

        # Draw X-Y coordinate system
        coord_origin = LEFT * 4 + DOWN * 1.5
        x_axis_motion = Arrow(coord_origin + LEFT * 0.5, coord_origin + RIGHT * 4, buff=0, color=RED, stroke_width=2)
        y_axis_motion = Arrow(coord_origin + DOWN * 0.5, coord_origin + UP * 3, buff=0, color=GREEN, stroke_width=2)
        x_label_motion = Text("X (World Frame)", font_size=14, color=RED).next_to(x_axis_motion, RIGHT, buff=0.1)
        y_label_motion = Text("Y", font_size=14, color=GREEN).next_to(y_axis_motion, UP, buff=0.1)
        origin_label = Text("O", font_size=14, color=WHITE).next_to(coord_origin, DOWN + LEFT, buff=0.1)

        self.play(
            ShowCreation(x_axis_motion), ShowCreation(y_axis_motion),
            FadeIn(x_label_motion), FadeIn(y_label_motion), FadeIn(origin_label),
        )

        # Robot at initial position (x, y, θ)
        robot_pos = coord_origin + RIGHT * 1.5 + UP * 1
        robot = VGroup(
            Circle(radius=0.25, color=WHITE, fill_opacity=0.3),
            Arrow(ORIGIN, RIGHT * 0.35, buff=0, color=YELLOW, stroke_width=3),
        )
        robot.move_to(robot_pos)
        # Rotate robot to show initial θ
        robot.rotate(PI/6, about_point=robot_pos)

        # Position labels
        x_line = DashedLine(robot_pos, coord_origin + RIGHT * 1.5, color=GREY)
        y_line = DashedLine(robot_pos, coord_origin + UP * 1, color=GREY)
        x_val = Text("x", font_size=14, color=WHITE).next_to(coord_origin + RIGHT * 1.5, DOWN, buff=0.1)
        y_val = Text("y", font_size=14, color=WHITE).next_to(coord_origin + UP * 1, LEFT, buff=0.1)

        # θ angle arc
        theta_arc = Arc(
            radius=0.5,
            start_angle=0,
            angle=PI/6,
            color=ORANGE,
            stroke_width=2
        )
        theta_arc.move_to(robot_pos)
        theta_label = Text("θ", font_size=16, color=ORANGE).next_to(theta_arc, RIGHT, buff=0.1)

        state_label = Text("Current: (x, y, θ)", font_size=16, color=WHITE)
        state_label.next_to(robot, UP + RIGHT, buff=0.2)

        self.play(
            ShowCreation(robot),
            ShowCreation(x_line), ShowCreation(y_line),
            FadeIn(x_val), FadeIn(y_val),
            ShowCreation(theta_arc), FadeIn(theta_label),
            FadeIn(state_label),
        )
        self.wait(0.5)

        # Show motion inputs Δs and Δθ
        input_label = Text("Input: Δs (displacement), Δθ (rotation)", font_size=16, color=BLUE)
        input_label.move_to(UP * 1.8)
        self.play(FadeIn(input_label))

        # New position after motion
        delta_s = 1.5
        delta_theta = PI/4
        mid_theta = PI/6 + delta_theta / 2  # θ + Δθ/2

        # Calculate new position
        new_x = 1.5 + delta_s * np.cos(mid_theta)
        new_y = 1 + delta_s * np.sin(mid_theta)
        robot_new_pos = coord_origin + RIGHT * new_x + UP * new_y

        # Draw the motion path (arc)
        motion_path = Arrow(
            robot_pos,
            robot_new_pos,
            buff=0,
            color=ORANGE,
            stroke_width=3
        )
        ds_label = Text("Δs", font_size=16, color=ORANGE)
        ds_label.next_to(motion_path, UP, buff=0.1)

        self.play(ShowCreation(motion_path), FadeIn(ds_label))

        # Create robot at new position
        robot_new = VGroup(
            Circle(radius=0.25, color=YELLOW, fill_opacity=0.3),
            Arrow(ORIGIN, RIGHT * 0.35, buff=0, color=YELLOW, stroke_width=3),
        )
        robot_new.move_to(robot_new_pos)
        robot_new.rotate(PI/6 + delta_theta, about_point=robot_new_pos)

        # New position labels
        x_line_new = DashedLine(robot_new_pos, coord_origin + RIGHT * new_x, color=YELLOW)
        y_line_new = DashedLine(robot_new_pos, coord_origin + UP * new_y, color=YELLOW)
        x_val_new = Text("x'", font_size=14, color=YELLOW).next_to(coord_origin + RIGHT * new_x, DOWN, buff=0.1)
        y_val_new = Text("y'", font_size=14, color=YELLOW).next_to(coord_origin + UP * new_y, LEFT, buff=0.1)

        new_state_label = Text("New: (x', y', θ')", font_size=16, color=YELLOW)
        new_state_label.next_to(robot_new, UP + RIGHT, buff=0.2)

        self.play(
            ShowCreation(robot_new),
            ShowCreation(x_line_new), ShowCreation(y_line_new),
            FadeIn(x_val_new), FadeIn(y_val_new),
            FadeIn(new_state_label),
        )

        # Motion equations (right side)
        eq_box = VGroup(
            Rectangle(width=5.5, height=3.5, color=RED),
            Text("Motion Equations:", font_size=18, color=RED).shift(UP * 1.2),
            Text("x' = x + Δs·cos(θ + Δθ/2)", font_size=18).shift(UP * 0.5),
            Text("y' = y + Δs·sin(θ + Δθ/2)", font_size=18).shift(DOWN * 0.1),
            Text("θ' = θ + Δθ", font_size=18).shift(DOWN * 0.7),
            Text("(θ+Δθ/2 = average heading)", font_size=14, color=GREY).shift(DOWN * 1.2),
        )
        eq_box.move_to(RIGHT * 3.5 + DOWN * 0.5)

        self.play(ShowCreation(eq_box))

        # Highlight nonlinear parts
        cos_highlight = SurroundingRectangle(eq_box[2], color=YELLOW, buff=0.05)
        sin_highlight = SurroundingRectangle(eq_box[3], color=YELLOW, buff=0.05)
        nonlinear_note = Text("cos, sin → Nonlinear!", font_size=18, color=YELLOW)
        nonlinear_note.next_to(eq_box, DOWN, buff=0.2)

        self.play(
            ShowCreation(cos_highlight), ShowCreation(sin_highlight),
            FadeIn(nonlinear_note),
        )
        self.wait(1.5)

        # ========== Part 5: Apply to Particles ==========
        self.play(
            FadeOut(motion_title), FadeOut(input_label),
            FadeOut(x_axis_motion), FadeOut(y_axis_motion),
            FadeOut(x_label_motion), FadeOut(y_label_motion), FadeOut(origin_label),
            FadeOut(robot), FadeOut(x_line), FadeOut(y_line),
            FadeOut(x_val), FadeOut(y_val), FadeOut(theta_arc), FadeOut(theta_label),
            FadeOut(state_label), FadeOut(motion_path), FadeOut(ds_label),
            FadeOut(robot_new), FadeOut(x_line_new), FadeOut(y_line_new),
            FadeOut(x_val_new), FadeOut(y_val_new), FadeOut(new_state_label),
            FadeOut(eq_box), FadeOut(cos_highlight), FadeOut(sin_highlight), FadeOut(nonlinear_note),
        )

        particle_title = Text("5. Apply to N Particles + Add Noise", font_size=28, color=PURPLE)
        particle_title.move_to(UP * 2.5)
        self.play(FadeIn(particle_title))

        # Equations with noise
        noise_eq = VGroup(
            Text("For each particle i:", font_size=18, color=PURPLE).shift(UP * 0.8),
            Text("x'ᵢ = xᵢ + Δs·cos(θᵢ + Δθ/2) + εₓ", font_size=18).shift(UP * 0.2),
            Text("y'ᵢ = yᵢ + Δs·sin(θᵢ + Δθ/2) + εᵧ", font_size=18).shift(DOWN * 0.3),
            Text("θ'ᵢ = θᵢ + Δθ + εθ", font_size=18).shift(DOWN * 0.8),
        )
        noise_eq.move_to(UP * 1)

        noise_note = Text("εₓ, εᵧ, εθ ~ N(0, σ²)  (noise ∝ motion)", font_size=16, color=RED)
        noise_note.next_to(noise_eq, DOWN, buff=0.3)

        self.play(Write(noise_eq))
        self.play(FadeIn(noise_note))
        self.wait(0.5)

        # Particle visualization
        np.random.seed(42)
        n_particles = 30

        # Initial particles (concentrated)
        init_x = np.random.normal(0, 0.15, n_particles)
        init_y = np.random.normal(0, 0.15, n_particles)

        particles_before = VGroup(*[
            Dot(point=LEFT * 3.5 + DOWN * 1.5 + RIGHT * init_x[i] + UP * init_y[i], radius=0.08, color=BLUE)
            for i in range(n_particles)
        ])

        before_label = Text("Before (concentrated)", font_size=16, color=BLUE)
        before_label.next_to(particles_before, DOWN, buff=0.3)

        self.play(ShowCreation(particles_before), FadeIn(before_label))
        self.wait(0.3)

        # Arrow showing motion
        motion_arrow = Arrow(LEFT * 2 + DOWN * 1.5, RIGHT * 1 + DOWN * 1.5, buff=0, color=WHITE)
        motion_label = Text("Motion model + noise", font_size=14)
        motion_label.next_to(motion_arrow, UP, buff=0.1)

        self.play(ShowCreation(motion_arrow), FadeIn(motion_label))

        # After particles (spread out)
        spread_x = np.random.normal(0, 0.5, n_particles)
        spread_y = np.random.normal(0, 0.5, n_particles)

        particles_after = VGroup(*[
            Dot(point=RIGHT * 3 + DOWN * 1.5 + RIGHT * spread_x[i] + UP * spread_y[i], radius=0.08, color=ORANGE)
            for i in range(n_particles)
        ])

        after_label = Text("After (spread out)", font_size=16, color=ORANGE)
        after_label.next_to(particles_after, DOWN, buff=0.3)

        # Animate
        self.play(
            *[Transform(particles_before[i].copy(), particles_after[i]) for i in range(n_particles)],
            ShowCreation(particles_after),
            FadeIn(after_label),
            run_time=2
        )

        # Final note
        final_note = Text("Uncertainty grows after prediction! (Need observation to correct)", font_size=20, color=YELLOW)
        final_note.to_edge(DOWN)
        self.play(FadeIn(final_note))

        self.wait(2)


# Run: manimgl pf_motion_model.py PFMotionModel -w -r 1920x1080
