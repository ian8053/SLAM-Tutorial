from manimlib import *
import numpy as np


class PFUpdateStep(Scene):
    """
    Particle Filter Update Step Animation (English version)
    1. LiDAR data explanation
    2. Distance Field (Distance Transform)
    3. Weight calculation (= Bayesian Update)
    4. Resampling (bowl flip concept)
    5. Final Output (Simple Average)
    """

    def construct(self):
        # Title
        title = Text("Particle Filter: Update Step", font_size=36, color=YELLOW)
        title.to_edge(UP)
        self.play(Write(title))
        self.wait(0.3)

        # ========== Part 1: LiDAR Data ==========
        lidar_title = Text("1. LiDAR Sensor", font_size=28, color=RED)
        lidar_title.move_to(UP * 2.5)
        self.play(FadeIn(lidar_title))

        # Robot with LiDAR sensor
        robot_center = LEFT * 3 + DOWN * 0.3
        robot = Circle(radius=0.4, color=WHITE, fill_opacity=0.3)
        robot.move_to(robot_center)
        lidar_sensor = Dot(robot_center, radius=0.1, color=RED)
        robot_label = Text("LiDAR", font_size=14, color=RED).next_to(robot, DOWN, buff=0.15)

        self.play(ShowCreation(robot), ShowCreation(lidar_sensor), FadeIn(robot_label))

        # Show 360 degree scanning concept
        scan_circle = DashedVMobject(Circle(radius=2.5, color=RED), num_dashes=36)
        scan_circle.move_to(robot_center)
        scan_label = Text("360° scan", font_size=14, color=RED)
        scan_label.move_to(robot_center + RIGHT * 3 + UP * 1)

        self.play(ShowCreation(scan_circle), FadeIn(scan_label))

        # Show individual rays
        rays_text = Text("Each ray: independent measurement", font_size=16, color=YELLOW)
        rays_text.move_to(UP * 1.5)
        self.play(FadeIn(rays_text))

        # Animate a few rays with measurements
        ray_data = [
            (0, 2.0),
            (45, 1.8),
            (90, 2.5),
            (135, 1.5),
            (180, 3.0),
        ]

        rays = VGroup()
        hits = VGroup()
        for theta_deg, r in ray_data:
            theta = theta_deg * DEGREES
            end = robot_center + r * np.array([np.cos(theta), np.sin(theta), 0])
            ray = Line(robot_center, end, color=RED, stroke_width=2)
            hit = Dot(end, radius=0.08, color=ORANGE)
            rays.add(ray)
            hits.add(hit)

        self.play(ShowCreation(rays), ShowCreation(hits), run_time=1.5)

        # Show one measurement detail
        example_theta = 45 * DEGREES
        example_r = 1.8
        example_end = robot_center + example_r * np.array([np.cos(example_theta), np.sin(example_theta), 0])

        highlight_ray = Line(robot_center, example_end, color=YELLOW, stroke_width=4)
        theta_arc = Arc(radius=0.5, start_angle=0, angle=example_theta, color=GREEN, stroke_width=3)
        theta_arc.move_to(robot_center + 0.25 * (RIGHT + UP))
        theta_label = Text("θᵢ = 45°", font_size=14, color=GREEN)
        theta_label.next_to(theta_arc, RIGHT, buff=0.1)
        r_label = Text("rᵢ = 1.8m", font_size=14, color=YELLOW)
        r_label.next_to(highlight_ray, UP, buff=0.1)

        self.play(
            ShowCreation(highlight_ray),
            ShowCreation(theta_arc), FadeIn(theta_label),
            FadeIn(r_label),
        )

        # Data format explanation
        data_box = VGroup(
            Rectangle(width=5, height=2.5, color=RED),
            Text("LiDAR Output Array:", font_size=16).shift(UP * 0.8),
            Text("[(r₀,θ₀), (r₁,θ₁), ..., (r₃₅₉,θ₃₅₉)]", font_size=14, color=YELLOW).shift(UP * 0.3),
            Text("Resolution: 1° → 360 measurements", font_size=14).shift(DOWN * 0.2),
            Text("Each beam is INDEPENDENT", font_size=14, color=GREEN).shift(DOWN * 0.7),
        )
        data_box.move_to(RIGHT * 3 + DOWN * 0.5)
        self.play(ShowCreation(data_box))

        # Independence assumption
        independence = Text("Assumption: P(z|x) = ∏ᵢ P(zᵢ|x)", font_size=16, color=PURPLE)
        independence.move_to(DOWN * 2.5)
        self.play(FadeIn(independence))
        self.wait(1)

        # ========== Clear and show Distance Field ==========
        self.play(
            FadeOut(lidar_title), FadeOut(robot), FadeOut(lidar_sensor), FadeOut(robot_label),
            FadeOut(scan_circle), FadeOut(scan_label), FadeOut(rays_text),
            FadeOut(rays), FadeOut(hits),
            FadeOut(highlight_ray), FadeOut(theta_arc), FadeOut(theta_label), FadeOut(r_label),
            FadeOut(data_box), FadeOut(independence),
        )

        # ========== Part 2: Distance Field (Distance Transform) ==========
        df_title = Text("2. Distance Transform", font_size=28, color=TEAL)
        df_title.move_to(UP * 2.5)
        self.play(FadeIn(df_title))

        # Left side: Grid Map
        grid_label = Text("Grid Map (.pgm)", font_size=18, color=BLUE)
        grid_label.move_to(LEFT * 4.5 + UP * 1.8)
        self.play(FadeIn(grid_label))

        # Draw grid map
        grid_origin = LEFT * 6 + DOWN * 0.8
        cell_size = 0.35
        grid_w, grid_h = 6, 5

        grid_cells = VGroup()
        walls = [(0, 4), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4),
                 (5, 3), (5, 2), (5, 1)]

        for i in range(grid_w):
            for j in range(grid_h):
                if (i, j) in walls:
                    color = GREY
                    opacity = 1.0
                else:
                    color = WHITE
                    opacity = 0.1

                cell = Square(side_length=cell_size, color=color, fill_opacity=opacity, stroke_width=1)
                cell.move_to(grid_origin + RIGHT * (i + 0.5) * cell_size + UP * (j + 0.5) * cell_size)
                grid_cells.add(cell)

        self.play(ShowCreation(grid_cells), run_time=1)

        # Arrow to Distance Field
        arrow_transform = Arrow(LEFT * 3.5 + UP * 0.3, LEFT * 2 + UP * 0.3, buff=0.1, color=YELLOW, stroke_width=4)
        transform_label = Text("Distance\nTransform", font_size=12, color=YELLOW)
        transform_label.next_to(arrow_transform, UP, buff=0.1)
        self.play(ShowCreation(arrow_transform), FadeIn(transform_label))

        # Right side: Distance Field
        df_label = Text("Distance Field", font_size=18, color=GREEN)
        df_label.move_to(RIGHT * 0 + UP * 1.8)
        self.play(FadeIn(df_label))

        # Draw distance field
        df_origin = LEFT * 1.5 + DOWN * 0.8
        df_cells = VGroup()

        def get_distance(i, j):
            if (i, j) in walls:
                return 0
            min_dist = float('inf')
            for wi, wj in walls:
                dist = np.sqrt((i - wi)**2 + (j - wj)**2)
                min_dist = min(min_dist, dist)
            return min_dist

        for i in range(grid_w):
            for j in range(grid_h):
                dist = get_distance(i, j)

                if dist == 0:
                    color = GREY
                    opacity = 1.0
                elif dist < 1.5:
                    color = RED
                    opacity = 0.8
                elif dist < 2.5:
                    color = ORANGE
                    opacity = 0.6
                elif dist < 3.5:
                    color = YELLOW
                    opacity = 0.4
                else:
                    color = GREEN
                    opacity = 0.3

                cell = Square(side_length=cell_size, color=color, fill_opacity=opacity, stroke_width=1)
                cell.move_to(df_origin + RIGHT * (i + 0.5) * cell_size + UP * (j + 0.5) * cell_size)

                dist_text = Text(f"{dist:.1f}", font_size=7, color=WHITE)
                dist_text.move_to(cell.get_center())
                df_cells.add(VGroup(cell, dist_text))

        self.play(ShowCreation(df_cells), run_time=1.5)

        # Benefit box - positioned to the right, not overlapping
        benefit = VGroup(
            Rectangle(width=4.5, height=1.8, color=TEAL),
            Text("Benefit:", font_size=16, color=TEAL).shift(UP * 0.5),
            Text("Precomputed → O(1) lookup", font_size=14).shift(UP * 0),
            Text("No ray-casting needed!", font_size=14).shift(DOWN * 0.5),
        )
        benefit.move_to(RIGHT * 4.5 + UP * 0.3)
        self.play(ShowCreation(benefit))

        # Color legend - at bottom
        legend = VGroup(
            Text("Distance: ", font_size=12),
            Square(side_length=0.2, color=GREY, fill_opacity=1).shift(RIGHT * 0.8),
            Text("0", font_size=10).shift(RIGHT * 1.1),
            Square(side_length=0.2, color=RED, fill_opacity=0.8).shift(RIGHT * 1.5),
            Text("~1", font_size=10).shift(RIGHT * 1.85),
            Square(side_length=0.2, color=ORANGE, fill_opacity=0.6).shift(RIGHT * 2.3),
            Text("~2", font_size=10).shift(RIGHT * 2.65),
            Square(side_length=0.2, color=GREEN, fill_opacity=0.3).shift(RIGHT * 3.1),
            Text(">3", font_size=10).shift(RIGHT * 3.45),
        )
        legend.move_to(DOWN * 2.8)
        self.play(FadeIn(legend))
        self.wait(1)

        # ========== Clear and show Weight Calculation ==========
        self.play(
            FadeOut(df_title), FadeOut(grid_label), FadeOut(grid_cells),
            FadeOut(arrow_transform), FadeOut(transform_label),
            FadeOut(df_label), FadeOut(df_cells), FadeOut(benefit), FadeOut(legend),
        )

        # ========== Part 3: Weight Calculation ==========
        weight_title = Text("3. Weight Calculation", font_size=28, color=ORANGE)
        weight_title.move_to(UP * 2.5)
        self.play(FadeIn(weight_title))

        concept = Text("Project LiDAR points to each particle's coordinate frame", font_size=16, color=YELLOW)
        concept.move_to(UP * 1.8)
        self.play(FadeIn(concept))

        # Draw map with wall
        map_rect = Rectangle(width=7, height=3.5, color=WHITE)
        map_rect.move_to(LEFT * 1.5 + DOWN * 0.2)

        wall = Rectangle(width=5.5, height=0.3, color=GREY, fill_opacity=0.9)
        wall.move_to(LEFT * 1.5 + UP * 1.2)
        wall_label = Text("Wall", font_size=14, color=GREY).next_to(wall, UP, buff=0.1)

        self.play(ShowCreation(map_rect), ShowCreation(wall), FadeIn(wall_label))

        lidar_info = Text("LiDAR: r = 2.0m, θ = 90° (front)", font_size=14, color=RED)
        lidar_info.move_to(DOWN * 2.3)
        self.play(FadeIn(lidar_info))

        # Three particles
        particle_data = [
            (LEFT * 3.5 + DOWN * 0.3, "P1", BLUE, "too far"),
            (LEFT * 1.5 + DOWN * 0.8, "P2", GREEN, "correct"),
            (RIGHT * 0.5 + DOWN * 0.3, "P3", RED, "too close"),
        ]

        particles = VGroup()
        for pos, name, color, _ in particle_data:
            p = Dot(pos, radius=0.2, color=color)
            label = Text(name, font_size=14, color=color).next_to(p, DOWN, buff=0.1)
            arrow = Arrow(pos, pos + UP * 0.4, buff=0, color=color, stroke_width=2)
            particles.add(VGroup(p, label, arrow))

        self.play(ShowCreation(particles))
        self.wait(0.3)

        # Project LiDAR for each particle
        projections = VGroup()
        distances = []

        for i, (pos, name, color, status) in enumerate(particle_data):
            proj_end = pos + UP * 2.0
            proj_line = DashedLine(pos, proj_end, color=color, stroke_width=2)
            proj_point = Dot(proj_end, radius=0.1, color=color)

            wall_y = 1.0
            proj_y = proj_end[1]
            d = abs(proj_y - wall_y)
            distances.append(d)

            projections.add(VGroup(proj_line, proj_point))

        self.play(ShowCreation(projections))

        # Show distances
        dist_labels = VGroup()
        for i, (pos, name, color, status) in enumerate(particle_data):
            proj_end = pos + UP * 2.0
            d = distances[i]

            wall_point = np.array([proj_end[0], 1.0, 0])
            if d > 0.1:
                dist_line = Line(proj_end, wall_point, color=YELLOW, stroke_width=2)
                d_label = Text(f"d={d:.1f}", font_size=11, color=YELLOW)
                d_label.next_to(dist_line, RIGHT, buff=0.05)
                dist_labels.add(VGroup(dist_line, d_label))
            else:
                checkmark = Text("d≈0 ✓", font_size=12, color=GREEN)
                checkmark.next_to(proj_end, RIGHT, buff=0.1)
                dist_labels.add(checkmark)

        self.play(ShowCreation(dist_labels))

        # Weight formula
        formula_box = VGroup(
            Rectangle(width=4.5, height=2.2, color=PURPLE),
            Text("Gaussian Likelihood:", font_size=14).shift(UP * 0.7),
            Text("P(z|x) = exp(-d²/2σ²)", font_size=18, color=YELLOW).shift(UP * 0.1),
            Text("d → 0: P → 1 (high)", font_size=11, color=GREEN).shift(DOWN * 0.4),
            Text("d → ∞: P → 0 (low)", font_size=11, color=RED).shift(DOWN * 0.8),
        )
        formula_box.move_to(RIGHT * 4.5 + UP * 0.5)
        self.play(ShowCreation(formula_box))

        weight_results = VGroup(
            Text("Weights:", font_size=14, color=ORANGE).shift(UP * 0.4),
            Text("P1: w ≈ 0.05 (low)", font_size=12, color=BLUE).shift(UP * 0),
            Text("P2: w ≈ 0.95 (high)", font_size=12, color=GREEN).shift(DOWN * 0.35),
            Text("P3: w ≈ 0.10 (low)", font_size=12, color=RED).shift(DOWN * 0.7),
        )
        weight_results.move_to(RIGHT * 4.5 + DOWN * 1.6)
        self.play(FadeIn(weight_results))
        self.wait(1)

        # ========== Clear and show Resampling ==========
        self.play(
            FadeOut(weight_title), FadeOut(concept), FadeOut(map_rect), FadeOut(wall), FadeOut(wall_label),
            FadeOut(lidar_info), FadeOut(particles), FadeOut(projections), FadeOut(dist_labels),
            FadeOut(formula_box), FadeOut(weight_results),
        )

        # ========== Part 4: Resampling (Bowl Flip Concept) ==========
        resample_title = Text("4. Resampling", font_size=28, color=GREEN)
        resample_title.move_to(UP * 2.5)
        self.play(FadeIn(resample_title))

        # Concept explanation
        concept = Text("Resampling = Sample from Posterior Distribution", font_size=18, color=YELLOW)
        concept.move_to(UP * 1.8)
        self.play(FadeIn(concept))

        # Draw inverted bowl (2D cross-section)
        bowl_center = LEFT * 3 + DOWN * 0.5

        # Inverted Gaussian curve (bowl shape)
        def inverted_gaussian(x):
            return -1.5 * np.exp(-x**2 / 0.8) + 0.5

        bowl_curve = FunctionGraph(
            inverted_gaussian,
            x_range=[-2.5, 2.5, 0.1],
            color=GREEN,
            stroke_width=4
        )
        bowl_curve.move_to(bowl_center)

        bowl_label = Text("Inverted P(x)", font_size=14, color=GREEN)
        bowl_label.next_to(bowl_curve, UP, buff=0.3)

        low_label = Text("LOW = High Prob", font_size=12, color=YELLOW)
        low_label.move_to(bowl_center + DOWN * 1.2)

        self.play(ShowCreation(bowl_curve), FadeIn(bowl_label), FadeIn(low_label))

        # Particles fall into bowl
        np.random.seed(42)
        n_particles = 12
        particles = VGroup()
        start_positions = []
        end_positions = []

        for i in range(n_particles):
            # Start: random x, above the bowl
            start_x = np.random.uniform(-2, 2)
            start_pos = bowl_center + RIGHT * start_x + UP * 1.5
            start_positions.append(start_pos)

            # End: sampled from distribution (concentrated at center)
            end_x = np.random.normal(0, 0.4)
            end_y = inverted_gaussian(end_x)
            end_pos = bowl_center + RIGHT * end_x + UP * (end_y + 0.15)
            end_positions.append(end_pos)

            p = Dot(start_pos, radius=0.12, color=YELLOW)
            particles.add(p)

        self.play(FadeIn(particles))

        # Animate particles rolling into bowl
        animations = [
            particles[i].animate.move_to(end_positions[i])
            for i in range(n_particles)
        ]
        self.play(*animations, run_time=1.5)

        roll_text = Text("Particles roll into the bowl!", font_size=14, color=YELLOW)
        roll_text.move_to(bowl_center + DOWN * 2)
        self.play(FadeIn(roll_text))
        self.wait(0.5)

        # Now flip the bowl
        self.play(FadeOut(roll_text), FadeOut(low_label), FadeOut(bowl_label))

        flip_text = Text("Flip!", font_size=20, color=WHITE)
        flip_text.move_to(DOWN * 2)
        self.play(FadeIn(flip_text))

        # Normal Gaussian curve (peak shape)
        def normal_gaussian(x):
            return 1.5 * np.exp(-x**2 / 0.8) - 0.5

        peak_curve = FunctionGraph(
            normal_gaussian,
            x_range=[-2.5, 2.5, 0.1],
            color=GREEN,
            stroke_width=4
        )
        peak_curve.move_to(bowl_center)

        # Calculate flipped particle positions
        flipped_positions = []
        for i in range(n_particles):
            end_x = end_positions[i][0] - bowl_center[0]
            end_y = normal_gaussian(end_x)
            flipped_pos = bowl_center + RIGHT * end_x + UP * (end_y + 0.15)
            flipped_positions.append(flipped_pos)

        # Animate flip
        particle_animations = [
            particles[i].animate.move_to(flipped_positions[i])
            for i in range(n_particles)
        ]

        self.play(
            Transform(bowl_curve, peak_curve),
            *particle_animations,
            run_time=1.5
        )

        self.play(FadeOut(flip_text))

        peak_label = Text("P(x)", font_size=14, color=GREEN)
        peak_label.next_to(bowl_curve, UP, buff=0.3)
        high_label = Text("HIGH = High Prob", font_size=12, color=YELLOW)
        high_label.move_to(bowl_center + UP * 1.8)

        self.play(FadeIn(peak_label), FadeIn(high_label))

        # Result explanation on right side
        result_box = VGroup(
            Rectangle(width=4.5, height=2.5, color=GREEN),
            Text("Result:", font_size=16, color=GREEN).shift(UP * 0.8),
            Text("Particles concentrated", font_size=14).shift(UP * 0.3),
            Text("on HIGH probability region!", font_size=14).shift(DOWN * 0.1),
            Text("", font_size=14).shift(DOWN * 0.4),
            Text("Same count, new positions", font_size=12, color=YELLOW).shift(DOWN * 0.8),
        )
        result_box.move_to(RIGHT * 3.5 + DOWN * 0.3)
        self.play(ShowCreation(result_box))

        self.wait(1.5)

        # ========== Clear and show Final Output ==========
        self.play(
            FadeOut(resample_title), FadeOut(concept),
            FadeOut(bowl_curve), FadeOut(peak_label), FadeOut(high_label),
            FadeOut(particles), FadeOut(result_box),
        )

        # ========== Part 5: Final Output - Average ==========
        output_title = Text("5. Final Output: Estimated Pose", font_size=28, color=TEAL)
        output_title.move_to(UP * 2.5)
        self.play(FadeIn(output_title))

        # Problem statement
        problem = Text("Problem: N particles → need ONE pose for navigation", font_size=16, color=YELLOW)
        problem.move_to(UP * 1.8)
        self.play(FadeIn(problem))

        # Solution - after resampling, weights are equal!
        solution = Text("After resampling: all weights = 1/N → Simple Average!", font_size=16, color=GREEN)
        solution.move_to(UP * 1.3)
        self.play(FadeIn(solution))

        # Formula
        formula = VGroup(
            Rectangle(width=6, height=1.2, color=TEAL),
            Text("(x̂, ŷ, θ̂) = (1/N) Σ (xᵢ, yᵢ, θᵢ)", font_size=20, color=YELLOW),
        )
        formula.move_to(UP * 0.3)
        self.play(ShowCreation(formula))

        # Visual example - particles SAME SIZE but CLUSTERED
        vis_label = Text("After Resampling:", font_size=14)
        vis_label.move_to(LEFT * 4.5 + DOWN * 0.6)
        self.play(FadeIn(vis_label))

        # Draw particles - ALL SAME SIZE, but clustered in center
        vis_center = DOWN * 1.5
        np.random.seed(123)

        # Clustered particles (most near center, few outliers)
        particle_positions = [
            # Cluster in center (many particles here due to resampling)
            (0.0, 0.0), (0.1, 0.05), (-0.05, 0.1), (0.15, -0.05),
            (0.0, 0.15), (-0.1, 0.0), (0.05, -0.1), (-0.08, 0.08),
            # Few outliers
            (-1.5, 0.3), (1.2, -0.2),
        ]

        vis_particles = VGroup()
        total_x, total_y = 0, 0
        n_particles = len(particle_positions)

        for x, y in particle_positions:
            pos = vis_center + RIGHT * x * 2 + UP * y * 2
            # ALL SAME SIZE (equal weight after resampling)
            p = Dot(pos, radius=0.12, color=GREEN)
            vis_particles.add(p)
            total_x += x
            total_y += y

        self.play(ShowCreation(vis_particles))

        # Show that they're all same weight
        same_weight_text = Text("All particles: w = 1/N (same size!)", font_size=12, color=WHITE)
        same_weight_text.move_to(LEFT * 3.5 + DOWN * 2.5)
        self.play(FadeIn(same_weight_text))

        # Calculate average (simple average since all weights equal)
        avg_x = total_x / n_particles
        avg_y = total_y / n_particles
        avg_pos = vis_center + RIGHT * avg_x * 2 + UP * avg_y * 2

        # Draw estimated pose (star)
        estimate = Dot(avg_pos, radius=0.2, color=TEAL)
        star = Text("★", font_size=20, color=WHITE)
        star.move_to(avg_pos)
        estimate_label = Text("Estimated\nPose", font_size=11, color=TEAL)
        estimate_label.next_to(estimate, RIGHT, buff=0.2)

        self.play(ShowCreation(estimate), FadeIn(star), FadeIn(estimate_label))

        # Explanation
        explain_box = VGroup(
            Rectangle(width=4, height=2, color=YELLOW),
            Text("Why biased to center?", font_size=12, color=YELLOW).shift(UP * 0.6),
            Text("More particles there!", font_size=11).shift(UP * 0.2),
            Text("(duplicated during", font_size=10).shift(DOWN * 0.15),
            Text("resampling)", font_size=10).shift(DOWN * 0.45),
        )
        explain_box.move_to(RIGHT * 4 + DOWN * 1.2)
        self.play(ShowCreation(explain_box))

        # Final destination
        final_box = VGroup(
            Rectangle(width=5, height=0.8, color=GREEN),
            Text("This pose → /amcl_pose → move_base", font_size=13, color=GREEN),
        )
        final_box.to_edge(DOWN)
        self.play(ShowCreation(final_box))

        self.wait(2)


# Run: manimgl pf_update_step.py PFUpdateStep -w -r 1920x1080
