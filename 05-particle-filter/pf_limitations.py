from manimlib import *
import numpy as np


class PFLimitations(Scene):
    """
    Particle Filter Practical Limitations + Solutions (Animated):
    1. Dynamic Obstacles → z_rand (tolerance)
    2. Symmetric Environment → Visual landmarks
    3. Kidnapped Robot → Recovery mode
    4. Computational Cost → KLD-Sampling
    """

    def construct(self):
        # Title
        title = Text("Particle Filter: Practical Limitations", font_size=36, color=RED)
        title.to_edge(UP)
        self.play(Write(title))
        self.wait(0.5)

        # ========== Part 1: Dynamic Obstacles ==========
        part1_title = Text("1. Dynamic Obstacles", font_size=28, color=ORANGE)
        part1_title.move_to(UP * 2.5)
        self.play(FadeIn(part1_title))

        # Draw map with static wall
        map_rect = Rectangle(width=5, height=3, color=WHITE)
        map_rect.move_to(LEFT * 3.5 + DOWN * 0.3)

        static_wall = Rectangle(width=4, height=0.25, color=GREY, fill_opacity=0.9)
        static_wall.move_to(LEFT * 3.5 + UP * 0.8)
        wall_label = Text("Wall (in map)", font_size=11, color=GREY)
        wall_label.next_to(static_wall, UP, buff=0.1)

        self.play(ShowCreation(map_rect), ShowCreation(static_wall), FadeIn(wall_label))

        # Robot
        robot_pos = LEFT * 4.5 + DOWN * 0.8
        robot = Circle(radius=0.25, color=BLUE, fill_opacity=0.5)
        robot.move_to(robot_pos)

        self.play(ShowCreation(robot))

        # LiDAR ray hitting wall (correct)
        lidar_ray = Line(robot_pos, robot_pos + UP * 1.35, color=GREEN, stroke_width=3)
        hit_point = Dot(robot_pos + UP * 1.35, radius=0.08, color=GREEN)

        self.play(ShowCreation(lidar_ray), ShowCreation(hit_point))

        # Person appears and blocks
        person = VGroup(
            Circle(radius=0.12, color=RED, fill_opacity=0.8),
            Line(ORIGIN, DOWN * 0.35, color=RED, stroke_width=3),
        )
        person.move_to(LEFT * 2.5 + DOWN * 0.3)

        self.play(FadeIn(person))
        self.play(person.animate.move_to(LEFT * 4 + DOWN * 0.1), run_time=0.8)

        # Ray now hits person
        wrong_ray = Line(robot_pos, LEFT * 4 + DOWN * 0.1, color=RED, stroke_width=3)
        wrong_hit = Dot(LEFT * 4 + DOWN * 0.1, radius=0.08, color=RED)
        problem_text = Text("Large error → weight ≈ 0?", font_size=12, color=RED)
        problem_text.move_to(LEFT * 3 + DOWN * 1.5)

        self.play(
            Transform(lidar_ray, wrong_ray),
            Transform(hit_point, wrong_hit),
            FadeIn(problem_text),
        )

        self.wait(0.5)

        # ===== Solution: z_rand =====
        solution_title = Text("Solution: z_rand (tolerance)", font_size=18, color=GREEN)
        solution_title.move_to(RIGHT * 3 + UP * 1.5)
        self.play(FadeIn(solution_title))

        # Show probability distribution
        axes = Axes(
            x_range=[0, 3, 1],
            y_range=[0, 1.2, 0.5],
            width=4,
            height=2,
        )
        axes.move_to(RIGHT * 3 + DOWN * 0.3)

        x_label = Text("distance error", font_size=10)
        x_label.next_to(axes, DOWN, buff=0.1)
        y_label = Text("P", font_size=10)
        y_label.next_to(axes, LEFT, buff=0.1)

        self.play(ShowCreation(axes), FadeIn(x_label), FadeIn(y_label))

        # Gaussian only (without z_rand) - problem!
        gaussian = axes.get_graph(
            lambda x: np.exp(-x**2 / 0.3),
            x_range=[0, 2.5],
            color=BLUE
        )
        gaussian_label = Text("Gaussian only", font_size=10, color=BLUE)
        gaussian_label.move_to(RIGHT * 2 + UP * 0.5)

        self.play(ShowCreation(gaussian), FadeIn(gaussian_label))

        # Show large error point → P ≈ 0
        error_point = Dot(axes.c2p(2.0, 0.01), radius=0.08, color=RED)
        error_label = Text("Large error\nP ≈ 0 !", font_size=9, color=RED)
        error_label.move_to(RIGHT * 4.5 + UP * 0.2)
        error_arrow = Arrow(error_label.get_left(), axes.c2p(2.0, 0.05), buff=0.1, color=RED, stroke_width=2)

        self.play(ShowCreation(error_point), FadeIn(error_label), ShowCreation(error_arrow))
        self.wait(0.5)

        # Add z_rand - baseline
        self.play(FadeOut(error_point), FadeOut(error_label), FadeOut(error_arrow))

        add_zrand = Text("Add z_rand baseline:", font_size=11, color=YELLOW)
        add_zrand.move_to(RIGHT * 3 + DOWN * 1.5)
        self.play(FadeIn(add_zrand))

        uniform_line = Line(
            axes.c2p(0, 0.1),
            axes.c2p(2.5, 0.1),
            color=YELLOW,
            stroke_width=3
        )
        uniform_label = Text("z_rand", font_size=10, color=YELLOW)
        uniform_label.move_to(RIGHT * 4.8 + DOWN * 0.6)

        self.play(ShowCreation(uniform_line), FadeIn(uniform_label))

        # Now show: even large error gets some probability
        new_error_point = Dot(axes.c2p(2.0, 0.1), radius=0.08, color=GREEN)
        new_error_label = Text("Now P ≈ 0.1\n(not zero!)", font_size=9, color=GREEN)
        new_error_label.move_to(RIGHT * 4.5 + UP * 0.2)
        new_error_arrow = Arrow(new_error_label.get_left(), axes.c2p(2.0, 0.12), buff=0.1, color=GREEN, stroke_width=2)

        self.play(ShowCreation(new_error_point), FadeIn(new_error_label), ShowCreation(new_error_arrow))

        # Explanation
        explain = Text("Particle not killed by one bad reading", font_size=12, color=GREEN)
        explain.move_to(RIGHT * 3 + DOWN * 2.2)
        self.play(FadeIn(explain))

        limitation = Text("⚠ Only works for sparse obstacles", font_size=11, color=GREY)
        limitation.move_to(RIGHT * 3 + DOWN * 2.7)
        self.play(FadeIn(limitation))

        self.wait(1)

        # Clear
        self.play(
            FadeOut(part1_title), FadeOut(map_rect), FadeOut(static_wall), FadeOut(wall_label),
            FadeOut(robot), FadeOut(lidar_ray), FadeOut(hit_point),
            FadeOut(person), FadeOut(problem_text),
            FadeOut(solution_title), FadeOut(axes), FadeOut(x_label), FadeOut(y_label),
            FadeOut(gaussian), FadeOut(gaussian_label),
            FadeOut(uniform_line), FadeOut(uniform_label), FadeOut(add_zrand),
            FadeOut(new_error_point), FadeOut(new_error_label), FadeOut(new_error_arrow),
            FadeOut(explain), FadeOut(limitation),
        )

        # ========== Part 2: Symmetric Environment ==========
        part2_title = Text("2. Symmetric Environment", font_size=28, color=PURPLE)
        part2_title.move_to(UP * 2.5)
        self.play(FadeIn(part2_title))

        # Draw corridor
        corridor = VGroup()
        top_wall = Rectangle(width=10, height=0.15, color=GREY, fill_opacity=0.9)
        top_wall.move_to(UP * 1.2)
        bottom_wall = Rectangle(width=10, height=0.15, color=GREY, fill_opacity=0.9)
        bottom_wall.move_to(UP * 0.2)
        corridor.add(top_wall, bottom_wall)

        for x in [-3, 0, 3]:
            divider = Line(UP * 1.1, UP * 0.3, color=GREY_A, stroke_width=1)
            divider.move_to(RIGHT * x)
            corridor.add(divider)

        self.play(ShowCreation(corridor))

        # Robot with question mark
        robot2 = Circle(radius=0.2, color=BLUE, fill_opacity=0.5)
        robot2.move_to(LEFT * 1.5 + UP * 0.7)
        question = Text("?", font_size=20, color=RED)
        question.move_to(LEFT * 1.5 + UP * 0.7)

        self.play(ShowCreation(robot2), FadeIn(question))

        # Particles in multiple segments (problem)
        particles = VGroup()
        for x in [-4.5, -4.2, -1.5, -1.2, 1.5, 1.8]:
            p = Dot(RIGHT * x + UP * 0.7, radius=0.1, color=YELLOW)
            particles.add(p)

        self.play(ShowCreation(particles))

        problem2 = Text("LiDAR sees same thing in all segments!", font_size=14, color=RED)
        problem2.move_to(DOWN * 0.3)
        self.play(FadeIn(problem2))

        self.wait(0.5)

        # ===== Solution: Visual landmarks =====
        solution2_title = Text("Solution: Visual Landmarks", font_size=18, color=GREEN)
        solution2_title.move_to(DOWN * 1)
        self.play(FadeIn(solution2_title), FadeOut(problem2))

        # Add door signs to corridor
        sign_a = VGroup(
            Rectangle(width=0.4, height=0.25, color=GREEN, fill_opacity=0.8),
            Text("A", font_size=10, color=WHITE),
        )
        sign_a.move_to(LEFT * 4.5 + UP * 1.5)

        sign_b = VGroup(
            Rectangle(width=0.4, height=0.25, color=BLUE, fill_opacity=0.8),
            Text("B", font_size=10, color=WHITE),
        )
        sign_b.move_to(LEFT * 1.5 + UP * 1.5)

        sign_c = VGroup(
            Rectangle(width=0.4, height=0.25, color=RED, fill_opacity=0.8),
            Text("C", font_size=10, color=WHITE),
        )
        sign_c.move_to(RIGHT * 1.5 + UP * 1.5)

        self.play(FadeIn(sign_a), FadeIn(sign_b), FadeIn(sign_c))

        # Camera on robot sees sign
        camera_ray = DashedLine(
            LEFT * 1.5 + UP * 0.7,
            LEFT * 1.5 + UP * 1.5,
            color=TEAL,
            stroke_width=2
        )

        self.play(ShowCreation(camera_ray))

        # Camera reads "B"
        sees_b = Text("Camera sees: B", font_size=14, color=TEAL)
        sees_b.move_to(RIGHT * 3 + UP * 0.7)
        self.play(FadeIn(sees_b))

        # Particles converge to segment B only
        converge_anims = []
        target_positions = [LEFT * 1.5, LEFT * 1.3, LEFT * 1.7, LEFT * 1.4, LEFT * 1.6, LEFT * 1.5]
        for i, p in enumerate(particles):
            converge_anims.append(p.animate.move_to(target_positions[i] + UP * 0.7))

        self.play(*converge_anims, run_time=1)

        # Remove question mark
        self.play(FadeOut(question))

        result2 = Text("Disambiguated!", font_size=14, color=GREEN)
        result2.move_to(DOWN * 1.8)
        self.play(FadeIn(result2))

        limitation2 = Text("⚠ Needs camera + landmark detection", font_size=11, color=GREY)
        limitation2.move_to(DOWN * 2.3)
        self.play(FadeIn(limitation2))

        self.wait(1)

        # Clear
        self.play(
            FadeOut(part2_title), FadeOut(corridor), FadeOut(robot2),
            FadeOut(particles), FadeOut(solution2_title),
            FadeOut(sign_a), FadeOut(sign_b), FadeOut(sign_c),
            FadeOut(camera_ray), FadeOut(sees_b),
            FadeOut(result2), FadeOut(limitation2),
        )

        # ========== Part 3: Kidnapped Robot ==========
        part3_title = Text("3. Kidnapped Robot", font_size=28, color=RED)
        part3_title.move_to(UP * 2.5)
        self.play(FadeIn(part3_title))

        # Map
        map3 = Rectangle(width=10, height=2, color=WHITE)
        map3.move_to(UP * 0.8)
        self.play(ShowCreation(map3))

        # Robot and particles at original position
        orig_pos = LEFT * 3 + UP * 0.8
        robot3 = Circle(radius=0.25, color=BLUE, fill_opacity=0.5)
        robot3.move_to(orig_pos)

        particles3 = VGroup()
        np.random.seed(42)
        for _ in range(10):
            offset = np.random.normal(0, 0.2, 2)
            p = Dot(orig_pos + RIGHT * offset[0] + UP * offset[1], radius=0.08, color=GREEN)
            particles3.add(p)

        self.play(ShowCreation(robot3), ShowCreation(particles3))

        # Kidnap!
        kidnap = Text("*Kidnapped!*", font_size=18, color=RED)
        kidnap.move_to(UP * 2)
        self.play(FadeIn(kidnap))

        new_pos = RIGHT * 3 + UP * 0.8
        self.play(robot3.animate.move_to(new_pos), run_time=0.8)
        self.play(FadeOut(kidnap))

        # Problem: no particles near robot
        problem3 = Text("No particles here → Lost!", font_size=14, color=RED)
        problem3.move_to(RIGHT * 3 + DOWN * 0.2)
        arrow_problem = Arrow(problem3.get_top(), new_pos + DOWN * 0.3, buff=0.1, color=RED)
        self.play(FadeIn(problem3), ShowCreation(arrow_problem))

        self.wait(0.5)

        # ===== Solution: Recovery mode =====
        solution3_title = Text("Solution: Recovery Mode", font_size=18, color=GREEN)
        solution3_title.move_to(DOWN * 1)
        self.play(FadeIn(solution3_title), FadeOut(problem3), FadeOut(arrow_problem))

        # Inject random particles across map
        random_particles = VGroup()
        np.random.seed(123)
        for _ in range(15):
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(0.3, 1.3)
            p = Dot(RIGHT * x + UP * y, radius=0.08, color=ORANGE)
            random_particles.add(p)

        inject_text = Text("Inject random particles (when avg weight low)", font_size=12, color=ORANGE)
        inject_text.move_to(DOWN * 1.6)
        self.play(FadeIn(random_particles), FadeIn(inject_text))

        self.wait(0.5)

        # Particles near robot survive, others die
        self.play(FadeOut(inject_text))

        # Keep only particles near new_pos
        surviving = VGroup()
        dying = VGroup()
        for p in random_particles:
            dist = np.linalg.norm(p.get_center() - new_pos)
            if dist < 1.5:
                surviving.add(p.copy())
            else:
                dying.add(p)

        # Also add from original particles3 - they all die
        self.play(FadeOut(dying), FadeOut(particles3), run_time=0.5)

        # Surviving particles converge
        converge3 = []
        for i, p in enumerate(surviving):
            offset = np.random.normal(0, 0.15, 2)
            target = new_pos + RIGHT * offset[0] + UP * offset[1]
            converge3.append(p.animate.move_to(target))

        recover_text = Text("Particles near robot survive!", font_size=12, color=GREEN)
        recover_text.move_to(DOWN * 1.6)
        self.play(*converge3, FadeIn(recover_text), run_time=1)

        limitation3 = Text("⚠ Slow, may false trigger", font_size=11, color=GREY)
        limitation3.move_to(DOWN * 2.2)
        self.play(FadeIn(limitation3))

        self.wait(1)

        # Clear
        self.play(
            FadeOut(part3_title), FadeOut(map3), FadeOut(robot3),
            FadeOut(surviving), FadeOut(random_particles),
            FadeOut(solution3_title), FadeOut(recover_text), FadeOut(limitation3),
        )

        # ========== Part 4: Computational Cost ==========
        part4_title = Text("4. Computational Cost", font_size=28, color=TEAL)
        part4_title.move_to(UP * 2.5)
        self.play(FadeIn(part4_title))

        # Show computation formula
        formula = Text("Cost = N particles × 360 rays × calculation", font_size=16, color=YELLOW)
        formula.move_to(UP * 1.7)
        self.play(FadeIn(formula))

        # Two scenarios side by side
        # Left: Few particles (fast but bad)
        left_box = Rectangle(width=4.5, height=2.5, color=RED)
        left_box.move_to(LEFT * 3 + DOWN * 0.2)
        left_title = Text("100 particles", font_size=14, color=RED)
        left_title.move_to(LEFT * 3 + UP * 1)

        few_particles = VGroup()
        np.random.seed(1)
        for _ in range(8):
            x = np.random.uniform(-1.5, 1.5)
            y = np.random.uniform(-0.8, 0.8)
            p = Dot(LEFT * 3 + RIGHT * x + DOWN * 0.2 + UP * y, radius=0.1, color=RED)
            few_particles.add(p)

        self.play(ShowCreation(left_box), FadeIn(left_title), ShowCreation(few_particles))

        left_speed = Text("Fast: 10ms ✓", font_size=12, color=GREEN)
        left_speed.move_to(LEFT * 3 + DOWN * 1.2)
        left_accuracy = Text("May lose track ✗", font_size=12, color=RED)
        left_accuracy.move_to(LEFT * 3 + DOWN * 1.6)
        self.play(FadeIn(left_speed), FadeIn(left_accuracy))

        # Right: Many particles (accurate but slow)
        right_box = Rectangle(width=4.5, height=2.5, color=GREEN)
        right_box.move_to(RIGHT * 3 + DOWN * 0.2)
        right_title = Text("5000 particles", font_size=14, color=GREEN)
        right_title.move_to(RIGHT * 3 + UP * 1)

        many_particles = VGroup()
        for _ in range(40):
            x = np.random.uniform(-1.5, 1.5)
            y = np.random.uniform(-0.8, 0.8)
            p = Dot(RIGHT * 3 + RIGHT * x + DOWN * 0.2 + UP * y, radius=0.05, color=GREEN)
            many_particles.add(p)

        self.play(ShowCreation(right_box), FadeIn(right_title), ShowCreation(many_particles))

        right_speed = Text("Slow: 200ms ✗", font_size=12, color=RED)
        right_speed.move_to(RIGHT * 3 + DOWN * 1.2)
        right_accuracy = Text("Robust ✓", font_size=12, color=GREEN)
        right_accuracy.move_to(RIGHT * 3 + DOWN * 1.6)
        self.play(FadeIn(right_speed), FadeIn(right_accuracy))

        self.wait(0.8)

        # ===== Solution: KLD-Sampling =====
        self.play(
            FadeOut(left_box), FadeOut(left_title), FadeOut(few_particles),
            FadeOut(left_speed), FadeOut(left_accuracy),
            FadeOut(right_box), FadeOut(right_title), FadeOut(many_particles),
            FadeOut(right_speed), FadeOut(right_accuracy),
        )

        solution4_title = Text("Solution: KLD-Sampling (adaptive)", font_size=20, color=GREEN)
        solution4_title.move_to(UP * 1)
        self.play(FadeIn(solution4_title), FadeOut(formula))

        # Animated demonstration
        # State 1: Uncertain - many particles
        state1_label = Text("Uncertain → Many particles", font_size=14, color=ORANGE)
        state1_label.move_to(UP * 0.5)
        self.play(FadeIn(state1_label))

        kld_particles = VGroup()
        np.random.seed(7)
        for _ in range(30):
            x = np.random.uniform(-3, 3)
            y = np.random.uniform(-1.5, 0.5)
            p = Dot(RIGHT * x + DOWN * 0.5 + UP * y, radius=0.08, color=ORANGE)
            kld_particles.add(p)

        particle_count = Text("N = 2000", font_size=16, color=ORANGE)
        particle_count.move_to(RIGHT * 4 + DOWN * 0.5)

        self.play(ShowCreation(kld_particles), FadeIn(particle_count))
        self.wait(0.5)

        # State 2: Converging
        self.play(FadeOut(state1_label))
        state2_label = Text("Converging...", font_size=14, color=YELLOW)
        state2_label.move_to(UP * 0.5)
        self.play(FadeIn(state2_label))

        # Particles move toward center
        converge_center = DOWN * 0.5
        converge4 = []
        for p in kld_particles:
            offset = np.random.normal(0, 0.8, 2)
            target = converge_center + RIGHT * offset[0] + UP * offset[1]
            converge4.append(p.animate.move_to(target))

        new_count = Text("N = 800", font_size=16, color=YELLOW)
        new_count.move_to(RIGHT * 4 + DOWN * 0.5)

        # Remove some particles
        self.play(*converge4[:20], Transform(particle_count, new_count), run_time=1)
        self.play(FadeOut(kld_particles[20:]), run_time=0.3)

        # State 3: Converged - few particles
        self.play(FadeOut(state2_label))
        state3_label = Text("Converged → Few particles (fast!)", font_size=14, color=GREEN)
        state3_label.move_to(UP * 0.5)
        self.play(FadeIn(state3_label))

        # Tight cluster
        final_converge = []
        for i, p in enumerate(kld_particles[:15]):
            offset = np.random.normal(0, 0.3, 2)
            target = converge_center + RIGHT * offset[0] + UP * offset[1]
            final_converge.append(p.animate.move_to(target))

        final_count = Text("N = 200", font_size=16, color=GREEN)
        final_count.move_to(RIGHT * 4 + DOWN * 0.5)

        self.play(*final_converge, Transform(particle_count, final_count), run_time=1)
        self.play(FadeOut(kld_particles[15:20]), run_time=0.3)

        # Change color to green
        self.play(*[p.animate.set_color(GREEN) for p in kld_particles[:15]])

        limitation4 = Text("⚠ Requires tuning min/max particles", font_size=11, color=GREY)
        limitation4.move_to(DOWN * 2)
        self.play(FadeIn(limitation4))

        self.wait(2)


# Run: manimgl pf_limitations.py PFLimitations -w -r 1920x1080
