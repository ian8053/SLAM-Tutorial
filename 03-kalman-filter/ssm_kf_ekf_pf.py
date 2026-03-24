from manimlib import *
import numpy as np


class SSMtoEKF(ThreeDScene):
    """
    SSM -> KF -> EKF -> Real World (N peaks)
    Using real 3D surface with mesh grid
    """

    def construct(self):
        # Setup 3D camera
        frame = self.camera.frame
        frame.set_euler_angles(
            theta=-30 * DEGREES,
            phi=70 * DEGREES,
        )

        # Create axes using ManimGL syntax
        axes = ThreeDAxes(
            x_range=[-3, 3, 1],
            y_range=[-3, 3, 1],
            z_range=[0, 2, 0.5],
        )
        axes.scale(0.8)
        axes.shift(DOWN * 0.5)

        self.play(ShowCreation(axes), run_time=1)

        # Title - use fix_in_frame for ManimGL
        title = Text("Kalman Filter", font_size=36, color=YELLOW)
        title.to_corner(UL)
        title.fix_in_frame()
        self.play(Write(title))

        # ============ Part 1: Initial state x_{k-1} ============
        def gaussian_2d(u, v, mu_x=0, mu_y=0, sigma_x=1, sigma_y=1, height=1.5):
            x = u
            y = v
            z = height * np.exp(-((x - mu_x)**2 / (2 * sigma_x**2) + (y - mu_y)**2 / (2 * sigma_y**2)))
            return axes.c2p(x, y, z)

        # Initial Gaussian surface
        surface1 = ParametricSurface(
            lambda u, v: gaussian_2d(u, v, mu_x=0, mu_y=0, sigma_x=0.8, sigma_y=0.8, height=1.5),
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(30, 30),
        )
        surface1.set_fill_by_value(axes, colors=[BLUE, TEAL, GREEN, YELLOW], axis=2)
        surface1.set_opacity(0.8)

        # Formula
        eq1 = Text("prior state x", font_size=20, color=BLUE)
        eq1.to_corner(UR).shift(DOWN * 0.5)
        eq1.fix_in_frame()

        self.play(ShowCreation(surface1), FadeIn(eq1), run_time=1.5)
        self.wait(0.5)

        # ============ Part 2: Prediction - surface spreads ============
        eq2 = Text("Predict: x = A x + B u", font_size=18, color=ORANGE)
        eq2.next_to(eq1, DOWN, buff=0.25, aligned_edge=RIGHT)
        eq2.fix_in_frame()
        self.play(FadeIn(eq2))

        # Predicted surface (wider, shorter = more uncertainty)
        surface2 = ParametricSurface(
            lambda u, v: gaussian_2d(u, v, mu_x=0.5, mu_y=0.3, sigma_x=1.2, sigma_y=1.2, height=1.0),
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(30, 30),
        )
        surface2.set_fill_by_value(axes, colors=[ORANGE, YELLOW, GREEN], axis=2)
        surface2.set_opacity(0.8)

        eq3 = Text("P = A P A' + Q", font_size=16, color=ORANGE)
        eq3.next_to(eq2, DOWN, buff=0.15, aligned_edge=RIGHT)
        eq3.fix_in_frame()

        self.play(
            Transform(surface1, surface2),
            FadeIn(eq3),
            run_time=1.5
        )

        note1 = Text("wider = uncertain", font_size=14, color=ORANGE)
        note1.next_to(eq3, DOWN, buff=0.15, aligned_edge=RIGHT)
        note1.fix_in_frame()
        self.play(FadeIn(note1))
        self.wait(0.5)

        # ============ Part 3: Observation - tall thin Gaussian ============
        eq4 = Text("Observe: z = H x + v", font_size=18, color=RED)
        eq4.next_to(note1, DOWN, buff=0.25, aligned_edge=RIGHT)
        eq4.fix_in_frame()
        self.play(FadeIn(eq4))

        # Observation surface (tall, narrow)
        surface_obs = ParametricSurface(
            lambda u, v: gaussian_2d(u, v, mu_x=0.8, mu_y=0.5, sigma_x=0.3, sigma_y=0.3, height=1.8),
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(30, 30),
        )
        surface_obs.set_fill_by_value(axes, colors=[RED, PINK], axis=2)
        surface_obs.set_opacity(0.6)

        self.play(ShowCreation(surface_obs), run_time=1)

        note2 = Text("tall = certain", font_size=14, color=RED)
        note2.next_to(eq4, DOWN, buff=0.1, aligned_edge=RIGHT)
        note2.fix_in_frame()
        self.play(FadeIn(note2))
        self.wait(0.5)

        # ============ Part 4: Update - fusion ============
        eq5 = Text("K = P H'/(H P H'+R)", font_size=16, color=YELLOW)
        eq5.next_to(note2, DOWN, buff=0.2, aligned_edge=RIGHT)
        eq5.fix_in_frame()
        self.play(FadeIn(eq5))

        eq6 = Text("x = x + K(z - Hx)", font_size=16, color=GREEN)
        eq6.next_to(eq5, DOWN, buff=0.1, aligned_edge=RIGHT)
        eq6.fix_in_frame()
        self.play(FadeIn(eq6))

        # Fused surface (narrower, between prediction and observation)
        surface_fused = ParametricSurface(
            lambda u, v: gaussian_2d(u, v, mu_x=0.7, mu_y=0.4, sigma_x=0.5, sigma_y=0.5, height=1.6),
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(30, 30),
        )
        surface_fused.set_fill_by_value(axes, colors=[GREEN, TEAL, BLUE], axis=2)
        surface_fused.set_opacity(0.8)

        self.play(
            FadeOut(surface_obs),
            Transform(surface1, surface_fused),
            run_time=1.5
        )

        note3 = Text("fused estimate", font_size=14, color=GREEN)
        note3.next_to(eq6, DOWN, buff=0.1, aligned_edge=RIGHT)
        note3.fix_in_frame()
        self.play(FadeIn(note3))
        self.wait(1)

        # ============ Part 5: EKF - multi-modal ============
        # Clear old formulas
        old_formulas = VGroup(eq1, eq2, eq3, note1, eq4, note2, eq5, eq6, note3)
        self.play(FadeOut(old_formulas))

        ekf_title = Text("EKF: Nonlinear", font_size=22, color=PURPLE)
        ekf_title.to_corner(UR).shift(DOWN * 0.3)
        ekf_title.fix_in_frame()
        self.play(FadeIn(ekf_title))

        ekf_note = Text("A -> F = df/dx", font_size=16, color=PURPLE)
        ekf_note.next_to(ekf_title, DOWN, buff=0.15, aligned_edge=RIGHT)
        ekf_note.fix_in_frame()
        self.play(FadeIn(ekf_note))

        # Multi-modal surface (3 peaks)
        def multi_modal_3(u, v):
            x, y = u, v
            # 3 Gaussian peaks
            z1 = 1.2 * np.exp(-((x - 0)**2 / 0.5 + (y - 0)**2 / 0.5))
            z2 = 0.9 * np.exp(-((x - 1.5)**2 / 0.4 + (y - 1)**2 / 0.4))
            z3 = 0.7 * np.exp(-((x + 1)**2 / 0.3 + (y - 1.5)**2 / 0.3))
            z = z1 + z2 + z3
            return axes.c2p(x, y, z)

        surface_multi3 = ParametricSurface(
            multi_modal_3,
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(40, 40),
        )
        surface_multi3.set_fill_by_value(axes, colors=[BLUE, GREEN, YELLOW, ORANGE], axis=2)
        surface_multi3.set_opacity(0.8)

        self.play(Transform(surface1, surface_multi3), run_time=2)

        problem = Text("EKF: single Gaussian!", font_size=16, color=RED)
        problem.next_to(ekf_note, DOWN, buff=0.15, aligned_edge=RIGHT)
        problem.fix_in_frame()
        self.play(FadeIn(problem))
        self.wait(1)

        # ============ Part 6: Real World - N peaks ============
        real_title = Text("Real: N hypotheses", font_size=20, color=RED)
        real_title.next_to(problem, DOWN, buff=0.2, aligned_edge=RIGHT)
        real_title.fix_in_frame()
        self.play(FadeIn(real_title))

        # N-modal surface (8 peaks)
        def multi_modal_n(u, v):
            x, y = u, v
            z = 0
            # 8 random peaks
            peaks = [
                (0, 0, 1.0, 0.4),
                (1.5, 1.0, 0.8, 0.35),
                (-1.2, 1.5, 0.6, 0.3),
                (1.8, -1.2, 0.7, 0.32),
                (-1.5, -1.0, 0.5, 0.28),
                (0.5, 2.0, 0.55, 0.25),
                (-2.0, 0.5, 0.45, 0.22),
                (2.2, 0.3, 0.5, 0.26),
            ]
            for px, py, h, s in peaks:
                z += h * np.exp(-((x - px)**2 / s + (y - py)**2 / s))
            return axes.c2p(x, y, z)

        surface_multi_n = ParametricSurface(
            multi_modal_n,
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(50, 50),
        )
        surface_multi_n.set_fill_by_value(axes, colors=[PURPLE, BLUE, TEAL, GREEN, YELLOW, ORANGE, RED], axis=2)
        surface_multi_n.set_opacity(0.8)

        self.play(Transform(surface1, surface_multi_n), run_time=2)

        solution = Text("-> Particle Filter", font_size=20, color=YELLOW)
        solution.next_to(real_title, DOWN, buff=0.15, aligned_edge=RIGHT)
        solution.fix_in_frame()
        self.play(FadeIn(solution))

        # Rotate camera for better view
        self.play(
            frame.animate.set_euler_angles(theta=-60 * DEGREES, phi=65 * DEGREES),
            run_time=2
        )

        self.wait(2)


# ================================================================
# Run:
# cd /home/ian/Desktop/slam_visualization
# manimgl ssm_kf_ekf_pf.py SSMtoEKF -w --low_quality
# ================================================================
