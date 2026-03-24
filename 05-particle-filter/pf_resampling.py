from manimlib import *
import numpy as np


class PFResampling(ThreeDScene):
    """
    Resampling visualization:
    1. Inverted surface (bowl shape - low = high probability)
    2. Particles fall and roll into the bowl
    3. Flip the surface back - particles now on peak
    """

    def construct(self):
        frame = self.camera.frame
        frame.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES)

        # Title
        title = Text("Resampling: Particles → High Probability", font_size=28, color=GREEN)
        title.to_corner(UL)
        title.fix_in_frame()
        self.play(Write(title))

        # Create axes
        axes = ThreeDAxes(
            x_range=[-3, 3, 1],
            y_range=[-3, 3, 1],
            z_range=[-2, 2, 0.5],
        )
        axes.scale(0.9)
        self.play(ShowCreation(axes), run_time=0.5)

        # Parameters
        post_mu_x, post_mu_y = 0.2, 0.1
        post_sigma = 0.5

        # ===== INVERTED surface (bowl shape) =====
        def inverted_func(u, v):
            # Negative z = bowl shape, center is LOW
            z = -1.5 * np.exp(-((u - post_mu_x)**2 / (2 * post_sigma**2) +
                                (v - post_mu_y)**2 / (2 * post_sigma**2)))
            return axes.c2p(u, v, z)

        inverted_surface = ParametricSurface(
            inverted_func,
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(25, 25)
        )
        inverted_surface.set_color(GREEN)
        inverted_surface.set_opacity(0.7)
        inverted_mesh = SurfaceMesh(inverted_surface, resolution=(25, 25))
        inverted_mesh.set_stroke(WHITE, width=0.5, opacity=0.5)

        self.play(ShowCreation(inverted_surface), ShowCreation(inverted_mesh), run_time=1.5)

        # Label
        label = Text("Low = High Probability (inverted)", font_size=18, color=GREEN)
        label.to_corner(UR)
        label.fix_in_frame()
        self.play(FadeIn(label))

        self.wait(0.5)

        # ===== Particles fall and roll into bowl =====
        np.random.seed(42)
        n_particles = 40

        particles = Group()
        start_positions = []
        bowl_positions = []

        for i in range(n_particles):
            # Start: random position, z = 0 (flat plane level)
            start_x = np.random.uniform(-2.5, 2.5)
            start_y = np.random.uniform(-2.5, 2.5)
            start_pos = axes.c2p(start_x, start_y, 0.5)
            start_positions.append(start_pos)

            # End: in the bowl (sampled from distribution)
            end_x = np.random.normal(post_mu_x, post_sigma * 0.6)
            end_y = np.random.normal(post_mu_y, post_sigma * 0.6)
            end_z = -1.5 * np.exp(-((end_x - post_mu_x)**2 / (2 * post_sigma**2) +
                                    (end_y - post_mu_y)**2 / (2 * post_sigma**2)))
            bowl_pos = axes.c2p(end_x, end_y, end_z + 0.08)
            bowl_positions.append((end_x, end_y, bowl_pos))

            particle = Sphere(radius=0.06, color=YELLOW)
            particle.move_to(start_pos)
            particles.add(particle)

        # Show particles
        self.play(FadeIn(particles), run_time=0.5)

        # Particles roll into bowl
        animations = [
            particles[i].animate.move_to(bowl_positions[i][2])
            for i in range(n_particles)
        ]
        self.play(*animations, run_time=2)

        # Rotate to see bowl
        self.play(
            frame.animate.set_euler_angles(theta=-60 * DEGREES, phi=60 * DEGREES),
            run_time=1.5
        )

        msg1 = Text("Particles roll into the bowl!", font_size=20, color=YELLOW)
        msg1.to_edge(DOWN)
        msg1.fix_in_frame()
        self.play(FadeIn(msg1))
        self.wait(1)

        # ===== FLIP the surface =====
        self.play(FadeOut(msg1), FadeOut(label))

        flip_msg = Text("Now flip the surface...", font_size=20, color=WHITE)
        flip_msg.to_edge(DOWN)
        flip_msg.fix_in_frame()
        self.play(FadeIn(flip_msg))

        # Normal surface (peak shape)
        def normal_func(u, v):
            z = 1.5 * np.exp(-((u - post_mu_x)**2 / (2 * post_sigma**2) +
                               (v - post_mu_y)**2 / (2 * post_sigma**2)))
            return axes.c2p(u, v, z)

        normal_surface = ParametricSurface(
            normal_func,
            u_range=[-3, 3],
            v_range=[-3, 3],
            resolution=(25, 25)
        )
        normal_surface.set_color(GREEN)
        normal_surface.set_opacity(0.7)
        normal_mesh = SurfaceMesh(normal_surface, resolution=(25, 25))
        normal_mesh.set_stroke(WHITE, width=0.5, opacity=0.5)

        # Calculate new particle positions on flipped surface
        peak_positions = []
        for i in range(n_particles):
            end_x, end_y, _ = bowl_positions[i]
            end_z = 1.5 * np.exp(-((end_x - post_mu_x)**2 / (2 * post_sigma**2) +
                                   (end_y - post_mu_y)**2 / (2 * post_sigma**2)))
            peak_pos = axes.c2p(end_x, end_y, end_z + 0.08)
            peak_positions.append(peak_pos)

        # Animate flip: surface transforms + particles move up
        particle_animations = [
            particles[i].animate.move_to(peak_positions[i])
            for i in range(n_particles)
        ]

        self.play(
            Transform(inverted_surface, normal_surface),
            Transform(inverted_mesh, normal_mesh),
            *particle_animations,
            run_time=2
        )

        self.play(FadeOut(flip_msg))

        # New label
        label2 = Text("High = High Probability", font_size=18, color=GREEN)
        label2.to_corner(UR)
        label2.fix_in_frame()
        self.play(FadeIn(label2))

        # Final message
        msg2 = Text("Particles now on the PEAK!", font_size=22, color=YELLOW)
        msg2.to_edge(DOWN)
        msg2.fix_in_frame()
        self.play(FadeIn(msg2))

        # Rotate to see peak
        self.play(
            frame.animate.set_euler_angles(theta=-120 * DEGREES, phi=55 * DEGREES),
            run_time=2
        )
        self.wait(1)

        # Top view
        self.play(
            frame.animate.set_euler_angles(theta=0 * DEGREES, phi=10 * DEGREES),
            run_time=2
        )
        self.wait(2)


# Run: manimgl pf_resampling.py PFResampling -w -r 1920x1080
