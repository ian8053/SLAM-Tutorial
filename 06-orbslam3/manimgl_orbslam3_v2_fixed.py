"""
ORB-SLAM3 Tutorial Animation - Fixed Version
All text in English, minimal words
"""

from manimlib import *
import numpy as np

# Font setting
FONT = "Consolas"

# Light blue color for all text
LIGHT_BLUE = "#87CEEB"

# KF Colors - each KF's Map Points use different color
KF_COLORS = [
    "#3498db",  # KF0: blue
    "#e74c3c",  # KF1: red
    "#2ecc71",  # KF2: green
    "#f39c12",  # KF3: orange
    "#9b59b6",  # KF4: purple
]


class ORBSLAM3V2Fixed(Scene):
    """ORB-SLAM3 Three-Thread Tutorial"""

    def construct(self):
        # 3D view setup
        frame = self.camera.frame
        frame.set_euler_angles(phi=55 * DEGREES, theta=-35 * DEGREES)
        frame.set_height(14)

        # Progress label
        self.step_label = Text("", font_size=24, color=LIGHT_BLUE, font=FONT)
        self.step_label.fix_in_frame()
        self.step_label.to_corner(UL)
        self.add(self.step_label)

        # Progress bar
        self.progress_bar = self.create_progress_bar()
        self.add(self.progress_bar)

        # Storage
        self.keyframes = []  # [(pos, marker, label)]
        self.map_points = []
        self.map_positions = []
        self.map_point_kf_idx = []
        self.kf_edges = []
        self.trajectory_lines = []
        self.cov_panel_group = None
        self.current_kf_idx = 0

        # ==================== PART 1: TRACKING ====================
        self.show_step(1, "Tracking: Init")
        self.setup_scene()

        self.show_step(2, "Tracking: Cam A + ORB")
        self.camera_a_capture()

        self.show_step(3, "Tracking: Move to B")
        self.camera_move_to_b()

        self.show_step(4, "Tracking: Feature Match")
        self.feature_matching()

        self.show_step(5, "Tracking: Triangulation")
        self.triangulation()

        self.show_step(6, "Tracking: Map Points")
        self.create_map_points()

        self.show_step(7, "Tracking: Keyframes")
        self.create_keyframes()

        # ==================== PART 2: LOCAL MAPPING ====================
        self.show_step(8, "Local Mapping: Covisibility")
        self.covisibility_graph()

        self.show_step(9, "Local Mapping: BA")
        self.bundle_adjustment()

        # ==================== PART 3: Continue TRACKING ====================
        self.show_step(10, "Tracking: PnP")
        self.pnp_tracking()

        self.show_step(11, "Tracking: Loop Path")
        self.loop_trajectory()

        # ==================== PART 4: LOOP CLOSING ====================
        self.show_step(12, "Loop Closing: BoW Detect")
        self.loop_detection()

        self.show_step(13, "Loop Closing: Sim(3) Merge")
        self.map_fusion()

        self.show_step(14, "Loop Closing: Global BA")
        self.global_ba()

        self.show_step(15, "Done!")
        self.final_view()

    def create_progress_bar(self):
        bar_bg = Rectangle(width=6, height=0.2, fill_color=GREY, fill_opacity=0.3)
        bar_bg.fix_in_frame()
        bar_bg.to_edge(DOWN, buff=0.3)
        self.bar_fill = Rectangle(width=0, height=0.2, fill_color=GREEN, fill_opacity=0.8)
        self.bar_fill.fix_in_frame()
        self.bar_fill.align_to(bar_bg, LEFT)
        self.bar_fill.to_edge(DOWN, buff=0.3)
        return VGroup(bar_bg, self.bar_fill)

    def show_step(self, step_num, description):
        total = 15
        new_label = Text(f"[{step_num}/{total}] {description}", font_size=20, color=LIGHT_BLUE, font=FONT)
        new_label.fix_in_frame()
        new_label.to_corner(UL)

        progress = step_num / total
        new_bar = Rectangle(width=6 * progress, height=0.2, fill_color=GREEN, fill_opacity=0.8)
        new_bar.fix_in_frame()
        new_bar.align_to(self.bar_fill, LEFT)
        new_bar.to_edge(DOWN, buff=0.3)

        self.play(
            Transform(self.step_label, new_label),
            Transform(self.bar_fill, new_bar),
            run_time=0.4
        )
        self.wait(1.2)

    def update_covisibility_panel(self, kf_names, has_loop=False):
        """Update covisibility panel - merge KF0 and KF_last when loop exists"""
        if hasattr(self, 'cov_panel_group') and self.cov_panel_group is not None:
            self.play(FadeOut(self.cov_panel_group), run_time=0.15)

        # Panel at bottom right
        panel = Rectangle(width=3.2, height=2.2, fill_color=BLACK, fill_opacity=1.0, stroke_width=0)
        panel.fix_in_frame()
        panel.to_corner(DR, buff=0.5)

        title = Text("Covisibility", font_size=11, color=LIGHT_BLUE, font=FONT)
        title.fix_in_frame()
        title.next_to(panel, UP, buff=0.05)

        n = len(kf_names)
        panel_center = panel.get_center()
        nodes = []
        labels = []
        edges = []

        if has_loop and n >= 3:
            # Loop closed: KF0 and KF_last merge into one node
            # Show n-1 nodes in arc, with merged node (KF0+KF_last) at top
            n_display = n - 1  # KF_last merges with KF0
            radius = 0.5

            for i in range(n_display):
                angle = PI/2 - (2 * PI * i / n_display)  # Start from top, go clockwise
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)

                # Merged node at position 0
                if i == 0:
                    # Merged KF0 + KF_last - larger node with green color
                    node = Circle(radius=0.16, fill_color=GREEN, fill_opacity=1.0, stroke_color=GREEN, stroke_width=2)
                    node.fix_in_frame()
                    node.move_to(panel_center + RIGHT * x + UP * y)
                    nodes.append(node)

                    # Label shows both KF names
                    last_kf = kf_names[-1]
                    merged_label = Text(f"KF0={last_kf[-1]}", font_size=6, color=WHITE, font=FONT)
                    merged_label.fix_in_frame()
                    merged_label.move_to(node.get_center())
                    labels.append(merged_label)
                else:
                    node = Circle(radius=0.12, fill_color=LIGHT_BLUE, fill_opacity=1.0, stroke_color=LIGHT_BLUE)
                    node.fix_in_frame()
                    node.move_to(panel_center + RIGHT * x + UP * y)
                    nodes.append(node)

                    label = Text(kf_names[i], font_size=6, color=LIGHT_BLUE, font=FONT)
                    label.fix_in_frame()
                    label.move_to(node.get_center())
                    labels.append(label)

            # Draw edges in circle (closed loop)
            for i in range(n_display):
                next_i = (i + 1) % n_display
                edge = Line(nodes[i].get_center(), nodes[next_i].get_center(), color=PURPLE, stroke_width=2)
                edge.fix_in_frame()
                edges.append(edge)
        else:
            # Linear layout (no loop)
            spacing = min(0.5, 2.5 / max(n, 1))
            start_x = -(n - 1) * spacing / 2

            for i, name in enumerate(kf_names):
                x = start_x + i * spacing

                node = Circle(radius=0.12, fill_color=LIGHT_BLUE, fill_opacity=1.0, stroke_color=LIGHT_BLUE)
                node.fix_in_frame()
                node.move_to(panel_center + RIGHT * x)
                nodes.append(node)

                label = Text(name, font_size=6, color=LIGHT_BLUE, font=FONT)
                label.fix_in_frame()
                label.move_to(node.get_center())
                labels.append(label)

            # Draw sequential edges only
            for i in range(len(nodes) - 1):
                edge = Line(nodes[i].get_right(), nodes[i+1].get_left(), color=PURPLE, stroke_width=2)
                edge.fix_in_frame()
                edges.append(edge)

        self.cov_panel_group = VGroup(panel, title, *nodes, *labels, *edges)
        self.play(FadeIn(self.cov_panel_group), run_time=0.25)

    # ==================== TRACKING ====================

    def setup_scene(self):
        """Scene init"""
        ground = NumberPlane(x_range=[-10, 10, 1], y_range=[-4, 16, 1])
        ground.rotate(PI/2, axis=RIGHT)
        ground.set_opacity(0.1)
        self.play(ShowCreation(ground), run_time=0.4)
        self.ground = ground

        self.scene_objects = []
        positions = [
            np.array([0, 6, 2]),
            np.array([-2, 8, 1.5]),
            np.array([2, 7, 2.5]),
            np.array([1, 10, 1]),
            np.array([-1, 5, 3]),
        ]
        for pos in positions:
            obj = Sphere(radius=0.15, color=RED)
            obj.move_to(pos)
            self.scene_objects.append((pos, obj))
            self.play(GrowFromCenter(obj), run_time=0.1)

    def camera_a_capture(self):
        """Camera A capture"""
        self.cam_a_pos = np.array([-4, 0, 1.5])
        self.camera_a = Cube(side_length=0.5, color=BLUE)
        self.camera_a.move_to(self.cam_a_pos)

        cam_label = Text("A", font_size=12, color=LIGHT_BLUE, font=FONT)
        cam_label.next_to(self.camera_a, DOWN, buff=0.1)

        self.play(FadeIn(self.camera_a), Write(cam_label), run_time=0.3)

        self.play(self.camera_a.animate.set_color(WHITE), run_time=0.08)
        self.play(self.camera_a.animate.set_color(BLUE), run_time=0.08)

        self.view_lines_a = []
        for pos, obj in self.scene_objects:
            line = Line3D(self.cam_a_pos, pos, color=BLUE)
            self.view_lines_a.append(line)
        self.play(*[ShowCreation(l) for l in self.view_lines_a], run_time=0.4)

        self.img_frame_a = Rectangle(width=2.0, height=1.4, color=BLUE, stroke_width=2)
        self.img_frame_a.fix_in_frame()
        self.img_frame_a.to_corner(UR, buff=0.3)
        self.img_frame_a.shift(LEFT * 2.3 + DOWN * 0.3)

        self.img_label_a = Text("A", font_size=12, color=LIGHT_BLUE, font=FONT)
        self.img_label_a.fix_in_frame()
        self.img_label_a.next_to(self.img_frame_a, UP, buff=0.05)

        self.play(ShowCreation(self.img_frame_a), Write(self.img_label_a), run_time=0.3)

        self.orb_dots_a = []
        for i, (pos, obj) in enumerate(self.scene_objects):
            dot = Dot(color=GREEN, radius=0.05)
            dot.fix_in_frame()
            offset = np.array([0.3 * (i - 2), 0.2 * (i % 3 - 1), 0])
            dot.move_to(self.img_frame_a.get_center() + offset)
            self.orb_dots_a.append(dot)
        self.play(*[FadeIn(d) for d in self.orb_dots_a], run_time=0.2)

    def camera_move_to_b(self):
        """Camera move to B"""
        self.cam_b_pos = np.array([4, 0, 1.5])

        path = Line3D(self.cam_a_pos, self.cam_b_pos, color=TEAL)
        self.play(ShowCreation(path), run_time=0.3)
        self.trajectory_lines.append(path)

        self.camera_b = Cube(side_length=0.5, color=ORANGE)
        self.camera_b.move_to(self.cam_a_pos)
        self.play(FadeIn(self.camera_b), run_time=0.2)
        self.play(self.camera_b.animate.move_to(self.cam_b_pos), run_time=0.5)

        cam_label = Text("B", font_size=12, color=LIGHT_BLUE, font=FONT)
        cam_label.next_to(self.camera_b, DOWN, buff=0.1)
        self.play(Write(cam_label), run_time=0.2)

        self.play(self.camera_b.animate.set_color(WHITE), run_time=0.08)
        self.play(self.camera_b.animate.set_color(ORANGE), run_time=0.08)

        self.view_lines_b = []
        for pos, obj in self.scene_objects:
            line = Line3D(self.cam_b_pos, pos, color=ORANGE)
            self.view_lines_b.append(line)
        self.play(*[ShowCreation(l) for l in self.view_lines_b], run_time=0.4)

    def feature_matching(self):
        """Feature matching"""
        self.img_frame_b = Rectangle(width=2.0, height=1.4, color=ORANGE, stroke_width=2)
        self.img_frame_b.fix_in_frame()
        self.img_frame_b.next_to(self.img_frame_a, RIGHT, buff=0.15)

        self.img_label_b = Text("B", font_size=12, color=LIGHT_BLUE, font=FONT)
        self.img_label_b.fix_in_frame()
        self.img_label_b.next_to(self.img_frame_b, UP, buff=0.05)

        self.play(ShowCreation(self.img_frame_b), Write(self.img_label_b), run_time=0.3)

        self.orb_dots_b = []
        for i, (pos, obj) in enumerate(self.scene_objects):
            dot = Dot(color=GREEN, radius=0.05)
            dot.fix_in_frame()
            offset = np.array([0.3 * (i - 2) - 0.15, 0.2 * (i % 3 - 1), 0])
            dot.move_to(self.img_frame_b.get_center() + offset)
            self.orb_dots_b.append(dot)
        self.play(*[FadeIn(d) for d in self.orb_dots_b], run_time=0.2)

        self.match_lines = []
        for da, db in zip(self.orb_dots_a, self.orb_dots_b):
            line = Line(da.get_center(), db.get_center(), color=LIGHT_BLUE, stroke_width=1)
            line.fix_in_frame()
            self.match_lines.append(line)
        self.play(*[ShowCreation(l) for l in self.match_lines], run_time=0.3)

        parallax_text = Text("Parallax", font_size=14, color=LIGHT_BLUE, font=FONT)
        parallax_text.fix_in_frame()
        parallax_text.next_to(self.img_frame_b, LEFT, buff=0.2)
        self.play(Write(parallax_text), run_time=0.2)
        self.parallax_text = parallax_text

    def triangulation(self):
        """Triangulation"""
        tri_label = Text("Triangulation: rays -> 3D", font_size=12, color=LIGHT_BLUE, font=FONT)
        tri_label.fix_in_frame()
        tri_label.to_corner(DL, buff=0.5)
        self.play(Write(tri_label), run_time=0.3)

        self.play(
            *[l.animate.set_color(YELLOW) for l in self.view_lines_a],
            *[l.animate.set_color(YELLOW) for l in self.view_lines_b],
            run_time=0.3
        )

        intersections = []
        for pos, obj in self.scene_objects:
            flash = Sphere(radius=0.25, color=LIGHT_BLUE)
            flash.move_to(pos)
            intersections.append(flash)
        self.play(*[GrowFromCenter(i) for i in intersections], run_time=0.4)
        self.wait(0.3)
        self.play(*[FadeOut(i) for i in intersections], run_time=0.2)

        for pos, obj in self.scene_objects:
            self.play(obj.animate.set_color(GREEN), run_time=0.05)

        self.play(FadeOut(tri_label), run_time=0.2)

    def create_map_points(self):
        """Create Map Points"""
        self.play(
            *[FadeOut(l) for l in self.view_lines_a],
            *[FadeOut(l) for l in self.view_lines_b],
            *[FadeOut(l) for l in self.match_lines],
            FadeOut(self.parallax_text),
            run_time=0.3
        )

        kf0_color = KF_COLORS[0]

        for pos, obj in self.scene_objects:
            self.play(obj.animate.set_color(kf0_color), run_time=0.05)
            self.map_points.append(obj)
            self.map_positions.append(pos)
            self.map_point_kf_idx.append(0)

        np.random.seed(42)
        for i in range(10):
            pos = np.array([
                np.random.uniform(-5, 5),
                np.random.uniform(4, 12),
                np.random.uniform(0.5, 3.5)
            ])
            pt = Sphere(radius=0.1, color=kf0_color)
            pt.move_to(pos)
            self.map_points.append(pt)
            self.map_positions.append(pos)
            self.map_point_kf_idx.append(0)
            self.play(GrowFromCenter(pt), run_time=0.03)

        mp_label = Text("Map Points", font_size=14, color=LIGHT_BLUE, font=FONT)
        mp_label.fix_in_frame()
        mp_label.to_edge(LEFT, buff=0.3)
        mp_label.shift(DOWN * 1)
        self.play(Write(mp_label), run_time=0.2)

        self.current_kf_idx = 1

    def create_keyframes(self):
        """Create Keyframes"""
        kf0 = Square(side_length=0.35, color=LIGHT_BLUE, fill_opacity=0.8)
        kf0.move_to(self.cam_a_pos + UP * 0.6)
        kf0.rotate(PI/2, axis=RIGHT)
        kf0_label = Text("KF0", font_size=10, color=LIGHT_BLUE, font=FONT)
        kf0_label.next_to(kf0, UP, buff=0.1)

        kf1 = Square(side_length=0.35, color=LIGHT_BLUE, fill_opacity=0.8)
        kf1.move_to(self.cam_b_pos + UP * 0.6)
        kf1.rotate(PI/2, axis=RIGHT)
        kf1_label = Text("KF1", font_size=10, color=LIGHT_BLUE, font=FONT)
        kf1_label.next_to(kf1, UP, buff=0.1)

        self.play(
            GrowFromCenter(kf0), Write(kf0_label),
            GrowFromCenter(kf1), Write(kf1_label),
            run_time=0.3
        )

        self.keyframes.append((self.cam_a_pos.copy(), kf0, kf0_label))
        self.keyframes.append((self.cam_b_pos.copy(), kf1, kf1_label))

    # ==================== LOCAL MAPPING ====================

    def covisibility_graph(self):
        """Covisibility Graph"""
        self.play(
            FadeOut(self.img_frame_a), FadeOut(self.img_frame_b),
            FadeOut(self.img_label_a), FadeOut(self.img_label_b),
            *[FadeOut(d) for d in self.orb_dots_a],
            *[FadeOut(d) for d in self.orb_dots_b],
            run_time=0.3
        )

        self.update_covisibility_panel(["KF0", "KF1"], has_loop=False)

        # 3D edge between KF0 and KF1 only
        kf0_pos, kf0, _ = self.keyframes[0]
        kf1_pos, kf1, _ = self.keyframes[1]

        edge = Line3D(kf0_pos + UP * 0.6, kf1_pos + UP * 0.6, color=PURPLE)
        self.play(ShowCreation(edge), run_time=0.4)
        self.kf_edges.append(edge)

        # Show shared MPs
        kf_mp_lines = []
        for mp_pos in self.map_positions[:5]:
            line0 = DashedLine(kf0_pos + UP * 0.6, mp_pos, color=PURPLE, stroke_width=0.5)
            line1 = DashedLine(kf1_pos + UP * 0.6, mp_pos, color=PURPLE, stroke_width=0.5)
            kf_mp_lines.extend([line0, line1])
        self.play(*[ShowCreation(l) for l in kf_mp_lines], run_time=0.4)

        self.wait(0.5)
        self.play(*[FadeOut(l) for l in kf_mp_lines], run_time=0.2)

    def bundle_adjustment(self):
        """Bundle Adjustment"""
        ba_panel = Rectangle(width=2.8, height=1.2, fill_color=BLACK, fill_opacity=1.0, stroke_width=0)
        ba_panel.fix_in_frame()
        ba_panel.to_corner(UR, buff=0.2)

        ba_title = Text("Bundle Adjustment", font_size=14, color=LIGHT_BLUE, font=FONT)
        ba_title.fix_in_frame()
        ba_title.next_to(ba_panel, UP, buff=0.05)

        ba_content = Text("Optimize KF + MP", font_size=10, color=LIGHT_BLUE, font=FONT)
        ba_content.fix_in_frame()
        ba_content.move_to(ba_panel.get_center())

        self.play(FadeIn(ba_panel), Write(ba_title), Write(ba_content), run_time=0.4)

        before_text = Text("Optimizing...", font_size=12, color=LIGHT_BLUE, font=FONT)
        before_text.fix_in_frame()
        before_text.next_to(ba_panel, DOWN, buff=0.1)
        self.play(Write(before_text), run_time=0.2)

        for iteration in range(5):
            for pos, kf, label in self.keyframes:
                offset = np.array([np.random.uniform(-0.03, 0.03), 0, np.random.uniform(-0.03, 0.03)])
                self.play(kf.animate.shift(offset), run_time=0.03)

            for mp in self.map_points[:8]:
                offset = np.array([np.random.uniform(-0.03, 0.03)] * 3)
                self.play(mp.animate.shift(offset), run_time=0.02)

        done = Text("BA Done!", font_size=12, color=LIGHT_BLUE, font=FONT)
        done.fix_in_frame()
        done.move_to(before_text.get_center())
        self.play(Transform(before_text, done), run_time=0.2)

        self.wait(0.3)
        self.play(FadeOut(ba_panel), FadeOut(ba_title), FadeOut(ba_content), FadeOut(before_text), run_time=0.2)

    # ==================== Continue TRACKING ====================

    def pnp_tracking(self):
        """PnP Tracking - NO edge to KF0!"""
        pnp_box = Rectangle(width=2.8, height=1.2, fill_color=BLACK, fill_opacity=1.0, stroke_width=0)
        pnp_box.fix_in_frame()
        pnp_box.to_corner(UR, buff=0.2)

        pnp_text = VGroup(
            Text("PnP Tracking", font_size=12, color=LIGHT_BLUE, font=FONT),
            Text("3D MP -> Pose", font_size=9, color=LIGHT_BLUE, font=FONT),
        )
        pnp_text.arrange(DOWN, buff=0.08)
        pnp_text.fix_in_frame()
        pnp_text.move_to(pnp_box.get_center())

        self.play(FadeIn(pnp_box), Write(pnp_text), run_time=0.3)

        camera_c = Cube(side_length=0.5, color=PURPLE)
        camera_c.move_to(self.cam_b_pos)
        self.play(FadeIn(camera_c), run_time=0.2)

        positions = [
            self.cam_b_pos,
            np.array([2, 3, 1.5]),
            np.array([0, 5, 1.5]),
        ]

        for i in range(1, len(positions)):
            path = Line3D(positions[i-1], positions[i], color=TEAL)
            self.trajectory_lines.append(path)
            self.add(path)
            self.play(camera_c.animate.move_to(positions[i]), run_time=0.4)

            rays = []
            for mp_pos in self.map_positions[:4]:
                ray = Line3D(mp_pos, positions[i], color=PURPLE)
                rays.append(ray)
            self.play(*[ShowCreation(r) for r in rays], run_time=0.8)
            self.wait(0.3)
            self.play(*[FadeOut(r) for r in rays], run_time=0.3)

        # KF2
        kf2 = Square(side_length=0.35, color=LIGHT_BLUE, fill_opacity=0.8)
        kf2.move_to(positions[-1] + UP * 0.6)
        kf2.rotate(PI/2, axis=RIGHT)
        kf2_label = Text("KF2", font_size=10, color=LIGHT_BLUE, font=FONT)
        kf2_label.next_to(kf2, UP, buff=0.1)
        self.play(GrowFromCenter(kf2), Write(kf2_label), run_time=0.2)
        self.keyframes.append((positions[-1].copy(), kf2, kf2_label))

        # Only connect KF2 to KF1 (previous KF), NOT to KF0!
        prev_kf = self.keyframes[-2][1]  # KF1
        curr_kf = self.keyframes[-1][1]  # KF2
        edge = Line3D(prev_kf.get_center(), curr_kf.get_center(), color=PURPLE)
        self.kf_edges.append(edge)
        self.play(ShowCreation(edge), run_time=0.15)

        self.update_covisibility_panel(["KF0", "KF1", "KF2"], has_loop=False)

        self.camera_c = camera_c
        self.cam_c_pos = positions[-1]
        self.pnp_box = pnp_box

    def loop_trajectory(self):
        """Loop trajectory"""
        waypoints = [
            (self.cam_c_pos, None),
            (np.array([-2, 6, 1.5]), "KF3"),
            (np.array([-3.5, 0.5, 1.5]), "KF4"),  # Near start!
        ]

        prev_pos = waypoints[0][0]
        for i in range(1, len(waypoints)):
            pos, kf_name = waypoints[i]

            path = Line3D(prev_pos, pos, color=TEAL)
            self.trajectory_lines.append(path)
            self.add(path)
            self.play(self.camera_c.animate.move_to(pos), run_time=0.4)

            kf_idx = len(self.keyframes)
            kf_color = KF_COLORS[kf_idx % len(KF_COLORS)]

            for j in range(3):
                mp_pos = pos + np.array([
                    np.random.uniform(-1.5, 1.5),
                    np.random.uniform(2, 4),
                    np.random.uniform(0, 2)
                ])
                pt = Sphere(radius=0.08, color=kf_color)
                pt.move_to(mp_pos)
                self.map_points.append(pt)
                self.map_positions.append(mp_pos)
                self.map_point_kf_idx.append(kf_idx)
                self.add(pt)

            if kf_name:
                kf = Square(side_length=0.35, color=LIGHT_BLUE, fill_opacity=0.8)
                kf.move_to(pos + UP * 0.6)
                kf.rotate(PI/2, axis=RIGHT)
                kf_label = Text(kf_name, font_size=10, color=LIGHT_BLUE, font=FONT)
                kf_label.next_to(kf, UP, buff=0.1)
                self.play(GrowFromCenter(kf), Write(kf_label), run_time=0.2)
                self.keyframes.append((pos.copy(), kf, kf_label))

                # Connect to previous KF only (sequential)
                if len(self.keyframes) >= 2:
                    prev_kf = self.keyframes[-2][1]
                    curr_kf = self.keyframes[-1][1]
                    edge = Line3D(prev_kf.get_center(), curr_kf.get_center(), color=PURPLE)
                    self.kf_edges.append(edge)
                    self.play(ShowCreation(edge), run_time=0.2)

                kf_list = [f"KF{k}" for k in range(len(self.keyframes))]
                self.update_covisibility_panel(kf_list, has_loop=False)

            prev_pos = pos

        self.cam_last_pos = waypoints[-1][0]

    # ==================== LOOP CLOSING ====================

    def loop_detection(self):
        """Loop detection"""
        self.play(FadeOut(self.pnp_box), run_time=0.1)

        loop_panel = Rectangle(width=3.5, height=2.0, fill_color=BLACK, fill_opacity=1.0, stroke_width=0)
        loop_panel.fix_in_frame()
        loop_panel.to_corner(UR, buff=0.2)

        loop_title = Text("Loop Closing", font_size=14, color=LIGHT_BLUE, font=FONT)
        loop_title.fix_in_frame()
        loop_title.next_to(loop_panel, UP, buff=0.08)

        loop_content = VGroup(
            Text("BoW Detection", font_size=11, color=LIGHT_BLUE, font=FONT),
            Text("Compare current vs history", font_size=10, color=GREY, font=FONT),
        )
        loop_content.arrange(DOWN, buff=0.15)
        loop_content.fix_in_frame()
        loop_content.move_to(loop_panel.get_center())

        self.play(FadeIn(loop_panel), Write(loop_title), Write(loop_content), run_time=0.3)

        kf0_pos = self.keyframes[0][0]
        kf_last_pos = self.keyframes[-1][0]

        loop_line = DashedLine(kf0_pos + UP * 0.6, kf_last_pos + UP * 0.6, color=RED, stroke_width=3)
        self.play(ShowCreation(loop_line), run_time=0.5)

        found = Text("Loop Found: KF4 ~ KF0", font_size=12, color=LIGHT_BLUE, font=FONT)
        found.fix_in_frame()
        found.move_to(loop_panel.get_center() + DOWN * 0.4)
        self.play(Write(found), run_time=0.3)

        self.loop_line = loop_line
        self.loop_panel = loop_panel
        self.loop_title = loop_title

    def map_fusion(self):
        """Map fusion with Sim(3)"""

        # Step 1: Sim(3)
        sim3_panel = Rectangle(width=3.5, height=1.8, fill_color=BLACK, fill_opacity=1.0, stroke_width=0)
        sim3_panel.fix_in_frame()
        sim3_panel.to_corner(UR, buff=0.15)

        sim3_title = Text("Sim(3) Correction", font_size=13, color=LIGHT_BLUE, font=FONT)
        sim3_title.fix_in_frame()
        sim3_title.next_to(sim3_panel, UP, buff=0.05)

        sim3_content = VGroup(
            Text("P' = s*R*P + t", font_size=12, color=LIGHT_BLUE, font=FONT),
            Text("Fix drift error", font_size=10, color=LIGHT_BLUE, font=FONT),
        )
        sim3_content.arrange(DOWN, buff=0.1)
        sim3_content.fix_in_frame()
        sim3_content.move_to(sim3_panel.get_center())

        self.play(FadeIn(sim3_panel), Write(sim3_title), Write(sim3_content), run_time=0.5)

        kf0_pos = self.keyframes[0][0]
        kf_last_pos = self.keyframes[-1][0]

        match_line = DashedLine(kf0_pos + UP * 0.6, kf_last_pos + UP * 0.6, color=ORANGE, stroke_width=3)
        self.play(ShowCreation(match_line), run_time=0.3)

        error_dist = np.linalg.norm(kf_last_pos - kf0_pos)
        error_label = Text(f"Drift: {error_dist:.1f}m", font_size=10, color=LIGHT_BLUE, font=FONT)
        error_label.move_to((kf0_pos + kf_last_pos) / 2 + UP * 1.5 + RIGHT * 0.5)
        self.play(Write(error_label), run_time=0.2)

        self.wait(0.4)

        # Step 2: MP fusion
        step2_content = VGroup(
            Text("MP Fusion", font_size=13, color=LIGHT_BLUE, font=FONT),
            Text("Merge duplicates", font_size=10, color=LIGHT_BLUE, font=FONT),
        )
        step2_content.arrange(DOWN, buff=0.1)
        step2_content.fix_in_frame()
        step2_content.move_to(sim3_panel.get_center())

        self.play(
            FadeOut(sim3_content),
            FadeIn(step2_content),
            run_time=0.3
        )

        self.wait(0.5)

        # Step 3: KF pose correction
        step3_content = VGroup(
            Text("KF Correction", font_size=13, color=LIGHT_BLUE, font=FONT),
            Text("Propagate Sim(3)", font_size=10, color=LIGHT_BLUE, font=FONT),
        )
        step3_content.arrange(DOWN, buff=0.1)
        step3_content.fix_in_frame()
        step3_content.move_to(sim3_panel.get_center())

        self.play(
            FadeOut(step2_content),
            FadeIn(step3_content),
            run_time=0.3
        )

        correction = (kf0_pos - kf_last_pos) * 0.8

        animations = []
        mp_animations = []

        for i, (pos, kf, label) in enumerate(self.keyframes[1:], 1):
            ratio = i / (len(self.keyframes) - 1)
            offset = correction * ratio
            animations.append(kf.animate.shift(offset))
            animations.append(label.animate.shift(offset))

        if hasattr(self, 'camera_c'):
            animations.append(self.camera_c.animate.shift(correction))

        num_mps = len(self.map_points)
        for j, mp in enumerate(self.map_points):
            ratio = (j / max(num_mps - 1, 1)) * 0.8
            offset = correction * ratio
            mp_animations.append(mp.animate.shift(offset))

        all_anims = animations + mp_animations
        self.play(*all_anims, run_time=1.2)

        # Redraw edges
        self.play(*[FadeOut(edge) for edge in self.kf_edges], run_time=0.1)

        new_edges = []
        for i in range(len(self.keyframes) - 1):
            kf1 = self.keyframes[i][1]
            kf2 = self.keyframes[i + 1][1]
            pos1 = kf1.get_center()
            pos2 = kf2.get_center()
            edge = Line3D(pos1, pos2, color=PURPLE)
            new_edges.append(edge)

        # Add loop edge
        kf0 = self.keyframes[0][1]
        kf_last = self.keyframes[-1][1]
        loop_edge = Line3D(kf0.get_center(), kf_last.get_center(), color=GREEN)
        new_edges.append(loop_edge)

        self.play(*[ShowCreation(e) for e in new_edges], run_time=0.3)
        self.kf_edges = new_edges

        self.play(
            self.loop_line.animate.set_color(GREEN),
            FadeOut(match_line),
            FadeOut(error_label),
            run_time=0.3
        )

        # Update covisibility panel with loop (circular layout)
        n_kfs = len(self.keyframes)
        kf_list = [f"KF{i}" for i in range(n_kfs)]
        self.update_covisibility_panel(kf_list, has_loop=True)

        # Done
        done_content = VGroup(
            Text("Fusion Done!", font_size=14, color=GREEN, font=FONT),
        )
        done_content.fix_in_frame()
        done_content.move_to(sim3_panel.get_center())

        self.play(
            FadeOut(step3_content),
            FadeIn(done_content),
            run_time=0.3
        )

        self.wait(0.5)
        self.sim3_panel = sim3_panel
        self.sim3_content = done_content
        self.sim3_title = sim3_title

    def global_ba(self):
        """Global BA"""
        self.play(
            FadeOut(self.sim3_panel), FadeOut(self.sim3_content), FadeOut(self.sim3_title),
            run_time=0.2
        )

        gba_panel = Rectangle(width=3.5, height=1.5, fill_color=BLACK, fill_opacity=1.0, stroke_width=0)
        gba_panel.fix_in_frame()
        gba_panel.to_corner(UR, buff=0.2)

        gba_content = VGroup(
            Text("Global BA", font_size=14, color=LIGHT_BLUE, font=FONT),
            Text("Optimize all", font_size=10, color=LIGHT_BLUE, font=FONT),
        )
        gba_content.arrange(DOWN, buff=0.08)
        gba_content.fix_in_frame()
        gba_content.move_to(gba_panel.get_center())

        self.play(FadeIn(gba_panel), Write(gba_content), run_time=0.3)

        gba_status = Text("Optimizing...", font_size=12, color=LIGHT_BLUE, font=FONT)
        gba_status.fix_in_frame()
        gba_status.to_edge(LEFT, buff=0.3)
        gba_status.shift(UP * 1.5)
        self.play(Write(gba_status), run_time=0.2)

        for iteration in range(4):
            kf_anims = []
            for pos, kf, label in self.keyframes:
                offset = np.array([np.random.uniform(-0.02, 0.02), 0, np.random.uniform(-0.02, 0.02)])
                kf_anims.append(kf.animate.shift(offset))
            self.play(*kf_anims, run_time=0.1)

            mp_anims = []
            for mp in self.map_points[:15]:
                offset = np.array([np.random.uniform(-0.025, 0.025)] * 3)
                mp_anims.append(mp.animate.shift(offset))
            self.play(*mp_anims, run_time=0.08)

        done = Text("Global BA Done!", font_size=12, color=LIGHT_BLUE, font=FONT)
        done.fix_in_frame()
        done.move_to(gba_status.get_center())
        self.play(Transform(gba_status, done), run_time=0.2)

        self.play(
            FadeOut(self.loop_panel), FadeOut(self.loop_title),
            FadeOut(gba_panel), FadeOut(gba_content),
            FadeOut(gba_status),
            run_time=0.3
        )

    def final_view(self):
        """Final view"""
        done_text = Text("ORB-SLAM3 Complete!", font_size=22, color=LIGHT_BLUE, font=FONT)
        done_text.fix_in_frame()
        done_text.to_edge(UP, buff=1.5)
        self.play(Write(done_text), run_time=0.3)

        stats = VGroup(
            Text(f"Keyframes: {len(self.keyframes)}", font_size=12, color=LIGHT_BLUE, font=FONT),
            Text(f"Map Points: {len(self.map_points)}", font_size=12, color=GREEN, font=FONT),
        )
        stats.arrange(DOWN, buff=0.1)
        stats.fix_in_frame()
        stats.to_edge(LEFT, buff=0.3)
        stats.shift(DOWN * 0.5)
        self.play(Write(stats), run_time=0.2)

        frame = self.camera.frame
        self.play(
            frame.animate.set_euler_angles(theta=-180 * DEGREES, phi=45 * DEGREES),
            run_time=2.5
        )

        self.wait(1)
