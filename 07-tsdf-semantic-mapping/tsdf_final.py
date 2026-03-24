from manimlib import *
import numpy as np
from scipy.spatial import Delaunay


class TSDFCompleteFlow(ThreeDScene):
    """
    Complete Flow:
    1. Photo → Mask R-CNN pixel labeling chair
    2. Pixel + Depth → Project to voxel, form chair shape
    3. Multi-angle Bayesian confirmation
    4. Voxel triangulation → Mesh (chair shape)
    5. Scene Graph extraction
    6. LLM queries Scene Graph: "Where is Chair?"
    """
    def construct(self):
        frame = self.camera.frame
        frame.set_euler_angles(theta=0 * DEGREES, phi=0 * DEGREES)

        # ========== Step 1: Photo → Mask R-CNN Segmentation ==========
        title1 = Text("Step 1: Photo → Mask R-CNN Semantic Segmentation", font_size=28, color=BLUE)
        title1.to_edge(UP)
        title1.fix_in_frame()
        self.play(FadeIn(title1, run_time=0.5))

        frame.set_euler_angles(theta=0 * DEGREES, phi=90 * DEGREES)

        photo_frame = Rectangle(width=5, height=3.5, color=WHITE)
        photo_frame.move_to(ORIGIN)
        photo_frame.fix_in_frame()

        photo_bg = Rectangle(width=4.8, height=3.3, color=GREY_D, fill_opacity=0.3)
        photo_bg.move_to(photo_frame)
        photo_bg.fix_in_frame()

        self.play(ShowCreation(photo_frame), FadeIn(photo_bg), run_time=0.5)

        chair_2d_seat = Rectangle(width=1.5, height=0.4, color=GREY_B, fill_opacity=0.8)
        chair_2d_seat.move_to(photo_frame.get_center() + DOWN * 0.3)
        chair_2d_seat.fix_in_frame()

        chair_2d_back = Rectangle(width=1.5, height=1.0, color=GREY_B, fill_opacity=0.8)
        chair_2d_back.move_to(photo_frame.get_center() + UP * 0.5)
        chair_2d_back.fix_in_frame()

        chair_2d_leg1 = Rectangle(width=0.15, height=0.6, color=GREY_B, fill_opacity=0.8)
        chair_2d_leg1.move_to(photo_frame.get_center() + DOWN * 0.8 + LEFT * 0.5)
        chair_2d_leg1.fix_in_frame()

        chair_2d_leg2 = Rectangle(width=0.15, height=0.6, color=GREY_B, fill_opacity=0.8)
        chair_2d_leg2.move_to(photo_frame.get_center() + DOWN * 0.8 + RIGHT * 0.5)
        chair_2d_leg2.fix_in_frame()

        chair_2d = VGroup(chair_2d_seat, chair_2d_back, chair_2d_leg1, chair_2d_leg2)

        photo_label = Text("RGB Image", font_size=14)
        photo_label.next_to(photo_frame, DOWN, buff=0.2)
        photo_label.fix_in_frame()

        self.play(FadeIn(chair_2d), FadeIn(photo_label), run_time=0.5)

        maskrcnn_label = Text("Mask R-CNN", font_size=16, color=YELLOW)
        maskrcnn_label.next_to(photo_frame, RIGHT, buff=0.5)
        maskrcnn_label.fix_in_frame()

        note1 = Text("Label pixels as 'Chair'", font_size=16, color=ORANGE)
        note1.to_corner(DR, buff=0.5)
        note1.fix_in_frame()
        self.play(FadeIn(maskrcnn_label), FadeIn(note1), run_time=0.5)

        self.play(
            chair_2d_seat.animate.set_color(ORANGE),
            chair_2d_back.animate.set_color(ORANGE),
            chair_2d_leg1.animate.set_color(ORANGE),
            chair_2d_leg2.animate.set_color(ORANGE),
            run_time=0.8
        )

        chair_label_2d = Text("Chair", font_size=14, color=ORANGE)
        chair_label_2d.move_to(photo_frame.get_center() + UP * 1.2)
        chair_label_2d.fix_in_frame()
        self.play(FadeIn(chair_label_2d), run_time=0.3)

        self.wait(0.5)

        # ========== Step 2: Pixel + Depth → Project to Voxel ==========
        self.play(FadeOut(title1), FadeOut(note1), FadeOut(maskrcnn_label), run_time=0.5)

        title2 = Text("Step 2: Pixel + Depth → Project to 3D Voxel", font_size=28, color=GREEN)
        title2.to_edge(UP)
        title2.fix_in_frame()
        self.play(FadeIn(title2, run_time=0.5))

        depth_note = Text("Each pixel has depth value (from Depth camera)", font_size=14, color=TEAL)
        depth_note.next_to(photo_frame, DOWN, buff=0.5)
        depth_note.fix_in_frame()
        self.play(FadeOut(photo_label), FadeIn(depth_note), run_time=0.5)

        self.wait(0.3)

        photo_frame.clear_updaters()
        photo_bg.clear_updaters()
        chair_2d_seat.clear_updaters()
        chair_2d_back.clear_updaters()
        chair_2d_leg1.clear_updaters()
        chair_2d_leg2.clear_updaters()
        chair_label_2d.clear_updaters()

        voxel_size = 0.04
        spacing = 0.12
        voxel_positions = []

        # Seat
        for x in range(-4, 5, 2):
            for y in range(-4, 5, 2):
                voxel_positions.append([x * spacing, y * spacing, 0])
        # Back
        for x in range(-4, 5, 2):
            for z in range(1, 10, 2):
                voxel_positions.append([x * spacing, 4 * spacing, z * spacing])
        # Legs
        for lx, ly in [(-4, -4), (-4, 4), (4, -4), (4, 4)]:
            for z in range(-6, 0, 2):
                voxel_positions.append([lx * spacing, ly * spacing, z * spacing])

        photo_final_pos = np.array([-5, 0, 0])
        photo_group = Group(photo_frame, photo_bg, chair_2d, chair_label_2d)

        self.play(
            photo_group.animate.move_to(photo_final_pos),
            FadeOut(depth_note),
            run_time=0.8
        )

        self.play(
            photo_group.animate.rotate(-60 * DEGREES, axis=UP),
            frame.animate.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES),
            run_time=1
        )

        chair_voxels = Group()
        for voxel_pos in voxel_positions:
            v = Cube(side_length=voxel_size)
            v.set_color(ORANGE)
            v.set_opacity(0.9)
            v.move_to(voxel_pos)
            chair_voxels.add(v)

        depth_note2 = Text("Pixel + Depth → Project to voxel", font_size=14, color=ORANGE)
        depth_note2.to_corner(DR, buff=0.5)
        depth_note2.fix_in_frame()
        self.play(FadeIn(depth_note2), run_time=0.3)
        self.play(ShowCreation(chair_voxels), run_time=1)

        self.play(
            FadeOut(photo_group), FadeOut(depth_note2),
            run_time=0.5
        )

        voxel_label = Text("These voxels are labeled as 'Chair'", font_size=14, color=ORANGE)
        voxel_label.shift(DOWN * 2.5)
        voxel_label.fix_in_frame()
        self.play(FadeIn(voxel_label), run_time=0.3)

        self.wait(0.5)

        # ========== Step 3: Bayesian Filtering ==========
        self.play(FadeOut(title2), FadeOut(voxel_label), run_time=0.5)

        title3 = Text("Step 3: Multi-view → Bayesian Semantic Confirmation", font_size=28, color=YELLOW)
        title3.to_edge(UP)
        title3.fix_in_frame()
        self.play(FadeIn(title3, run_time=0.5))

        note3 = Text("Confirm voxels are 'Chair' from different views", font_size=14, color=YELLOW)
        note3.to_corner(DR, buff=0.5)
        note3.fix_in_frame()
        self.play(FadeIn(note3), run_time=0.3)

        # Camera 1
        cam1_pos = np.array([3, -2, 2])
        cam1 = Sphere(radius=0.15)
        cam1.set_color(WHITE)
        cam1.move_to(cam1_pos)

        cam1_label = Text("View 1", font_size=12, color=WHITE)
        cam1_label.move_to(cam1_pos + UP * 0.3)
        cam1_label.fix_in_frame()

        self.play(ShowCreation(cam1), FadeIn(cam1_label), run_time=0.5)

        rays1 = Group()
        for v in list(chair_voxels)[:5]:
            r = Line3D(cam1_pos, v.get_center(), color=YELLOW)
            r.set_opacity(0.5)
            rays1.add(r)
        self.play(ShowCreation(rays1), run_time=0.4)

        obs1_info = Text("View 1: All Chair (conf. 50%)", font_size=12, color=GREEN)
        obs1_info.shift(DOWN * 2.5)
        obs1_info.fix_in_frame()
        self.play(FadeIn(obs1_info), run_time=0.3)
        self.wait(0.3)

        # Camera 2
        self.play(FadeOut(rays1), FadeOut(obs1_info), run_time=0.3)

        cam2_pos = np.array([-3, 2, 1.5])
        cam2 = Sphere(radius=0.15)
        cam2.set_color(WHITE)
        cam2.move_to(cam2_pos)

        cam2_label = Text("View 2", font_size=12, color=WHITE)
        cam2_label.move_to(cam2_pos + UP * 0.3)
        cam2_label.fix_in_frame()

        self.play(
            FadeOut(cam1), FadeOut(cam1_label),
            ShowCreation(cam2), FadeIn(cam2_label),
            frame.animate.set_euler_angles(theta=60 * DEGREES, phi=65 * DEGREES),
            run_time=1
        )

        rays2 = Group()
        for v in list(chair_voxels)[5:10]:
            r = Line3D(cam2_pos, v.get_center(), color=YELLOW)
            r.set_opacity(0.5)
            rays2.add(r)
        self.play(ShowCreation(rays2), run_time=0.4)

        obs2_info = Text("View 2: All Chair (conf. 80%)", font_size=12, color=GREEN)
        obs2_info.shift(DOWN * 2.5)
        obs2_info.fix_in_frame()
        self.play(FadeIn(obs2_info), run_time=0.3)

        for v in chair_voxels:
            v.set_opacity(0.8)

        self.wait(0.3)

        # Camera 3
        self.play(FadeOut(rays2), FadeOut(obs2_info), run_time=0.3)

        cam3_pos = np.array([0, 4, 2])
        cam3 = Sphere(radius=0.15)
        cam3.set_color(WHITE)
        cam3.move_to(cam3_pos)

        cam3_label = Text("View 3", font_size=12, color=WHITE)
        cam3_label.move_to(cam3_pos + UP * 0.3)
        cam3_label.fix_in_frame()

        self.play(
            FadeOut(cam2), FadeOut(cam2_label),
            ShowCreation(cam3), FadeIn(cam3_label),
            frame.animate.set_euler_angles(theta=150 * DEGREES, phi=60 * DEGREES),
            run_time=1
        )

        rays3 = Group()
        for v in list(chair_voxels)[10:15]:
            r = Line3D(cam3_pos, v.get_center(), color=YELLOW)
            r.set_opacity(0.5)
            rays3.add(r)
        self.play(ShowCreation(rays3), run_time=0.4)

        obs3_info = Text("View 3: All Chair (conf. 98%)", font_size=12, color=GREEN)
        obs3_info.shift(DOWN * 2.5)
        obs3_info.fix_in_frame()
        self.play(FadeIn(obs3_info), run_time=0.3)

        for v in chair_voxels:
            v.set_opacity(0.95)

        self.wait(0.3)

        self.play(FadeOut(rays3), FadeOut(obs3_info), FadeOut(note3), run_time=0.3)

        confirm_text = Text("Bayesian Confirmed: These Voxels are 'Chair'!", font_size=16, color=GREEN)
        confirm_text.shift(DOWN * 2.5)
        confirm_text.fix_in_frame()
        self.play(FadeIn(confirm_text), run_time=0.3)

        self.wait(0.5)

        self.play(FadeOut(cam3), FadeOut(cam3_label), FadeOut(confirm_text), run_time=0.3)

        # ========== Step 4: Voxel Triangulation → Mesh ==========
        self.play(FadeOut(title3), run_time=0.3)

        title4 = Text("Step 4: Voxel Triangulation → Mesh", font_size=28, color=PURPLE)
        title4.to_edge(UP)
        title4.fix_in_frame()
        self.play(FadeIn(title4, run_time=0.5))

        self.play(
            frame.animate.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES),
            run_time=0.8
        )

        note4 = Text("Connect chair voxel surface with triangles", font_size=14, color=PURPLE)
        note4.to_corner(DR, buff=0.5)
        note4.fix_in_frame()
        self.play(FadeIn(note4), run_time=0.3)

        corner_points = Group()
        voxel_centers = []
        for v in chair_voxels:
            voxel_centers.append(v.get_center())

        for i, pos in enumerate(voxel_centers):
            p = Sphere(radius=0.02)
            p.set_color(GREEN)
            p.move_to(pos)
            corner_points.add(p)

        self.play(ShowCreation(corner_points), run_time=0.8)

        self.play(FadeOut(note4), run_time=0.3)
        note4b = Text("Connect points with triangles → Mesh surface", font_size=14, color=TEAL)
        note4b.to_corner(DR, buff=0.5)
        note4b.fix_in_frame()
        self.play(FadeIn(note4b), run_time=0.3)

        self.play(FadeOut(chair_voxels), run_time=0.3)

        mesh_triangles = VGroup()
        spacing = 0.12

        # Seat (z=0 plane)
        seat_points = []
        for x in range(-4, 5, 2):
            for y in range(-4, 5, 2):
                seat_points.append([x * spacing, y * spacing, 0])

        if len(seat_points) >= 3:
            pts_2d = np.array([[p[0], p[1]] for p in seat_points])
            try:
                tri = Delaunay(pts_2d)
                for simplex in tri.simplices:
                    p0 = seat_points[simplex[0]]
                    p1 = seat_points[simplex[1]]
                    p2 = seat_points[simplex[2]]
                    triangle = Polygon(np.array(p0), np.array(p1), np.array(p2),
                                       color=ORANGE, fill_opacity=0.7,
                                       stroke_color=WHITE, stroke_width=1)
                    mesh_triangles.add(triangle)
            except:
                pass

        # Back (y=4*spacing plane)
        back_points = []
        for x in range(-4, 5, 2):
            for z in range(1, 10, 2):
                back_points.append([x * spacing, 4 * spacing, z * spacing])

        if len(back_points) >= 3:
            pts_2d = np.array([[p[0], p[2]] for p in back_points])
            try:
                tri = Delaunay(pts_2d)
                for simplex in tri.simplices:
                    p0 = back_points[simplex[0]]
                    p1 = back_points[simplex[1]]
                    p2 = back_points[simplex[2]]
                    triangle = Polygon(np.array(p0), np.array(p1), np.array(p2),
                                       color=ORANGE, fill_opacity=0.7,
                                       stroke_color=WHITE, stroke_width=1)
                    mesh_triangles.add(triangle)
            except:
                pass

        # Legs
        for lx, ly in [(-4, -4), (-4, 4), (4, -4), (4, 4)]:
            leg_points = []
            for z in range(-6, 0, 2):
                leg_points.append([lx * spacing, ly * spacing, z * spacing])

            for i in range(len(leg_points) - 1):
                p0 = np.array(leg_points[i])
                p1 = np.array(leg_points[i + 1])
                offset = np.array([0.02, 0.02, 0])
                quad = Polygon(
                    p0 - offset, p0 + offset, p1 + offset, p1 - offset,
                    color=ORANGE, fill_opacity=0.7,
                    stroke_color=WHITE, stroke_width=1
                )
                mesh_triangles.add(quad)

        self.play(ShowCreation(mesh_triangles), run_time=1.5)

        self.play(FadeOut(note4b), FadeOut(corner_points), run_time=0.3)

        result = Text("Done! Triangle Mesh = Chair Shape", font_size=20, color=YELLOW)
        result.shift(DOWN * 2.5)
        result.fix_in_frame()
        self.play(FadeIn(result), run_time=0.3)

        # Rotation display (faster)
        self.play(
            frame.animate.set_euler_angles(theta=-90 * DEGREES, phi=60 * DEGREES),
            run_time=1.2
        )
        self.play(
            frame.animate.set_euler_angles(theta=90 * DEGREES, phi=70 * DEGREES),
            run_time=1.2
        )

        self.wait(0.3)

        # ========== Step 5: Scene Graph Extraction ==========
        self.play(FadeOut(title4), FadeOut(result), run_time=0.3)

        title5 = Text("Step 5: Scene Graph Extraction", font_size=28, color=TEAL)
        title5.to_edge(UP)
        title5.fix_in_frame()
        self.play(FadeIn(title5, run_time=0.5))

        self.play(
            frame.animate.set_euler_angles(theta=-20 * DEGREES, phi=60 * DEGREES).move_to(ORIGIN + OUT * 3),
            run_time=0.8
        )

        # Layer heights
        z_l1 = 0
        z_l2 = 1.5
        z_l3 = 3.0
        z_l4 = 4.5

        # ====== Layer 1: Object Nodes + Relationship Edges ======
        note_l1 = Text("Layer 1: Object Nodes + Relationship Edges", font_size=16, color=TEAL)
        note_l1.to_corner(DR, buff=0.4)
        note_l1.fix_in_frame()
        self.play(FadeIn(note_l1), run_time=0.3)

        chair_target_pos = np.array([-1.0, -1.0, z_l1])
        self.play(
            mesh_triangles.animate.scale(0.3).move_to(chair_target_pos),
            run_time=1
        )

        # Other objects
        table = Cube(side_length=0.3)
        table.set_color(BLUE)
        table.set_opacity(0.8)
        table.scale([1.5, 1, 0.4])
        table.move_to(np.array([0, -1.0, z_l1]))

        sofa = Cube(side_length=0.3)
        sofa.set_color(GREEN)
        sofa.set_opacity(0.8)
        sofa.scale([1.3, 1, 0.6])
        sofa.move_to(np.array([1.2, -1.0, z_l1]))

        bed = Cube(side_length=0.35)
        bed.set_color(PINK)
        bed.set_opacity(0.8)
        bed.scale([1.2, 1.5, 0.3])
        bed.move_to(np.array([-2.0, 1.5, z_l1]))

        wardrobe = Cube(side_length=0.25)
        wardrobe.set_color(MAROON)
        wardrobe.set_opacity(0.8)
        wardrobe.scale([1, 0.5, 1.5])
        wardrobe.move_to(np.array([-1.2, 1.5, z_l1]))

        fridge = Cube(side_length=0.25)
        fridge.set_color(WHITE)
        fridge.set_opacity(0.8)
        fridge.scale([0.8, 0.6, 1.5])
        fridge.move_to(np.array([1.5, 1.5, z_l1]))

        stove = Cube(side_length=0.25)
        stove.set_color(GREY)
        stove.set_opacity(0.8)
        stove.scale([1.2, 0.8, 0.4])
        stove.move_to(np.array([2.3, 1.5, z_l1]))

        self.play(
            ShowCreation(table), ShowCreation(sofa),
            ShowCreation(bed), ShowCreation(wardrobe),
            ShowCreation(fridge), ShowCreation(stove),
            run_time=0.8
        )

        # Node positions
        chair_node_pos = np.array([-1.2, -1.8, z_l1 + 0.5])
        table_node_pos = np.array([0, -0.8, z_l1 + 0.5])
        sofa_node_pos = np.array([1.2, -1.8, z_l1 + 0.5])
        bed_node_pos = np.array([-2.5, 1.5, z_l1 + 0.5])
        wardrobe_node_pos = np.array([-1.2, 1.5, z_l1 + 0.5])
        fridge_node_pos = np.array([1.2, 1.5, z_l1 + 0.4])
        stove_node_pos = np.array([2.5, 1.5, z_l1 + 0.4])

        chair_node = Sphere(radius=0.18)
        chair_node.set_color(ORANGE)
        chair_node.move_to(chair_node_pos)

        table_node = Sphere(radius=0.18)
        table_node.set_color(BLUE)
        table_node.move_to(table_node_pos)

        sofa_node = Sphere(radius=0.18)
        sofa_node.set_color(GREEN)
        sofa_node.move_to(sofa_node_pos)

        bed_node = Sphere(radius=0.18)
        bed_node.set_color(PINK)
        bed_node.move_to(bed_node_pos)

        wardrobe_node = Sphere(radius=0.15)
        wardrobe_node.set_color(MAROON)
        wardrobe_node.move_to(wardrobe_node_pos)

        fridge_node = Sphere(radius=0.15)
        fridge_node.set_color(WHITE)
        fridge_node.move_to(fridge_node_pos)

        stove_node = Sphere(radius=0.15)
        stove_node.set_color(GREY)
        stove_node.move_to(stove_node_pos)

        # Labels
        chair_label = Text("Chair", font_size=14, color=ORANGE)
        chair_label.move_to(chair_node_pos + np.array([0, 0.5, 0]))

        table_label = Text("Table", font_size=14, color=BLUE)
        table_label.move_to(table_node_pos + np.array([0, 0.5, 0]))

        sofa_label = Text("Sofa", font_size=14, color=GREEN)
        sofa_label.move_to(sofa_node_pos + np.array([0, 0.5, 0]))

        bed_label = Text("Bed", font_size=14, color=PINK)
        bed_label.move_to(bed_node_pos + np.array([0, 0.5, 0]))

        wardrobe_label = Text("Wardrobe", font_size=14, color=MAROON)
        wardrobe_label.move_to(wardrobe_node_pos + np.array([0, 0.5, 0]))

        fridge_label = Text("Fridge", font_size=14, color=WHITE)
        fridge_label.move_to(fridge_node_pos + np.array([0, 0.5, 0]))

        stove_label = Text("Stove", font_size=14, color=GREY)
        stove_label.move_to(stove_node_pos + np.array([0, 0.5, 0]))

        self.play(
            ShowCreation(chair_node), ShowCreation(table_node), ShowCreation(sofa_node),
            ShowCreation(bed_node), ShowCreation(wardrobe_node),
            ShowCreation(fridge_node), ShowCreation(stove_node),
            FadeIn(chair_label), FadeIn(table_label), FadeIn(sofa_label),
            FadeIn(bed_label), FadeIn(wardrobe_label),
            FadeIn(fridge_label), FadeIn(stove_label),
            run_time=1
        )

        # Relationship edges between nodes
        edge_ct = Line3D(chair_node_pos, table_node_pos, color=WHITE)
        edge_ct.set_opacity(0.7)
        edge_ct_label = Text("beside", font_size=11, color=WHITE)
        edge_ct_label.move_to((chair_node_pos + table_node_pos) / 2 + np.array([0, -0.3, 0]))

        edge_ts = Line3D(table_node_pos, sofa_node_pos, color=WHITE)
        edge_ts.set_opacity(0.7)
        edge_ts_label = Text("across", font_size=11, color=WHITE)
        edge_ts_label.move_to((table_node_pos + sofa_node_pos) / 2 + np.array([0, -0.3, 0]))

        edge_cs = Line3D(chair_node_pos, sofa_node_pos, color=WHITE)
        edge_cs.set_opacity(0.5)
        edge_cs_label = Text("facing", font_size=11, color=WHITE)
        edge_cs_label.move_to((chair_node_pos + sofa_node_pos) / 2 + np.array([0, 0.35, 0]))

        edge_bw = Line3D(bed_node_pos, wardrobe_node_pos, color=WHITE)
        edge_bw.set_opacity(0.7)
        edge_bw_label = Text("beside", font_size=11, color=WHITE)
        edge_bw_label.move_to((bed_node_pos + wardrobe_node_pos) / 2 + np.array([0, -0.3, 0]))

        edge_fs = Line3D(fridge_node_pos, stove_node_pos, color=WHITE)
        edge_fs.set_opacity(0.7)
        edge_fs_label = Text("beside", font_size=11, color=WHITE)
        edge_fs_label.move_to((fridge_node_pos + stove_node_pos) / 2 + np.array([0, -0.3, 0]))

        self.play(
            ShowCreation(edge_ct), ShowCreation(edge_ts), ShowCreation(edge_cs),
            ShowCreation(edge_bw), ShowCreation(edge_fs),
            FadeIn(edge_ct_label), FadeIn(edge_ts_label), FadeIn(edge_cs_label),
            FadeIn(edge_bw_label), FadeIn(edge_fs_label),
            run_time=0.8
        )

        self.wait(0.3)

        layer1_objs = Group(mesh_triangles, table, sofa, bed, wardrobe, fridge, stove,
                           chair_node, table_node, sofa_node, bed_node, wardrobe_node, fridge_node, stove_node,
                           chair_label, table_label, sofa_label, bed_label, wardrobe_label, fridge_label, stove_label,
                           edge_ct, edge_ts, edge_cs, edge_bw, edge_fs,
                           edge_ct_label, edge_ts_label, edge_cs_label, edge_bw_label, edge_fs_label)

        # ====== Layer 2: Relationship Nodes (edges become nodes, connect to Layer 1 NODES) ======
        self.play(FadeOut(note_l1), run_time=0.3)
        note_l2 = Text("Layer 2: Relationship Nodes", font_size=14, color=PURPLE)
        note_l2.to_corner(DR, buff=0.4)
        note_l2.fix_in_frame()
        self.play(FadeIn(note_l2), run_time=0.3)

        # Relationship node positions at z_l2
        rel_ct_pos = np.array([-0.6, -1.4, z_l2])   # Chair-Table
        rel_ts_pos = np.array([0.6, -1.4, z_l2])    # Table-Sofa
        rel_cs_pos = np.array([0, -2.0, z_l2])      # Chair-Sofa
        rel_bw_pos = np.array([-1.8, 1.5, z_l2])    # Bed-Wardrobe
        rel_fs_pos = np.array([1.8, 1.5, z_l2])     # Fridge-Stove

        rel_ct_node = Sphere(radius=0.12)
        rel_ct_node.set_color(PURPLE_A)
        rel_ct_node.move_to(rel_ct_pos)

        rel_ts_node = Sphere(radius=0.12)
        rel_ts_node.set_color(PURPLE_A)
        rel_ts_node.move_to(rel_ts_pos)

        rel_cs_node = Sphere(radius=0.12)
        rel_cs_node.set_color(PURPLE_A)
        rel_cs_node.move_to(rel_cs_pos)

        rel_bw_node = Sphere(radius=0.12)
        rel_bw_node.set_color(PURPLE_A)
        rel_bw_node.move_to(rel_bw_pos)

        rel_fs_node = Sphere(radius=0.12)
        rel_fs_node.set_color(PURPLE_A)
        rel_fs_node.move_to(rel_fs_pos)

        rel_ct_label = Text("C-T beside", font_size=10, color=PURPLE_A)
        rel_ct_label.move_to(rel_ct_pos + np.array([0, 0.35, 0]))

        rel_ts_label = Text("T-S across", font_size=10, color=PURPLE_A)
        rel_ts_label.move_to(rel_ts_pos + np.array([0, 0.35, 0]))

        rel_cs_label = Text("C-S facing", font_size=10, color=PURPLE_A)
        rel_cs_label.move_to(rel_cs_pos + np.array([0, 0.35, 0]))

        rel_bw_label = Text("B-W beside", font_size=10, color=PURPLE_A)
        rel_bw_label.move_to(rel_bw_pos + np.array([0, 0.35, 0]))

        rel_fs_label = Text("F-S beside", font_size=10, color=PURPLE_A)
        rel_fs_label.move_to(rel_fs_pos + np.array([0, 0.35, 0]))

        # Connect relationship nodes to Layer 1 OBJECT NODES (not edge midpoints!)
        # C-T connects to Chair and Table
        rel_ct_to_chair = Line3D(chair_node_pos, rel_ct_pos, color=PURPLE)
        rel_ct_to_chair.set_opacity(0.5)
        rel_ct_to_table = Line3D(table_node_pos, rel_ct_pos, color=PURPLE)
        rel_ct_to_table.set_opacity(0.5)

        # T-S connects to Table and Sofa
        rel_ts_to_table = Line3D(table_node_pos, rel_ts_pos, color=PURPLE)
        rel_ts_to_table.set_opacity(0.5)
        rel_ts_to_sofa = Line3D(sofa_node_pos, rel_ts_pos, color=PURPLE)
        rel_ts_to_sofa.set_opacity(0.5)

        # C-S connects to Chair and Sofa
        rel_cs_to_chair = Line3D(chair_node_pos, rel_cs_pos, color=PURPLE)
        rel_cs_to_chair.set_opacity(0.5)
        rel_cs_to_sofa = Line3D(sofa_node_pos, rel_cs_pos, color=PURPLE)
        rel_cs_to_sofa.set_opacity(0.5)

        # B-W connects to Bed and Wardrobe
        rel_bw_to_bed = Line3D(bed_node_pos, rel_bw_pos, color=PURPLE)
        rel_bw_to_bed.set_opacity(0.5)
        rel_bw_to_wardrobe = Line3D(wardrobe_node_pos, rel_bw_pos, color=PURPLE)
        rel_bw_to_wardrobe.set_opacity(0.5)

        # F-S connects to Fridge and Stove
        rel_fs_to_fridge = Line3D(fridge_node_pos, rel_fs_pos, color=PURPLE)
        rel_fs_to_fridge.set_opacity(0.5)
        rel_fs_to_stove = Line3D(stove_node_pos, rel_fs_pos, color=PURPLE)
        rel_fs_to_stove.set_opacity(0.5)

        self.play(
            ShowCreation(rel_ct_to_chair), ShowCreation(rel_ct_to_table),
            ShowCreation(rel_ts_to_table), ShowCreation(rel_ts_to_sofa),
            ShowCreation(rel_cs_to_chair), ShowCreation(rel_cs_to_sofa),
            ShowCreation(rel_bw_to_bed), ShowCreation(rel_bw_to_wardrobe),
            ShowCreation(rel_fs_to_fridge), ShowCreation(rel_fs_to_stove),
            run_time=0.8
        )
        self.play(
            ShowCreation(rel_ct_node), FadeIn(rel_ct_label),
            ShowCreation(rel_ts_node), FadeIn(rel_ts_label),
            ShowCreation(rel_cs_node), FadeIn(rel_cs_label),
            ShowCreation(rel_bw_node), FadeIn(rel_bw_label),
            ShowCreation(rel_fs_node), FadeIn(rel_fs_label),
            run_time=0.6
        )

        layer2_objs = Group(
            rel_ct_node, rel_ct_label, rel_ts_node, rel_ts_label,
            rel_cs_node, rel_cs_label, rel_bw_node, rel_bw_label,
            rel_fs_node, rel_fs_label,
            rel_ct_to_chair, rel_ct_to_table,
            rel_ts_to_table, rel_ts_to_sofa,
            rel_cs_to_chair, rel_cs_to_sofa,
            rel_bw_to_bed, rel_bw_to_wardrobe,
            rel_fs_to_fridge, rel_fs_to_stove)

        self.wait(0.3)

        # ====== Layer 3: Room Nodes (aggregate objects) ======
        self.play(FadeOut(note_l2), run_time=0.3)
        note_l3 = Text("Layer 3: Room Aggregation", font_size=14, color=RED)
        note_l3.to_corner(DR, buff=0.4)
        note_l3.fix_in_frame()
        self.play(FadeIn(note_l3), run_time=0.3)

        # Room positions at z_l3
        room1_pos = np.array([0, -1.2, z_l3])       # Living Room
        room2_pos = np.array([-1.8, 1.5, z_l3])     # Bedroom
        room3_pos = np.array([1.8, 1.5, z_l3])      # Kitchen

        room1_node = Sphere(radius=0.2)
        room1_node.set_color(RED)
        room1_node.move_to(room1_pos)

        room2_node = Sphere(radius=0.2)
        room2_node.set_color(RED_A)
        room2_node.move_to(room2_pos)

        room3_node = Sphere(radius=0.2)
        room3_node.set_color(RED_B)
        room3_node.move_to(room3_pos)

        room1_label = Text("Living Room", font_size=14, color=RED)
        room1_label.move_to(room1_pos + np.array([0, 0.45, 0]))

        room2_label = Text("Bedroom", font_size=14, color=RED_A)
        room2_label.move_to(room2_pos + np.array([0, 0.45, 0]))

        room3_label = Text("Kitchen", font_size=14, color=RED_B)
        room3_label.move_to(room3_pos + np.array([0, 0.45, 0]))

        # Connect rooms to relationship nodes (Layer 2)
        # Living Room connects to C-T, T-S, C-S
        to_room1_ct = Line3D(rel_ct_pos, room1_pos, color=RED)
        to_room1_ct.set_opacity(0.4)
        to_room1_ts = Line3D(rel_ts_pos, room1_pos, color=RED)
        to_room1_ts.set_opacity(0.4)
        to_room1_cs = Line3D(rel_cs_pos, room1_pos, color=RED)
        to_room1_cs.set_opacity(0.4)

        # Bedroom connects to B-W
        to_room2_bw = Line3D(rel_bw_pos, room2_pos, color=RED_A)
        to_room2_bw.set_opacity(0.4)

        # Kitchen connects to F-S
        to_room3_fs = Line3D(rel_fs_pos, room3_pos, color=RED_B)
        to_room3_fs.set_opacity(0.4)

        self.play(
            ShowCreation(to_room1_ct), ShowCreation(to_room1_ts), ShowCreation(to_room1_cs),
            ShowCreation(to_room2_bw),
            ShowCreation(to_room3_fs),
            run_time=0.8
        )
        self.play(
            ShowCreation(room1_node), FadeIn(room1_label),
            ShowCreation(room2_node), FadeIn(room2_label),
            ShowCreation(room3_node), FadeIn(room3_label),
            run_time=0.6
        )

        layer3_objs = Group(room1_node, room1_label, room2_node, room2_label, room3_node, room3_label,
                           to_room1_ct, to_room1_ts, to_room1_cs,
                           to_room2_bw, to_room3_fs)

        self.wait(0.3)

        # ====== Layer 4: Building ======
        self.play(FadeOut(note_l3), run_time=0.3)
        note_l4 = Text("Layer 4: Building", font_size=14, color=BLUE)
        note_l4.to_corner(DR, buff=0.4)
        note_l4.fix_in_frame()
        self.play(FadeIn(note_l4), run_time=0.3)

        building_pos = np.array([0, 0.3, z_l4])
        building_node = Sphere(radius=0.25)
        building_node.set_color(BLUE)
        building_node.move_to(building_pos)

        building_label = Text("Building", font_size=16, color=BLUE)
        building_label.move_to(building_pos + np.array([0, 0.5, 0]))

        to_building1 = Line3D(room1_pos, building_pos, color=BLUE)
        to_building1.set_opacity(0.6)
        to_building2 = Line3D(room2_pos, building_pos, color=BLUE)
        to_building2.set_opacity(0.6)
        to_building3 = Line3D(room3_pos, building_pos, color=BLUE)
        to_building3.set_opacity(0.6)

        self.play(
            ShowCreation(to_building1), ShowCreation(to_building2), ShowCreation(to_building3),
            run_time=0.6
        )
        self.play(ShowCreation(building_node), FadeIn(building_label), run_time=0.5)

        layer4_objs = Group(building_node, building_label, to_building1, to_building2, to_building3)

        self.wait(0.3)
        self.play(FadeOut(note_l4), run_time=0.3)

        all_3d_objs = Group(layer1_objs, layer2_objs, layer3_objs, layer4_objs)

        # Quick rotation
        self.play(
            frame.animate.set_euler_angles(theta=45 * DEGREES, phi=55 * DEGREES).move_to(np.array([0, 0, 2])),
            run_time=1.5
        )
        self.play(
            frame.animate.set_euler_angles(theta=-30 * DEGREES, phi=65 * DEGREES),
            run_time=1.5
        )

        # ========== Step 6: LLM Query ==========
        self.play(FadeOut(title5), run_time=0.3)

        title6 = Text("Step 6: LLM Queries Scene Graph", font_size=28, color=GOLD)
        title6.to_edge(UP)
        title6.fix_in_frame()
        self.play(FadeIn(title6, run_time=0.5))

        # Show LLM query box — at bottom edge to not block the graph
        query_bg = Rectangle(width=6, height=0.6, color=GOLD, fill_opacity=0.15, stroke_width=2)
        query_bg.to_edge(DOWN, buff=0.3)
        query_bg.fix_in_frame()

        query_text = Text('LLM Query: "Where is the Chair?"', font_size=16, color=GOLD)
        query_text.move_to(query_bg.get_center())
        query_text.fix_in_frame()

        self.play(FadeIn(query_bg), FadeIn(query_text), run_time=0.5)
        self.wait(0.3)

        # LLM traverses Building → Living Room → C-T Relationship → Chair
        traverse_note = Text("Traverse: Building → Living Room → C-T → Chair", font_size=14, color=YELLOW)
        traverse_note.to_corner(DR, buff=0.4)
        traverse_note.fix_in_frame()
        self.play(FadeIn(traverse_note), run_time=0.3)

        # Highlight Building node (Layer 4)
        self.play(
            building_node.animate.set_color(GOLD),
            run_time=0.3
        )
        self.wait(0.2)

        # Highlight path Building → Living Room (Layer 4 → Layer 3)
        highlight_b_to_lr = Line3D(building_pos, room1_pos, color=GOLD)
        highlight_b_to_lr.set_opacity(0.9)
        self.play(
            ShowCreation(highlight_b_to_lr),
            room1_node.animate.set_color(GOLD),
            run_time=0.5
        )
        self.wait(0.2)

        # Highlight path Living Room → C-T Relationship (Layer 3 → Layer 2)
        highlight_lr_to_ct = Line3D(room1_pos, rel_ct_pos, color=GOLD)
        highlight_lr_to_ct.set_opacity(0.9)
        self.play(
            ShowCreation(highlight_lr_to_ct),
            rel_ct_node.animate.set_color(GOLD),
            run_time=0.5
        )
        self.wait(0.2)

        # Highlight path C-T → Chair (Layer 2 → Layer 1)
        highlight_ct_to_chair = Line3D(rel_ct_pos, chair_node_pos, color=GOLD)
        highlight_ct_to_chair.set_opacity(0.9)
        self.play(
            ShowCreation(highlight_ct_to_chair),
            chair_node.animate.set_color(GOLD),
            run_time=0.5
        )
        self.wait(0.2)

        # Show answer
        self.play(FadeOut(query_text), run_time=0.2)

        answer_text = Text('Answer: "Chair is in Living Room, beside Table"', font_size=16, color=GREEN)
        answer_text.move_to(query_bg.get_center())
        answer_text.fix_in_frame()
        self.play(FadeIn(answer_text), run_time=0.5)

        # Highlight the chair mesh
        self.play(
            mesh_triangles.animate.set_color(GOLD),
            run_time=0.5
        )

        # Camera focus on the chair
        self.play(
            frame.animate.set_euler_angles(theta=-45 * DEGREES, phi=70 * DEGREES).move_to(np.array([-0.5, -0.5, 1])),
            run_time=1.5
        )

        self.wait(0.5)

        # Zoom back out
        self.play(
            frame.animate.set_euler_angles(theta=30 * DEGREES, phi=55 * DEGREES).move_to(np.array([0, 0, 2])),
            run_time=1.5
        )

        # Final summary — move camera up so graph is in upper half, text at bottom edge
        self.play(
            FadeOut(traverse_note), FadeOut(answer_text), FadeOut(query_bg),
            frame.animate.set_euler_angles(theta=20 * DEGREES, phi=50 * DEGREES).move_to(np.array([0, 0, 3])),
            run_time=0.8
        )

        summary_line1 = Text("LLM + Scene Graph = Spatial Reasoning", font_size=16, color=TEAL)
        summary_line1.to_edge(DOWN, buff=0.6)
        summary_line1.fix_in_frame()

        summary_line2 = Text("Photo → Semantic → Voxel → Mesh → Graph → LLM Query", font_size=12, color=GREY_A)
        summary_line2.next_to(summary_line1, DOWN, buff=0.15)
        summary_line2.fix_in_frame()

        self.play(FadeIn(summary_line1), FadeIn(summary_line2), run_time=0.5)

        self.wait(1.5)
