from manimlib import *
import numpy as np


class KF7Equations(ThreeDScene):
    """
    Kalman Filter 七條公式動畫 — Step 1 & 2: Prior → Predict → Top-down covariance

    Run: cd /home/ian/Desktop/slam_visualization
         DISPLAY=:0 PATH="/home/ian/.local/bin:/usr/bin:/home/ian/miniconda3/bin:$PATH" manimgl kf_7equations.py KF7Equations -w -r 1920x1080
    """

    def construct(self):
        frame = self.camera.frame
        frame.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES)

        axes = ThreeDAxes(
            x_range=[-6, 6, 1],
            y_range=[-6, 6, 1],
            z_range=[0, 3, 0.5],
        )
        axes.scale(0.55)
        self.play(ShowCreation(axes), run_time=0.8)

        # --- 參數 ---
        # Prior 位置
        mu0x, mu0y = -3.0, -2.0
        s0x, s0y = 0.7, 0.7
        h0 = 2.5

        # F 轉移後位置（移動 + 變形：正圓→橢圓，代表 F P F^T）
        mu1x, mu1y = 1.5, 1.0
        sF_x, sF_y = 1.2, 0.5   # F 讓 x 拉長 y 壓扁（不再是正圓）
        hF = 1.8                  # 高度也因為拉扁而降低

        # 加 Q 後（橢圓整體再變大）
        s_pred_x, s_pred_y = 1.8, 1.2
        h_pred = 0.85

        # --- Helpers ---
        def lerp(a, b, t):
            return a + (b - a) * t

        def make_gauss(mu_x, mu_y, sx, sy, h, color, opacity=0.85):
            # 用極座標參數化，曲面邊界是橢圓形（不是方形）
            # u = 半徑 [0, 3σ], v = 角度 [0, 2π]
            spread = 3.0
            def func(u, v):
                # u = 半徑比例 (0~spread), v = 角度
                x = mu_x + u * sx * np.cos(v)
                y = mu_y + u * sy * np.sin(v)
                z = h * np.exp(-u ** 2 / 2.0)
                return axes.c2p(x, y, z)
            surf = ParametricSurface(func, u_range=[0, spread], v_range=[0, TAU], resolution=(30, 60))
            surf.set_color(color)
            surf.set_opacity(opacity)
            mesh = SurfaceMesh(surf, resolution=(30, 60))
            mesh.set_stroke(WHITE, width=2.0, opacity=0.7)
            return surf, mesh

        def make_ellipse(mx, my, sx, sy, color, ns=2, width=6, opacity=1.0,
                         fill_opacity=0.0, fill_color=None):
            c = ParametricCurve(
                lambda t, _mx=mx, _my=my, _sx=sx, _sy=sy: axes.c2p(
                    _mx + ns * _sx * np.cos(t), _my + ns * _sy * np.sin(t), 0
                ),
                t_range=[0, TAU, 0.02],
            )
            c.set_stroke(color, width=width, opacity=opacity)
            if fill_opacity > 0 and fill_color:
                c.set_fill(fill_color, opacity=fill_opacity)
            return c

        def step_animate(dyn_s, dyn_m, dyn_e,
                         start_mx, start_my, start_sx, start_sy, start_h,
                         end_mx, end_my, end_sx, end_sy, end_h,
                         color, n_steps, total_time):
            """分步 Transform 做連續移動+變形"""
            for i in range(1, n_steps + 1):
                a = i / n_steps
                nmx = lerp(start_mx, end_mx, a)
                nmy = lerp(start_my, end_my, a)
                nsx = lerp(start_sx, end_sx, a)
                nsy = lerp(start_sy, end_sy, a)
                nh = lerp(start_h, end_h, a)

                ns, nm = make_gauss(nmx, nmy, nsx, nsy, nh, color)
                ne = make_ellipse(nmx, nmy, nsx, nsy, color,
                                  width=10)

                self.play(
                    Transform(dyn_s, ns),
                    Transform(dyn_m, nm),
                    Transform(dyn_e, ne),
                    run_time=total_time / n_steps,
                    rate_func=linear,
                )

        # ================================================================
        # STEP 1: Prior x_{k-1} — 藍色高斯
        # ================================================================
        s1 = Text("Step 1: Prior State", font_size=38, color=YELLOW)
        s1.to_corner(UL, buff=0.3)
        s1.fix_in_frame()
        self.play(Write(s1), run_time=0.8)

        f1 = Tex(r"\hat{x}_{k-1}", font_size=42, color=BLUE)
        f1.next_to(s1, DOWN, buff=0.2, aligned_edge=LEFT)
        f1.fix_in_frame()
        self.play(FadeIn(f1), run_time=0.5)

        prior_s, prior_m = make_gauss(mu0x, mu0y, s0x, s0y, h0, BLUE)
        self.play(ShowCreation(prior_s), ShowCreation(prior_m), run_time=2)

        lb0 = Tex(r"\hat{x}_{k-1}", font_size=32, color=BLUE)
        lb0.move_to(axes.c2p(mu0x, mu0y - 1.8, 0))
        lb0.fix_in_frame()
        self.play(FadeIn(lb0), run_time=0.5)

        prior_ell = make_ellipse(mu0x, mu0y, s0x, s0y, BLUE,
                                 width=10)
        self.play(ShowCreation(prior_ell), run_time=1)
        self.wait(1.5)

        # ================================================================
        # STEP 2a: F x_{k-1} — 只移動，形狀不變
        # ================================================================
        self.play(FadeOut(s1), FadeOut(f1), run_time=0.3)

        s2 = Text("Step 2: State Prediction", font_size=38, color=ORANGE)
        s2.to_corner(UL, buff=0.3)
        s2.fix_in_frame()
        self.play(Write(s2), run_time=0.5)

        # 公式分段：先顯示 x̂_k^- = F x_{k-1}
        f2_lhs = Tex(r"\hat{x}_k^- =", font_size=34, color=ORANGE)
        f2_lhs.next_to(s2, DOWN, buff=0.2, aligned_edge=LEFT)
        f2_lhs.fix_in_frame()
        self.play(FadeIn(f2_lhs), run_time=0.5)

        f2_Fx = Tex(r"F \hat{x}_{k-1}", font_size=34, color=ORANGE)
        f2_Fx.next_to(f2_lhs, RIGHT, buff=0.12)
        f2_Fx.fix_in_frame()

        self.play(FadeOut(lb0), run_time=0.2)

        # 把舊的 prior 換成動態版本
        self.play(FadeOut(prior_s), FadeOut(prior_m), FadeOut(prior_ell), run_time=0.01)

        dyn_s, dyn_m = make_gauss(mu0x, mu0y, s0x, s0y, h0, ORANGE)
        dyn_e = make_ellipse(mu0x, mu0y, s0x, s0y, ORANGE,
                             width=10)
        self.add(dyn_s, dyn_m, dyn_e)

        # F x_{k-1}: 移動 + 變形（正圓→橢圓，代表 F P F^T 的效果）
        self.play(FadeIn(f2_Fx), run_time=0.5)

        step_animate(dyn_s, dyn_m, dyn_e,
                     mu0x, mu0y, s0x, s0y, h0,
                     mu1x, mu1y, sF_x, sF_y, hF,  # 移動 + 變形！
                     ORANGE, n_steps=20, total_time=2.5)

        self.wait(0.8)

        # ================================================================
        # STEP 2b: + Bu_k + w_k — 加了 Q，才變矮變寬
        # ================================================================
        f2_Bu = Tex(r"+ B u_k + w_k", font_size=34, color=YELLOW)
        f2_Bu.next_to(f2_Fx, RIGHT, buff=0.12)
        f2_Bu.fix_in_frame()

        self.play(FadeIn(f2_Bu), run_time=0.5)

        # 加 Q：橢圓整體再變大（位置不動）
        step_animate(dyn_s, dyn_m, dyn_e,
                     mu1x, mu1y, sF_x, sF_y, hF,
                     mu1x, mu1y, s_pred_x, s_pred_y, h_pred,  # 位置不動，整體變大！
                     ORANGE, n_steps=25, total_time=3.0)

        # 標籤
        pred_lb = Tex(r"\hat{x}_k^-", font_size=32, color=ORANGE)
        pred_lb.move_to(axes.c2p(mu1x, mu1y - 2.5, 0))
        pred_lb.fix_in_frame()
        self.play(FadeIn(pred_lb), run_time=0.5)

        self.wait(2)

        # ================================================================
        # STEP 3: 轉成俯視 → 顯示 covariance 公式 → 對比橢圓
        # ================================================================
        self.play(
            FadeOut(s2), FadeOut(f2_lhs), FadeOut(f2_Fx), FadeOut(f2_Bu),
            FadeOut(pred_lb),
            run_time=0.4
        )

        # 淡出 3D surface
        self.play(FadeOut(dyn_s), FadeOut(dyn_m), run_time=0.5)

        # 轉俯視
        self.play(
            frame.animate.set_euler_angles(theta=0 * DEGREES, phi=0 * DEGREES),
            run_time=2.5
        )
        self.wait(0.5)

        # covariance 公式
        cov_title = Text("Covariance Prediction", font_size=38, color=YELLOW)
        cov_title.to_corner(UL, buff=0.3)
        cov_title.fix_in_frame()
        self.play(Write(cov_title), run_time=0.5)

        cov_eq = Tex(r"P_k^- = F P_{k-1} F^T + Q", font_size=40, color=ORANGE)
        cov_eq.next_to(cov_title, DOWN, buff=0.2, aligned_edge=LEFT)
        cov_eq.fix_in_frame()
        self.play(FadeIn(cov_eq), run_time=1)
        self.wait(1)

        # --- 俯視：兩個橢圓對比 ---
        # prior 橢圓（藍色小圓，原始位置）
        prior_ell2 = make_ellipse(mu0x, mu0y, s0x, s0y, BLUE,
                                  width=10)
        prior_label2 = Tex(r"P_{k-1}", font_size=32, color=BLUE)
        prior_label2.move_to(axes.c2p(mu0x, mu0y, 0))
        prior_label2.fix_in_frame()
        prior_dot2 = Dot(axes.c2p(mu0x, mu0y, 0), color=BLUE, radius=0.1)

        # predict 橢圓（橙色大圓，F 轉移位置）
        # dyn_e 已經在正確位置了，但重新畫一個更明顯的
        pred_ell2 = make_ellipse(mu1x, mu1y, s_pred_x, s_pred_y, ORANGE,
                                 width=10)
        pred_label2 = Tex(r"P_k^-", font_size=32, color=ORANGE)
        pred_label2.move_to(axes.c2p(mu1x, mu1y, 0))
        pred_label2.fix_in_frame()
        pred_dot2 = Dot(axes.c2p(mu1x, mu1y, 0), color=ORANGE, radius=0.1)

        self.play(
            ShowCreation(prior_ell2), FadeIn(prior_label2), FadeIn(prior_dot2),
            ShowCreation(pred_ell2), FadeIn(pred_label2), FadeIn(pred_dot2),
            run_time=1
        )
        self.wait(1)

        # 箭頭
        connect_arrow = Arrow(
            axes.c2p(mu0x, mu0y, 0),
            axes.c2p(mu1x, mu1y, 0),
            color=YELLOW, stroke_width=5, buff=0.5,
        )
        arrow_label = Text("F + Q", font_size=22, color=YELLOW)
        mid_ax = (mu0x + mu1x) / 2
        mid_ay = (mu0y + mu1y) / 2
        arrow_label.move_to(axes.c2p(mid_ax - 1.0, mid_ay + 0.8, 0))
        arrow_label.fix_in_frame()

        self.play(ShowCreation(connect_arrow), FadeIn(arrow_label), run_time=1)

        self.wait(1)

        # --- 綠色 morph：從 prior 移動+變大到 predict ---
        morph_ell = make_ellipse(mu0x, mu0y, s0x, s0y, GREEN,
                                 width=8)
        morph_dot = Dot(axes.c2p(mu0x, mu0y, 0), color=GREEN, radius=0.12)

        self.add(morph_ell, morph_dot)
        self.play(ShowCreation(morph_ell), FadeIn(morph_dot), run_time=0.5)

        # 分步：先移動+變形（F: 正圓→橢圓），再整體變大（+Q）
        # Phase 1: F — 移動 + 正圓→橢圓
        n_m1 = 20
        for i in range(1, n_m1 + 1):
            a = i / n_m1
            nmx = lerp(mu0x, mu1x, a)
            nmy = lerp(mu0y, mu1y, a)
            nsx = lerp(s0x, sF_x, a)
            nsy = lerp(s0y, sF_y, a)
            te = make_ellipse(nmx, nmy, nsx, nsy, GREEN,
                              width=8)
            td = Dot(axes.c2p(nmx, nmy, 0), color=GREEN, radius=0.12)
            self.play(
                Transform(morph_ell, te), Transform(morph_dot, td),
                run_time=2.0 / n_m1, rate_func=linear,
            )

        # Phase 2: +Q — 橢圓整體再變大
        q_appear = Text("+Q", font_size=28, color=RED)
        q_appear.move_to(axes.c2p(mu1x + 2.5, mu1y + 1.5, 0))
        q_appear.fix_in_frame()
        self.play(FadeIn(q_appear), run_time=0.3)

        n_m2 = 20
        for i in range(1, n_m2 + 1):
            a = i / n_m2
            nsx = lerp(sF_x, s_pred_x, a)
            nsy = lerp(sF_y, s_pred_y, a)
            te = make_ellipse(mu1x, mu1y, nsx, nsy, GREEN,
                              width=8)
            self.play(
                Transform(morph_ell, te),
                run_time=2.0 / n_m2, rate_func=linear,
            )

        self.wait(3)

        # Clean up Step 2 俯視對比
        self.play(
            FadeOut(cov_title), FadeOut(cov_eq),
            FadeOut(prior_ell2), FadeOut(prior_label2), FadeOut(prior_dot2),
            FadeOut(pred_ell2), FadeOut(pred_label2), FadeOut(pred_dot2),
            FadeOut(connect_arrow), FadeOut(arrow_label),
            FadeOut(morph_ell), FadeOut(morph_dot),
            FadeOut(dyn_e), FadeOut(q_appear),
            run_time=0.5
        )

        # 回到 3D 繼續
        self.play(
            frame.animate.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES),
            run_time=2
        )
        self.wait(0.5)

        # ================================================================
        # STEP 3: Measurement — 觀測 z_k（綠色高斯）
        # ================================================================
        # K 值（R 小，偏信觀測）
        k_val = 0.65

        # 觀測參數：位置和 predict 分開，避免 3D 重疊擋住
        mu_zx, mu_zy = 3.5, -1.5
        s_zx, s_zy = 0.5, 0.5  # 觀測通常比較精確（小 sigma）
        h_z = 2.8               # 高且窄 = 比較確定

        s3 = Text("Step 3: Measurement", font_size=38, color=GREEN)
        s3.to_corner(UL, buff=0.3)
        s3.fix_in_frame()
        self.play(Write(s3), run_time=0.5)

        f3_eq = Tex(r"z_k = H \hat{x}_k + v_k", font_size=34, color=GREEN)
        f3_eq.next_to(s3, DOWN, buff=0.2, aligned_edge=LEFT)
        f3_eq.fix_in_frame()
        self.play(FadeIn(f3_eq), run_time=0.8)

        # 先重新顯示 predict 分佈（橙色，在 mu1x, mu1y）
        pred_s3, pred_m3 = make_gauss(mu1x, mu1y, s_pred_x, s_pred_y, h_pred, ORANGE)
        self.play(ShowCreation(pred_s3), ShowCreation(pred_m3), run_time=1)

        pred_lb3 = Tex(r"\hat{x}_k^-", font_size=28, color=ORANGE)
        pred_lb3.move_to(axes.c2p(mu1x, mu1y - 2.5, 0))
        pred_lb3.fix_in_frame()
        self.play(FadeIn(pred_lb3), run_time=0.3)

        # 觀測分佈出現（綠色，高且窄）
        obs_s, obs_m = make_gauss(mu_zx, mu_zy, s_zx, s_zy, h_z, GREEN)
        self.play(ShowCreation(obs_s), ShowCreation(obs_m), run_time=2)

        obs_lb = Tex(r"z_k", font_size=28, color=GREEN)
        obs_lb.move_to(axes.c2p(mu_zx, mu_zy - 1.5, 0))
        obs_lb.fix_in_frame()
        self.play(FadeIn(obs_lb), run_time=0.3)

        self.wait(2)

        # ================================================================
        # STEP 4: Innovation (殘差) y_k = z_k - H x̂_k^-
        # ================================================================
        self.play(FadeOut(s3), FadeOut(f3_eq), run_time=0.3)

        s4 = Text("Step 4: Innovation (Residual)", font_size=38, color=YELLOW)
        s4.to_corner(UL, buff=0.3)
        s4.fix_in_frame()
        self.play(Write(s4), run_time=0.5)

        f4_eq = Tex(r"\tilde{y}_k = z_k - H \hat{x}_k^-", font_size=34, color=YELLOW)
        f4_eq.next_to(s4, DOWN, buff=0.2, aligned_edge=LEFT)
        f4_eq.fix_in_frame()
        self.play(FadeIn(f4_eq), run_time=0.8)

        # 箭頭從 predict 到 observation（殘差的視覺化）
        innov_arrow = Arrow(
            axes.c2p(mu1x, mu1y, 0),
            axes.c2p(mu_zx, mu_zy, 0),
            color=YELLOW, stroke_width=6, buff=0.15,
        )
        innov_label = Tex(r"\tilde{y}_k", font_size=28, color=YELLOW)
        innov_mid_x = (mu1x + mu_zx) / 2
        innov_mid_y = (mu1y + mu_zy) / 2
        innov_label.move_to(axes.c2p(innov_mid_x + 1.2, innov_mid_y + 0.8, 0))
        innov_label.fix_in_frame()

        self.play(ShowCreation(innov_arrow), FadeIn(innov_label), run_time=1)

        self.wait(2)

        # ================================================================
        # STEP 5: Kalman Gain
        # ================================================================
        self.play(
            FadeOut(s4), FadeOut(f4_eq),
            FadeOut(innov_arrow), FadeOut(innov_label),
            run_time=0.3
        )

        s5 = Text("Step 5: Kalman Gain", font_size=38, color=TEAL)
        s5.to_corner(UL, buff=0.3)
        s5.fix_in_frame()
        self.play(Write(s5), run_time=0.5)

        # --- Kalman Gain 推導 ---
        # Step 5a: 定義 error
        d1 = Tex(r"e_k = x_k - \hat{x}_k", font_size=30, color=TEAL)
        d1.next_to(s5, DOWN, buff=0.2, aligned_edge=LEFT)
        d1.fix_in_frame()
        self.play(FadeIn(d1), run_time=0.8)
        self.wait(1)

        # Step 5b: P = E[e e^T]
        d2 = Tex(r"P_k = E[e_k \, e_k^T]", font_size=30, color=TEAL)
        d2.next_to(d1, DOWN, buff=0.15, aligned_edge=LEFT)
        d2.fix_in_frame()
        self.play(FadeIn(d2), run_time=0.8)
        self.wait(1)

        # Step 5c: dtr/dK = 0
        d4 = Tex(r"\frac{d\,\mathrm{tr}(P_k)}{dK_k} = 0", font_size=30, color=YELLOW)
        d4.next_to(d2, DOWN, buff=0.15, aligned_edge=LEFT)
        d4.fix_in_frame()
        self.play(FadeIn(d4), run_time=0.8)
        self.wait(1)

        # Step 5e: 結果
        d5 = Tex(r"\Rightarrow K_k = P_k^- H^T (H P_k^- H^T + R)^{-1}",
                 font_size=28, color=TEAL)
        d5.next_to(d4, DOWN, buff=0.2, aligned_edge=LEFT)
        d5.fix_in_frame()
        self.play(FadeIn(d5), run_time=1)
        self.wait(2.5)

        # Clean up Kalman Gain
        self.play(
            FadeOut(s5), FadeOut(d1), FadeOut(d2),
            FadeOut(d4), FadeOut(d5),
            run_time=0.4
        )

        # ================================================================
        # STEP 6: State Update — 融合 predict + measurement → posterior
        # ================================================================
        s6 = Text("Step 6: State Update", font_size=38, color=PURPLE)
        s6.to_corner(UL, buff=0.3)
        s6.fix_in_frame()
        self.play(Write(s6), run_time=0.5)

        f6_eq = Tex(
            r"\hat{x}_k = \hat{x}_k^- + K_k \tilde{y}_k",
            font_size=34, color=PURPLE
        )
        f6_eq.next_to(s6, DOWN, buff=0.2, aligned_edge=LEFT)
        f6_eq.fix_in_frame()
        self.play(FadeIn(f6_eq), run_time=0.8)

        # 後驗分佈：在 predict 和 observation 之間，偏向觀測（因為 K≈0.65）
        # posterior = predict + K * (z - predict)
        mu_post_x = mu1x + k_val * (mu_zx - mu1x)
        mu_post_y = mu1y + k_val * (mu_zy - mu1y)
        # posterior 比 predict 和 observation 都窄（融合後更確定）
        s_post_x, s_post_y = 0.45, 0.40
        h_post = 2.8

        # 動畫：predict 和 observation 融合成 posterior
        # 建立 posterior 分佈（紫色，高且窄）
        post_s, post_m = make_gauss(mu_post_x, mu_post_y, s_post_x, s_post_y,
                                     h_post, PURPLE)

        # 兩個高斯同時 Transform 融合成 posterior
        self.play(
            Transform(pred_s3, post_s),
            Transform(pred_m3, post_m),
            FadeOut(obs_s), FadeOut(obs_m),
            run_time=2.5
        )

        post_lb = Tex(r"\hat{x}_k", font_size=28, color=PURPLE)
        post_lb.move_to(axes.c2p(mu_post_x, mu_post_y - 1.5, 0))
        post_lb.fix_in_frame()
        self.play(FadeIn(post_lb), run_time=0.3)

        self.wait(2.5)

        # ================================================================
        # STEP 7: Covariance Update
        # ================================================================
        self.play(
            FadeOut(s6), FadeOut(f6_eq),
            run_time=0.3
        )

        s7 = Text("Step 7: Covariance Update", font_size=38, color=PURPLE)
        s7.to_corner(UL, buff=0.3)
        s7.fix_in_frame()
        self.play(Write(s7), run_time=0.5)

        f7_eq = Tex(
            r"P_k = (I - K_k H) P_k^-",
            font_size=34, color=PURPLE
        )
        f7_eq.next_to(s7, DOWN, buff=0.2, aligned_edge=LEFT)
        f7_eq.fix_in_frame()
        self.play(FadeIn(f7_eq), run_time=0.8)

        self.wait(1)

        # 轉俯視看 covariance 對比
        self.play(
            FadeOut(pred_lb3), FadeOut(obs_lb),
            FadeOut(pred_s3), FadeOut(pred_m3), FadeOut(post_lb),
            run_time=0.5
        )

        self.play(
            frame.animate.set_euler_angles(theta=0 * DEGREES, phi=0 * DEGREES),
            run_time=2
        )
        self.wait(0.5)

        # 三個橢圓對比：predict（橙色大）、observation（綠色中）、posterior（紫色小）
        ell_pred_final = make_ellipse(mu1x, mu1y, s_pred_x, s_pred_y, ORANGE,
                                      width=8)
        ell_obs_final = make_ellipse(mu_zx, mu_zy, s_zx, s_zy, GREEN,
                                     width=8)
        ell_post_final = make_ellipse(mu_post_x, mu_post_y, s_post_x, s_post_y,
                                      PURPLE, width=10)

        lb_pred_f = Tex(r"P_k^-", font_size=26, color=ORANGE)
        lb_pred_f.move_to(axes.c2p(mu1x, mu1y + 2.5, 0))
        lb_pred_f.fix_in_frame()
        lb_obs_f = Tex(r"R", font_size=26, color=GREEN)
        lb_obs_f.move_to(axes.c2p(mu_zx, mu_zy - 1.5, 0))
        lb_obs_f.fix_in_frame()
        lb_post_f = Tex(r"P_k", font_size=26, color=PURPLE)
        lb_post_f.move_to(axes.c2p(mu_post_x, mu_post_y - 1.2, 0))
        lb_post_f.fix_in_frame()

        self.play(
            ShowCreation(ell_pred_final), FadeIn(lb_pred_f),
            ShowCreation(ell_obs_final), FadeIn(lb_obs_f),
            ShowCreation(ell_post_final), FadeIn(lb_post_f),
            run_time=1.5
        )

        self.wait(3)

        # ================================================================
        # Ending: 回到 3D + 七條公式總結
        # ================================================================
        self.play(
            FadeOut(s7), FadeOut(f7_eq),
            FadeOut(ell_pred_final), FadeOut(ell_obs_final), FadeOut(ell_post_final),
            FadeOut(lb_pred_f), FadeOut(lb_obs_f), FadeOut(lb_post_f),
            run_time=0.5
        )

        self.play(
            frame.animate.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES),
            run_time=2
        )

        # 七條公式總結
        summary_title = Text("Kalman Filter: 7 Equations", font_size=36, color=YELLOW)
        summary_title.to_corner(UL, buff=0.3)
        summary_title.fix_in_frame()

        eq_list = VGroup(
            Tex(r"1.\ \hat{x}_k^- = F \hat{x}_{k-1} + B u_k + w_k", font_size=26, color=ORANGE),
            Tex(r"2.\ P_k^- = F P_{k-1} F^T + Q", font_size=26, color=ORANGE),
            Tex(r"3.\ z_k = H x_k + v_k", font_size=26, color=GREEN),
            Tex(r"4.\ \tilde{y}_k = z_k - H \hat{x}_k^-", font_size=26, color=YELLOW),
            Tex(r"5.\ K_k = P_k^- H^T (H P_k^- H^T + R)^{-1}", font_size=26, color=TEAL),
            Tex(r"6.\ \hat{x}_k = \hat{x}_k^- + K_k \tilde{y}_k", font_size=26, color=PURPLE),
            Tex(r"7.\ P_k = (I - K_k H) P_k^-", font_size=26, color=PURPLE),
        )
        eq_list.arrange(DOWN, buff=0.18, aligned_edge=LEFT)
        eq_list.next_to(summary_title, DOWN, buff=0.25, aligned_edge=LEFT)
        for eq in eq_list:
            eq.fix_in_frame()
        summary_title.fix_in_frame()

        self.play(FadeIn(summary_title), run_time=0.5)
        for eq in eq_list:
            self.play(FadeIn(eq), run_time=0.4)
        self.wait(4)

        # Final clean
        self.play(
            FadeOut(summary_title), FadeOut(eq_list), FadeOut(axes),
            run_time=1
        )
