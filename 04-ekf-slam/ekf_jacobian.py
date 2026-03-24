from manimlib import *
import numpy as np


class EKFJacobian(ThreeDScene):
    """
    EKF Jacobian 線性化限制動畫

    Run: cd /home/ian/Desktop/slam_visualization
         DISPLAY=:0 PATH="/home/ian/.local/bin:/usr/bin:/home/ian/miniconda3/bin:$PATH" manimgl ekf_jacobian.py EKFJacobian -w -r 1920x1080
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

        # --- 顏色 ---
        CLR_PRIOR = BLUE_C
        CLR_TRUE = BLUE
        CLR_EKF = RED
        CLR_ERR = RED

        # --- Helpers ---
        def make_gauss(mu_x, mu_y, sx, sy, h, color, opacity=0.85):
            spread = 3.0
            def func(u, v):
                x = mu_x + u * sx * np.cos(v)
                y = mu_y + u * sy * np.sin(v)
                z = h * np.exp(-u ** 2 / 2.0)
                return axes.c2p(x, y, z)
            surf = ParametricSurface(func, u_range=[0, spread], v_range=[0, TAU],
                                      resolution=(30, 60))
            surf.set_color(color)
            surf.set_opacity(opacity)
            mesh = SurfaceMesh(surf, resolution=(30, 60))
            mesh.set_stroke(WHITE, width=2.0, opacity=0.7)
            return surf, mesh

        def make_ellipse(mx, my, sx, sy, color, ns=2, width=6, opacity=1.0,
                         fill_color=None, fill_opacity=0.0):
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

        # --- 非線性函數 ---
        def f_nonlinear(x, y):
            r = np.sqrt(x ** 2 + y ** 2) + 1e-8
            theta = np.arctan2(y, x)
            new_r = r ** 1.4
            new_theta = theta + r * 0.5
            return new_r * np.cos(new_theta), new_r * np.sin(new_theta)

        def jacobian_at(mx, my, eps=1e-5):
            fx0, fy0 = f_nonlinear(mx, my)
            fx1, fy1 = f_nonlinear(mx + eps, my)
            fx2, fy2 = f_nonlinear(mx, my + eps)
            J = np.array([
                [(fx1 - fx0) / eps, (fx2 - fx0) / eps],
                [(fy1 - fy0) / eps, (fy2 - fy0) / eps],
            ])
            return J

        # 參數
        mu0x, mu0y = 2.0, 1.0
        s0x, s0y = 0.8, 0.8
        h0 = 2.5
        spread = 3.0

        # 預先計算
        J = jacobian_at(mu0x, mu0y)
        P0 = np.diag([s0x ** 2, s0y ** 2])
        P_lin = J @ P0 @ J.T
        s_lin_x = np.sqrt(P_lin[0, 0])
        s_lin_y = np.sqrt(P_lin[1, 1])
        mu_lin_x, mu_lin_y = f_nonlinear(mu0x, mu0y)
        h_lin = 2.5 / (s_lin_x * s_lin_y / (s0x * s0y))
        h_lin = min(h_lin, 2.5)

        np.random.seed(42)
        samples = np.random.multivariate_normal([mu0x, mu0y], P0, 2000)
        mapped = np.array([f_nonlinear(s[0], s[1]) for s in samples])
        true_cx = np.mean(mapped[:, 0])
        true_cy = np.mean(mapped[:, 1])
        true_sx = np.std(mapped[:, 0])
        true_sy = np.std(mapped[:, 1])

        # ================================================================
        # STEP 1: EKF 公式
        # ================================================================
        title = Text("Extended Kalman Filter", font_size=42, color=YELLOW)
        title.to_corner(UL, buff=0.3)
        title.fix_in_frame()
        self.play(Write(title), run_time=0.8)

        eq1 = Tex(r"x_k = f(x_{k-1}, u_k) + w_{k-1}", font_size=32, color=CLR_PRIOR)
        eq1.next_to(title, DOWN, buff=0.2, aligned_edge=LEFT)
        eq1.fix_in_frame()
        eq2 = Tex(r"z_k = h(x_k) + v_k", font_size=32, color=GREEN)
        eq2.next_to(eq1, DOWN, buff=0.15, aligned_edge=LEFT)
        eq2.fix_in_frame()

        self.play(FadeIn(eq1), run_time=0.8)
        self.play(FadeIn(eq2), run_time=0.8)
        self.wait(2)
        self.play(FadeOut(eq1), FadeOut(eq2), run_time=0.3)

        # ================================================================
        # STEP 2: Prior — EKF 假設是高斯
        # ================================================================
        s2 = Text("EKF Assumption", font_size=36, color=CLR_PRIOR)
        s2.to_corner(UL, buff=0.3)
        s2.fix_in_frame()
        self.play(FadeOut(title), run_time=0.2)
        self.play(Write(s2), run_time=0.5)

        assume_eq = Tex(r"\hat{x}_{k-1} \sim \mathcal{N}(\mu, P_{k-1})",
                        font_size=32, color=CLR_PRIOR)
        assume_eq.next_to(s2, DOWN, buff=0.2, aligned_edge=LEFT)
        assume_eq.fix_in_frame()
        self.play(FadeIn(assume_eq), run_time=0.8)

        prior_s, prior_m = make_gauss(mu0x, mu0y, s0x, s0y, h0, CLR_PRIOR)
        self.play(ShowCreation(prior_s), ShowCreation(prior_m), run_time=2)

        prior_lb = Tex(r"\hat{x}_{k-1}", font_size=32, color=CLR_PRIOR)
        prior_lb.move_to(axes.c2p(mu0x, mu0y - 2.0, 0))
        prior_lb.fix_in_frame()
        self.play(FadeIn(prior_lb), run_time=0.3)
        self.wait(2)

        # ================================================================
        # STEP 3: 非線性變換 → 扭曲（Transform 過去）
        # ================================================================
        self.play(FadeOut(s2), FadeOut(assume_eq), run_time=0.2)

        s3 = Text("Nonlinear Transform", font_size=36, color=CLR_TRUE)
        s3.to_corner(UL, buff=0.3)
        s3.fix_in_frame()
        self.play(Write(s3), run_time=0.5)

        f_eq = Tex(r"x_k = f(x_{k-1})", font_size=32, color=CLR_TRUE)
        f_eq.next_to(s3, DOWN, buff=0.2, aligned_edge=LEFT)
        f_eq.fix_in_frame()
        self.play(FadeIn(f_eq), run_time=0.5)

        # 扭曲的真實分佈
        def warped_func(u, v):
            ox = mu0x + u * s0x * np.cos(v)
            oy = mu0y + u * s0y * np.sin(v)
            nx, ny = f_nonlinear(ox, oy)
            z = h0 * np.exp(-u ** 2 / 2.0)
            return axes.c2p(nx, ny, z)

        warped_surf = ParametricSurface(warped_func, u_range=[0, spread],
                                         v_range=[0, TAU], resolution=(40, 80))
        warped_surf.set_color(CLR_TRUE)
        warped_surf.set_opacity(0.75)
        warped_mesh = SurfaceMesh(warped_surf, resolution=(40, 80))
        warped_mesh.set_stroke(WHITE, width=2.0, opacity=0.7)

        # 藍色高斯 Transform 成扭曲的青綠形狀
        self.play(
            Transform(prior_s, warped_surf),
            Transform(prior_m, warped_mesh),
            run_time=3.0
        )

        true_lb = Tex(r"P_{\text{true}}", font_size=36, color=CLR_TRUE)
        true_lb.move_to(axes.c2p(true_cx, true_cy - 3.0, 0))
        true_lb.fix_in_frame()
        self.play(FadeOut(prior_lb), FadeIn(true_lb), run_time=0.5)
        self.wait(2)

        # ================================================================
        # STEP 4: 真實分佈消失 → EKF 線性化高斯出現（分開看）
        # ================================================================
        self.play(FadeOut(s3), FadeOut(f_eq), run_time=0.2)

        s4 = Text("Jacobian Linearization", font_size=36, color=CLR_EKF)
        s4.to_corner(UL, buff=0.3)
        s4.fix_in_frame()
        self.play(Write(s4), run_time=0.5)

        jac_eq = Tex(r"F_k = \left.\frac{\partial f}{\partial x}\right|_{\hat{x}_{k-1}}",
                     font_size=32, color=CLR_EKF)
        jac_eq.next_to(s4, DOWN, buff=0.2, aligned_edge=LEFT)
        jac_eq.fix_in_frame()
        self.play(FadeIn(jac_eq), run_time=0.8)

        # 真實分佈淡出
        self.play(
            FadeOut(prior_s), FadeOut(prior_m), FadeOut(true_lb),
            run_time=1.5
        )

        # EKF 線性化高斯出現（粉紅）
        ekf_s, ekf_m = make_gauss(mu_lin_x, mu_lin_y, s_lin_x, s_lin_y,
                                   h_lin, CLR_EKF, opacity=0.7)
        self.play(ShowCreation(ekf_s), ShowCreation(ekf_m), run_time=2)

        ekf_lb = Tex(r"P_{\text{EKF}}", font_size=36, color=CLR_EKF)
        ekf_lb.move_to(axes.c2p(mu_lin_x + 3.0, mu_lin_y + 2.0, 0))
        ekf_lb.fix_in_frame()
        self.play(FadeIn(ekf_lb), run_time=0.3)
        self.wait(2)

        # ================================================================
        # STEP 5: 誤差區域 — 紅色立體（真實 - EKF 的差）
        # ================================================================
        self.play(FadeOut(s4), FadeOut(jac_eq), run_time=0.2)

        s5 = Text("Linearization Error", font_size=36, color=CLR_ERR)
        s5.to_corner(UL, buff=0.3)
        s5.fix_in_frame()
        self.play(Write(s5), run_time=0.5)

        err_eq = Tex(r"E[f(x)] \neq f(E[x])", font_size=30, color=YELLOW)
        err_eq.next_to(s5, DOWN, buff=0.2, aligned_edge=LEFT)
        err_eq.fix_in_frame()
        self.play(FadeIn(err_eq), run_time=0.8)

        # 重新顯示真實分佈（半透明）疊上去看差異
        warped_surf2 = ParametricSurface(warped_func, u_range=[0, spread],
                                          v_range=[0, TAU], resolution=(40, 80))
        warped_surf2.set_color(CLR_TRUE)
        warped_surf2.set_opacity(0.4)
        warped_mesh2 = SurfaceMesh(warped_surf2, resolution=(40, 80))
        warped_mesh2.set_stroke(WHITE, width=1.5, opacity=0.4)
        self.play(ShowCreation(warped_surf2), ShowCreation(warped_mesh2), run_time=1.5)

        self.wait(1)

        # 誤差區域：真實分佈中 EKF 沒有覆蓋到的部分（紅色實心）
        # 方法：在真實分佈的座標上，如果 EKF 高斯高度 < 真實高度的一半，就顯示紅色
        def error_func(u, v):
            ox = mu0x + u * s0x * np.cos(v)
            oy = mu0y + u * s0y * np.sin(v)
            nx, ny = f_nonlinear(ox, oy)
            z_true = h0 * np.exp(-u ** 2 / 2.0)

            # EKF 在這個位置的高度
            dx = (nx - mu_lin_x) / max(s_lin_x, 0.01)
            dy = (ny - mu_lin_y) / max(s_lin_y, 0.01)
            z_ekf = h_lin * np.exp(-0.5 * (dx ** 2 + dy ** 2))

            # 誤差 = 真實比 EKF 高出的部分（EKF 沒 cover 到的）
            z_err = max(z_true - z_ekf, 0)
            return axes.c2p(nx, ny, z_err)

        err_surf = ParametricSurface(error_func, u_range=[0, spread],
                                      v_range=[0, TAU], resolution=(40, 80))
        err_surf.set_color(RED)
        err_surf.set_opacity(0.95)
        # 不加 mesh，純紅色實心區塊
        self.play(ShowCreation(err_surf), run_time=2)

        err_label = Tex(r"\text{Error}", font_size=36, color=RED)
        err_label.move_to(axes.c2p(mu_lin_x - 3.0, mu_lin_y - 2.0, 0))
        err_label.fix_in_frame()
        self.play(FadeIn(err_label), run_time=0.3)

        # 旋轉看清楚
        self.play(
            frame.animate.set_euler_angles(theta=-60 * DEGREES, phi=55 * DEGREES),
            run_time=2
        )
        self.wait(3)

        # 清除誤差表面，留下俯視對比
        self.play(
            FadeOut(err_surf), FadeOut(err_label),
            FadeOut(warped_surf2), FadeOut(warped_mesh2),
            FadeOut(ekf_s), FadeOut(ekf_m), FadeOut(ekf_lb),
            FadeOut(err_eq),
            run_time=0.5
        )

        # ================================================================
        # STEP 6: 俯視橢圓對比 — 標籤跟著動
        # ================================================================
        self.play(FadeOut(s5), run_time=0.2)

        s6 = Text("Overconfidence", font_size=36, color=YELLOW)
        s6.to_corner(UL, buff=0.3)
        s6.fix_in_frame()
        self.play(Write(s6), run_time=0.5)

        ov_eq = Tex(r"P_{\text{EKF}} \ll P_{\text{true}}", font_size=30, color=YELLOW)
        ov_eq.next_to(s6, DOWN, buff=0.2, aligned_edge=LEFT)
        ov_eq.fix_in_frame()
        self.play(FadeIn(ov_eq), run_time=0.5)

        # 轉俯視
        self.play(
            frame.animate.set_euler_angles(theta=0 * DEGREES, phi=0 * DEGREES),
            run_time=2
        )

        # EKF 橢圓（粉紅邊框 + 粉紅填色）
        ekf_ell = make_ellipse(mu_lin_x, mu_lin_y, s_lin_x, s_lin_y,
                                CLR_EKF, width=12,
                                fill_color=CLR_EKF, fill_opacity=0.25)

        # 真實橢圓（青綠邊框 + 青綠填色）
        true_ell = make_ellipse(true_cx, true_cy, true_sx, true_sy,
                                 CLR_TRUE, width=12,
                                 fill_color=CLR_TRUE, fill_opacity=0.2)

        # 先出現 EKF 橢圓
        self.play(ShowCreation(ekf_ell), run_time=1)

        # EKF 標籤 — 粉紅色背景框，箭頭指向橢圓
        ekf_ell_lb = Tex(r"P_{\text{EKF}}", font_size=40, color=WHITE)
        ekf_ell_lb.move_to(axes.c2p(mu_lin_x - 4.0, mu_lin_y - 3.0, 0))
        ekf_ell_lb.fix_in_frame()
        ekf_bg = SurroundingRectangle(ekf_ell_lb, color=CLR_EKF, fill_opacity=0.8,
                                       fill_color=CLR_EKF, buff=0.15)
        ekf_bg.fix_in_frame()
        ekf_arrow = Arrow(
            axes.c2p(mu_lin_x - 2.5, mu_lin_y - 2.5, 0),
            axes.c2p(mu_lin_x, mu_lin_y, 0),
            color=CLR_EKF, stroke_width=6, buff=0.1,
        )
        self.play(FadeIn(ekf_bg), FadeIn(ekf_ell_lb), ShowCreation(ekf_arrow), run_time=0.8)
        self.wait(1)

        # 再出現 真實橢圓
        self.play(ShowCreation(true_ell), run_time=1)

        # 真實標籤 — 青綠色背景框，箭頭指向橢圓
        true_ell_lb = Tex(r"P_{\text{true}}", font_size=40, color=WHITE)
        true_ell_lb.move_to(axes.c2p(true_cx + 4.0, true_cy + 3.0, 0))
        true_ell_lb.fix_in_frame()
        true_bg = SurroundingRectangle(true_ell_lb, color=CLR_TRUE, fill_opacity=0.8,
                                        fill_color=CLR_TRUE, buff=0.15)
        true_bg.fix_in_frame()
        true_arrow = Arrow(
            axes.c2p(true_cx + 2.5, true_cy + 2.5, 0),
            axes.c2p(true_cx + true_sx * 0.8, true_cy + true_sy * 0.8, 0),
            color=CLR_TRUE, stroke_width=6, buff=0.1,
        )
        self.play(FadeIn(true_bg), FadeIn(true_ell_lb), ShowCreation(true_arrow), run_time=0.8)
        self.wait(3)

        # ================================================================
        # STEP 7: 發散 — EKF 縮小，真實擴大，標籤跟著動
        # ================================================================
        self.play(FadeOut(s6), FadeOut(ov_eq), run_time=0.2)

        s7 = Text("EKF Divergence", font_size=36, color=CLR_ERR)
        s7.to_corner(UL, buff=0.3)
        s7.fix_in_frame()
        self.play(Write(s7), run_time=0.5)

        # 發散原因公式
        div_eq = Tex(r"z_k \notin P_{\text{EKF}} \Rightarrow \text{reject } z_k",
                     font_size=28, color=YELLOW)
        div_eq.next_to(s7, DOWN, buff=0.2, aligned_edge=LEFT)
        div_eq.fix_in_frame()
        self.play(FadeIn(div_eq), run_time=0.5)

        # 移除箭頭和背景框（動畫中不好跟著動）
        self.play(
            FadeOut(ekf_arrow), FadeOut(true_arrow),
            FadeOut(ekf_bg), FadeOut(true_bg),
            run_time=0.3
        )

        n_div = 20
        for i in range(1, n_div + 1):
            a = i / n_div
            # EKF 越來越小（過度自信）
            ekf_sx_now = s_lin_x * (1 - 0.7 * a)
            ekf_sy_now = s_lin_y * (1 - 0.7 * a)
            ekf_shrink = make_ellipse(mu_lin_x, mu_lin_y,
                                       ekf_sx_now, ekf_sy_now,
                                       CLR_EKF, width=10,
                                       fill_color=CLR_EKF, fill_opacity=0.25)
            # 真實越來越大
            true_sx_now = true_sx * (1 + 0.4 * a)
            true_sy_now = true_sy * (1 + 0.4 * a)
            true_grow = make_ellipse(true_cx, true_cy,
                                      true_sx_now, true_sy_now,
                                      CLR_TRUE, width=10,
                                      fill_color=CLR_TRUE, fill_opacity=0.2)

            # 標籤跟著橢圓邊緣動
            new_ekf_lb = Tex(r"P_{\text{EKF}}", font_size=34, color=CLR_EKF)
            new_ekf_lb.move_to(axes.c2p(mu_lin_x, mu_lin_y - 2 * ekf_sy_now - 0.8, 0))
            new_ekf_lb.fix_in_frame()

            new_true_lb = Tex(r"P_{\text{true}}", font_size=34, color=CLR_TRUE)
            new_true_lb.move_to(axes.c2p(true_cx, true_cy + 2 * true_sy_now + 0.8, 0))
            new_true_lb.fix_in_frame()

            self.play(
                Transform(ekf_ell, ekf_shrink),
                Transform(true_ell, true_grow),
                Transform(ekf_ell_lb, new_ekf_lb),
                Transform(true_ell_lb, new_true_lb),
                run_time=3.0 / n_div, rate_func=linear,
            )

        self.wait(3)

        # ================================================================
        # ENDING: 總結
        # ================================================================
        self.play(
            FadeOut(s7), FadeOut(div_eq),
            FadeOut(ekf_ell), FadeOut(ekf_ell_lb),
            FadeOut(true_ell), FadeOut(true_ell_lb),
            run_time=0.5
        )

        self.play(
            frame.animate.set_euler_angles(theta=-30 * DEGREES, phi=70 * DEGREES),
            run_time=2
        )

        summary_title = Text("EKF Limitation", font_size=36, color=YELLOW)
        summary_title.to_corner(UL, buff=0.3)
        summary_title.fix_in_frame()
        self.play(Write(summary_title), run_time=0.5)

        sum_eqs = VGroup(
            Tex(r"x_k = f(x_{k-1}, u_k) + w_{k-1}", font_size=28, color=CLR_PRIOR),
            Tex(r"z_k = h(x_k) + v_k", font_size=28, color=GREEN),
            Tex(r"F_k = \left.\frac{\partial f}{\partial x}\right|_{\hat{x}_{k-1}}",
                font_size=28, color=CLR_EKF),
            Tex(r"E[f(x)] \neq f(E[x])", font_size=28, color=YELLOW),
        )
        sum_eqs.arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        sum_eqs.next_to(summary_title, DOWN, buff=0.2, aligned_edge=LEFT)
        for eq in sum_eqs:
            eq.fix_in_frame()

        self.play(*[FadeIn(eq) for eq in sum_eqs], run_time=1.5)
        self.wait(4)

        self.play(
            FadeOut(summary_title),
            *[FadeOut(eq) for eq in sum_eqs],
            FadeOut(axes),
            run_time=1
        )
