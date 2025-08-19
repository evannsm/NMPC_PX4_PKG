from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
from nmpc_px4_pkg.quad_casadi_model import Quadrotor
import importlib, sys, os
from casadi import vertcat, horzcat, diag
from scipy.linalg import block_diag
import time

class QuadrotorMPC2:
    def __init__(self, generate_c_code: bool, quadrotor: Quadrotor, horizon: float, num_steps: int):
        self.model = AcadosModel()
        self.quad = quadrotor
        self.model_name = 'holybro'
        self.horizon = horizon
        self.num_steps = num_steps

        self.hover_ctrl = np.array([self.quad.m * self.quad.g, 0., 0., 0.])

        self.ocp_solver = None
        self.generate_c_code = generate_c_code

        # === Put all generated artifacts under <this package>/acados_generated_files ===
        pkg_dir = os.path.dirname(os.path.abspath(__file__))   # .../nmpc_px4_pkg
        parts = pkg_dir.split(os.sep)
        parts = ["src" if part in ("build","install") else part for part in parts] # Replace 'build' with 'src' if it exists in the path
        pkg_dir = os.sep.join(parts)
        gen_root = os.path.join(pkg_dir, "acados_generated_files")
        os.makedirs(gen_root, exist_ok=True)

        # C code export dir and JSON path (both inside acados_generated_files)
        self.code_export_directory = os.path.join(gen_root, f"{self.model_name}_mpc_c_generated_code")
        self.json_path = os.path.join(gen_root, f"{self.model_name}_mpc_acados_ocp.json")

        if self.generate_c_code:
            print("\n[acados] Generating/compiling fresh MPC...\n")
            self.generate_mpc()
            print("[acados] Done.")
        else:
            try:
                print("\n[acados] Trying to load compiled MPC; will generate if not found...\n")
                sys.path.append(self.code_export_directory)
                acados_ocp_solver_pyx = importlib.import_module('acados_ocp_solver_pyx')
                self.ocp_solver = acados_ocp_solver_pyx.AcadosOcpSolverCython(
                    self.model_name, 'SQP', self.num_steps
                )
            except ImportError:
                print("[acados] Compiled MPC not found. Generating now...")
                self.generate_mpc()
                print("[acados] Done! Control stack should begin in two seconds...")
                time.sleep(2)



    def generate_mpc(self):
        f_expl, x, u = self.quad.dynamics()

        # --- Define Acados Model ---
        model = AcadosModel()
        model.f_expl_expr = f_expl
        model.x = x
        model.u = u
        model.name = self.model_name

        # --- Define Acados OCP ---
        ocp = AcadosOcp()
        ocp.model = model
        ocp.code_export_directory = self.code_export_directory

        # --- Define Dimensions ---
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu
        ny_e = nx

        # --- Temporal Parameters ---
        Tf = self.horizon
        N = self.num_steps
        ocp.dims.N = N
        ocp.solver_options.tf = Tf

        # --- Define Cost Matrices ---
        Q = 50 * np.diag([20., 20., 20., 2., 2., 2., 1., 1., 20.])
        R = 1 * diag(horzcat(1., 1., 1., 1.))
        W = block_diag(Q, R)

        # --- Define Cost Functions ---
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.Vx = np.vstack([np.identity(nx), np.zeros((nu, nx))])
        ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.identity(nu)])
        ocp.cost.W = W
        ocp.cost.yref = np.zeros(ny)

        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W_e = Q
        ocp.cost.Vx_e = np.identity(nx)
        ocp.cost.yref_e = np.zeros(ny_e)

        # --- Input Bounds ---
        max_rate = 0.8
        max_thrust = 27.0
        min_thrust = 0.0
        ocp.constraints.lbu = np.array([min_thrust, -max_rate, -max_rate, -max_rate])
        ocp.constraints.ubu = np.array([max_thrust,  max_rate,  max_rate,  max_rate])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])

        # --- State Bounds ---
        ocp.constraints.x0 = np.zeros(9)

        # --- Define Solver Options ---
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP'

        # --- Generate JSON into our custom folder ---
        AcadosOcpSolver.generate(ocp, json_file=self.json_path)


        # --- Compile with cython ---
        AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)

        # --- Import compiled module from our folder ---
        mod_path = os.path.join(self.code_export_directory, "acados_ocp_solver_pyx.so")
        spec = importlib.util.spec_from_file_location("acados_ocp_solver_pyx", mod_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        acados_ocp_solver_pyx = mod   # use it as usual

        # sys.path.append(self.code_export_directory)  # Add the directory to sys.path
        # acados_ocp_solver_pyx = importlib.import_module('acados_ocp_solver_pyx.o')
        print(f"nonononono\n\n")
        self.ocp_solver = acados_ocp_solver_pyx.AcadosOcpSolverCython(
            self.model_name, 'SQP', self.num_steps
        )

    def solve_mpc_control(self, x0, xd):
        N = self.num_steps
        if xd.shape[1] != N:
            raise ValueError("The reference trajectory should have the same length as the number of steps")

        for i in range(N):
            y_ref = np.hstack((xd[:, i], self.hover_ctrl))
            self.ocp_solver.set(i, 'y_ref', y_ref)

        self.ocp_solver.set(0, 'lbx', x0)
        self.ocp_solver.set(0, 'ubx', x0)

        status = self.ocp_solver.solve()
        x_mpc = self.ocp_solver.get(0, 'x')
        u_mpc = self.ocp_solver.get(0, 'u')
        return status, x_mpc, u_mpc
