import numpy as np
from cs import CanonicalSystem
from scipy import interpolate

class ForceDMP:

    def __init__(self, T, dt, n_bfs=10):
        self.T = T
        self.dt = dt
        self.n_bfs = n_bfs

        # canonical system
        a = 1.0
        self.cs = CanonicalSystem(a, T, dt)

        # initialize basis functions for LWR
        self.w = np.zeros(n_bfs)
        self.centers = None
        self.widths = None
        self.set_basis_functions()

        # executed trajectory
        self.f = None

        # desired path
        self.path = None

        self.reset()
    
    def reset(self):
        self.f = 0.0
        self.cs.reset()
    
    def set_basis_functions(self):
        time = np.linspace(0, self.T, self.n_bfs)
        self.centers = np.zeros(self.n_bfs)
        self.centers = np.exp(-self.cs.a * time)
        self.widths = np.ones(self.n_bfs) * self.n_bfs ** 1.5 / self.centers / self.cs.a

    def psi(self, theta):
        if isinstance(theta, np.ndarray):
            theta = theta[:, None]
        return np.exp(-self.widths * (theta - self.centers) ** 2)
    
    def step(self, tau=1.0, error=0.0, alpha=0.0):
        
        # error_coupling = 1.0 / (1.0 + alpha * error)
        error_coupling = 1.0 - alpha
        theta = self.cs.step(tau, error_coupling)
        psi = self.psi(theta)

        self.f = np.dot(self.w, psi) / np.sum(psi)

        return self.f
    
    def fit(self, f_demo, tau=1.0):
        self.path = f_demo
        self.f0 = f_demo[0].copy()
        self.fg = f_demo[-1].copy()

        f_demo = interpolate_path(self, f_demo)

        f_target = f_demo.copy()

        theta_seq = self.cs.all_steps()
        psi_funs = self.psi(theta_seq)

        # Locally Weighted Regression
        aa = np.multiply(theta_seq.reshape((1, theta_seq.shape[0])), psi_funs.T)
        aa = np.multiply(aa, f_target.reshape((1, theta_seq.shape[0])))
        aa = np.sum(aa, axis=1)

        bb = np.multiply(theta_seq.reshape((1, theta_seq.shape[0])), psi_funs.T)
        bb = np.sum(bb, axis=1)
        self.w = aa / bb

        self.reset()
    
    def run_sequence(self, tau=1.0):
        f = np.zeros(self.cs.N)
        for i in range(self.cs.N):
            f[i] = self.step(tau=tau)
        return f
    
def interpolate_path(dmp, path):
    time = np.linspace(0, dmp.cs.T, path.shape[0])
    inter = interpolate.interp1d(time, path)
    y = np.array([inter(i * dmp.dt) for i in range(dmp.cs.N)])
    return y