from ctypes import sizeof
import numpy as np

class EKF():
    def __init__(self):
        self.H = 0 # Jacobian for the observation
        self.z = 0 # Predicted location of the given landmark 
        self.K = 0 # Kalmann Gain
        self.conta_landmarks = 0 
        self.r = 0
        self.phi = 0

    def correct_prediction(self, Q, modelo, landmarks, idx_landmarks):
        k = 0
        for i in landmarks:
            j = idx_landmarks[k] 
            self.obs_predict(j, landmarks[k], modelo)
            self.KalmanGain(modelo, Q)
            land_range_bearing = np.array([self.r, self.phi])
            modelo.x = modelo.x + self.K @ (land_range_bearing - self.z)
            l,m = ((self.K @ self.H).shape)
            I = np.identity(l)
            modelo.sigma = (I - self.K @ self.H) @ modelo.sigma
            k = k + 1

    def obs_predict(self, j, land_pos, modelo): # Recebe a posicao das features ja no frame global 
        n = len(modelo.x)
        x = modelo.x[0]
        y = modelo.x[1]
        theta = modelo.x[2]

        land_x = land_pos[0] 
        land_y = land_pos[1]

        land_x_antigo = modelo.x[3 + 2*(j-1)]
        land_y_antigo = modelo.x[4 + 2*(j-1)]

        if (j > self.conta_landmarks):
            lx = land_x
            ly = land_y
            pos_land = np.array([lx, ly])
            self.conta_landmarks = self.conta_landmarks + 1
        else:
            pos_land = np.array([land_x, land_y])

        deltax = land_x_antigo - x
        deltay = land_y_antigo - y
        self.r = np.sqrt((deltax)**2 + (deltay)**2)
        self.phi = np.arctan2(deltay, deltax) - theta

        pos_mod = np.array([x,y])
        delta = np.array(pos_land - pos_mod)
        q = delta.T @ delta

        self.z = np.array([np.sqrt(q), np.arctan2(delta[0], delta[1]) - theta]) 

        h = 1/q * np.array(([-np.sqrt(q) * delta[0], -np.sqrt(q) * delta[1], 0, np.sqrt(q) * delta[0], np.sqrt(q) * delta[0]], 
                    [delta[1], -delta[0], -q, -delta[1], delta[0]]))
       
        F = np.zeros((5,n))
        j = int(j)
        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1
        F[3][3 + 2*(j-1)] = 1
        F[4][4 + 2*(j-1)] = 1
        self.H = h @ F 

    def KalmanGain(self, modelo, Q):
        sigma = modelo.sigma
        H = (self.H)
        self.K = sigma @ (H.T) @ np.linalg.inv(H @ sigma @ (H.T) + Q)

class modelo():
    def __init__(self, x0, dt, sigma0): # x = [x, y, theta], u = [velocity, angular_velocity] 
        self.x = x0 # State vector for the model, [x_coord, y_coord, theta, land_1_x, land_1_y, land_2_x, land_2_y ...]
        self.dt = dt # Time step 
        self.sigma = sigma0 # Covariances matrix 
        
    def move(self, u): # updates de pose and covariance of the robot for a given control input 
        n = len(self.x)
        F = np.zeros((3,n))
        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1

        dist  = u[0]*self.dt
        dth = u[1]*self.dt
        dx = F.T @ np.array([np.cos(self.x[2]+ dth)*dist, np.sin(self.x[2]+ dth)*dist, dth])
        self.x = np.add(self.x, dx)
        self.update_cov(u) 
    
    def update_cov(self, u): # updates de covariance matrix for a given input
        # This changes the uncertainty in the robot pose but not in the landmar position
        v = u[0]
        w = u[1]
        theta = self.x[2]
        dt = self.dt
        n = len(self.x)

        Res = np.zeros((n,n)) # FALTA COMPLETAR - User defined uncertainty in the range and bearing of the model 
        Res[0][0] = 1e-1
        Res[1][1] = 1e-1
        Res[2][2] = 1e-1

        G = np.identity(n)
        G[0][2] = -np.sin(w * dt + theta) * (v * dt)
        G[1][2] = np.cos(w * dt + theta) * (v * dt)
        self.sigma = G @ self.sigma @ (G.T) + Res 

if __name__ == '__main__':

    n = 2 # number of landmarks 

    # inicializa o estado inicial com n*2 landmark e pose (3)
    x0 = np.zeros(n*2 + 3)
    x0[0] = 0
    x0[1] = 0
    x0[2] = 0

    # Inicializar as incertezas 
    infinito = 10000
    sigma0 = np.identity(n*2 + 3) * infinito
    sigma0[0][0] = 0
    sigma0[1][1] = 0
    sigma0[2][2] = 0

    Q = np.identity(2) # uncertanty in the measurement, bearing and range 
    Q[0][0] = 1
    Q[1][1] = 1

    u = [1,0]
    dt = 1

    teste = modelo(x0, dt, sigma0)
    modelo.move(teste, u)
    teste_ekf = EKF()
    obs = np.array([[2, 2], [1,1]])
    teste_ekf.correct_prediction(Q, teste, obs, [1, 2])




