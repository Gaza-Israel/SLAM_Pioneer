from ctypes import sizeof
import numpy as np

class EKF():
    def __init__(self):
        self.H = 0
        self.z = 0
        self.K = 0
        self.conta_landmarks = 0

    def predict(self,u,modelo):
        pass
    def correct_prediction(self,landmarks,idx_landmarks,modelo):
        pass


    def atualiza_landmarks(self, land_indices, land_pos, modelo):
        k = 0
        for i in land_indices:
            self.obs_predict(i, land_pos[k], modelo)
            k = k + 1

    def obs_predict(self, j, land_pos, modelo): # Range-bearing observation [x,y] * n landmarks que observa nesse momento 
        n = len(modelo.x)
        x = modelo.x[0]
        y = modelo.x[1]
        theta = modelo.x[2]

        if (j > self.conta_landmarks):
            deltax = (x-land_pos[0])
            deltay = (y-land_pos[1])
            r = np.sqrt(deltax**2 + deltay**2)
            phi = np.arctan2(deltay,deltax)

            lx = x + r * np.cos(phi + theta)
            ly = y + r * np.sin(phi + theta)
            pos_land = np.array([lx, ly])
            self.conta_landmarks = self.conta_landmarks + 1
        else:
            pos_land = np.array([land_pos[0], land_pos[1]])

        pos_mod = np.array([x,y])
        delta = np.array(pos_land - pos_mod)
        q = delta.T @ delta

        self.z = np.array([np.sqrt(q), np.arctan2(delta[0], delta[1]) - theta])

        h = 1/q * np.array(([-np.sqrt(q) * delta[0], -np.sqrt(q) * delta[1], 0, np.sqrt(q) * delta[0], np.sqrt(q) * delta[0]], 
                    [delta[1], -delta[0], -q, -delta[1], delta[0]]))
       
       
        F = np.zeros((5,n))
        F[0][0] = 1
        F[1][1] = 1
        F[2][2] = 1
        F[3][3 + 2*(j-1)] = 1
        F[4][4 + 2*(j-1)] = 1
        self.H = h @ F 

    def KalmanGain(self, modelo, Q):
        sigma = modelo.sigma
        H = self.H
        Q = (H @ sigma @ (H.T)) * 0.0001
        self.K = sigma @ (H.T) @ np.linalg.inv(H @ sigma @ (H.T) + Q)

    def correct_prediction(self, modelo, obs):
        x = modelo.x + self.K @ (obs - self.z)
        l,m = ((self.K @ self.H).shape)
        I = np.identity(l)
        sigma = (I - self.K @ self.H) @ modelo.sigma
        return(x,sigma)

class modelo():
    def __init__(self, x0, dt, sigma0): # x = [x, y, theta], u = [velocity, angular_velocity] 
        self.x = x0
        self.dt = dt
        self.sigma = sigma0
        
    def move(self, u):
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
    
    def update_cov(self, u):
        v = u[0]
        w = u[1]
        theta = self.x[2]
        dt = self.dt
        n = len(self.x)

        Res = np.zeros((n,n)) # FALTA COMPLETAR 
        Res[0][0] = 0.001
        Res[1][1] = 0.001
        Res[2][2] = 0.001

        G = np.identity(n)
        G[0][2] = -np.sin(w * dt + theta) * (v * dt)
        G[1][2] = np.cos(w * dt + theta) * (v * dt)
        self.sigma = G @ self.sigma @ (G.T) + Res 
if __name__ == '__main__':
    n = 2 # number of landmarks 
    infinito = 10000

    x0 = np.zeros(n*2 + 3)
    x0[0] = 0
    x0[1] = 0
    x0[2] = 0

    sigma0 = np.identity(n*2 + 3) * 0.001 
    Q = np.identity(n*2 + 3) # uncertanty in the measurement, bearing and range 
    Q[0][0] = 0.001
    Q[1][1] = 0.001

    u = [1,0]
    dt = 1
    teste = modelo(x0, dt, sigma0)
    modelo.move(teste, u)
    teste_ekf = EKF()
    obs = np.array([2, 2])
    teste_ekf.atualiza_landmarks([1,2], [[1,1],[2,2]], teste)
    teste_ekf.KalmanGain(teste, Q)
    print(teste_ekf.correct_prediction(teste, obs))


    #     class modelo:
    #     measure
        
    #     kalman.predict()
    #     feture.extract()
    #     feature.match()
    #     kalma.correct()



