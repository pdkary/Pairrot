import numpy as np
from matplotlib import pyplot as plt
class FiniteHorizonLQR():
    def __init__(self,A,B,Q,R,N):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.N = N
        self.T = self.get_T(A,N)
        self.S  = self.get_S(A,B,N)

        self.Q_hat = self.get_Q_or_R_hat(Q,N)
        self.R_hat = self.get_Q_or_R_hat(R,N)

    def get_k(self):
        StQT = self.S.transpose().dot(self.Q_hat).dot(self.T)
        StQS = self.R_hat + self.S.transpose().dot(self.Q_hat).dot(self.S)
        inv_StQS = np.linalg.inv(StQS)
        return -1*inv_StQS.dot(StQT)
    
    def get_S(self,A,B,N):
        A_rows = A.shape[0]
        ## (A^n)*B will be 1xArows
        ## output will be Nx(N*Arows)
        N_rows = N*A_rows
        N_cols = N
        S = np.ndarray(shape=(N_rows,N_cols),dtype=A.dtype)
        for i in range(N):
            for j in range(N):
                looking = S[i*A_rows:(i+1)*A_rows,j:j+1]
                if i < j:
                    S[i*A_rows:(i+1)*A_rows,j:j+1] = np.zeros_like(B)
                elif i == j:
                    S[i*A_rows:(i+1)*A_rows,j:j+1] = B
                else:
                    v = np.linalg.matrix_power(A,i-j).dot(B)
                    S[i*A_rows:(i+1)*A_rows,j:j+1] = v
        return S

    def get_Q_or_R_hat(self,Q,N):
        Q_rows = Q.shape[0]
        Q_cols = Q.shape[1]
        new_Q_rows = N*Q_rows
        new_Q_cols = N*Q_cols
        new_Q = np.ndarray(shape=(new_Q_rows,new_Q_cols),dtype=Q.dtype)
        for i in range(N):
            for j in range(N):
                if i==j:
                    new_Q[i*Q_rows:(i+1)*Q_rows,j*(Q_cols):(j+1)*Q_cols] = Q
                else:
                    new_Q[i*Q_rows:(i+1)*Q_rows,j*(Q_cols):(j+1)*Q_cols] = np.zeros_like(Q)
        return new_Q

    def get_T(self,A,N):
        A_rows = A.shape[0]
        A_cols = A.shape[1]
        T_rows = N*A_rows
        T_cols = A_rows
        T = np.ndarray(shape=(T_rows,T_cols),dtype=Q.dtype)
        for i in range(N):
            T[i*A_rows:(i+1)*A_rows,0:A_cols] = np.linalg.matrix_power(A,i)
        return T

"""test
state = [y y']
A = [[1 1],[0,1]]
B = [[0],[1]] (can only affect velocity)

Q = [[1 0],[0,10]] penalize velocity over position
R = [1] only penalize velocity change

N = 10
"""
if __name__ == '__main__':
    A = np.array([[1,1],[0,1]])
    B = np.array([[0],[1]])

    Q = np.array([[1,0],[0,10]])
    R = np.array([[1]])
    N=20

    fhLQR = FiniteHorizonLQR(A,B,Q,R,N)

    k = fhLQR.get_k()

    state0 = np.array([[100],[0]]) #100m above ground

    moves = k.dot(state0)
    s  = [state0]
    for m in moves:
        s.append(A.dot(s[-1]) +m[0]*B)

    y = [l[0] for l in s]
    yp = [l[1] for l in s]

    plt.plot(list(range(N+1)),y)
    plt.savefig("lqr_test_position.png")
    
    plt.plot(list(range(N+1)),yp)
    plt.savefig("lqr_test_velocity.png")
