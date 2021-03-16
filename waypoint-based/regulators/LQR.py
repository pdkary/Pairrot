import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
"""
This is a Finite Horizon Linear Quadratic Regulator

Uses a state space system of state X (nx1), and inputs U (ux,uy,uz)
in the below example ux,uy,uz are accelerations

X(n+1) = AX + B0*ux + B1*uy + B2*uz
X(n+1) = AX + [B0 B1 B2]*[ux uy uz]^T
"""
class FiniteHorizonLQR():
    def __init__(self,A,B,Q,R,N):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.N = N
        self.beta = B.shape[1] # number of inputs
        self.T = self.get_T(A,N)
        self.S  = self.get_S(A,B,N)

        self.Q_hat = self.get_Q_or_R_hat(Q,N)
        self.R_hat = self.get_Q_or_R_hat(R,N)

    def get_k(self):
        sTQ = np.matmul(self.S.transpose(),self.Q_hat)
        sTQT = np.matmul(sTQ,self.T)
        sTQS = self.R_hat + np.matmul(sTQ,self.S)

        inv_StQS = np.linalg.inv(sTQS)
        return -1*np.matmul(inv_StQS,sTQT)
    
    def get_S(self,A,B,N):
        
        ## A := (nxn)
        ## B := (nxbeta)
        ## (A^n)*B will be nxbeta
        ## output will be (N*n,N*beta)
        ele_rows = A.shape[0]
        ele_cols = B.shape[1]
        S_rows = N*ele_rows
        S_cols = N*self.beta
        S = np.ndarray(shape=(S_rows,S_cols),dtype=A.dtype)
        for i in range(N):
            for j in range(N):
                looking = S[i*ele_rows:(i+1)*ele_rows,j*ele_cols:(j+1)*ele_cols]
                if i < j:
                    S[i*ele_rows:(i+1)*ele_rows,j*ele_cols:(j+1)*ele_cols] = np.zeros_like(B)
                elif i == j:
                    S[i*ele_rows:(i+1)*ele_rows,j*ele_cols:(j+1)*ele_cols] = B
                else:
                    v = np.matmul(np.linalg.matrix_power(A,i-j),B)
                    S[i*ele_rows:(i+1)*ele_rows,j*ele_cols:(j+1)*ele_cols] = v
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
        T = np.ndarray(shape=(T_rows,T_cols),dtype=A.dtype)
        for i in range(N):
            T[i*A_rows:(i+1)*A_rows,0:A_cols] = np.linalg.matrix_power(A,i)
        return T


"""
This test is to test the system in which the controller can control acceleration in 3 axes
"""
if __name__ == '__main__':
    T = 1/10
    hT2 = (.5)*T*T
    A = np.array(
        [
            [1,T,hT2,0,0,0,0,0,0],
            [0,1,T,0,0,0,0,0,0],
            [0,0,0,1,0,0,0,0,0],
            [0,0,0,1,T,hT2,0,0,0],
            [0,0,0,0,1,T,0,0,0],
            [0,0,0,0,0,0,1,0,0],
            [0,0,0,0,0,0,1,T,hT2],
            [0,0,0,0,0,0,0,1,T],
            [0,0,0,0,0,0,0,0,1]
        ]
    )
    B = np.array(
        [
            [0,0,0],
            [0,0,0],
            [1,0,0],
            [0,0,0],
            [0,0,0],
            [0,1,0],
            [0,0,0],
            [0,0,0],
            [0,0,1]
            ]
    )

    C = np.array([[1,0,0,0,0,0,0,0,0],[0,0,0,1,0,0,0,0,0],[0,0,0,0,0,0,1,0,0]])

    Q = np.array([
        [100,0,0,0,0,0,0,0,0],
        [0,50,0,0,0,0,0,0,0],
        [0,0,1,0,0,0,0,0,0],
        [0,0,0,100,0,0,0,0,0],
        [0,0,0,0,50,0,0,0,0],
        [0,0,0,0,0,1,0,0,0],
        [0,0,0,0,0,0,100,0,0],
        [0,0,0,0,0,0,0,50,0],
        [0,0,0,0,0,0,0,0,1]])

    R = np.array([[1,0,0],[0,1,0],[0,0,1]])
    N=100

    LQR = FiniteHorizonLQR(A,B,Q,R,N)

    k = LQR.get_k()
    # state0 = np.array([[0],[0],[0],[10],[0],[0],[0],[0],[0]]) #100m in x direction
    # state0 = np.array([[0],[0],[0],[100],[0],[0],[0],[0],[0]]) #100m in y direction
    # state0 = np.array([[0],[0],[0],[0],[0],[0],[100],[0],[0]]) #100m in z direction
    state0 = np.array([[100],[0],[0],[100],[0],[0],[100],[0],[0]]) #100m in all direction
    moves = np.reshape(np.matmul(k,state0),newshape=(N,3))

    states = [state0]
    ys = [np.matmul(C,state0)]
    for i in range(500):
        ax = np.matmul(A,states[-1])
        u = np.matmul(k,states[-1])
        u = np.reshape(u,newshape=(N,3))
        bu = np.matmul(B,u[0])
        bu = np.reshape(bu,newshape=(9,1))
        ax_bu = ax + bu
        states.append(ax_bu)
        y = np.matmul(C,ax_bu)
        ys.append(y)
        if np.all(y<=0.001):
            break

    for p in ys:
        print("({},{},{})".format(round(p[0][0],2),round(p[1][0],2),round(p[2][0],2)))

    x = [l[0][0] for l in ys]
    y = [l[1][0] for l in ys]
    z = [l[2][0] for l in ys]
    
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter3D(x,y,z)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.savefig("lqr_test_position.png")
