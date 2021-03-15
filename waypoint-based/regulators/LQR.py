import numpy as np

def get_S(A,B,N):
    A_rows = A.shape[0]
    B_cols = B.shape[1]
    
    new_N = N*A_rows*A_cols
    S = np.array(shape=(new_N,new_N),dtype=A.dtype)
    for i in range(N):
        for j in range(N):


def get_QR_hat(Q,N):
    def get_QR_row(i):
        return [0 if j != i else Q for j in range(N)]
    return np.array([get_QR_row(i) for i in range(N)])


class FiniteHorizonLQR():
    def __init__(self,A,B,Q,R,N):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.N = N
        self.T = np.array([np.linalg.matrix_power(A,n) for n in range(N)])
        self.S  = get_S(A,B,N)

        self.Q_hat = get_QR_hat(Q,N)
        self.R_hat = get_QR_hat(R,N)

        print(self.T)
        print(self.S)
        print(self.Q_hat)
        print(self.R_hat)

    def get_k(self):
        STQT = self.S.transpose()*self.Q_hat*self.T
        R_SQS = self.R_hat + self.S.transpose()*self.Q_hat*self.S
        return -STQT*np.linalg.inv(R_SQS)

"""test
state = [y y']
A = [[1 1],[0,1]]
B = [[0],[1]] (can only affect velocity)

Q = [[1 0],[0,10]] penalize velocity over position
R = [[0],[1]] only penalize velocity change

N = 10
"""
if __name__ == '__main__':
    A = np.array([[1,1],[0,1]])
    B = np.array([[0],[1]])

    Q = np.array([[1,0],[0,10]])
    R = np.array([[0],[1]])
    N=10

    fhLQR = FiniteHorizonLQR(A,B,Q,R,N)

    print(fhLQR.get_k())
    



