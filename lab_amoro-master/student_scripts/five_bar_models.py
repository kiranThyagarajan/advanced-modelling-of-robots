from math import *
import numpy as np

# Geometric parameters
l = 0.09
d = 0.118
# Dynamic parameters
ZZ1R = 1.0 * 0.045 * 0.045
ZZ2R = 1.0 * 0.045 * 0.045
mR = 0.5


def dgm(q11, q21, assembly_mode):
    A22H = 0.5 * np.array([[- l * np.cos(q21) - d + l * np.cos(q11)], [-l * np.sin(q21) + l * np.sin(q11)]])
    OA21 = np.array([[0.5 * d], [0.0]])
    A21A22 = np.array([[l * np.cos(q21)], [l * np.sin(q21)]])
    a = 0.5 * d + l * np.cos(q21)
    h = np.sqrt(l*2 - a*2)
    HA13 = assembly_mode * (h/a) * (np.array([[0.0, -1.0], [1.0, 0.0]]) @ A22H)
    A22H = np.array([[-a], [0.0]])
    OA13 = OA21 + A21A22 + A22H + HA13
    x = OA13[0]
    y = OA13[1] 
    return x, y


def igm(x, y, gamma1, gamma2):
    qA11A13 = np.array([x + 0.5 * d, y])
    A11M1 = 0.5 * A11A13
    c = np.linalg.norm(A11M1)
    b = np.sqrt(l*2 - c*2)
    M1A12 = gamma1 * (b/c) * (np.array([[0, -1], [1.0, 0.0]]) @ A11M1)
    A11A12 = A11M1 + M1A12  
    q11 = np.arctan2(A11A12[1], A11A12[0])
    
    A21A13 = np.array([ x - d * 0.5, y])
    A21M2 = 0.5 * A21A13
    c = np.linalg.norm(A21M2)
    b = np.sqrt(l*2 - c*2)
    M2A22 = gamma2 * (b/c) * (np.array([[0, -1], [1.0, 0.0]]) @ A21M2)
    A21A22 = A21M2 + M2A22
    q21 = np.arctan2(A21A22[1], A21A22[0])
    return q11, q21


def dgm_passive(q11, q21, assembly_mode):
    x,y = dgm(q11, q21, assembly_mode)
    q12 = np.arctan2(y/l - np.sin(q11), x/l + d/(2*l) - np.cos(q11)) - q11
    q22 = np.arctan2(y/l - np.sin(q21), x/l + d/(2*l) - np.cos(q21)) - q21
    return q12, q22


# You can create intermediate functions to avoid redundant code
def compute_A_B(q11, q12, q21, q22, ):
    A = 0
    B = 0
    return A, B


def dkm(q11, q12, q21, q22, q11D, q21D):
    xD = 0
    yD = 0
    return xD, yD


def ikm(q11, q12, q21, q22, xD, yD):
    q11D = 0
    q21D = 0
    return q11D, q21D


def dkm_passive(q11, q12, q21, q22, q11D, q21D, xD, yD):
    q12D = 0
    q22D = 0
    return q12D, q22D


def dkm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, q11DD, q21DD):
    xDD = 0
    yDD = 0
    return xDD, yDD


def ikm2(q11, q12, q21, q22, q11D, q12D, q21D, q22D, xDD, yDD):
    q11DD = 0
    q21DD = 0
    return q11DD, q21DD


def dkm2_passive(q11, q12, q21, q22, q11D, q12D, q21D, q22D, q11DD, q21DD, xDD, yDD):
    q12DD = 0
    q22DD = 0
    return q12DD, q22DD


def dynamic_model(q11, q12, q21, q22, q11D, q12D, q21D, q22D):
    M = np.zeros((2,2))
    c = 0
    return M, c
