import sympy as sym


# Fossen p.20
def S(l):
    return sym.Matrix([
        [    0, -l[2],  l[1]],
        [ l[2],     0, -l[0]],
        [-l[1],  l[0],     0]
    ])


def J(state):
    phi, theta, psi = state[3:6]
    ans = sym.zeros(6, 6)
    s = lambda x: sym.sin(x)
    c = lambda x: sym.cos(x)
    t = lambda x: sym.tan(x)
    ans[0:3, 0:3] = sym.Matrix([
                    [c(psi)*c(theta), -s(psi)*c(phi) + c(psi)*s(theta)*s(phi), s(psi)*s(phi) + c(psi)*c(phi)*s(theta)],
                    [s(psi)*c(theta), c(psi)*c(phi) + s(phi)*s(theta)*s(psi), -c(psi)*s(phi) + s(theta)*s(psi)*c(theta)],
                    [-s(theta), c(theta)*s(phi), c(theta)*c(phi)]
                ])
    ans[3:6, 3:6] = sym.Matrix([
                    [1, s(phi)*t(theta), c(phi)*t(theta)],
                    [0, s(phi), -s(phi)],
                    [0, s(phi)/c(theta), c(phi)/c(theta)]
                ])
    return ans


### Rigid Body Kinetics ###
# Rigid Body mass, Fossen
def getMrb(m, xg, yg, zg, Ix, Iy, Iz, Ixy, Ixz, Iyz):
    return sym.Matrix([
            [    m,     0,     0,     0,  m*zg, -m*yg],
            [    0,     m,     0, -m*zg,     0,  m*xg],
            [    0,     0,     m,  m*yg, -m*xg,     0],
            [    0, -m*zg,  m*yg,    Ix,  -Ixy,  -Ixz],
            [ m*zg,     0, -m*xg,  -Ixy,    Iy,  -Iyz],
            [-m*yg,  m*xg,     0,  -Ixz,  -Iyz,    Iz]
        ])


# Coriolis force (Crb and Ca)
def getCoriolis(M, state):
    v1 = sym.Matrix(state[6:9])
    v2 = sym.Matrix(state[9:12])
    c12 = -S(M[0:3, 0:3]*v1 + M[0:3, 3:6]*v2)
    c22 = -S(M[3:6, 0:3]*v1 + M[3:6, 3:6]*v2)
    C = sym.eye(6)
    C[0:3, 0:3] = sym.Matrix(3, 3, lambda i, j: 0)
    C[0:3, 3:6] = sym.Matrix(3, 3, lambda i, j: c12[i, j])
    C[3:6, 0:3] = sym.Matrix(3, 3, lambda i, j: c12[i, j])
    C[3:6, 3:6] = sym.Matrix(3, 3, lambda i, j: c22[i, j])
    return C


### Hydrostatics ###
# Gravity
def getG(state, W, B, center_of_gravity, center_of_buoyancy):
    phi, theta = tuple(state[3:5])
    gx, gy, gz = center_of_gravity
    bx, by, bz = center_of_buoyancy
    return sym.Matrix([
        (W - B) * sym.sin(theta),
        -(W - B) * sym.cos(theta) * sym.sin(phi),
        -(W - B) * sym.cos(theta) * sym.cos(phi),
        -(gy * W - by * B) * sym.cos(theta) * sym.cos(phi) + (gz * W - bz * B) * sym.cos(theta) * sym.sin(phi),
        (gz * W - bz * B) * sym.sin(theta) + (gx * W - bx * B) * sym.cos(theta) * sym.cos(phi),
        -(gx * W - bx * B) * sym.cos(theta) * sym.sin(phi) - (gy * W - by * B) * sym.sin(theta)
    ])


### Hydrodynamics ###
# Linear Damping - need to implement
def getDLinear():
    return sym.zeros(6, 6)

# Quadratic Damping - need to implement
def getDQuad():
    return sym.zeros(6, 6)


# Added mass
def getMa(X_dot, Y_dot, Z_dot, K_dot, M_dot, N_dot):
    Ma = sym.eye(6)
    for i in range(6):
        Ma[0, i] = X_dot[i]
        Ma[1, i] = Y_dot[i]
        Ma[2, i] = Z_dot[i]
        Ma[3, i] = K_dot[i]
        Ma[4, i] = M_dot[i]
        Ma[5, i] = N_dot[i]
    return -Ma

def getSystem(var):
    M = getMa(var["Xdot"], var["Ydot"], var["Zdot"], var["Kdot"], var["Mdot"], var["Ndot"]) + \
        getMrb(var["m"], var["xg"], var["yg"], var["zg"], var["Ix"], var["Iy"], var["Iz"],
               var["Ixy"], var["Ixz"], var["Iyz"])
    C = getCoriolis(M, var["state"])
    D = getDLinear() + getDQuad()
    G = getG(var["state"], var["m"]*var["gravity"], var["rho"]*var["V"]*var["gravity"],
             var["center_of_gravity"], var["center_of_buoyancy"])

    f1 = sym.zeros(12, 12)
    f1[0:6, 6:12] = J(var["state"])
    f1[6:12, 6:12] = -M.inv()*(C + D)
    f2 = sym.zeros(12, 1)
    f2[6:12, 0] = -M.inv() * G
    f = f1*var["state"] + f2
    g = sym.zeros(12, 1)
    g[6:12, :] = M.inv()*var["thruster"]*var["output"]
    state_model = f + g

    # Linearize model
    return state_model.jacobian(var["state"]), state_model.jacobian(var["output"])


