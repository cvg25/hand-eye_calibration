import numpy as np

def degrees_to_radians(degrees):
    """
    Converts degrees to radians
    @param: degrees
    """
    radians = (degrees/360.0)*2*np.pi
    return radians

def V2T( V ):
    """
    V2T converts 1x6 vector into 4x4 transformation matrix
    @param: V - 1x6 vector of form [x,y,z,rx,ry,rz] where x,y,z is the translation
    and rx,ry,rz is an angle-axis representation of the angle where the
    unit vector representing the axis has been multipled by the angle of
    rotation about it.
    @returns: T - a standard 4x4 transformation matrix
    """
    assert(V.shape == (1,6))
    T = np.eye(4)
    T[0:3, 3] = V[0, 0:3]
    T[0:3, 0:3] = V2R(V[0,3:6])
    return T

def V2R( V ):
    """
    V2R converts a 1x3 angle-axis vector into a 3x3 rotation matrix
    @param: V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis
    representation of the angle where the unit vector representing the axis
    has been multipled by the angle of rotation about it.
    @returns: R - a standard 3x3 transformation matrix
    """
    V = V.transpose()
    s = np.linalg.norm(V)
    if s == 0:
        R = eye(3)
    else:
        V = V[:] / s
        V = V.reshape(3,1)
        V = np.insert(V, 3, s)
        R = vrrotvec2mat(V)

    return R

def T2V( T ):
    """
    T2V converts 4x4 transformation matrix into a 1x6 vector
    @param: T - a standard 4x4 transformation matrix
    @returns:V - 1x6 vector of form [x,y,z,rx,ry,rz] where x,y,z is the translation
    and rx,ry,rz is an angle-axis representation of the angle where the
    unit vector representing the axis has been multipled by the angle of
    rotation about it
    """
    assert(T.shape == (4,4))
    V = np.zeros((1,6))
    V[0, 0:3] = T[0:3, 3]
    V[0, 3:6] = R2V(T[0:3, 0:3])

    return V

def R2V( R ):
    """
    R2V converts 3x3 rotation matrix into a 1x3 angle-axis vector
    @param: R - a standard 3x3 transformation matrix
    @returns: V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis
    representation of the angle where the unit vector representing the axis
    has been multipled by the angle of rotation about it
    """
    assert(R.shape == (3,3))
    R = vrrotmat2vec(R)
    V = R[0, 0:3] * R[0, 2]

    return V

def vrrotvec2mat(ax_ang):
    """
    Create a rotation matrix corresponding to the rotation around a general
    axis by a specified angle.
    """
    if ax_ang.ndim == 1:
        if np.size(ax_ang) == 5:
            ax_ang = np.reshape(ax_ang, (5, 1))
            msz = 1
        elif np.size(ax_ang) == 4:
            ax_ang = np.reshape(np.hstack((ax_ang, np.array([1]))), (5, 1))
            msz = 1
        else:
            raise Exception('Wrong Input Type')
    elif ax_ang.ndim == 2:
        if np.shape(ax_ang)[0] == 5:
            msz = np.shape(ax_ang)[1]
        elif np.shape(ax_ang)[1] == 5:
            ax_ang = ax_ang.transpose()
            msz = np.shape(ax_ang)[1]
        else:
            raise Exception('Wrong Inpuqt Type')
    else:
        raise Exception('Wrong Input Type')

    direction = ax_ang[0:3, :]
    angle = ax_ang[3, :]

    d = np.array(direction, dtype=np.float64)
    d /= np.linalg.norm(d, axis=0)
    x = d[0, :]
    y = d[1, :]
    z = d[2, :]
    c = np.cos(angle)
    s = np.sin(angle)
    tc = 1 - c

    mt11 = tc*x*x + c
    mt12 = tc*x*y - s*z
    mt13 = tc*x*z + s*y

    mt21 = tc*x*y + s*z
    mt22 = tc*y*y + c
    mt23 = tc*y*z - s*x

    mt31 = tc*x*z - s*y
    mt32 = tc*y*z + s*x
    mt33 = tc*z*z + c

    mtx = np.column_stack((mt11, mt12, mt13, mt21, mt22, mt23, mt31, mt32, mt33))

    inds1 = np.where(ax_ang[4, :] == -1)
    mtx[inds1, :] = -mtx[inds1, :]

    if msz == 1:
        mtx = mtx.reshape(3, 3)
    else:
        mtx = mtx.reshape(msz, 3, 3)

    return mtx


def vrrotmat2vec(mat1, rot_type='proper'):
    """
    Create an axis-angle np.array from Rotation Matrix:
    ====================

    @param mat:  The nx3x3 rotation matrices to convert
    @type mat:   nx3x3 numpy array

    @param rot_type: 'improper' if there is a possibility of
                      having improper matrices in the input,
                      'proper' otherwise. 'proper' by default
    @type  rot_type: string ('proper' or 'improper')

    @return:    The 3D rotation axis and angle (ax_ang)
                5 entries:
                   First 3: axis
                   4: angle
                   5: 1 for proper and -1 for improper
    @rtype:     numpy 5xn array

    """
    mat = np.copy(mat1)
    if mat.ndim == 2:
        if np.shape(mat) == (3, 3):
            mat = np.copy(np.reshape(mat, (1, 3, 3)))
        else:
            raise Exception('Wrong Input Typef')
    elif mat.ndim == 3:
        if np.shape(mat)[1:] != (3, 3):
            raise Exception('Wrong Input Typez')
    else:
        raise Exception('Wrong Input Type')

    msz = np.shape(mat)[0]
    ax_ang = np.zeros((5, msz))

    epsilon = 1e-12
    if rot_type == 'proper':
        ax_ang[4, :] = np.ones(np.shape(ax_ang[4, :]))
    elif rot_type == 'improper':
        for i in range(msz):
            det1 = np.linalg.det(mat[i, :, :])
            if abs(det1 - 1) < epsilon:
                ax_ang[4, i] = 1
            elif abs(det1 + 1) < epsilon:
                ax_ang[4, i] = -1
                mat[i, :, :] = -mat[i, :, :]
            else:
                raise Exception('Matrix is not a rotation: |det| != 1')
    else:
        raise Exception('Wrong Input parameter for rot_type')



    mtrc = mat[:, 0, 0] + mat[:, 1, 1] + mat[:, 2, 2]


    ind1 = np.where(abs(mtrc - 3) <= epsilon)[0]
    ind1_sz = np.size(ind1)
    if np.size(ind1) > 0:
        ax_ang[:4, ind1] = np.tile(np.array([0, 1, 0, 0]), (ind1_sz, 1)).transpose()


    ind2 = np.where(abs(mtrc + 1) <= epsilon)[0]
    ind2_sz = np.size(ind2)
    if ind2_sz > 0:
        # phi = pi
        # This singularity requires elaborate sign ambiguity resolution

        # Compute axis of rotation, make sure all elements >= 0
        # real signs are obtained by flipping algorithm below
        diag_elems = np.concatenate((mat[ind2, 0, 0].reshape(ind2_sz, 1),
                                     mat[ind2, 1, 1].reshape(ind2_sz, 1),
                                     mat[ind2, 2, 2].reshape(ind2_sz, 1)), axis=1)
        axis = np.sqrt(np.maximum((diag_elems + 1)/2, np.zeros((ind2_sz, 3))))
        # axis elements that are <= epsilon are set to zero
        axis = axis*((axis > epsilon).astype(int))

        # Flipping
        #
        # The algorithm uses the elements above diagonal to determine the signs
        # of rotation axis coordinate in the singular case Phi = pi.
        # All valid combinations of 0, positive and negative values lead to
        # 3 different cases:
        # If (Sum(signs)) >= 0 ... leave all coordinates positive
        # If (Sum(signs)) == -1 and all values are non-zero
        #   ... flip the coordinate that is missing in the term that has + sign,
        #       e.g. if 2AyAz is positive, flip x
        # If (Sum(signs)) == -1 and 2 values are zero
        #   ... flip the coord next to the one with non-zero value
        #   ... ambiguous, we have chosen shift right

        # construct vector [M23 M13 M12] ~ [2AyAz 2AxAz 2AxAy]
        # (in the order to facilitate flipping):    ^
        #                                  [no_x  no_y  no_z ]

        m_upper = np.concatenate((mat[ind2, 1, 2].reshape(ind2_sz, 1),
                                  mat[ind2, 0, 2].reshape(ind2_sz, 1),
                                  mat[ind2, 0, 1].reshape(ind2_sz, 1)), axis=1)

        # elements with || smaller than epsilon are considered to be zero
        signs = np.sign(m_upper)*((abs(m_upper) > epsilon).astype(int))

        sum_signs = np.sum(signs, axis=1)
        t1 = np.zeros(ind2_sz,)
        tind1 = np.where(sum_signs >= 0)[0]
        t1[tind1] = np.ones(np.shape(tind1))

        tind2 = np.where(np.all(np.vstack(((np.any(signs == 0, axis=1) == False), t1 == 0)), axis=0))[0]
        t1[tind2] = 2*np.ones(np.shape(tind2))

        tind3 = np.where(t1 == 0)[0]
        flip = np.zeros((ind2_sz, 3))
        flip[tind1, :] = np.ones((np.shape(tind1)[0], 3))
        flip[tind2, :] = np.copy(-signs[tind2, :])

        t2 = np.copy(signs[tind3, :])

        shifted = np.column_stack((t2[:, 2], t2[:, 0], t2[:, 1]))
        flip[tind3, :] = np.copy(shifted + (shifted == 0).astype(int))

        axis = axis*flip
        ax_ang[:4, ind2] = np.vstack((axis.transpose(), np.pi*(np.ones((1, ind2_sz)))))

    ind3 = np.where(np.all(np.vstack((abs(mtrc + 1) > epsilon, abs(mtrc - 3) > epsilon)), axis=0))[0]
    ind3_sz = np.size(ind3)
    if ind3_sz > 0:
        phi = np.arccos((mtrc[ind3]-1)/2)
        den = 2*np.sin(phi)
        a1 = (mat[ind3, 2, 1]-mat[ind3, 1, 2])/den
        a2 = (mat[ind3, 0, 2]-mat[ind3, 2, 0])/den
        a3 = (mat[ind3, 1, 0]-mat[ind3, 0, 1])/den
        axis = np.column_stack((a1, a2, a3))
        ax_ang[:4, ind3] = np.vstack((axis.transpose(), phi.transpose()))

    return ax_ang
