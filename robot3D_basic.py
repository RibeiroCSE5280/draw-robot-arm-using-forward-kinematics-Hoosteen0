#!/usr/bin/env python
# coding: utf-8


from vedo import *
from functools import reduce


def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)

    rotation_matrix = np.array([[0, 0, 0],
                                [0, 0, 0],
                                [0, 0, 0]])
    if axis_name == 'x':
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, c, -s],
                                    [0, s, c]])
    if axis_name == 'y':
        rotation_matrix = np.array([[c, 0, s],
                                    [0, 1, 0],
                                    [-s, 0, c]])
    elif axis_name == 'z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s, c, 0],
                                    [0, 0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1

    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)

    originDot = Sphere(pos=[0, 0, 0],
                       c="black",
                       r=0.10)

    # Combine the axes together to form a frame as a single mesh object
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot

    return F


def getLocalFrameMatrix(R_ij, t_ij):
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij, t_ij],
                     [np.zeros((1, 3)), 1]])

    return T_ij


def forward_kinematics(phi, length_list):
    # returns the translation component of T_05

    # origin of robot base
    base = np.array([[3], [2], [0.0]])

    # construct the individual transformations
    transformations = []
    for i in range(len(length_list)):
        rot_mat = RotationMatrix(phi[i], 'z')
        translation_mat = base
        if i != 0:
            translation_mat = np.array([[length_list[i-1]], [0.0], [0.0]])
        transformation_mat = getLocalFrameMatrix(rot_mat, translation_mat)
        transformations.append(transformation_mat)

    # compute frames in world coordinates
    frames_0_i = []
    for i in range(2, len(length_list) + 2):
        frames_0_i.append(reduce(np.matmul, transformations[0:i-1]))
    return frames_0_i


def end_effector_location(phi, length_list):
    frames = forward_kinematics(phi, length_list)
    p = frames[-1][:, -1]
    return p


def draw_parts(length_list, phi):
    # Set the limits of the graph x, y, and z ranges
    axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))

    transformation_matrices = forward_kinematics(phi, length_list)
    frames = []
    for i in range(len(transformation_matrices)):
        current_frame_arrows = createCoordinateFrameMesh()
        link_mesh = Cylinder(r=0.4,
                             height=length_list[i],
                             pos=(length_list[i] / 2, 0, 0),
                             c="red",
                             alpha=.8,
                             axis=(1, 0, 0)
                             )
        if i == 0:
            sphere = Sphere(r=0.4).pos(-0.4, 0, 0).color("gray").alpha(.8)
            current_frame = current_frame_arrows + link_mesh + sphere
        else:
            current_frame = current_frame_arrows + link_mesh
        current_frame.apply_transform(transformation_matrices[i])
        frames.append(current_frame)

    # draw the end effector
    Frame4 = createCoordinateFrameMesh()

    # Transform the part to position it at its correct location and orientation
    Frame4.apply_transform(transformation_matrices[-1])
    frames.append(Frame4)

    show(frames, axes, viewup="z").close()

def original_draw():
    # Set the limits of the graph x, y, and z ranges
    axes = Axes(xrange=(0, 20), yrange=(-2, 10), zrange=(0, 6))

    # Lengths of arm parts
    L1 = 5  # Length of link 1
    L2 = 8  # Length of link 2
    L3 = 5  # link 3

    # Joint angles
    phi1 = 30  # Rotation angle of part 1 in degrees
    phi2 = -10  # Rotation angle of part 2 in degrees
    phi3 = 20  # link 3 in degrees
    phi4 = 0  # Rotation angle of the end-effector in degrees

    # Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame)
    R_01 = RotationMatrix(phi1, axis_name='z')  # Rotation matrix
    p1 = np.array([[3], [2], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_01 = p1  # Translation vector

    T_01 = getLocalFrameMatrix(R_01, t_01)  # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)

    # Create the coordinate frame mesh and transform
    Frame1Arrows = createCoordinateFrameMesh()

    # Now, let's create a cylinder and add it to the local coordinate frame
    link1_mesh = Cylinder(r=0.4,
                          height=L1,
                          pos=(L1 / 2, 0, 0),
                          c="yellow",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Also create a sphere to show as an example of a joint
    r1 = 0.4
    sphere1 = Sphere(r=r1).pos(-r1, 0, 0).color("gray").alpha(.8)

    # Combine all parts into a single object
    Frame1 = Frame1Arrows + link1_mesh + sphere1

    # Transform the part to position it at its correct location and orientation
    Frame1.apply_transform(T_01)

    # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame)
    R_12 = RotationMatrix(phi2, axis_name='z')  # Rotation matrix
    p2 = np.array([[L1], [0.0], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_12 = p2  # Translation vector

    # Matrix of Frame 2 w.r.t. Frame 1
    T_12 = getLocalFrameMatrix(R_12, t_12)

    # Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12

    # Create the coordinate frame mesh and transform
    Frame2Arrows = createCoordinateFrameMesh()

    # Now, let's create a cylinder and add it to the local coordinate frame
    link2_mesh = Cylinder(r=0.4,
                          height=L2,
                          pos=(L2 / 2, 0, 0),
                          c="red",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Combine all parts into a single object
    Frame2 = Frame2Arrows + link2_mesh

    # Transform the part to position it at its correct location and orientation
    Frame2.apply_transform(T_02)

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_23 = RotationMatrix(phi3, axis_name='z')  # Rotation matrix
    p3 = np.array([[L2], [0.0], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_23 = p3  # Translation vector

    # Matrix of Frame 3 w.r.t. Frame 2
    T_23 = getLocalFrameMatrix(R_23, t_23)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_01 @ T_12 @ T_23

    # Create the coordinate frame mesh and transform
    Frame3Arrows = createCoordinateFrameMesh()

    # Now, let's create a cylinder and add it to the local coordinate frame
    link3_mesh = Cylinder(r=0.4,
                          height=L3,
                          pos=(L3 / 2, 0, 0),
                          c="blue",
                          alpha=.8,
                          axis=(1, 0, 0)
                          )

    # Combine all parts into a single object
    Frame3 = Frame3Arrows + link3_mesh

    # Transform the part to position it at its correct location and orientation
    Frame3.apply_transform(T_03)

    # Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame)
    R_34 = RotationMatrix(phi4, axis_name='z')  # Rotation matrix
    p4 = np.array([[L3], [0.0], [0.0]])  # Frame's origin (w.r.t. previous frame)
    t_34 = p4  # Translation vector

    # Matrix of Frame 3 w.r.t. Frame 2
    T_34 = getLocalFrameMatrix(R_34, t_34)

    # Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_04 = T_01 @ T_12 @ T_23 @ T_34

    # Create the coordinate frame mesh and transform. This point is the end-effector. So, I am
    # just creating the coordinate frame.
    Frame4 = createCoordinateFrameMesh()

    # Transform the part to position it at its correct location and orientation
    Frame4.apply_transform(T_04)

    # Show everything
    show([Frame1, Frame2, Frame3, Frame4], axes, viewup="z").close()

def main():
    # length list
    length_list = [5, 8, 5]
    # run the program several times to see an animation
    for i in range(7):
        turn = i*5
        phi = np.array([30 + turn, -10 + turn, 20 + turn, 0 + turn])
        draw_parts(length_list=length_list, phi=phi)
    # original_draw()


if __name__ == '__main__':
    main()
