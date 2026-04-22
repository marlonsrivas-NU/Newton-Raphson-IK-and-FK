import mujoco
import mujoco.viewer
import numpy as np
from matplotlib import pyplot as plt
import modern_robotics as mr

np.set_printoptions(precision=4, suppress=True, linewidth=120)
DEBUG = 1

#Panda Joint Limits
q_min = np.array([
    -2.8973,   # joint1
    -1.7628,   # joint2
    -2.8973,   # joint3
    -3.0718,   # joint4
    -2.8973,   # joint5
    -0.0175,   # joint6
    -2.8973    # joint7
])

q_max = np.array([
     2.8973,   # joint1
     1.7628,   # joint2
     2.8973,   # joint3
    -0.0698,   # joint4
     2.8973,   # joint5
     3.7525,   # joint6
     2.8973    # joint7
])


#Load Mujoco model from XML
# MjModel contains the model description (geometry, joints, actuators, etc)
model = mujoco.MjModel.from_xml_path(r"C:\Users\marli\OneDrive\Escritorio\Grad_2026\RobMechControls\ME5250-RMC\project2\franka_emika_panda/mjx_scene.xml")

# MjData contains the simulation state (pos, vels, forces, etc)
data = mujoco.MjData(model)

# Set the simulation state to the default config
mujoco.mj_resetData(model,data)

print(f"Number of degrees of freedom (DOF): {model.nv}")
print(f"Number of actuators: {model.nu}")
print(f"Number of bodies: {model.nbody}")

mujoco.mj_forward(model,data)

print(f"Number of qpos: {model.nq}")
print(f"Number of joints: {model.njnt}")
print(f"Number of sites: {model.nsite}")


#Reset and set a specific configuration
mujoco.mj_resetData(model,data)

#Get EE site ID
site_id = model.site('gripper').id

#Compute forward kinematics
mujoco.mj_forward(model,data)

ee_pos = data.site_xpos[site_id]
ee_rot = data.site_xmat[site_id].reshape(3,3)


if DEBUG == 1:
    print(f"\tEE position: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
    print(f"EE Rotation: [{ee_rot}]")

def Get_Screw_axis(n):
    #Get body id for link
    link =f"link{n}"
    body_id = model.body(link).id
    #print(f"\nlink{n} id: {body_id}")

    #obtain world position for Link1
    L = data.xpos[body_id]
    #print(f"Link{n}= {L}")

    omega_local = [0,0,1]

    #Get the world orientation of Link1
    R = data.xmat[body_id].reshape(3,3)
    #print(f"\nRotation:\n {R}")
    #Rotate local frame to world coordinates
    omega_world = R @ omega_local
    #print(f"\n omega world = {omega_world}")
    #Compute Screw Axis
    v = np.cross(omega_world,-L)
    #print(f"Screw_axis{n} = {v}")

    Screw_axis = np.hstack((omega_world,v))
    print(f"Screw axis{n} = {Screw_axis}")

    return Screw_axis
def POE_FK(M,slist,thetalist):
    #obtain Screw Axis
    T = mr.FKinSpace(M,slist,thetalist)
    return T
def Pose_error(T_mj, T_fk):
    Rot_fk = T_fk[:3, :3]
    pos_fk = T_fk[:3, 3]

    Rot_mj = T_mj[:3, :3]
    pos_mj = T_mj[:3, 3]
    # Check for error:

    # Check Translational Error
    pos_error = np.linalg.norm(pos_fk - pos_mj)

    # Check Rotational Error if False then Rotation is not similar/ or similar enough.
    # True - means rotation passed and is close enough
    R_err = Rot_fk.T @ Rot_mj

    cos_theta = (np.trace(R_err) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    rot_error = np.arccos(cos_theta)

    return pos_error, rot_error
#Get M transform matrix from home position
M = np.eye(4)
M[:3,:3] = ee_rot
M[:3,3] = ee_pos
print(f"M matrix: \n {M}")

joints = model.njnt - 2
slist = []


for i in range(1,joints+1):
    screw = Get_Screw_axis(i)
    slist.append(screw)

slist = np.column_stack(slist)
#Home position Joint angles
theta_list = np.array((0.5,1,0.8,0.3,0.5,0.8,1))

#Using Modern Robotics Library
T_fk = POE_FK(M,slist,theta_list)

print(f"\nObtained POE T = \n{T_fk}")

#Use mujoco to check (everything up to the 7th link): 
data.qpos[:7] = theta_list

#Get Mujoco EE pose
mujoco.mj_forward(model,data)

#Get end effector site ID (from xml file)
site_id = model.site("gripper").id

pos_mj = data.site_xpos[site_id].copy()
Rot_mj = data.site_xmat[site_id].reshape(3,3).copy()

T_mj = np.eye(4)

T_mj[:3,:3] = Rot_mj
T_mj[:3,3] = pos_mj

print(f"\nObtained mujoco T_mj = \n{T_mj}")


# Test many configurations
total_pos_error = []
total_rot_error = []

if DEBUG == 1:
    for k in range(100):

        upper_limit = 2.9
        lower_limit = -2.9

        theta = np.random.uniform(low=q_min,high=q_max)
        theta_list = theta

        T_fk = POE_FK(M,slist,theta_list)

        #Use mujoco to check (everything up to the 7th link): 
        data.qpos[:7] = theta_list
        data.qpos[7:] = 0.0

        #Get Mujoco EE pose
        mujoco.mj_forward(model,data)

        #Get end effector site ID (from xml file)
        site_id = model.site("gripper").id

        pos_mj = data.site_xpos[site_id].copy()
        Rot_mj = data.site_xmat[site_id].reshape(3,3).copy()

        T_mj = np.eye(4)

        T_mj[:3,:3] = Rot_mj
        T_mj[:3,3] = pos_mj
        
        #print(f"Mujoco pose \n {T_mj}")
        #print(f"\nPOE pose \n {T_fk}")

        pos_error, rot_error = Pose_error(T_mj,T_fk)
        print(f"\nTranslational Error is within: {pos_error}\n")
        print(f"Rotational Error is within: {rot_error}\n")
        total_pos_error.append(pos_error)
        total_rot_error.append(rot_error)

    max_pos_error = np.max(total_pos_error)
    min_pos_error = np.min(total_pos_error)
    mean_pos_error = np.mean(total_pos_error)


    max_rot_error = np.max(total_rot_error)
    min_rot_error = np.min(total_rot_error)
    mean_rot_error = np.mean(total_rot_error)

    print("\nFK Validation Summary")
    print(f"Position Error  | min: {min_pos_error:.6e} m, mean: {mean_pos_error:.6e} m, max: {max_pos_error:.6e} m")
    print(f"Rotation Error  | min: {min_rot_error:.6e} rad, mean: {mean_rot_error:.6e} rad, max: {max_rot_error:.6e} rad")
