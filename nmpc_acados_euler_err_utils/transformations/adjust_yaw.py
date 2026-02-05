import numpy as np

def adjust_yaw(node, yaw: float) -> float:
    """Adjust yaw angle to account for full rotations and return the adjusted yaw."""
    mocap_psi = yaw
    psi = None

    if not node.mocap_initialized:
        node.mocap_initialized = True
        node.prev_mocap_psi = mocap_psi
        psi = mocap_psi
        return psi

    if node.prev_mocap_psi > np.pi*0.9 and mocap_psi < -np.pi*0.9:
        node.full_rotations += 1
    elif node.prev_mocap_psi < -np.pi*0.9 and mocap_psi > np.pi*0.9:
        node.full_rotations -= 1

    psi = mocap_psi + 2*np.pi * node.full_rotations
    node.prev_mocap_psi = mocap_psi

    return psi
