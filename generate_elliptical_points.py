import numpy as np

def generate_elliptical_points(center, radii, theta, n_points, n=2):
    # Define the rotation matrix
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta), np.cos(theta), 0],
                                [0, 0, 1]])

    # Generate n_points from the ellipse
    angles = np.linspace(0, 2*np.pi, n_points, endpoint=False)
    points = np.stack((radii[0]*np.cos(angles), radii[1]*np.sin(angles), np.zeros(n_points)), axis=1)
    points = np.dot(points, rotation_matrix) + center

    # Generate n turns points from the ellipse
    pts = np.tile(points, (n, 1))
    
    return pts