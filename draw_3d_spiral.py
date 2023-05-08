import numpy as np
# Function to generate a 3D spiral points
def draw_3d_spiral(num_points=100, num_turns=2, radius=0.005, height=0.002):
    # Generate spiral points
    t = np.linspace(0, num_turns * 2 * np.pi, num_points)
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    z = height * (t / (2 * np.pi))
    return x,y,z