import numpy as np
import matplotlib.pyplot as plt

arc1 = 10
arc2 = 20
length = 10

radius_of_curvature = ((-length) * (arc1 + arc2) / (arc1 - arc2))

# The calculation for theta is not used later, it might be part of an intended calculation for the arc.
theta = arc1 / (radius_of_curvature - length/2)

starting_pt = np.array([0, 0])
starting_orientation = np.array([0,1])

# Render the starting point and orientation
plt.plot(starting_pt[0], starting_pt[1], 'ro')
plt.arrow(starting_pt[0], starting_pt[1], starting_orientation[0], starting_orientation[1], head_width=1, head_length=10, fc='k', ec='k')

# Draw the first arc
arc_center = starting_pt - np.array([(radius_of_curvature), 0])
print(arc_center)
print(radius_of_curvature)
plt.plot(arc_center[0], arc_center[1], 'ro')

# Create a circle and add it to the plot
circle = plt.Circle((arc_center[0], arc_center[1]), radius_of_curvature, color='b', fill=False)
plt.gca().add_patch(circle)  # This line adds the circle to the current axes


# Rotate the orientation vector and starting point around the arc center by the angle theta
# This is a simple rotation matrix
rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
ending_orientation = np.dot(rotation_matrix, starting_orientation - arc_center) + arc_center
ending_position = np.dot(rotation_matrix, starting_pt - arc_center) + arc_center

# Draw the ending point and orientation
plt.plot(ending_position[0], ending_position[1], 'ro')
plt.arrow(ending_position[0], ending_position[1], ending_orientation[0], ending_orientation[1], head_width=1, head_length=10, fc='k', ec='k')

plt.axis('equal')
plt.show()
