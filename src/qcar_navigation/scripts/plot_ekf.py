import rosbag
import matplotlib.pyplot as plt
import numpy as np

bag_path = 'ekf_comparison_2026-04-09-11-21-09.bag'

raw_x, raw_y, raw_t = [], [], []
filtered_x, filtered_y, filtered_t = [], [], []

bag = rosbag.Bag(bag_path)

for topic, msg, t in bag.read_messages(topics=['/odom']):
    raw_x.append(msg.pose.pose.position.x)
    raw_y.append(msg.pose.pose.position.y)
    raw_t.append(t.to_sec())

for topic, msg, t in bag.read_messages(topics=['/odometry/filtered']):
    filtered_x.append(msg.pose.pose.position.x)
    filtered_y.append(msg.pose.pose.position.y)
    filtered_t.append(t.to_sec())

bag.close()

raw_t = np.array(raw_t) - raw_t[0]
filtered_t = np.array(filtered_t) - filtered_t[0]

print(f"Raw messages: {len(raw_x)}")
print(f"Filtered messages: {len(filtered_x)}")

# Plot 1 - XY path
plt.figure(figsize=(8, 6))
plt.plot(raw_x, raw_y, label='Raw odometry', color='red', alpha=0.6, linewidth=1)
plt.plot(filtered_x, filtered_y, label='EKF filtered', color='blue', linewidth=1.5)
plt.xlabel('X position (m)')
plt.ylabel('Y position (m)')
plt.title('Raw vs EKF filtered path')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.savefig('path_comparison.png', dpi=150)
plt.show()

# Plot 2 - X over time
plt.figure(figsize=(10, 4))
plt.plot(raw_t, raw_x, label='Raw X', color='red', alpha=0.6)
plt.plot(filtered_t, filtered_x, label='EKF filtered X', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('X position (m)')
plt.title('X position over time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('x_comparison.png', dpi=150)
plt.show()

# Plot 3 - Y over time
plt.figure(figsize=(10, 4))
plt.plot(raw_t, raw_y, label='Raw Y', color='red', alpha=0.6)
plt.plot(filtered_t, filtered_y, label='EKF filtered Y', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Y position (m)')
plt.title('Y position over time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('y_comparison.png', dpi=150)
plt.show()