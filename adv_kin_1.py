import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.tri import Triangulation
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib as mpl

# Set a modern style
plt.style.use('seaborn-v0_8-darkgrid')
mpl.rcParams['font.family'] = 'Arial'

def rotation_matrix(roll, pitch, yaw):
    """Calculate rotation matrix from roll, pitch, yaw angles (in radians)"""
    Rx = np.array([[1, 0, 0],
                  [0, np.cos(roll), -np.sin(roll)],
                  [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                  [0, 1, 0],
                  [-np.sin(pitch), 0, np.cos(pitch)]])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                  [np.sin(yaw), np.cos(yaw), 0],
                  [0, 0, 1]])

    return Rz @ Ry @ Rx  # Matrix multiplication

def calculate_stewart_platform(r_B, r_P, servo_arm_length, rod_length, alpha_B, alpha_P, trans, orient):
    """
    Calculate servo angles for given platform parameters and desired position

    Parameters:
    - r_B: radius of base platform
    - r_P: radius of top platform
    - servo_arm_length: length of servo arms
    - rod_length: length of connecting rods
    - alpha_B: angle offset for base joints (radians)
    - alpha_P: angle offset for platform joints (radians)
    - trans: [x, y, z] translation vector
    - orient: [roll, pitch, yaw] orientation in radians

    Returns:
    - servo angles in radians
    """
    # Calculate base platform points
    B = np.zeros((6, 3))
    for i in range(6):
        angle = i * np.pi/3 + alpha_B if i % 2 == 0 else (i-1) * np.pi/3 - alpha_B
        B[i] = [r_B * np.cos(angle), r_B * np.sin(angle), 0]

    # Calculate top platform points in home position
    P_home = np.zeros((6, 3))
    for i in range(6):
        angle = i * np.pi/3 + alpha_P if i % 2 == 0 else (i-1) * np.pi/3 - alpha_P
        P_home[i] = [r_P * np.cos(angle), r_P * np.sin(angle), 0]

    # Apply rotation and translation to platform
    R = rotation_matrix(orient[0], orient[1], orient[2])
    P = np.zeros((6, 3))
    for i in range(6):
        P[i] = R @ P_home[i] + trans + [0, 0, rod_length + servo_arm_length]

    # Calculate servo angles
    angles = np.zeros(6)
    for i in range(6):
        # Vector from servo pivot to platform attachment point
        L = P[i] - B[i]

        # Project L onto XY plane
        L_xy = np.array([L[0], L[1], 0])
        L_xy_norm = np.linalg.norm(L_xy)

        # Calculate angle between L_xy and the servo arm's projection
        if L_xy_norm <= servo_arm_length:
            # Calculate servo angle using law of cosines
            cos_theta = L_xy_norm / servo_arm_length
            angles[i] = np.arccos(cos_theta)

            # Adjust angle based on quadrant
            if L[2] < rod_length + servo_arm_length:
                angles[i] = -angles[i]
        else:
            # If unreachable, assign complex number
            angles[i] = np.arccos(1.1)  # Value > 1 to force complex result

    return angles

class StewartPlatform:
    def __init__(self):
        # Default geometry parameters
        self.r_B = 10.0
        self.r_P = 8.0
        self.servo_arm_length = 2.5
        self.rod_length = 15.0
        self.alpha_B = np.radians(10)
        self.alpha_P = np.radians(10)

        # Default position and orientation
        self.trans = np.array([0.0, 0.0, 0.0])
        self.orient = np.array([0.0, 0.0, 0.0])

        # Color scheme
        self.base_color = '#3498db'  # Blue
        self.platform_color = '#2ecc71'  # Green
        self.servo_color = '#e74c3c'  # Red
        self.rod_color = '#f39c12'  # Orange
        self.axis_color = '#7f8c8d'  # Gray
        self.background_color = '#ecf0f1'  # Light gray
        self.text_color = '#2c3e50'  # Dark blue

        # Initialize plot with custom figure
        self.fig = plt.figure(figsize=(18, 10), facecolor=self.background_color)
        self.fig.suptitle('Stewart Platform Simulation', fontsize=18, color=self.text_color, fontweight='bold', y=0.97)

        # Create better layout with more breathing room
        # 3D plot on the left
        self.ax = self.fig.add_axes([0.05, 0.05, 0.55, 0.85], projection='3d', facecolor=self.background_color)
        
        # Info panel on the top right corner
        self.info_ax = self.fig.add_axes([0.65, 0.6, 0.3, 0.3], facecolor=self.background_color)
        self.info_ax.axis('off')

        # Create sliders with improved spacing and organization
        self.create_sliders()

        # Create reset button - positioned properly below sliders
        self.reset_button_ax = self.fig.add_axes([0.75, 0.08, 0.1, 0.05])
        self.reset_button = Button(self.reset_button_ax, 'Reset', color=self.base_color, hovercolor=self.platform_color)
        self.reset_button.on_clicked(self.reset)

        # Initial render
        self.update_platform()

        plt.show()

    def create_sliders(self):
        """Create position and orientation sliders with better spacing"""
        # Slider properties - increased width and better vertical spacing
        slider_width = 0.25
        slider_height = 0.02
        slider_x = 0.68
        
        # Improved organization with group titles and proper spacing
        # Translation sliders (first group)
        # Add translation group title
        title_ax = self.fig.add_axes([slider_x, 0.54, slider_width, 0.02])
        title_ax.text(0, 0, "Translation Controls", fontsize=12, fontweight='bold')
        title_ax.axis('off')
        
        self.slider_x_ax = self.fig.add_axes([slider_x, 0.5, slider_width, slider_height])
        self.slider_x = Slider(self.slider_x_ax, 'X Translation', -5.0, 5.0, valinit=0, 
                               color=self.base_color, alpha=0.6, valstep=0.1)
        
        self.slider_y_ax = self.fig.add_axes([slider_x, 0.45, slider_width, slider_height])
        self.slider_y = Slider(self.slider_y_ax, 'Y Translation', -5.0, 5.0, valinit=0,
                               color=self.base_color, alpha=0.6, valstep=0.1)
        
        self.slider_z_ax = self.fig.add_axes([slider_x, 0.4, slider_width, slider_height])
        self.slider_z = Slider(self.slider_z_ax, 'Z Translation', -5.0, 5.0, valinit=0,
                               color=self.base_color, alpha=0.6, valstep=0.1)
        
        # Rotation sliders (second group) with proper spacing and a group title
        # Add rotation group title
        title_rot_ax = self.fig.add_axes([slider_x, 0.34, slider_width, 0.02])
        title_rot_ax.text(0, 0, "Rotation Controls", fontsize=12, fontweight='bold')
        title_rot_ax.axis('off')
        
        self.slider_roll_ax = self.fig.add_axes([slider_x, 0.3, slider_width, slider_height])
        self.slider_roll = Slider(self.slider_roll_ax, 'Roll (°)', -15, 15, valinit=0,
                                  color=self.base_color, alpha=0.6, valstep=0.1)
        
        self.slider_pitch_ax = self.fig.add_axes([slider_x, 0.25, slider_width, slider_height])
        self.slider_pitch = Slider(self.slider_pitch_ax, 'Pitch (°)', -15, 15, valinit=0,
                                   color=self.base_color, alpha=0.6, valstep=0.1)
        
        self.slider_yaw_ax = self.fig.add_axes([slider_x, 0.2, slider_width, slider_height])
        self.slider_yaw = Slider(self.slider_yaw_ax, 'Yaw (°)', -15, 15, valinit=0,
                                 color=self.base_color, alpha=0.6, valstep=0.1)

        # Connect update functions
        self.slider_x.on_changed(self.update_platform)
        self.slider_y.on_changed(self.update_platform)
        self.slider_z.on_changed(self.update_platform)
        self.slider_roll.on_changed(self.update_platform)
        self.slider_pitch.on_changed(self.update_platform)
        self.slider_yaw.on_changed(self.update_platform)

    def reset(self, event):
        """Reset all sliders to initial values"""
        self.slider_x.reset()
        self.slider_y.reset()
        self.slider_z.reset()
        self.slider_roll.reset()
        self.slider_pitch.reset()
        self.slider_yaw.reset()
        self.update_platform()

    def update_platform(self, val=None):
        """Update platform position and orientation based on sliders"""
        # Get values from sliders
        self.trans[0] = self.slider_x.val
        self.trans[1] = self.slider_y.val
        self.trans[2] = self.slider_z.val

        # Convert degrees to radians
        self.orient[0] = np.radians(self.slider_roll.val)
        self.orient[1] = np.radians(self.slider_pitch.val)
        self.orient[2] = np.radians(self.slider_yaw.val)

        # Calculate servo angles
        angles = calculate_stewart_platform(
            self.r_B, self.r_P, self.servo_arm_length, self.rod_length,
            self.alpha_B, self.alpha_P, self.trans, self.orient
        )

        # Display platform
        self.plot_platform(angles)

    def plot_platform(self, angles):
        """Plot the stewart platform with current configuration and enhanced visuals"""
        self.ax.clear()
        self.info_ax.clear()
        self.info_ax.axis('off')

        # Calculate base platform points
        B = np.zeros((6, 3))
        for i in range(6):
            angle = i * np.pi/3 + self.alpha_B if i % 2 == 0 else (i-1) * np.pi/3 - self.alpha_B
            B[i] = [self.r_B * np.cos(angle), self.r_B * np.sin(angle), 0]

        # Calculate servo arm positions
        S = np.zeros((6, 3))
        for i in range(6):
            # Servo angle in 3D
            theta = angles[i].real  # Take real part for visualization
            S[i] = B[i] + [0, 0, self.servo_arm_length * np.sin(theta)]
            S[i][0] += self.servo_arm_length * np.cos(theta) * np.cos(i * np.pi/3)
            S[i][1] += self.servo_arm_length * np.cos(theta) * np.sin(i * np.pi/3)

        # Calculate top platform points
        P_home = np.zeros((6, 3))
        for i in range(6):
            angle = i * np.pi/3 + self.alpha_P if i % 2 == 0 else (i-1) * np.pi/3 - self.alpha_P
            P_home[i] = [self.r_P * np.cos(angle), self.r_P * np.sin(angle), 0]

        # Apply rotation and translation
        R = rotation_matrix(self.orient[0], self.orient[1], self.orient[2])
        P = np.zeros((6, 3))
        home_height = self.rod_length + self.servo_arm_length
        for i in range(6):
            P[i] = R @ P_home[i] + self.trans + [0, 0, home_height]

        # Plot base (hexagon)
        x_base = [B[i, 0] for i in range(6)] + [B[0, 0]]
        y_base = [B[i, 1] for i in range(6)] + [B[0, 1]]
        z_base = [B[i, 2] for i in range(6)] + [B[0, 2]]
        self.ax.plot(x_base, y_base, z_base, color=self.base_color, linewidth=2)

        # Plot platform (hexagon)
        x_plat = [P[i, 0] for i in range(6)] + [P[0, 0]]
        y_plat = [P[i, 1] for i in range(6)] + [P[0, 1]]
        z_plat = [P[i, 2] for i in range(6)] + [P[0, 2]]
        self.ax.plot(x_plat, y_plat, z_plat, color=self.platform_color, linewidth=2)

        # Plot servo arms and connecting rods with improved appearance
        for i in range(6):
            # Servo arm with gradient coloring
            x_servo = [B[i, 0], S[i, 0]]
            y_servo = [B[i, 1], S[i, 1]]
            z_servo = [B[i, 2], S[i, 2]]
            self.ax.plot(x_servo, y_servo, z_servo, color=self.servo_color, linewidth=3, alpha=0.8)

            # Connecting rod with gradient coloring
            x_rod = [S[i, 0], P[i, 0]]
            y_rod = [S[i, 1], P[i, 1]]
            z_rod = [S[i, 2], P[i, 2]]
            self.ax.plot(x_rod, y_rod, z_rod, color=self.rod_color, linewidth=2, alpha=0.8)

            # Add small spheres at joints for better visualization
            self.ax.scatter(B[i, 0], B[i, 1], B[i, 2], color=self.base_color, s=50, edgecolor='black', linewidth=0.5)
            self.ax.scatter(S[i, 0], S[i, 1], S[i, 2], color=self.servo_color, s=40, edgecolor='black', linewidth=0.5)
            self.ax.scatter(P[i, 0], P[i, 1], P[i, 2], color=self.platform_color, s=50, edgecolor='black', linewidth=0.5)

        # Fill base surface using plot_trisurf with gradient
        x_base_arr = np.array(x_base[:-1])
        y_base_arr = np.array(y_base[:-1])
        z_base_arr = np.array(z_base[:-1])
        tri_base = Triangulation(x_base_arr, y_base_arr)
        base_surf = self.ax.plot_trisurf(x_base_arr, y_base_arr, z_base_arr, triangles=tri_base.triangles,
                             color=self.base_color, alpha=0.4, shade=True)

        # Fill platform surface using plot_trisurf with gradient
        x_plat_arr = np.array(x_plat[:-1])
        y_plat_arr = np.array(y_plat[:-1])
        z_plat_arr = np.array(z_plat[:-1])
        tri_plat = Triangulation(x_plat_arr, y_plat_arr)
        plat_surf = self.ax.plot_trisurf(x_plat_arr, y_plat_arr, z_plat_arr, triangles=tri_plat.triangles,
                             color=self.platform_color, alpha=0.4, shade=True)

        # Set axis properties for better appearance
        self.ax.set_xlim([-15, 15])
        self.ax.set_ylim([-15, 15])
        self.ax.set_zlim([0, 25])
        self.ax.set_xlabel('X', fontsize=12, color=self.text_color)
        self.ax.set_ylabel('Y', fontsize=12, color=self.text_color)
        self.ax.set_zlabel('Z', fontsize=12, color=self.text_color)

        # Set grid and ticks
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False

        # Make 3D plot more interactive
        self.ax.view_init(elev=30, azim=45)  # Better default viewing angle

        # Display servo angles with improved formatting and location
        any_unreachable = any(np.iscomplex(angles))

        # Title for angles display (bold and properly positioned)
        self.info_ax.text(0.05, 0.55, "Servo Angles (degrees):", fontsize=12,
                          fontweight='bold', color=self.text_color)

        # Display status message with proper position
        if any_unreachable:
            self.info_ax.text(0.05, 0.65, "Status: UNREACHABLE",
                             color='red', fontsize=10, fontweight='bold')
        else:
            self.info_ax.text(0.05, 0.65, "Status: Stable Position",
                             color='green', fontsize=10, fontweight='bold')

        # Current position and orientation - Organized and well-spaced
        pos_text = f"Position:\n"
        pos_text += f"X: {self.trans[0]:.1f}, Y: {self.trans[1]:.1f}, Z: {self.trans[2]:.1f}\n\n"
        pos_text += f"Orientation:\n"
        pos_text += f"Roll: {np.degrees(self.orient[0]):.1f}°, Pitch: {np.degrees(self.orient[1]):.1f}°, Yaw: {np.degrees(self.orient[2]):.1f}°"

        self.info_ax.text(0.05, 0.78, pos_text, fontsize=10, color=self.text_color, linespacing=1.5)

        # Servo angles with color-coding - Improved spacing
        for i in range(6):
            y_pos = 0.45 - i * 0.06  # Better vertical spacing
            if np.iscomplex(angles[i]):
                angle_text = f"Servo {i+1}: Unreachable"
                angle_color = 'red'
            else:
                angle_text = f"Servo {i+1}: {np.degrees(angles[i]):.1f}°"
                angle_color = 'black'
            self.info_ax.text(0.05, y_pos, angle_text, fontsize=10, color=angle_color)

        # Add timestamp and credits in a better position
        self.info_ax.text(0.05, 0.05, "Stewart Platform Sim v2.0",
                         fontsize=8, style='italic', color=self.text_color)

        # Refresh canvas
        self.fig.canvas.draw_idle()

# Main function to run the simulation
def main():
    app = StewartPlatform()

if __name__ == "__main__":
    main()