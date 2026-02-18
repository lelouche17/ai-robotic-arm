import numpy as np
import matplotlib.pyplot as plt

class TwoLinkRobot:
    def __init__(self, L1, L2):
        """
        L1: length of first link
        L2: length of second link
        """
        self.L1 = L1
        self.L2 = L2
    
    def homogeneous_transform(self, theta, d, a, alpha):
        """
        Create a 4x4 homogeneous transformation matrix using DH parameters
        
        theta: joint angle (rotation about z-axis)
        d: link offset (translation along z-axis)
        a: link length (translation along x-axis)
        alpha: link twist (rotation about x-axis)
        
        This is the standard DH transformation!
        """
        T = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                 np.cos(alpha),                d              ],
            [0,              0,                             0,                            1              ]
        ])
        return T
    
    def forward_kinematics_homogeneous(self, theta1, theta2):
        """
        Calculate forward kinematics using homogeneous transformations
        This is the PROPER robotics way!
        
        For a 2-link planar robot:
        - Both joints rotate in the plane (2D)
        - DH parameters for Link 1: theta=theta1, d=0, a=L1, alpha=0
        - DH parameters for Link 2: theta=theta2, d=0, a=L2, alpha=0
        """
        # Transformation from base to joint 1
        T_0_1 = self.homogeneous_transform(theta1, d=0, a=self.L1, alpha=0)
        
        # Transformation from joint 1 to joint 2 (end effector)
        T_1_2 = self.homogeneous_transform(theta2, d=0, a=self.L2, alpha=0)
        
        # Total transformation from base to end effector
        T_0_2 = T_0_1 @ T_1_2
        
        # Extract position from transformation matrix
        # Position is in the last column, first 3 rows
        x = T_0_2[0, 3]
        y = T_0_2[1, 3]
        
        return x, y, T_0_1, T_0_2
    
    def visualize(self, theta1, theta2):
        """
        Draw the robot arm with coordinate frames
        """
        x, y, T_0_1, T_0_2 = self.forward_kinematics_homogeneous(theta1, theta2)
        
        # Joint 1 position (extract from T_0_1)
        x1 = T_0_1[0, 3]
        y1 = T_0_1[1, 3]
        
        # Create figure
        plt.figure(figsize=(10, 10))
        
        # Draw the robot links
        plt.plot([0, x1, x], [0, y1, y], 'o-', linewidth=4, markersize=12, 
                label='Robot Arm', color='blue')
        
        # Draw base
        plt.plot(0, 0, 'ks', markersize=15, label='Base')
        
        # Draw joints
        plt.plot(x1, y1, 'ro', markersize=12, label='Joint 1 (Elbow)')
        
        # Draw end effector
        plt.plot(x, y, 'g^', markersize=15, label='End Effector')
        
        # Draw coordinate frame at base
        arrow_length = 0.3
        plt.arrow(0, 0, arrow_length, 0, head_width=0.1, head_length=0.1, 
                 fc='red', ec='red', linewidth=2)
        plt.arrow(0, 0, 0, arrow_length, head_width=0.1, head_length=0.1, 
                 fc='green', ec='green', linewidth=2)
        plt.text(arrow_length+0.1, 0, 'X₀', fontsize=12, color='red')
        plt.text(0, arrow_length+0.1, 'Y₀', fontsize=12, color='green')
        
        # Settings
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        max_reach = self.L1 + self.L2 + 0.5
        plt.xlim(-max_reach, max_reach)
        plt.ylim(-max_reach, max_reach)
        plt.xlabel('X (meters)', fontsize=12)
        plt.ylabel('Y (meters)', fontsize=12)
        plt.title(f'2-DOF Robot Arm\nθ₁={np.degrees(theta1):.1f}°, θ₂={np.degrees(theta2):.1f}°\n'
                 f'End Effector Position: ({x:.3f}, {y:.3f})', fontsize=14)
        plt.legend(loc='upper right')
        
        plt.show()
    
    def print_transformation_matrices(self, theta1, theta2):
        """
        Print the transformation matrices (educational!)
        """
        x, y, T_0_1, T_0_2 = self.forward_kinematics_homogeneous(theta1, theta2)
        
        print("\n" + "="*60)
        print("HOMOGENEOUS TRANSFORMATION MATRICES")
        print("="*60)
        
        print("\nT₀₁ (Base to Joint 1):")
        print(T_0_1)
        print(f"\nJoint 1 Position: ({T_0_1[0,3]:.3f}, {T_0_1[1,3]:.3f})")
        
        print("\nT₁₂ (Joint 1 to End Effector):")
        T_1_2 = self.homogeneous_transform(theta2, d=0, a=self.L2, alpha=0)
        print(T_1_2)
        
        print("\nT₀₂ (Base to End Effector) = T₀₁ × T₁₂:")
        print(T_0_2)
        print(f"\nEnd Effector Position: ({x:.3f}, {y:.3f})")
        print("="*60 + "\n")


def get_user_input():
    """
    Get robot parameters from user
    """
    print("\n" + "="*60)
    print("🤖 2-DOF ROBOT ARM FORWARD KINEMATICS SIMULATOR")
    print("="*60)
    
    # Get link lengths
    print("\n📏 LINK LENGTHS (in meters)")
    while True:
        try:
            L1 = float(input("Enter length of Link 1 (e.g., 1.5): "))
            L2 = float(input("Enter length of Link 2 (e.g., 1.0): "))
            if L1 > 0 and L2 > 0:
                break
            else:
                print("❌ Lengths must be positive! Try again.")
        except ValueError:
            print("❌ Invalid input! Please enter a number.")
    
    # Get joint angles
    print("\n📐 JOINT ANGLES")
    print("(You can enter in degrees, we'll convert to radians)")
    while True:
        try:
            theta1_deg = float(input("Enter θ₁ (Joint 1 angle in degrees, e.g., 45): "))
            theta2_deg = float(input("Enter θ₂ (Joint 2 angle in degrees, e.g., 60): "))
            
            # Convert to radians
            theta1 = np.radians(theta1_deg)
            theta2 = np.radians(theta2_deg)
            break
        except ValueError:
            print("❌ Invalid input! Please enter a number.")
    
    return L1, L2, theta1, theta2, theta1_deg, theta2_deg


def main():
    """
    Main program with user interaction
    """
    while True:
        # Get user input
        L1, L2, theta1, theta2, theta1_deg, theta2_deg = get_user_input()
        
        # Create robot
        robot = TwoLinkRobot(L1, L2)
        
        # Calculate forward kinematics
        x, y, _, _ = robot.forward_kinematics_homogeneous(theta1, theta2)
        
        # Print results
        print("\n" + "="*60)
        print("✅ RESULTS")
        print("="*60)
        print(f"Link 1 Length: {L1} m")
        print(f"Link 2 Length: {L2} m")
        print(f"Joint 1 Angle: {theta1_deg}° ({theta1:.4f} rad)")
        print(f"Joint 2 Angle: {theta2_deg}° ({theta2:.4f} rad)")
        print(f"\n🎯 End Effector Position: ({x:.4f}, {y:.4f}) meters")
        print(f"Distance from base: {np.sqrt(x**2 + y**2):.4f} meters")
        
        # Ask if user wants to see transformation matrices
        show_matrices = input("\nShow transformation matrices? (y/n): ").lower()
        if show_matrices == 'y':
            robot.print_transformation_matrices(theta1, theta2)
        
        # Visualize
        print("\n📊 Opening visualization window...")
        robot.visualize(theta1, theta2)
        
        # Ask if user wants to continue
        again = input("\n🔄 Try another configuration? (y/n): ").lower()
        if again != 'y':
            print("\n👋 Thanks for using the robot simulator! Goodbye!")
            break


if __name__ == "__main__":
    main()