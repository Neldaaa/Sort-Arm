import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
import random

# --- WAREHOUSE CONFIGURATION ---
try:
    num_packages = int(input("Enter the number of packages to sort: "))
except ValueError:
    num_packages = 12
    print("Invalid input. Defaulting to 12 packages.")

AVAILABLE_COLORS = ['red', 'blue', 'green']
PACKAGE_QUEUE = [random.choice(AVAILABLE_COLORS) for _ in range(num_packages)]

# Bin Coordinates
BIN_LOCATIONS = {
    'red':   [0.0, 0.6, 0.0],   
    'blue':  [0.0, -0.6, 0.0],  
    'green': [1.0, 0.0, 0.0]    
}

PICKUP_ZONE = [0.5, 0.0, 0.05]
# -------------------------------

class RobotSimulation:
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # --- VISUAL SETUP ---
        # Hide the default PyBullet menus (left/right)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        
        # Better Lighting (Shadows)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        
        # Set Initial Camera Position (User's Preference)
        p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.3,0,0.2])
        
        # 1. Load Environment
        self.planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        
        # 2. Conveyor Belt
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[0.12, 1, 0.05], rgbaColor=[0.2, 0.2, 0.2, 1])
        collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[0.1, 1, 0.05])
        self.conveyorId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collisionShapeId, 
                                          baseVisualShapeIndex=visualShapeId, basePosition=[0.5, 0.0, -0.025])
        p.changeDynamics(self.conveyorId, -1, lateralFriction=2.0)

        # 3. Create Baskets
        self.create_tray(BIN_LOCATIONS['red'], [1, 0, 0, 1])
        self.create_tray(BIN_LOCATIONS['blue'], [0, 0, 1, 1])
        self.create_tray(BIN_LOCATIONS['green'], [0, 1, 0, 1])

        # 4. Counters
        self.bin_item_counts = {'red': 0, 'blue': 0, 'green': 0}
        self.total_sorted = 0

        # 5. SETUP TWO DASHBOARDS
        # Turn off toolbar for all windows
        plt.rcParams['toolbar'] = 'None' 
        plt.rcParams.update({'figure.max_open_warning': 0})
        plt.ion() 

        self.joint_angles = []
        self.end_effector_positions = []
        
        # Initialize BOTH windows
        self.setup_graph_dashboard()  # Graphs (Angles/Path)
        self.setup_score_dashboard()  # Scoreboard

        p.setGravity(0, 0, -9.8)
        
    def create_tray(self, position, color):
        """Creates a physical tray."""
        base_pos = np.array(position)
        width = 0.2
        thickness = 0.005
        wall_height = 0.12 # Taller walls
        
        vis_base = p.createVisualShape(p.GEOM_BOX, halfExtents=[width, width, thickness], rgbaColor=color)
        col_base = p.createCollisionShape(p.GEOM_BOX, halfExtents=[width, width, thickness])
        baseId = p.createMultiBody(0, col_base, vis_base, basePosition=[base_pos[0], base_pos[1], 0.01])
        
        # High Friction & No Bounce to prevent jumping
        p.changeDynamics(baseId, -1, lateralFriction=5.0, restitution=0.0)
        
        wall_configs = [
            ([width, thickness, wall_height], [0, width, wall_height]),
            ([width, thickness, wall_height], [0, -width, wall_height]),
            ([thickness, width, wall_height], [width, 0, wall_height]),
            ([thickness, width, wall_height], [-width, 0, wall_height])
        ]
        darker_color = [color[0]*0.7, color[1]*0.7, color[2]*0.7, 1]
        for extents, offset in wall_configs:
            vis_wall = p.createVisualShape(p.GEOM_BOX, halfExtents=extents, rgbaColor=darker_color)
            col_wall = p.createCollisionShape(p.GEOM_BOX, halfExtents=extents)
            p.createMultiBody(0, col_wall, vis_wall, basePosition=[base_pos[0]+offset[0], base_pos[1]+offset[1], 0.01 + offset[2]])

    # --- DASHBOARD 1: TELEMETRY (New) ---
    def setup_graph_dashboard(self):
    # Create Figure 1 with placeholder axes
        self.fig_graph, (self.ax1, ax_placeholder) = plt.subplots(1, 2, figsize=(8, 3.5))
        self.fig_graph.canvas.manager.set_window_title("Telemetry (Joints & Path)")

    # Left subplot: Joint Angles
        self.ax1.set_title('Joint Angles', fontsize=10)

    # Remove right 2D placeholder axis
        self.fig_graph.delaxes(ax_placeholder)

    # Create REAL 3D axis in its place
        self.ax2 = self.fig_graph.add_subplot(122, projection='3d')
        self.ax2.set_title('End Effector Path', fontsize=10)

        self.fig_graph.tight_layout()
        plt.show(block=False)


    def update_graph_dashboard(self):
        self.ax1.clear()
        self.ax2.clear()

    # Joint angles
        self.ax1.set_title('Joint Angles', fontsize=10)
        angles = np.array(self.joint_angles)
        if angles.size > 0:
            for i in range(min(4, angles.shape[1])):
                self.ax1.plot(angles[:, i], label=f'J{i+1}')
            self.ax1.legend(loc='upper right', fontsize='x-small')

    # 3D path
        self.ax2.set_title('End Effector Path', fontsize=10)
        pos = np.array(self.end_effector_positions)
        if pos.size > 0:
           self.ax2.plot3D(pos[:, 0], pos[:, 1], pos[:, 2])

        self.fig_graph.canvas.draw()
        self.fig_graph.canvas.flush_events()


    # --- DASHBOARD 2: SCOREBOARD (Existing) ---
    def setup_score_dashboard(self):
        # Create Figure 2
        self.fig_score, (self.ax_bar, self.ax_text) = plt.subplots(2, 1, figsize=(3.5, 4.5), gridspec_kw={'height_ratios': [3, 1]})
        self.fig_score.canvas.manager.set_window_title("Sorting Score")
        
        self.colors = ['red', 'blue', 'green']
        self.labels = ['Red', 'Blue', 'Green']
        self.counts = [0, 0, 0]
        self.bars = self.ax_bar.bar(self.labels, self.counts, color=self.colors)
        self.ax_bar.set_ylim(0, num_packages)
        self.ax_bar.set_title("Sorted Items", fontsize=10, fontweight='bold')
        
        self.ax_text.axis('off')
        self.score_text = self.ax_text.text(0.5, 0.5, f'Total: 0 / {num_packages}', ha='center', va='center', fontsize=14, fontweight='bold')

        self.fig_score.tight_layout()
        plt.show(block=False)

    def update_score_dashboard(self):
        counts = [self.bin_item_counts['red'], self.bin_item_counts['blue'], self.bin_item_counts['green']]
        for bar, count in zip(self.bars, counts):
            bar.set_height(count)
        self.score_text.set_text(f'Total: {sum(counts)} / {num_packages}')
        self.fig_score.canvas.draw()
        self.fig_score.canvas.flush_events()

    def get_joint_states(self):
        joint_states = p.getJointStates(self.robotId, range(p.getNumJoints(self.robotId)))
        return [state[0] for state in joint_states]
    
    def move_arm(self, target_pos, steps, speed):
        # 1. Define the "Natural" Resting Pose (Elbow up, Wrist straight)
        # These specific angles keep the Kuka arm looking realistic
        ll = [-2.96]*7   # Lower Limits
        ul = [2.96]*7    # Upper Limits
        jr = [5.92]*7    # Joint Ranges
        rp = [0, -0.5, 0, -1.5, 0, 1.0, 0] # <-- THIS IS THE KEY FIX
        
        # 2. Calculate IK with these constraints
        joint_poses = p.calculateInverseKinematics(
            self.robotId, 
            6, 
            target_pos,
            lowerLimits=ll,
            upperLimits=ul,
            jointRanges=jr,
            restPoses=rp
        )
        
        p.setJointMotorControlArray(self.robotId, range(7), p.POSITION_CONTROL, targetPositions=joint_poses)
        
        for _ in range(steps):
            p.stepSimulation()
            time.sleep(speed)
            
            # Record Data
            self.joint_angles.append(self.get_joint_states())
            link_state = p.getLinkState(self.robotId, 6)
            self.end_effector_positions.append(link_state[0])
            
            # Update Graphs every 50 steps
            if len(self.joint_angles) % 50 == 0:
                self.update_graph_dashboard()

    def spawn_package(self, color_name):
        colors = {'red': [1, 0, 0, 1], 'green': [0, 1, 0, 1], 'blue': [0, 0, 1, 1]}
        start_pos = [0.5, -1.0, 0.05]
        
        col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
        vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03], rgbaColor=colors[color_name])
        
        # PHYSICS FIX: BaseMass 1.0 (Heavy) + Damping
        cubeId = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=vis_shape, basePosition=start_pos)
        p.changeDynamics(cubeId, -1, lateralFriction=5.0, restitution=0.0, linearDamping=0.9, angularDamping=0.9)
        
        print(f"New {color_name} package arrived...")
        for i in range(51): 
            y_pos = -1.0 + (1.0 * (i/50))
            p.resetBasePositionAndOrientation(cubeId, [0.5, y_pos, 0.05], [0,0,0,1])
            p.stepSimulation()
            time.sleep(0.002) 
        return cubeId

    def process_package(self, color_name):
        cubeId = self.spawn_package(color_name)
        
        # Speed settings
        if color_name == 'red':
            speed = 1./240.; steps = 250 
            print("Handling FRAGILE (Red) - Slow & Gentle")
        else:
            speed = 1./240.; steps = 50       
            print(f"Handling STANDARD ({color_name}) - Fast Mode") 
            
        self.bin_item_counts[color_name] += 1
        self.update_score_dashboard()

        # --- LOGIC FIX START ---
        # 1. Randomize Target: Spread items within +/- 6cm of center to avoid unstable piles
        rand_x = random.uniform(-0.06, 0.06) 
        rand_y = random.uniform(-0.06, 0.06)
        
        # 2. Fixed Height: Drop from 0.15m (safe for piles, but low enough to prevent bouncing)
        # Do NOT use stack_index here.
        drop_z = 0.20 
        
        bin_pos = BIN_LOCATIONS[color_name]
        drop_target = [bin_pos[0] + rand_x, bin_pos[1] + rand_y, drop_z]
        # --- LOGIC FIX END ---

        # 1. Pick
        self.move_arm([0.5, 0.0, 0.3], steps, speed)
        self.move_arm(PICKUP_ZONE, steps, speed)
        grasp_constraint = p.createConstraint(self.robotId, 6, cubeId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
        self.move_arm([0.5, 0.0, 0.3], steps, speed)
        
        # 2. Move to Bin (Stay high 0.4m during travel to avoid hitting bin walls)
        self.move_arm([drop_target[0], drop_target[1], 0.4], steps, speed)
        
        # 3. Lower to Drop Point
        self.move_arm(drop_target, 100, 1./120.)

        for _ in range(20): 
            p.stepSimulation()
        
        # 4. Release
        p.resetBaseVelocity(cubeId, [0, 0, 0], [0, 0, 0])
        p.removeConstraint(grasp_constraint)
        
        # 5. Settle Time
        for _ in range(40):
            p.stepSimulation()
            time.sleep(0.005)
        
        # 6. Return Home (Go UP first)
        self.move_arm([drop_target[0], drop_target[1], 0.4], steps, speed)
        self.move_arm([0.5, 0.0, 0.4], steps, speed)
        
        print(f"Sorted into {color_name} bin.\n")
    
    def run(self):
        try:
            print(f"Queue: {PACKAGE_QUEUE}")
            for package_color in PACKAGE_QUEUE:
                self.process_package(package_color)
            print("Done! Press Ctrl+C to exit.")
            while True:
                p.stepSimulation()
                time.sleep(0.1)
        except KeyboardInterrupt:
            p.disconnect()
            plt.close()

if __name__ == "__main__":
    sim = RobotSimulation()
    sim.run()