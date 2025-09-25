

import heapq
import random
import math
from collections import deque
import time
import argparse
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class GridCell:
    def _init_(self, terrain_cost=1, is_obstacle=False):
        self.terrain_cost = terrain_cost
        self.is_obstacle = is_obstacle

    def get_cost_at_time(self, time_step):
        if self.is_obstacle:
            return float('inf')
        return self.terrain_cost

class GridEnvironment:
    def _init_(self):
        self.width = 0
        self.height = 0
        self.grid = []
        self.packages = []
        self.agent_position = (0, 0)
        self.fuel_limit = 1000
        self.time_limit = 1000

    def load_from_file(self, filename):
        if filename == "small.map":
            self.width, self.height = 5, 5
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            # Place some obstacles
            self.grid[2][2].is_obstacle = True
            self.grid[3][1].is_obstacle = True
            # Package going from (1,1) to (4,4) - agent begins at (0,0)
            self.packages = [((1, 1), (4, 4))]
            self.agent_position = (0, 0)
        elif filename == "medium.map":
            self.width, self.height = 10, 10
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            # Add some obstacles and different terrain types
            for i in range(3, 7):
                for j in range(3, 7):
                    self.grid[i][j].terrain_cost = 3  # More difficult terrain
            self.grid[5][5].is_obstacle = True
            self.grid[6][2].is_obstacle = True
            self.grid[7][7].is_obstacle = True
            # Packages - agent starts at (0,0)
            self.packages = [((0, 1), (9, 9)), ((2, 2), (8, 8))]
            self.agent_position = (0, 0)
        elif filename == "large.map":
            self.width, self.height = 10, 10
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            # Add a few obstacles but make sure paths are still possible
            for i in range(self.height):
                for j in range(self.width):
                    if (i + j) % 5 == 0 and (i, j) != (0, 0) and (i, j) != (1, 1) and (i, j) != (9, 9):
                        self.grid[i][j].is_obstacle = True
            # Packages - agent starts at (0,0)
            self.packages = [((1, 1), (9, 9)), ((2, 2), (8, 8))]
            self.agent_position = (0, 0)
        elif filename == "dynamic.map":
            self.width, self.height = 8, 8
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            # Add obstacles
            self.grid[2][2].is_obstacle = True
            self.grid[2][3].is_obstacle = True
            self.grid[3][3].is_obstacle = True
            # Package - agent starts at (0,0)
            self.packages = [((1, 1), (7, 7))]
            self.agent_position = (0, 0)

class UninformedPlanner:
    def _init_(self, environment):
        self.env = environment

    def get_neighbors(self, position, time_step):
        x, y = position
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # Up, right, down, left
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.env.height and 0 <= ny < self.env.width:
                cost = self.env.grid[nx][ny].get_cost_at_time(time_step)
                if cost < float('inf'):
                    neighbors.append(((nx, ny), cost))
        return neighbors

class BFSPlanner(UninformedPlanner):
    def plan(self, start, goal):
        queue = deque([(start, [start])])
        visited = set([start])
        nodes_expanded = 0
        while queue:
            position, path = queue.popleft()
            nodes_expanded += 1
            if position == goal:
                return path[1:], nodes_expanded  # Skip the starting position
            for neighbor, _ in self.get_neighbors(position, len(path)):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None, nodes_expanded

class UniformCostPlanner(UninformedPlanner):
    def plan(self, start, goal):
        priority_queue = [(0, start, [start])]
        visited = set()
        nodes_expanded = 0
        while priority_queue:
            cost, position, path = heapq.heappop(priority_queue)
            nodes_expanded += 1
            if position in visited:
                continue
            if position == goal:
                return path[1:], nodes_expanded
            visited.add(position)
            for neighbor, move_cost in self.get_neighbors(position, len(path)):
                if neighbor not in visited:
                    new_cost = cost + move_cost
                    heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))
        return None, nodes_expanded

class AStarPlanner(UninformedPlanner):
    def heuristic(self, a, b):
        # Manhattan distance
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set = set()
        nodes_expanded = 0
        while open_set:
            _, current = heapq.heappop(open_set)
            nodes_expanded += 1
            if current == goal:
                # Build the path by backtracking
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[-2::-1], nodes_expanded  # Reverse and skip start
            closed_set.add(current)
            for neighbor, cost in self.get_neighbors(current, g_score[current]):
                if neighbor in closed_set:
                    continue
                tentative_g_score = g_score[current] + cost
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None, nodes_expanded

class HillClimbingPlanner(UninformedPlanner):
    def _init_(self, environment, restarts=5):
        super()._init_(environment)
        self.restarts = restarts

    def plan(self, start, goal):
        best_path = None
        best_cost = float('inf')
        nodes_expanded = 0
        for _ in range(self.restarts):
            current = start
            path = [start]
            visited = set([start])
            while current != goal:
                neighbors = [n for n, _ in self.get_neighbors(current, len(path)) if n not in visited]
                nodes_expanded += len(neighbors)
                if not neighbors:
                    break
                # Sort neighbors by how close they are to the goal
                neighbors.sort(key=lambda n: self.heuristic(n, goal))
                # Occasionally pick a random neighbor to avoid getting stuck
                if random.random() < 0.3:  # More exploration
                    next_pos = random.choice(neighbors[:3])
                else:
                    next_pos = neighbors[0]
                path.append(next_pos)
                visited.add(next_pos)
                current = next_pos
            if current == goal:
                path_cost = self.calculate_path_cost(path)
                if path_cost < best_cost:
                    best_cost = path_cost
                    best_path = path[1:]
        return best_path, nodes_expanded

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def calculate_path_cost(self, path):
        total_cost = 0
        for i, pos in enumerate(path):
            if i > 0:
                total_cost += self.env.grid[pos[0]][pos[1]].get_cost_at_time(i)
        return total_cost

class SimulatedAnnealingPlanner(UninformedPlanner):
    def _init_(self, environment, initial_temp=100, cooling_rate=0.97):
        super()._init_(environment)
        self.initial_temp = initial_temp
        self.cooling_rate = cooling_rate

    def plan(self, start, goal):
        current_path = self.generate_random_path(start, goal)
        if not current_path:
            return None, 0
        current_cost = self.calculate_path_cost(current_path)
        temperature = self.initial_temp
        nodes_expanded = 0
        while temperature > 1 and current_path[-1] != goal:
            new_path = self.mutate_path(current_path, goal)
            nodes_expanded += 1
            new_cost = self.calculate_path_cost(new_path)
            if new_cost < current_cost or random.random() < math.exp((current_cost - new_cost) / temperature):
                current_path = new_path
                current_cost = new_cost
            temperature *= self.cooling_rate
        if current_path[-1] == goal:
            return current_path[1:], nodes_expanded
        else:
            return None, nodes_expanded

    def generate_random_path(self, start, goal, max_length=50):
        path = [start]
        current = start
        for _ in range(max_length):
            if current == goal:
                break
            neighbors = [n for n, _ in self.get_neighbors(current, len(path))]
            if not neighbors:
                break
            next_pos = random.choice(neighbors)
            path.append(next_pos)
            current = next_pos
        return path

    def mutate_path(self, path, goal):
        new_path = path.copy()
        if len(new_path) > 2 and random.random() < 0.5:
            idx = random.randint(1, len(new_path)-1)
            neighbors = [n for n, _ in self.get_neighbors(new_path[idx-1], idx)]
            if neighbors:
                new_path[idx] = random.choice(neighbors)
        else:
            if new_path[-1] != goal:
                neighbors = [n for n, _ in self.get_neighbors(new_path[-1], len(new_path))]
                if neighbors:
                    new_path.append(random.choice(neighbors))
        return new_path

    def calculate_path_cost(self, path):
        total_cost = 0
        for i, pos in enumerate(path):
            if i > 0:
                total_cost += self.env.grid[pos[0]][pos[1]].get_cost_at_time(i)
        return total_cost

class DeliveryAgent:
    def _init_(self, environment, planner_type='astar', gui_callback=None):
        self.env = environment
        self.position = environment.agent_position
        self.packages = environment.packages.copy()
        self.delivered = []
        self.fuel_remaining = environment.fuel_limit
        self.time_elapsed = 0
        self.planner_type = planner_type
        self.planner = self.create_planner(planner_type)
        self.log = []
        self.gui_callback = gui_callback

    def create_planner(self, planner_type):
        if planner_type == 'bfs':
            return BFSPlanner(self.env)
        elif planner_type == 'ucs':
            return UniformCostPlanner(self.env)
        elif planner_type == 'astar':
            return AStarPlanner(self.env)
        elif planner_type == 'hillclimb':
            return HillClimbingPlanner(self.env)
        elif planner_type == 'simanneal':
            return SimulatedAnnealingPlanner(self.env)

    def plan_path(self, start, goal):
        return self.planner.plan(start, goal)

    def execute_delivery(self):
        self.log.append(f"Starting delivery using {self.planner_type} planner")
        self.log.append(f"Starting at: {self.position}, Fuel left: {self.fuel_remaining}")
        while self.packages and self.fuel_remaining > 0 and self.time_elapsed < self.env.time_limit:
            package_pos, destination = self.packages[0]
            # Head to the package
            self.log.append(f"Finding path to package at {package_pos}")
            path_to_package, nodes_expanded = self.plan_path(self.position, package_pos)
            self.log.append(f"Path search looked at {nodes_expanded} nodes")
            if not path_to_package:
                self.log.append("Could not find a way to the package!")
                return False
            if not self.follow_path(path_to_package, package_pos):
                return False
            # Pick up the package
            self.log.append(f"Got the package at {package_pos}")
            self.packages.pop(0)
            # Deliver the package
            self.log.append(f"Finding path to delivery point at {destination}")
            path_to_dest, nodes_expanded = self.plan_path(self.position, destination)
            self.log.append(f"Path search looked at {nodes_expanded} nodes")
            if not path_to_dest:
                self.log.append("Could not find a way to the destination!")
                return False
            if not self.follow_path(path_to_dest, destination):
                return False
            self.log.append(f"Package delivered to {destination}")
            self.delivered.append((package_pos, destination))
        success = len(self.packages) == 0
        self.log.append(f"Delivery finished: {success}")
        self.log.append(f"Packages delivered: {len(self.delivered)}")
        self.log.append(f"Fuel left: {self.fuel_remaining}")
        self.log.append(f"Time taken: {self.time_elapsed}")
        return success

    def follow_path(self, path, target):
        if not path:
            return False
        for next_pos in path:
            if self.fuel_remaining <= 0:
                self.log.append("Ran out of fuel!")
                return False
            cell_cost = self.env.grid[next_pos[0]][next_pos[1]].get_cost_at_time(self.time_elapsed)
            if cell_cost == float('inf'):
                self.log.append(f"Hit an obstacle at {next_pos}, finding new path...")
                new_path, nodes_expanded = self.plan_path(self.position, target)
                self.log.append(f"New path search looked at {nodes_expanded} nodes")
                if not new_path:
                    self.log.append("Could not find a new path!")
                    return False
                return self.follow_path(new_path, target)
            self.fuel_remaining -= cell_cost
            self.time_elapsed += 1
            self.position = next_pos
            # Update the display if we have a callback
            if self.gui_callback:
                self.gui_callback(self)
                time.sleep(0.3)  # Slow down so we can see what's happening
            if self.time_elapsed % 5 == 0:
                self.log.append(f"Time {self.time_elapsed}: Moved to {next_pos}, fuel: {self.fuel_remaining}")
        return True

class DeliveryApp:
    def _init_(self, root):
        self.root = root
        self.root.title("Autonomous Delivery Agent")
        self.root.geometry("1000x700")
        # Variables
        self.env = None
        self.agent = None
        self.running = False
        # Build the interface
        self.create_widgets()

    def create_widgets(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="5")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        # Map selection
        ttk.Label(control_frame, text="Map:").grid(row=0, column=0, sticky=tk.W)
        self.map_var = tk.StringVar(value="small.map")
        map_combo = ttk.Combobox(control_frame, textvariable=self.map_var,
                                 values=["small.map", "medium.map", "large.map", "dynamic.map"],
                                 width=15)
        map_combo.grid(row=0, column=1, padx=5)
        # Algorithm selection
        ttk.Label(control_frame, text="Algorithm:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.algo_var = tk.StringVar(value="astar")
        algo_combo = ttk.Combobox(control_frame, textvariable=self.algo_var,
                                  values=["bfs", "ucs", "astar", "hillclimb", "simanneal"], width=15)
        algo_combo.grid(row=0, column=3, padx=5)
        # Buttons
        ttk.Button(control_frame, text="Run", command=self.run_simulation).grid(row=0, column=4, padx=10)
        ttk.Button(control_frame, text="Compare All", command=self.compare_algorithms).grid(row=0, column=5, padx=10)
        ttk.Button(control_frame, text="Reset", command=self.reset).grid(row=0, column=6, padx=10)
        # Visualization frame
        viz_frame = ttk.LabelFrame(main_frame, text="Visualization", padding="5")
        viz_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        # Canvas for showing the grid
        self.canvas = tk.Canvas(viz_frame, width=600, height=600, bg="white")
        self.canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        # Info frame
        info_frame = ttk.LabelFrame(main_frame, text="Information", padding="5")
        info_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        # Info text area
        self.info_text = tk.Text(info_frame, width=40, height=35)
        self.info_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        # Scrollbar for the text area
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.info_text.configure(yscrollcommand=scrollbar.set)
        # Set up grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=0)
        main_frame.rowconfigure(1, weight=1)
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)
        info_frame.columnconfigure(0, weight=1)
        info_frame.rowconfigure(0, weight=1)

    def update_gui(self, agent):
        """Update the display with the current state of the agent"""
        self.info_text.delete(1.0, tk.END)
        for log_entry in agent.log[-20:]:  # Show the last 20 log messages
            self.info_text.insert(tk.END, log_entry + "\n")
        self.info_text.see(tk.END)
        self.draw_grid(agent)
        self.root.update()

    def draw_grid(self, agent):
        """Draw the grid on the canvas"""
        self.canvas.delete("all")
        if not self.env:
            return
        cell_size = min(600 // self.env.width, 600 // self.env.height)
        # Draw each cell in the grid
        for i in range(self.env.height):
            for j in range(self.env.width):
                x1, y1 = j * cell_size, i * cell_size
                x2, y2 = x1 + cell_size, y1 + cell_size
                # Choose color based on cell type
                if self.env.grid[i][j].is_obstacle:
                    color = "black"
                elif self.env.grid[i][j].terrain_cost > 1:
                    color = "lightgreen"
                else:
                    color = "white"
                # Draw the cell
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                # Show terrain cost if it's higher than normal
                if self.env.grid[i][j].terrain_cost > 1:
                    self.canvas.create_text(x1 + cell_size//2, y1 + cell_size//2,
                                            text=str(self.env.grid[i][j].terrain_cost))
        # Draw packages
        for package_pos, destination in