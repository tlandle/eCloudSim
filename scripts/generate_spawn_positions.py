import random
import os
from jinja2 import Environment, FileSystemLoader

# Add a seed value for random number generation
random.seed(42)

def generate_spawn_positions(num_cars, road_left, road_right, spawn=True):
    positions = []

    for _ in range(num_cars):
        x = random.uniform(-20, -200)
        y = random.uniform(road_left, road_right)

        while any(is_too_close(x, y, pos[0], pos[1], min_distance=5) for pos in positions) or not is_within_road(x, y, road_left, road_right):
            x = random.uniform(-20, -200)
            y = random.uniform(road_left, road_right)

        if spawn:
            positions.append([x, y, 0.3, 0, 180, 0])
        else:
            # Make the destinations some location ahead of the road
            positions.append([x + random.uniform(120, 200), y, 0.3, 0, 180, 0])

    return positions

def is_too_close(x1, y1, x2, y2, min_distance=1):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 < min_distance

def is_within_road(x, y, road_left, road_right):
    return road_left <= y <= road_right


num_cars = int(input("Enter the number of vehicles to generate: "))
road_left = -25
road_right = -12
spawn_positions = generate_spawn_positions(num_cars, road_left, road_right)
destinations = generate_spawn_positions(num_cars, road_left, road_right, spawn=False)

# Generate the vehicles as a list of dictionaries
vehicles = []
for i in range(num_cars):
    vehicles.append({
        "spawn_position": spawn_positions[i],
        "destination": destinations[i]
    })
    
# Get the absolute path to the templates directory
template_dir = os.path.abspath('../templates')

# Load the Jinja2 template and render it
env = Environment(loader=FileSystemLoader(template_dir))
template = env.get_template('ecloud_dist_template.yaml')
output_yaml = template.render(vehicles=vehicles)

output_file_name = f"../opencda/scenario_testing/config_yaml/ecloud_4lane_dist_{num_cars}_car.yaml"
with open(output_file_name, "w") as file:
    file.write(output_yaml)

print(f"Generated YAML file with {num_cars} vehicles: {output_file_name}")
