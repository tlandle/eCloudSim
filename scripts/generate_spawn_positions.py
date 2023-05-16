import random
import os
from jinja2 import FileSystemLoader, Environment

# Add a seed value for random number generation
random.seed(42)

def generate_spawn_positions(num_cars, road_left, road_right, spawn=True, x_range=(-20, -200), destination_x_range=(120, 200)):
    positions = []

    for _ in range(num_cars):
        x = random.uniform(*x_range)
        y = random.uniform(road_left, road_right)

        while any(is_too_close(x, y, pos[0], pos[1], min_distance=5) for pos in positions) or not is_within_road(x, y, road_left, road_right):
            x = random.uniform(*x_range)
            y = random.uniform(road_left, road_right)

        if spawn:
            positions.append([x, y, 0.3, 0, 0, 0])
        else:
            # Change the x coordinate for the destination position
            x += random.uniform(*destination_x_range)
            positions.append([x, y, 0.3, 0, 0, 0])

    return positions

def is_too_close(x1, y1, x2, y2, min_distance=1):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 < min_distance

def is_within_road(x, y, road_left, road_right):
    return road_left <= y <= road_right

def choose_template():
    template_dir = os.path.abspath('./templates')
    templates = [file for file in os.listdir(template_dir) if file.endswith('.yaml')]
    for i, template in enumerate(templates, start=1):
        print(f"{i}. {template}")
    choice = int(input("Choose a template by number: "))
    return templates[choice - 1]

num_cars = int(input("Enter the number of vehicles to generate: "))
road_left = 139
road_right = 152
spawn_positions = generate_spawn_positions(num_cars, road_left, road_right, x_range=(30, 200))
destinations = generate_spawn_positions(num_cars, road_left, road_right, spawn=False, x_range=(30, 200), destination_x_range=(200, 300))

# Generate the vehicles as a list of dictionaries
vehicles = []
for i in range(num_cars):
    vehicles.append({
        "spawn_position": spawn_positions[i],
        "destination": destinations[i]
    })
    
# Get the absolute path to the templates directory
template_dir = os.path.abspath('./templates')

# Ask the user to choose a template
template_file = choose_template()

# Load the Jinja2 template and render it
env = Environment(loader=FileSystemLoader(template_dir))
template = env.get_template(template_file)
output_yaml = template.render(vehicles=vehicles)

output_file_name = f"./opencda/scenario_testing/config_yaml/ecloud_4lane_dist_{num_cars}_car.yaml"
with open(output_file_name, "w") as file:
    file.write(output_yaml)

print(f"Generated YAML file with {num_cars} vehicles: {output_file_name}")

# Also create a copy of the chosen template
template_python_file_name = f"{template_file.rsplit('.', 1)[0]}.py"
output_py_file_name = f"./opencda/scenario_testing/ecloud_4lane_dist_{num_cars}_car.py"
with open(f'{template_dir}/{template_python_file_name}', 'r') as source:
    with open(output_py_file_name, 'w') as target:
        target.write(source.read())

print(f"Generated Python file from the template: {output_py_file_name}")