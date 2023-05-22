import carla
import random
import os
from agents.navigation.global_route_planner import GlobalRoutePlanner
from jinja2 import FileSystemLoader, Environment
from opencda.scenario_testing.utils.yaml_utils import load_yaml

# Add a seed value for random number generation
random.seed(42)

def filter_spawn_points(spawn_points, x_range=(40, 200), y_range=(130, 160)):
    valid_points = []
    for point in spawn_points:
        if x_range[0] <= point.location.x <= x_range[1] and y_range[0] <= point.location.y <= y_range[1]:
            valid_points.append(point)
    return valid_points


def filter_spawn_points_for_destinations(spawn_points, start_x, y_range=(137, 154)):
    valid_points = []
    for point in spawn_points:
        if point.location.x > start_x and y_range[0] <= point.location.y <= y_range[1]:
            valid_points.append(point)
    return valid_points


def generate_spawn_positions(num_cars, valid_spawn_points, spawn=True, dest_proximity=10):
    if len(valid_spawn_points) < num_cars:
        raise ValueError("Not enough unique locations for all vehicles.")
    
    positions = []
    chosen_points = random.sample(valid_spawn_points, num_cars)  # choose unique random spawn points

    first_dest = None

    for point in chosen_points:
        x, y = point.location.x, point.location.y
        if spawn:
            positions.append([x, y, 0.3, 0, 0, 0])
        else:
            # Generate valid destinations
            valid_destinations = filter_spawn_points_for_destinations(spawn_points, x, y_range=(137, 154))

            if not valid_destinations:
                raise ValueError(f"No valid destinations ahead of spawn point {x}, {y}")

            # if it's the first destination, choose any
            if first_dest is None:
                first_dest = random.choice(valid_destinations)
                x, y = first_dest.location.x, first_dest.location.y
            else:
                # for subsequent vehicles, their destination is chosen to be within `dest_proximity` of the first destination
                valid_destinations = [point for point in valid_destinations if ((point.location.x - first_dest.location.x) ** 2 + (point.location.y - first_dest.location.y) ** 2) ** 0.5 < dest_proximity]
                if not valid_destinations:
                    raise ValueError(f"No valid destinations close to the first destination {first_dest.location.x}, {first_dest.location.y}")

                destination_point = random.choice(valid_destinations)
                x, y = destination_point.location.x, destination_point.location.y

            positions.append([x, y, 0.3, 0, 0, 0])

    return positions


def choose_template():
    template_dir = os.path.abspath('./templates')
    templates = [file for file in os.listdir(template_dir) if file.endswith('.yaml')]
    for i, template in enumerate(templates, start=1):
        print(f"{i}. {template}")
    choice = int(input("Choose a template by number: "))
    return templates[choice - 1]

cloud_config = load_yaml("cloud_config.yaml")
CARLA_IP = cloud_config["carla_server_public_ip"]    

client = carla.Client(CARLA_IP, 2000)
client.set_timeout(10)
world = client.load_world('Town06')
amap = world.get_map()
sampling_resolution = 2
dao = GlobalRoutePlanner(amap, sampling_resolution)

spawn_points = world.get_map().get_spawn_points()
filtered_spawn_points = filter_spawn_points(spawn_points, x_range=(40, 200), y_range=(137, 154))
for point in filtered_spawn_points:
    print(point)

num_cars = int(input("Enter the number of vehicles to generate: "))
spawn_positions = generate_spawn_positions(num_cars, filtered_spawn_points, spawn=True)
destinations = generate_spawn_positions(num_cars, spawn_points, spawn=False)  # Note that we're passing all spawn_points here

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
