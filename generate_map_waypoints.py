import carla
import numpy as np

from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

def check_if_within_dist(waypoint, x, y, ncars=4):
    for i in range(0,ncars):
        dist = np.sqrt((waypoint.transform.location.x - x[i])**2 + (waypoint.transform.location.y - y[i])**2)
        if dist <= 0.5:
            return i
    return 1000

client = carla.Client("localhost", 2000)
client.set_timeout(10)
world = client.load_world('Town06')
amap = world.get_map()
sampling_resolution = 2
dao = GlobalRoutePlanner(amap, sampling_resolution)
# grp = GlobalRoutePlanner(dao)
# grp.setup()
spawn_points = world.get_map().get_spawn_points()
#for spawn_point in spawn_points:
    #print(spawn_point)
w1 = world.get_map().generate_waypoints(10)
#a = carla.Location(waypoints[219].transform.location)
#b = carla.Location(waypoints[141].transform.location)
#w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
i = 0
final_out = np.empty((0,3))

xsearch = [-39.52, -39.52, -39.52, -39.60]
ysearch = [-22.79, -19.292, -15.79, -12.29]

xdest = [-147.759079, -147.74, -147.73, -147.72]
ydest = [-22.39, -18.89, -15.79, -12.29]

indice_list_init = np.zeros((4,2))
indice_list_dest = np.zeros((4,2))

for w in w1:
    print(w)
    temp_array = np.array([i,w.transform.location.x,w.transform.location.y])
    final_out = np.vstack((final_out,temp_array.ravel()))
    mark=str(i)
    loc = check_if_within_dist(w, xsearch, ysearch)
    loc_dest = check_if_within_dist(w, xdest, ydest)
    if loc != 1000:
        indice_list_init[loc,:] = np.array([loc,i]).ravel()
        xsearch[loc] = -10000
        ysearch[loc] = -10000
    elif loc_dest != 1000:
        indice_list_dest[loc_dest,:] = np.array([loc_dest,i]).ravel()
        xdest[loc_dest] = -10000
        ydest[loc_dest] = -10000        

    if i % 10 == 0:
        world.debug.draw_string(w.transform.location,mark, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=120.0, persistent_lines=True)
    else:
        world.debug.draw_string(w.transform.location, mark, draw_shadow=False,
        color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
        persistent_lines=True)
    i += 1

print(indice_list_dest)
print(indice_list_init)
np.save('Indices_start.npy',indice_list_init)
np.save('Indices_dest.npy', indice_list_dest)
print(final_out.shape)
np.save('waypoints.npy',final_out)