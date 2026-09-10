#!/usr/bin/env python

"""Corridor overtake scenario (three-locale, two conflict zones).

Forked from scenario_1_1l.py. The single behavioural change is generalising
the stationary-blocker handling from ONE blocker (index 0) to N blockers
(the first ``CORRIDOR_BLOCKERS`` other_actors, default 2), so a corridor XML
can place one stopped blocker per conflict zone. Everything else -- the
oncoming velocity profile, ONCOMING_SPEED / FLOW_N / TRIGGER_DIST knobs, the
ConstantAccelWaypointFollower maneuvering panel -- is inherited unchanged.

The ego termination distance is env-configurable (CORRIDOR_DRIVE_DIST,
default 380 m) because a corridor route is far longer than the single-locale
overtake's 100 m.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenariomanager.timer import GameTime
from srunner.scenarios.basic_scenario import BasicScenario


class ConstantAccelWaypointFollower(WaypointFollower):
    """freeze-1l: WaypointFollower that ramps its target speed as a CONSTANT
    acceleration (m/s^2) from the moment the actor starts moving, capped at an
    optional ceiling.

    Reuses the base lane-following (LocalPlanner + PID); only the target speed
    is updated each tick as ``v(t) = v0 + a*t`` (clamped to ``[0, cap]``). The
    rising speed is therefore in the tracker's OBSERVED record, so a history-
    based migration can forecast the acceleration.
    """

    def __init__(self, actor, start_speed, acceleration, cap=None,
                 name="ConstantAccelFollower"):
        super(ConstantAccelWaypointFollower, self).__init__(
            actor, target_speed=start_speed, name=name)
        self._v0 = float(start_speed)
        self._accel = float(acceleration)
        self._cap = float(cap) if cap is not None else None
        self._accel_t0 = None

    def initialise(self):
        self._accel_t0 = GameTime.get_time()
        return super(ConstantAccelWaypointFollower, self).initialise()

    def update(self):
        if self._accel_t0 is not None:
            elapsed = GameTime.get_time() - self._accel_t0
            v = self._v0 + self._accel * elapsed
            if self._cap is not None:
                v = min(v, self._cap)
            v = max(0.0, v)
            self._target_speed = v
            lp = self._local_planner_dict.get(self._actor)
            if lp is not None and lp != "Walker":
                try:
                    lp.set_speed(v * 3.6)  # LocalPlanner target speed is km/h
                except Exception:  # noqa: BLE001
                    pass
        return super(ConstantAccelWaypointFollower, self).update()


class Scenario_1(BasicScenario):
    """Spawns N stationary blockers (conflict zones) plus an oncoming stream.

    self.other_actors[0 .. CORRIDOR_BLOCKERS-1] = stationary blockers
    self.other_actors[CORRIDOR_BLOCKERS ..]     = oncoming traffic
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, vehicle_index=-1, scenario_params=None, distributed=False):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Corridor Overtake Scenario")
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        import os as _os
        # Number of leading other_actors that are stationary blockers (one per
        # conflict zone). Default 2 for the two-conflict corridor; the flow
        # single-conflict XML would set CORRIDOR_BLOCKERS=1 (or run scenario_1).
        self._n_block = int(_os.environ.get('CORRIDOR_BLOCKERS', '2'))

        # Vehicle count follows the XML actor list. The first _n_block actors
        # are stationary blockers (velocity 0); the rest are oncoming traffic.
        self.num_vehicle = len(config.other_actors)
        _base = [(0 if i < self._n_block else 8)
                 for i in range(self.num_vehicle)]
        self.vehicle_velocities = list(_base)
        # ONCOMING_SPEED overrides every oncoming actor's constant speed (all but
        # the _n_block stationary blockers). A fast, constant oncoming is tracked
        # with good velocity SNR upstream and handed off at its true speed.
        _onc_spd = _os.environ.get('ONCOMING_SPEED')
        if _onc_spd is not None:
            _v = float(_onc_spd)
            self.vehicle_velocities = [
                (0 if i < self._n_block else _v)
                for i in range(self.num_vehicle)]
        # FLOW_N caps how many oncoming actors participate (first N of the
        # non-blocker actors lift; the rest stay parked): the crossings-per-
        # minute axis for the scale evaluation.
        _flow_n = _os.environ.get('FLOW_N')
        if _flow_n is not None:
            _n = int(_flow_n)
            for _i in range(self._n_block + _n, self.num_vehicle):
                self.vehicle_velocities[_i] = 0
        # Distance at which an actor begins moving (ego within this range of the
        # actor spawn). TRIGGER_DIST decouples the oncoming's start from the ego
        # start so the oncoming timeline (and its locale handoffs) is
        # independent of how far back the ego begins.
        self._trigger_distance = float(_os.environ.get('TRIGGER_DIST', 150))
        # Corridor route is ~400 m with two overtakes, so the ego drive-distance
        # end condition is far larger than the single-locale 100 m.
        self._drive_distance = float(_os.environ.get('CORRIDOR_DRIVE_DIST', 380))
        self.agents = []

        super(Scenario_1, self).__init__("Scenario_1",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Spawn vehicles
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=False)

        # Transformation that renders the vehicle visible
        for i in range(self.num_vehicle):
            car_transform = self.other_actors[i].get_transform()
            setattr(self, f"car_0{i + 1}_visible", carla.Transform(
                carla.Location(car_transform.location.x,
                               car_transform.location.y,
                               car_transform.location.z + 501),
                car_transform.rotation))

            # Trigger location for the actors
            setattr(self, f"vehicle_0{i + 1}_trigger_location", carla.Location(
                car_transform.location.x,
                car_transform.location.y,
                car_transform.location.z + 501, ))

    def _create_behavior(self):

        sequence_vehicle = []

        # Vehicle behaviors
        for i in range(self.num_vehicle):
            sequence_vehicle.append(py_trees.composites.Sequence(f"Vehicle_0{i + 1}"))
            trigger_location = getattr(self, f"vehicle_0{i + 1}_trigger_location")
            actor = self.other_actors[i]
            transform = getattr(self, f"car_0{i + 1}_visible")
            velocity = self.vehicle_velocities[i]

            trigger_behavior = InTriggerDistanceToLocation(self.ego_vehicles[0], trigger_location,
                                                           self._trigger_distance)
            set_transform_behavior = ActorTransformSetter(actor, transform)

            import os as _os
            # ONCOMING_ACCEL: FLOAT m/s^2 applied as constant acceleration from
            # start of motion (maneuvering panel). ONCOMING_STEP=1 keeps the
            # legacy exogenous ego-triggered speed step. Both apply only to
            # oncoming actors (index >= _n_block) that actually move.
            _step = _os.environ.get('ONCOMING_STEP', '0') == '1'
            _accel_raw = _os.environ.get('ONCOMING_ACCEL', '0')
            try:
                _accel_f = float(_accel_raw)
            except (TypeError, ValueError):
                _accel_f = 0.0
            _vcap = _os.environ.get('ONCOMING_VCAP')
            _vcap_f = float(_vcap) if _vcap else None
            if _step and i >= self._n_block and velocity > 0:
                # Aggressive oncoming: cruise slow, then floor it near the
                # conflict (exogenous step; kept as the step-limit control).
                car_transform = actor.get_transform()
                conflict_loc = carla.Location(
                    250.0, car_transform.location.y,
                    car_transform.location.z + 501)
                cruise_v = 5.0
                fast_v = 16.0
                phase1 = py_trees.composites.Parallel(
                    f"accel_gate_0{i + 1}",
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                phase1.add_child(WaypointFollower(actor, cruise_v))
                phase1.add_child(InTriggerDistanceToLocation(
                    self.ego_vehicles[0], conflict_loc, 30.0))
                fast = WaypointFollower(actor, fast_v)
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(phase1)
                sequence_vehicle[i].add_child(fast)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(fast)
            elif _accel_f > 0.0 and i >= self._n_block and velocity > 0:
                # Constant acceleration from start of motion (maneuvering panel).
                accel_follow = ConstantAccelWaypointFollower(
                    actor, velocity, _accel_f, cap=_vcap_f,
                    name=f"const_accel_0{i + 1}")
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(accel_follow)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(accel_follow)
            else:
                drive_behavior = WaypointFollower(actor, velocity)
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(drive_behavior)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(drive_behavior)

        # End condition: ego traverses the corridor route (env-configurable).
        termination = DriveDistance(self.ego_vehicles[0], self._drive_distance)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i in range(self.num_vehicle):
            root.add_child(sequence_vehicle[i])
        root.add_child(termination)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
