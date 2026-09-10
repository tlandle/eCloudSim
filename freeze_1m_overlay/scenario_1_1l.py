#!/usr/bin/env python

"""
Overtake Scenario:

The scripts simulate a scenario where an ego vehicle has to overtake a background vehicle
that is ahead of the ego vehicle and at a lower speed. There are two fearless pedestrians
that suddenly appear in front of the ego vehicle and the ego vehicle has to avoid a collision
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
    based migration can forecast the acceleration. This is the maneuvering-panel
    replacement for the exogenous, ego-triggered step (kept under ONCOMING_STEP=1
    as the exogenous-step-limit control), which is not in any actor history and
    so is unpredictable by construction.
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
    """
    The class spawns two background vehicles and two pedestrians in front of the ego vehicle.
    The ego vehicle is driving behind and overtaking the fast vehicle ahead

    self.other_actors[0] = fast car
    self.other_actors[1] = slow car
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, vehicle_index=-1, scenario_params=None, distributed=False):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Overtake Scenario")
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        # Vehicle count follows the XML actor list so denser oncoming
        # streams (two-locale variant) need only more <other_actor> rows.
        # First actor is the stationary blocker (velocity 0); the rest are
        # oncoming traffic. The single-locale XML (4 actors) keeps its
        # original [0, 8, 6, 6] profile exactly.
        self.num_vehicle = len(config.other_actors)
        _base = [0, 8, 6, 6]
        self.vehicle_velocities = [
            (_base[i] if i < len(_base) else 8)
            for i in range(self.num_vehicle)]
        # ONCOMING_SPEED overrides every oncoming actor's constant speed (all
        # but the stationary blocker at index 0). A fast, constant oncoming is
        # tracked with good velocity SNR upstream and handed off at its true
        # speed, which the timing-necessity arms (predictive vs reactive
        # migration) need; the default profile is unchanged when unset.
        import os as _os
        _onc_spd = _os.environ.get('ONCOMING_SPEED')
        if _onc_spd is not None:
            _v = float(_onc_spd)
            self.vehicle_velocities = [
                (0 if i == 0 else _v) for i in range(self.num_vehicle)]
        # FLOW_N caps how many oncoming actors participate (first N lift
        # from their parked pose; the rest stay parked): the crossings-per-
        # minute axis for the scale evaluation, one knob on one XML.
        _flow_n = _os.environ.get('FLOW_N')
        if _flow_n is not None:
            _n = int(_flow_n)
            for _i in range(1 + _n, self.num_vehicle):
                self.vehicle_velocities[_i] = 0
        # Distance at which an actor begins moving (ego within this range of
        # the actor spawn). TRIGGER_DIST decouples the oncoming's start from
        # the ego start so the oncoming's timeline (and its locale handoff) is
        # independent of how far back the ego begins; the two-locale accel
        # scenario needs the handoff to precede the ego's overtake commit.
        self._trigger_distance = float(_os.environ.get('TRIGGER_DIST', 150))
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
        # freeze-1m maneuvering panel: ONCOMING_SHIFT_X shifts ONLY the oncoming
        # actors (i >= 1) along x, so the oncoming crosses the destination boundary
        # nearer the conflict (shorter T_avail = fewer destination frames before the
        # ego's overtake decision). The truck / conflict (i == 0, x=278) and the ego
        # are NOT moved, so the ego's approach is identical across shifts. Positive
        # shift moves the eastbound oncoming's spawn toward the conflict; the
        # realized T_avail per shift is measured from the probe, not assumed.
        import os as _os_shift
        _onc_shift = float(_os_shift.environ.get('ONCOMING_SHIFT_X', '0') or 0.0)
        # freeze-1m PAIRED SPAWN-PHASE SEED (Tyler): the conflict scenario is
        # otherwise fully deterministic, so "seeds" were timing-jitter reps, not
        # samples. A per-run seed INDEX draws a deterministic offset on the
        # oncoming stream's spawn x (the whole stream shifts together = a phase
        # shift), identical across arms for the same index (paired), applied on
        # top of ONCOMING_SHIFT_X. So N seeds = N independent samples of the
        # spawn-phase distribution around the operating point, and the paper's
        # paired-seed intervals mean what they claim. seed 0 / unset = 0 offset
        # (base scenario, backward compatible). Band = KHONSU_SEED_BAND_M (m).
        import random as _rnd_seed
        _seed_idx = int(_os_shift.environ.get('KHONSU_SEED', '0') or 0)
        _seed_band = float(_os_shift.environ.get('KHONSU_SEED_BAND_M', '4.0') or 4.0)
        _seed_off = (_rnd_seed.Random(_seed_idx).uniform(-_seed_band, _seed_band)
                     if _seed_idx > 0 else 0.0)
        try:
            import logging as _lg_seed
            _lg_seed.getLogger(__name__).info(
                "[SEEDROW] seed_idx=%d band_m=%.2f spawn_offset_m=%.3f "
                "base_shift_m=%.1f eff_shift_m=%.3f",
                _seed_idx, _seed_band, _seed_off, _onc_shift,
                _onc_shift + _seed_off)
        except Exception:  # noqa: BLE001
            pass
        for i in range(self.num_vehicle):
            car_transform = self.other_actors[i].get_transform()
            _sx = (_onc_shift + _seed_off) if i >= 1 else 0.0
            setattr(self, f"car_0{i + 1}_visible", carla.Transform(
                carla.Location(car_transform.location.x + _sx,
                               car_transform.location.y,
                               car_transform.location.z + 501),
                car_transform.rotation))

            # Trigger location for the actors (shifted with the oncoming spawn)
            setattr(self, f"vehicle_0{i + 1}_trigger_location", carla.Location(
                car_transform.location.x + _sx,
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
            # freeze-1l: ONCOMING_ACCEL is now a FLOAT m/s^2 applied as constant
            # acceleration from start of motion (the maneuvering panel; rising
            # speed enters the observed record). The legacy exogenous ego-
            # triggered STEP (cruise -> floor near the ego) is kept for the
            # exogenous-step-limit control and now moves ONLY under ONCOMING_STEP=1.
            _step = _os.environ.get('ONCOMING_STEP', '0') == '1'
            _accel_raw = _os.environ.get('ONCOMING_ACCEL', '0')
            try:
                _accel_f = float(_accel_raw)
            except (TypeError, ValueError):
                _accel_f = 0.0
            _vcap = _os.environ.get('ONCOMING_VCAP')
            _vcap_f = float(_vcap) if _vcap else None
            # freeze-1m maneuvering panel: ENDOGENOUS acceleration onset. The
            # oncoming cruises slow (ONCOMING_CRUISE) until IT reaches the onset
            # point on ITS OWN route (ONCOMING_ACCEL_ONSET, an x-coord), then
            # accelerates from the cruise speed. Keyed on the ACTOR, not the ego,
            # so the onset is a property of the oncoming's route (not exogenous
            # like the ego-keyed STEP), the source locale observes the first
            # frames of the rise (history to migrate), and the ego decides while
            # the vehicle is still slow (CV says GO, the accel-aware history says
            # WAIT). Active only when ONCOMING_ACCEL_ONSET is set; else the accel
            # branch below is the from-spawn (ego-triggered) version.
            _cruise_f = float(_os.environ.get('ONCOMING_CRUISE', '5') or 5.0)
            _onset = _os.environ.get('ONCOMING_ACCEL_ONSET')
            _onset_f = float(_onset) if _onset else None
            if _step and i >= 1 and velocity > 0:
                # Aggressive oncoming: cruise slow, then floor it near the
                # conflict. A box+velocity SNAPSHOT migrated upstream (kf arm)
                # captures only the slow cruise speed, so the destination
                # predicts constant velocity and the ego thinks it has a gap;
                # the actor then accelerates and arrives early -> collision.
                # The full memo-bank latent carries the motion history, so the
                # destination predictor sees the acceleration and the ego waits.
                car_transform = actor.get_transform()
                conflict_loc = carla.Location(
                    278.0, car_transform.location.y,
                    car_transform.location.z + 501)
                cruise_v = 5.0     # slow approach; a snapshot sees this speed
                fast_v = 16.0      # floor it as the ego reaches the conflict
                # The oncoming cruises upstream (tracked and migrated by the
                # previous locale) and accelerates when the EGO nears the
                # conflict, so the fast approach overlaps the overtake window
                # regardless of the ego's exact deceleration profile. This is
                # the aggressive-driver case: a full-latent migration recovers
                # the accelerating speed and the ego waits; a snapshot
                # under-predicts it and the ego commits into a closing gap.
                phase1 = py_trees.composites.Parallel(
                    f"accel_gate_0{i + 1}",
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                phase1.add_child(WaypointFollower(actor, cruise_v))  # cruise
                phase1.add_child(InTriggerDistanceToLocation(
                    self.ego_vehicles[0], conflict_loc, 30.0))       # ego near conflict
                fast = WaypointFollower(actor, fast_v)               # floor it
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(phase1)
                sequence_vehicle[i].add_child(fast)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(fast)
            elif (_accel_f > 0.0 and _onset_f is not None
                  and i >= 1 and velocity > 0):
                # freeze-1m maneuvering panel (endogenous onset): cruise slow at
                # ONCOMING_CRUISE until the ACTOR reaches x=ONCOMING_ACCEL_ONSET on
                # its own route, then constant-accelerate from the cruise speed
                # (cap ONCOMING_VCAP). Placed shortly before the boundary so the
                # source observes the rise and the ego decides while slow: a queue
                # tail / stop-line departure a constant-velocity model is wrong
                # about for a physical reason (paper 2.3).
                car_transform = actor.get_transform()
                onset_loc = carla.Location(
                    _onset_f, car_transform.location.y,
                    car_transform.location.z + 501)
                phase1 = py_trees.composites.Parallel(
                    f"cruise_gate_0{i + 1}",
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                phase1.add_child(WaypointFollower(actor, _cruise_f))   # cruise slow
                phase1.add_child(InTriggerDistanceToLocation(
                    actor, onset_loc, 5.0))            # ACTOR reaches its onset point
                accel_follow = ConstantAccelWaypointFollower(
                    actor, _cruise_f, _accel_f, cap=_vcap_f,
                    name=f"cruise_accel_0{i + 1}")     # accelerate FROM the cruise speed
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(phase1)
                sequence_vehicle[i].add_child(accel_follow)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(accel_follow)
            elif _accel_f > 0.0 and i >= 1 and velocity > 0:
                # freeze-1l maneuvering panel: constant acceleration from start
                # of motion. The oncoming lifts off at its start speed (velocity,
                # i.e. ONCOMING_SPEED) and accelerates at _accel_f m/s^2, so the
                # rising speed is in the observed record by PREPARE and a
                # history-based migration can forecast it. Optional ONCOMING_VCAP
                # caps the top speed; pick v0/cap so the actor is still
                # accelerating at the launch decision and at the crossing.
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

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 100)

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
