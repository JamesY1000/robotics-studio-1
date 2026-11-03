import itertools
import math
import threading
from types import SimpleNamespace as NS

import pytest

from mission_control.ros_adapter import RosAdapter


def header(stamp=10):
    return NS(frame_id='world', stamp=NS(sec=stamp, nanosec=0))


def mission(identity='mission-1'):
    point = NS(x=0., y=0., z=0.)
    return NS(instance_id=identity, session_id='session', state='sampling',
              parrot_stage='Done', husky_stage='Measuring', message='', cleanup_complete=False, returning_home=False,
              control_mode='auto', manual_input='keyboard', input_revision=0, safety_instance_id='', safety_revision=0,
              roi=NS(header=header(), pose=NS(position=point)),
              roi_min=point, roi_max=NS(x=10., y=10., z=0.), roi_progress=1.)


def summary(identity='mission-1', stamp=10, **changes):
    values = dict(mission_id=identity, header=header(stamp), valid=True,
                  total_agb_kg=23.4, tree_count=2, measured_count=1, estimated_count=1,
                  message='Configured heights and wood densities')
    return NS(**(values | changes))


def adapter():
    value = RosAdapter.__new__(RosAdapter)
    value.lock = threading.Lock()
    value.data = {'mission': {}, 'trees': [], 'biomass': None, 'total_agb_kg': None}
    value.biomass_summaries = {}
    value.tree_map_key = None
    return value


@pytest.mark.parametrize('order', list(itertools.permutations(('mission', 'map', 'summary'))))
def test_late_subscriber_accepts_any_latched_arrival_order(order):
    value = adapter()
    callbacks = {'mission': lambda: value.on_mission(mission()),
                 'map': lambda: value.on_tree_map(NS(header=header(), trees=[])),
                 'summary': lambda: value.on_biomass(summary())}
    for name in order:
        callbacks[name]()
    assert value.data['total_agb_kg'] == 23.4


def test_new_mission_and_changed_map_do_not_reuse_previous_total():
    value = adapter()
    value.on_mission(mission())
    value.on_tree_map(NS(header=header(), trees=[]))
    value.on_biomass(summary())
    value.on_mission(mission('mission-2'))
    assert value.data['total_agb_kg'] is None
    value.on_biomass(summary())
    assert value.data['total_agb_kg'] is None
    value.on_tree_map(NS(header=header(20), trees=[]))
    value.on_biomass(summary('mission-2', 20, total_agb_kg=42.))
    assert value.data['total_agb_kg'] == 42.
    value.on_biomass(summary('mission-2', 10))
    assert value.data['total_agb_kg'] == 42.
    value.on_tree_map(NS(header=header(30), trees=[]))
    assert value.data['total_agb_kg'] is None
    value.on_biomass(summary('mission-2', 30, total_agb_kg=56.))
    assert value.data['total_agb_kg'] == 56.


@pytest.mark.parametrize('changes', [
    {'total_agb_kg': math.nan}, {'total_agb_kg': math.inf}, {'total_agb_kg': -1.},
    {'estimated_count': 2}, {'measured_count': 3}, {'valid': False},
])
def test_invalid_result_clears_the_display(changes):
    value = adapter()
    value.on_mission(mission())
    value.on_biomass(summary())
    assert value.data['total_agb_kg'] == 23.4
    value.on_biomass(summary(**changes))
    assert value.data['total_agb_kg'] is None and not value.data['biomass']['valid']


def test_unmeasured_or_unsupported_trees_are_not_displayed_as_zero_biomass():
    value = adapter()
    value.on_mission(mission())
    for measured in (0, 1):
        value.on_biomass(summary(total_agb_kg=0., estimated_count=0, measured_count=measured))
        assert value.data['total_agb_kg'] is None
        assert value.data['biomass']['measured_count'] == measured
