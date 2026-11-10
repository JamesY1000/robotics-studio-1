import itertools
from types import SimpleNamespace as NS

import pytest

from test_biomass import adapter, header, mission


def target(**changes):
    values = dict(header=header(), mission_id='mission-1', tree_index=0,
                  centre=NS(x=3.2, y=4.1), refined=True, active=True,
                  min_range_m=1., max_range_m=4.)
    return NS(**(values | changes))


@pytest.mark.parametrize('order', list(itertools.permutations(('mission', 'map', 'target'))))
def test_selected_tree_survives_latched_arrival_order(order):
    value = adapter()
    callbacks = {
        'mission': lambda: value.on_mission(mission()),
        'map': lambda: value.on_tree_map(NS(header=header(), trees=[
            NS(location=NS(x=3., y=4., z=0.), tree_type='PINE_TREE', observations=3)])),
        'target': lambda: value.on_sampling_target(target()),
    }
    for name in order:
        callbacks[name]()
    assert value.data['sampling_target']['x'] == 3.2
    assert value.data['sampling_target']['refined']
    value.on_sampling_target(target(active=False))
    assert value.data['sampling_target'] is None


@pytest.mark.parametrize('changes', [dict(mission_id='old'), dict(header=header(3)),
                                    dict(tree_index=1), dict(max_range_m=float('nan'))])
def test_invalid_or_stale_targets_are_hidden(changes):
    value = adapter()
    value.on_mission(mission())
    value.tree_map_key = value.map_key(header())
    value.data['trees'] = [{}]
    value.on_sampling_target(target(**changes))
    assert value.data['sampling_target'] is None
