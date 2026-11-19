"""Experiment scheduling and validation; live collider behavior needs Isaac."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from navigation.missions import SINGLE_OBSTACLE
from navigation.geometry import obstacle_distance
from popup_experiment import PopupExperiment
from check_experiment import assess


class PopupTests(unittest.TestCase):
    def test_activation_uses_navigation_clock_once_and_changes_only_metrics_environment(self):
        enable=Mock(); p=PopupExperiment(SINGLE_OBSTACLE,enable,3.)
        run=SimpleNamespace(nav_start_s=None,env=None)
        p.prepare(run)
        # Position inside the future box is free before activation.
        self.assertGreater(obstacle_distance(run.env,[[4,4]])[0],0)
        state=dict(time_s=20.,position_m=[2.,4.,1.],linear_velocity_m_s=[.5,0,0])
        p.update(state,run); self.assertIsNone(p.event)
        run.nav_start_s=20.
        p.update(state,run); self.assertIsNone(p.event)
        state['time_s']=23.
        p.update(state,run); p.update(state,run)
        self.assertEqual(enable.call_count,2)  # disable, then one enable
        self.assertEqual(p.event['navigation_time_s'],3.)
        self.assertLess(obstacle_distance(run.env,[[4,4]])[0],0)
        self.assertIsNone(p.first_detection)
        p.observe([dict(sample_time_s=23.1,delivery_time_s=23.2,direction='front',
                        hit_prim='/World/Obstacles/Box',range_mm=1400)])
        self.assertEqual(p.first_detection['direction'],'front')

    def test_refuses_overlapping_insertion(self):
        p=PopupExperiment(SINGLE_OBSTACLE,Mock(),3.)
        with self.assertRaisesRegex(RuntimeError,'overlap'):
            p.update(dict(time_s=3.,position_m=[4.,4.,1.],linear_velocity_m_s=[0,0,0]),
                     SimpleNamespace(nav_start_s=0))
        self.assertIsNone(p.event)

    def test_disabled_collider_cannot_be_seen_before_appearance(self):
        p=PopupExperiment(SINGLE_OBSTACLE,Mock())
        with self.assertRaisesRegex(RuntimeError,'Inactive popup'):
            p.observe([dict(hit_prim='/World/Obstacles/Box')])

    def test_video_and_arrival_do_not_mask_collision_or_missing_popup_detection(self):
        data=dict(status='encoded',summary=dict(arrived=True,n_targets=1,targets_reached=1,
                  collided=False,unreachable=[]),popup=dict(appearance=dict(speed_m_s=.5,
                  clearance_at_appearance_m=1.),first_detection=None))
        self.assertFalse(assess(data,True)['passed'])
        data['popup']['first_detection']={'direction':'front'}
        self.assertTrue(assess(data,True)['passed'])
        data['summary']['collided']=True
        self.assertFalse(assess(data,True)['passed'])


if __name__=='__main__': unittest.main()
