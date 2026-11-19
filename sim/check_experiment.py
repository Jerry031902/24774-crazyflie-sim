#!/usr/bin/env python3
"""Assess recorded runs; a produced MP4 alone does not mean the test passed."""
import argparse
import json
from pathlib import Path


def assess(data, video_exists):
    s = data.get('summary', {})
    checks = dict(recorded=data.get('status')=='encoded' and video_exists,
                  reached_goal=s.get('arrived') is True,
                  all_targets=s.get('n_targets',0)>0 and s.get('targets_reached')==s.get('n_targets'),
                  no_collision=s.get('collided') is False,
                  no_unreachable_targets=not s.get('unreachable',[]),
                  no_safety_error=not s.get('safety_last_error',''))
    if 'popup' in data:
        p = data['popup']; event = p.get('appearance') or {}
        checks.update(popup_appeared=bool(event),
                      popup_while_moving=event.get('speed_m_s',0)>.05,
                      popup_did_not_overlap=event.get('clearance_at_appearance_m',0)>0,
                      popup_seen_by_ranger=bool(p.get('first_detection')))
    return dict(passed=all(checks.values()),checks=checks,summary=s,
                scope='One noise-free firmware-PID/truth-feedback run; not hardware or CBF validation')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    data=json.loads((args.directory/'capture.json').read_text())
    result=assess(data,(args.directory/'flight.mp4').is_file())
    (args.directory/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__': main()
