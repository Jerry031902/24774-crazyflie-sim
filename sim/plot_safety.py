#!/usr/bin/env python3
"""Plot the optional Safety columns from an Isaac mission flight CSV (no GPU)."""

import argparse
from pathlib import Path
import numpy as np


def plot_safety(path):
    import matplotlib.pyplot as plt
    path=Path(path)
    d=np.genfromtxt(path,delimiter=',',names=True,dtype=None,encoding='utf-8')
    d=np.atleast_1d(d)
    if 'safety_state' not in d.dtype.names:
        raise ValueError('This log has no Safety columns; run a mission with --safety on')
    t=d['time_s']; states={'NORMAL':1,'BRAKE':2,'HOLD':3,'RECOVER':4}
    fig,axes=plt.subplots(3,2,figsize=(14,9),sharex=True)
    ax=axes[0,0]
    for key,label in [('safety_nominal_speed_m_s','Nominal'),('safety_candidate_speed_m_s','Candidate'),
                      ('safety_selected_speed_m_s','Selected')]: ax.plot(t,d[key],label=label,lw=1)
    ax.plot(t,np.hypot(d['vx_m_s'],d['vy_m_s']),label='True ground speed',color='black',lw=1)
    ax.set_ylabel('Speed (m/s)'); ax.legend(fontsize=8)
    ax=axes[0,1]
    ax.plot(t,d['safety_primary_margin_m'],label='Primary')
    ax.plot(t,d['safety_backup_margin_m'],label='Backup')
    ax.axhline(0,color='red',ls='--'); ax.set_ylabel('Distance margin (m)'); ax.legend()
    ax=axes[1,0]
    ax.step(t,[states[s] for s in d['safety_state']],where='post')
    ax.set_yticks(list(states.values()),list(states)); ax.set_ylabel('Safety state')
    ax=axes[1,1]
    ax.plot(t,d['safety_ranger_age_s']); ax.axhline(.35,color='red',ls='--')
    ax.set_ylabel('Sample age (s)')
    ax=axes[2,0]
    ax.step(t,d['safety_primary_trip'],label='Primary',where='post')
    ax.step(t,2*d['safety_backup_trip'],label='Backup (2)',where='post')
    ax.set_ylabel('Triggers'); ax.legend()
    ax=axes[2,1]
    for name in ['front','left','back','right']:
        ax.plot(t,np.minimum(d[f'range_{name}_m'],4),label=name,lw=.8)
    ax.set_ylabel('Processed ranges (m)'); ax.legend(fontsize=8)
    for ax in axes.flat: ax.grid(alpha=.2); ax.set_xlabel('Simulation time (s)')
    fig.suptitle(path.name+' | horizontal Safety supervisor')
    fig.tight_layout()
    output=path.with_suffix('.safety.png'); fig.savefig(output,dpi=150); plt.close(fig)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('csv',type=Path)
    print(plot_safety(parser.parse_args().csv))
