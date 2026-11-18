from types import SimpleNamespace as NS

import pytest

from omslam_system_monitor.sampling import HostSampler


def proc_fixture(tmp_path):
    proc=tmp_path/'proc';sys=tmp_path/'sys'
    (proc/'net').mkdir(parents=True);sys.mkdir()
    (proc/'stat').write_text('cpu 100 0 100 800 0 0 0 0 500 0\n')
    (proc/'meminfo').write_text('MemTotal: 1000 kB\nMemFree: 10 kB\nMemAvailable: 400 kB\n')
    (proc/'net'/'dev').write_text('eth0: 100 1 2 3 0 0 0 0 200 2 4 5 0 0 0 0\n')
    disk=NS(f_frsize=4096,f_blocks=100,f_bfree=40,f_bavail=35)
    return HostSampler(proc,sys,'/fixture',lambda path:disk),proc,sys


def test_resource_units_cpu_guest_accounting_and_network_rates(tmp_path):
    monitor,proc,sys=proc_fixture(tmp_path)
    first={sample.category:sample for sample in monitor.sample(10.)}
    assert not first['cpu'].valid and first['cpu'].values=={}
    assert first['memory'].values['used_bytes']==600*1024  # MemAvailable, not MemFree
    assert first['disk'].values['used_bytes']==60*4096
    assert first['disk'].values['available_bytes']==35*4096
    assert not first['temperature'].valid and first['temperature'].values=={}
    assert 'interface.eth0.rx_bytes_per_s' not in first['network'].values
    (proc/'stat').write_text('cpu 120 0 120 860 0 0 0 0 560 0\n')
    (proc/'net'/'dev').write_text('eth0: 500 5 2 3 0 0 0 0 800 8 4 5 0 0 0 0\n')
    zone=sys/'class'/'thermal'/'thermal_zone0';zone.mkdir(parents=True)
    (zone/'temp').write_text('42500\n');(zone/'type').write_text('cpu-package\n')
    second={sample.category:sample for sample in monitor.sample(12.)}
    assert second['cpu'].values['busy_percent']==pytest.approx(40.)  # guest excluded from sum
    assert second['network'].values['interface.eth0.rx_bytes_per_s']==200.
    assert second['network'].values['interface.eth0.tx_bytes_per_s']==300.
    assert second['temperature'].values['sensor.thermal_zone0.celsius']==42.5
    assert 'motor' in second['temperature'].message


def test_unavailable_reset_and_counter_rollbacks_never_become_zero_measurements(tmp_path):
    monitor,proc,sys=proc_fixture(tmp_path)
    monitor.sample(1.)
    (proc/'stat').write_text('cpu 1 0 1 8 0 0 0 0\n')
    (proc/'meminfo').write_text('MemTotal: 1000 kB\n')
    (proc/'net'/'dev').write_text('eth0: 1 0 0 0 0 0 0 0 2 0 0 0 0 0 0 0\n')
    monitor.statvfs=lambda path:(_ for _ in ()).throw(OSError('unmounted'))
    result={sample.category:sample for sample in monitor.sample(2.)}
    for category in ('cpu','memory','disk','temperature'):
        assert not result[category].valid and result[category].values=={}
    assert result['network'].valid
    assert 'interface.eth0.rx_bytes_per_s' not in result['network'].values
    assert 'interface.eth0.tx_bytes_per_s' not in result['network'].values
    (proc/'net'/'dev').unlink()
    assert not monitor.network(3.).valid and monitor.previous_network is None
