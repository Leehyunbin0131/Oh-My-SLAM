"""Small, nonblocking Linux snapshots; no subprocesses, devices, or probes."""
from dataclasses import dataclass
from pathlib import Path
import os
import time


@dataclass(frozen=True)
class ResourceSample:
    category: str
    valid: bool
    source: str
    message: str
    values: dict


class HostSampler:
    def __init__(self, proc_root='/proc', sys_root='/sys', disk_path='/', statvfs=os.statvfs):
        self.proc = Path(proc_root)
        self.sys = Path(sys_root)
        self.disk_path = str(disk_path)
        self.statvfs = statvfs
        self.previous_cpu = None
        self.previous_network = None

    @staticmethod
    def unavailable(category, source, reason):
        return ResourceSample(category, False, source, str(reason), {})

    def cpu(self):
        source = str(self.proc / 'stat')
        try:
            line = (self.proc / 'stat').read_text().splitlines()[0].split()
            if line[0] != 'cpu' or len(line) < 5:
                raise ValueError('aggregate CPU counters unavailable')
            # guest and guest_nice are already included in user/nice.
            ticks = [int(v) for v in line[1:9]]
            if any(v < 0 for v in ticks):
                raise ValueError('negative CPU counter')
            total, idle = sum(ticks), ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
            previous, self.previous_cpu = self.previous_cpu, (total, idle)
            if previous is None:
                return self.unavailable('cpu', source, 'waiting for second CPU counter sample')
            elapsed, idle_elapsed = total - previous[0], idle - previous[1]
            if elapsed <= 0 or idle_elapsed < 0 or idle_elapsed > elapsed:
                return self.unavailable('cpu', source, 'CPU counters reset or did not advance')
            return ResourceSample('cpu', True, source, 'CPU utilization measured over sampling interval',
                                  {'busy_percent': 100. * (elapsed - idle_elapsed) / elapsed})
        except (OSError, ValueError, IndexError) as exc:
            self.previous_cpu = None
            return self.unavailable('cpu', source, exc)

    def memory(self):
        source = str(self.proc / 'meminfo')
        try:
            values = {}
            for line in (self.proc / 'meminfo').read_text().splitlines():
                name, value = line.split(':', 1)
                if name in ('MemTotal', 'MemAvailable'):
                    parts = value.split()
                    if len(parts) != 2 or parts[1] != 'kB':
                        raise ValueError('unsupported memory unit')
                    values[name] = int(parts[0]) * 1024
            total, available = values['MemTotal'], values['MemAvailable']
            if total <= 0 or not 0 <= available <= total:
                raise ValueError('invalid available memory')
            return ResourceSample('memory', True, source, 'Linux MemAvailable measurement', {
                'total_bytes': total, 'available_bytes': available, 'used_bytes': total - available,
                'used_percent': 100. * (total - available) / total})
        except (OSError, ValueError, KeyError) as exc:
            return self.unavailable('memory', source, exc)

    def disk(self):
        source = 'statvfs:' + self.disk_path
        try:
            stat = self.statvfs(self.disk_path)
            total, free, available = (stat.f_frsize * n for n in (stat.f_blocks, stat.f_bfree, stat.f_bavail))
            if total <= 0 or not 0 <= available <= free <= total:
                raise ValueError('invalid filesystem statistics')
            return ResourceSample('disk', True, source, 'Filesystem capacity at configured path', {
                'path': self.disk_path, 'total_bytes': total, 'free_bytes': free,
                'available_bytes': available, 'used_bytes': total - free,
                'used_percent': 100. * (total - free) / total})
        except (OSError, ValueError) as exc:
            return self.unavailable('disk', source, exc)

    def temperature(self):
        source = str(self.sys / 'class')
        values = {}
        candidates = []
        try:
            for directory in sorted((self.sys / 'class' / 'thermal').glob('thermal_zone*'))[:32]:
                label_path = directory / 'type'
                candidates.append((directory.name, directory / 'temp', label_path))
            for directory in sorted((self.sys / 'class' / 'hwmon').glob('hwmon*'))[:16]:
                for path in sorted(directory.glob('temp*_input'))[:16]:
                    label = directory / (path.stem.removesuffix('_input') + '_label')
                    candidates.append((directory.name + '.' + path.stem, path,
                                       label if label.exists() else directory / 'name'))
            for identifier, path, label_path in candidates[:64]:
                try:
                    degrees = int(path.read_text().strip()) / 1000.
                    if not -100. <= degrees <= 250.:
                        continue
                    try:
                        label = label_path.read_text().strip()
                    except OSError:
                        label = identifier
                    values['sensor.' + identifier + '.celsius'] = degrees
                    values['sensor.' + identifier + '.label'] = label
                except (OSError, ValueError):
                    continue
        except OSError as exc:
            return self.unavailable('temperature', source, exc)
        if not values:
            return self.unavailable('temperature', source, 'host temperature sensors unavailable')
        return ResourceSample('temperature', True, source,
                              'Host sensors only; these are not motor temperatures', values)

    def network(self, now):
        source = str(self.proc / 'net' / 'dev')
        try:
            counters = {}
            for line in (self.proc / 'net' / 'dev').read_text().splitlines():
                if ':' not in line:
                    continue
                name, text = line.split(':', 1)
                numbers = [int(part) for part in text.split()]
                if len(numbers) != 16 or any(n < 0 for n in numbers):
                    raise ValueError('invalid network interface counters')
                counters[name.strip()] = {
                    'rx_bytes': numbers[0], 'rx_packets': numbers[1], 'rx_errors': numbers[2], 'rx_dropped': numbers[3],
                    'tx_bytes': numbers[8], 'tx_packets': numbers[9], 'tx_errors': numbers[10], 'tx_dropped': numbers[11]}
            if not counters:
                raise ValueError('network interface counters unavailable')
            previous, self.previous_network = self.previous_network, (now, counters)
            values = {}
            for name, readings in sorted(counters.items()):
                prefix = 'interface.' + name + '.'
                values.update({prefix + key: value for key, value in readings.items()})
                try:
                    values[prefix + 'operstate'] = (self.sys / 'class' / 'net' / name / 'operstate').read_text().strip()
                except OSError:
                    pass
                if previous is not None and now > previous[0] and name in previous[1]:
                    for direction in ('rx', 'tx'):
                        key = direction + '_bytes'
                        delta = readings[key] - previous[1][name][key]
                        if delta >= 0:
                            values[prefix + direction + '_bytes_per_s'] = delta / (now - previous[0])
            return ResourceSample('network', True, source,
                                  'Interface counters; remote reachability is not measured', values)
        except (OSError, ValueError) as exc:
            self.previous_network = None
            return self.unavailable('network', source, exc)

    def sample(self, now=None):
        now = time.monotonic() if now is None else now
        return [self.cpu(), self.memory(), self.disk(), self.temperature(), self.network(now)]
