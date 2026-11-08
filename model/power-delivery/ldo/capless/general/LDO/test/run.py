"""Reproduce the seven LDO SystemC acceptance measurements."""
from pathlib import Path
import csv
import hashlib
import json
import math
import re
import subprocess

ROOT = Path(__file__).resolve().parent.parent
OUTPUT = ROOT / 'test/build/results'


def run(name, mode, index):
    path = OUTPUT / (name + '.csv')
    command = [str(ROOT / 'test/build/model_tb'), str(ROOT / 'json/LDO.json'), mode,
               str(index), '5e-10', str(path)]
    process = subprocess.run(command, capture_output=True, text=True, timeout=600)
    text = process.stdout + process.stderr
    (OUTPUT / (name + '.log')).write_text(text, encoding='utf-8')
    process.check_returncode()
    return text, path


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    dc = []
    for index, load in [(1, .001), (2, .1), (8, 0), (9, .1)]:
        text, _ = run('dc_' + str(index), 'dc', index)
        config = list(map(float, re.search(r'^CONFIG (.+)$', text, re.M)[1].split()))
        assert len(config) == 6
        assert all(math.isclose(a, b, abs_tol=1e-10) for a, b in zip(config, [1.2, 1, 2.5, 1e-5, load, 27]))
        match = re.search(r'^DC (\S+) (\S+) (\S+) (\S+)$', text, re.M)
        dc.append(dict(vout=float(match[2]), current=float(match[3]), iq=float(match[4])))
        assert abs(dc[-1]['current']-load) < 1e-8
    text, path = run('ac_sweep', 'ac_sweep', 10)
    assert 'CONFIG 1.2 1 2.5 1e-05 0.1 27' in text
    with path.open() as source:
        ac = list(csv.DictReader(source))
    assert len(ac) == 31
    for index, row in enumerate(ac):
        assert math.isclose(float(row['frequency_hz']), 1000*10**(index/10), rel_tol=1e-9)
    values = [dc[0]['vout'], dc[1]['vout'], abs(dc[2]['vout']-dc[3]['vout'])*1e4,
              dc[2]['iq']*1e6, *[float(ac[index]['psrr_db']) for index in [0, 20, 30]]]
    rules = json.loads((ROOT / 'test/acceptance.json').read_text())
    results = []
    for rule, value in zip(rules, values):
        low, high = rule['typical']*.95, rule['typical']*1.05
        status = 'PASS' if math.isfinite(value) and low <= value <= high else 'FAIL'
        results.append(dict(rule, measured=value, low=low, high=high, status=status))
        print(rule['name'], format(value, '.10g'), rule['unit'], status, flush=True)
    files = [path for folder in ['include', 'source', 'json'] for path in (ROOT / folder).rglob('*') if path.is_file()]
    record = dict(model='LDO', version='1.1', measurements=results,
                  source_sha256={path.relative_to(ROOT).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in files})
    (OUTPUT / 'summary.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    return int(any(row['status'] == 'FAIL' for row in results))


if __name__ == '__main__':
    raise SystemExit(main())
