"""Run the supplied baseline scenarios against the compiled SystemC model."""
from pathlib import Path
import json
import math
import subprocess

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / 'test/build'


def main():
    binary = BUILD / 'model_tb'
    results = []
    for case_path in sorted((ROOT / 'test/cases').glob('*.json')):
        case = json.loads(case_path.read_text(encoding='utf-8'))
        output = BUILD / 'results' / case['id']
        output.mkdir(parents=True, exist_ok=True)
        config = json.loads((ROOT / case['config']).read_text(encoding='utf-8-sig'))
        for key, value in case.get('config_patch', {}).items():
            if isinstance(config[key], dict) and 'value' in config[key]:
                config[key]['value'] = value
            else:
                config[key] = value
        config_path = output / 'config.json'
        config_path.write_text(json.dumps(config, indent=2), encoding='utf-8')
        process = subprocess.run([str(binary), str(config_path), str(case_path), str(output / 'wave.csv')],
                                 capture_output=True, text=True, timeout=600)
        log = process.stdout + process.stderr
        (output / 'run.log').write_text(log, encoding='utf-8')
        process.check_returncode()
        line = next(line for line in log.splitlines() if line.startswith('EXPERT_RESULT '))
        measurements = json.loads(line[len('EXPERT_RESULT '):])
        assert measurements['nonfinite_samples'] == 0
        checks = []
        for metric in case['metrics']:
            actual = measurements['statistics'][metric['output']][metric.get('stat', 'mean')]
            passed = abs(actual-metric['expected']) <= metric['absolute_tolerance'] if 'expected' in metric else metric['min'] <= actual <= metric['max']
            checks.append(dict(metric, actual=actual, passed=math.isfinite(actual) and passed))
        status = 'PASS' if checks else 'OBSERVED'
        if any(not check['passed'] for check in checks):
            status = 'FAIL'
        result = dict(case=case['id'], status=status, checks=checks, measurements=measurements)
        (output / 'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        results.append(result)
        print(case['id'], status, flush=True)
    if not results:
        raise RuntimeError('No baseline scenarios found')
    (BUILD / 'results/summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    return int(any(result['status'] == 'FAIL' for result in results))


if __name__ == '__main__':
    raise SystemExit(main())
