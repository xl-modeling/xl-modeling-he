#!/usr/bin/env python3
"""Build S28 in Ubuntu/WSL and retain each run in a separate classified folder."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import traceback

PROJECT = Path(__file__).resolve().parent.parent
CASE_NAMES = {1: '01_resolution', 2: '02_range', 3: '03_static', 4: '04_dynamic'}
CASE_IDS = {1: 'ADC-001-10-1-1', 2: 'ADC-001-10-1-2', 3: 'ADC-001-10-1-3', 4: 'ADC-001-10-1-4'}
CASE_EXPECTED = {
    1: '平均量化步长为 28.125～34.375 mV/LSB。',
    2: 'V1 和 V63 的绝对值均大于 0.9*VRP，范围外输出饱和。',
    3: '各有效码 DNL、INL 均处于 -0.5～+0.5 LSB。',
    4: 'SNR 为 36.5～37.6 dB，THD 为 -60～-50 dB，ENOB 为 5.8～6.0 bit。'
}
SCENARIO_ID = '02-10-1'
METRIC_UNITS = {
    'resolution_bits':'bit','nominal_lsb_v':'V','average_lsb_v':'V','differential_lsb_v':'V','input_step_v':'V','pin_input_step_v':'V','vref_v':'V','lsb_v':'V','ramp_start_v':'V','ramp_stop_v':'V','ramp_step_v':'V','transition_code_1_v':'V','transition_code_63_v':'V',
    'code_count':'code','codes_seen':'code','code_range_min':'code','code_range_max':'code',
    'gain_error_percent':'%','offset_first_transition_lsb':'LSB','offset_midscale_lsb':'LSB',
    'dnl_min_lsb':'LSB','dnl_max_lsb':'LSB','inl_min_lsb':'LSB','inl_max_lsb':'LSB',
    'configured_sample_rate_sps':'S/s','configured_period_s':'s','configured_decision_interval_s':'s',
    'configured_latency_s':'s','configured_clockout_width_s':'s','snr_db':'dB','thd_db':'dB',
    'enob_bit':'bit'
}
METRIC_REFERENCES = {
    'resolution_bits':'6','average_lsb_v':'0.028125..0.034375','codes_seen':'64','code_range_min':'0','code_range_max':'63','saturation_verified':'true',
    'configured_sample_rate_sps':'100000000','dnl_min_lsb':'-0.5..0.5','dnl_max_lsb':'-0.5..0.5',
    'inl_min_lsb':'-0.5..0.5','inl_max_lsb':'-0.5..0.5',
    'snr_db':'37.63','thd_db':'-53.09','enob_bit':'5.93'
}
FOLDERS = ['01_测试报告', '02_性能指标', '03_曲线图', '04_原始数据',
           '05_模型与配置', '06_测试与复现脚本', '07_日志与校验']


def json_write(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def project_build_dir(project=PROJECT):
    # 现场测试目录：构建目录缺省放在模型目录内的 build/（原为 /mnt/d/systemc/build/s28_<hash>_systemc_ams）
    return project/'build'


def compatible_build_dir(requested, project=PROJECT):
    """Reuse this project's cache, leaving any other project's cache intact."""
    requested = Path(requested)
    project = project.resolve()
    key = hashlib.sha256(str(project).encode('utf-8')).hexdigest()[:12]
    candidate = requested
    counter = 0
    while True:
        cache = candidate/'CMakeCache.txt'
        if not cache.is_file():
            return candidate
        source = None
        for line in cache.read_text(encoding='utf-8', errors='replace').splitlines():
            if line.startswith('CMAKE_HOME_DIRECTORY:INTERNAL='):
                source = line.split('=', 1)[1]
                break
        if source and Path(source).resolve() == project:
            return candidate
        counter += 1
        suffix = '_'+key+('' if counter == 1 else '_'+str(counter))
        candidate = requested.with_name(requested.name+suffix)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--case', type=int, choices=range(1, 5))
    selection.add_argument('--cases', type=str, metavar='LIST', help='Run a comma-separated case list, for example 3,4')
    selection.add_argument('--all', action='store_true', help='Run all four scenario-10 test instances; also the default')
    parser.add_argument('--check-only', action='store_true', help='Check files/configuration without simulation')
    parser.add_argument('--no-vcd', action='store_true', help='Disable VCD waveform generation for this run')
    parser.add_argument('--systemc-root', type=Path, default=Path(os.environ.get('S28_SYSTEMC_ROOT', '/mnt/d/systemc')))
    parser.add_argument('--build-dir', type=Path, default=Path(os.environ.get('S28_BUILD_DIR', str(project_build_dir()))))
    # Keep simulation outputs outside the delivery tree by default.  The shell
    # wrappers set the same location explicitly; this fallback also protects
    # direct invocations of run_s28.py from writing into the package.
    parser.add_argument('--result-root', type=Path, default=Path(os.environ.get('S28_RESULT_ROOT', str(PROJECT/'out'))))
    parser.add_argument('--model-config', type=Path, default=Path(os.environ.get('S28_MODEL_CONFIG', str(PROJECT/'json/s28.json'))))
    parser.add_argument('--test-config', type=Path, default=Path(os.environ.get('S28_TEST_CONFIG', str(PROJECT/'json/test_cases.json'))))
    return parser.parse_args(argv)


def preflight(args, check_runtime=True):
    needed = ['CMakeLists.txt', 'source/s28.cpp', 'include/s28.h', 'include/detail/json_value.h',
              'test/CMakeLists.txt', 'test/s28_tb.cpp', 'test/metrics.h', 'test/run_s28.py']
    missing = [str(PROJECT/name) for name in needed if not (PROJECT/name).is_file()]
    if missing:
        raise RuntimeError('Missing files: '+', '.join(missing))
    model = json.loads(args.model_config.read_text(encoding='utf-8-sig'))
    t = json.loads(args.test_config.read_text(encoding='utf-8-sig'))
    if model['adc_bits']['value'] != 6:
        raise ValueError('S28 requires six bits')
    m, k = t['M'], t['k']
    expected_conditions = {'scenario_id':'02-10-1', 'supply_v':1.0, 'common_mode_v':0.5,
                           'cdac_reference_v':1.0, 'pad_ring_v':2.5, 'ambient_temperature_c':2.0,
                           'cdac_sampling_cap_pf':4.096}
    for key, expected in expected_conditions.items():
        valid = t.get(key) == expected if key == 'scenario_id' else (
            key in t and math.isclose(float(t[key]), float(expected), rel_tol=0, abs_tol=1e-12))
        if not valid:
            raise ValueError(f'Invalid scenario-10 condition: {key}')
    if not isinstance(m, int) or not isinstance(k, int) or m < 256 or m & (m-1) or not 0 < k < m//2 or math.gcd(m, k) != 1:
        raise ValueError('Invalid coherent M/k configuration')
    if not math.isclose(t['dynamic_frequency_hz'], k*t['sample_rate_hz']/m, rel_tol=0, abs_tol=1e-6):
        raise ValueError('Frequency must equal k*fs/M')
    if (t.get('static_samples') != m or t.get('static_k') != k or
            not math.isclose(float(t.get('static_differential_peak_v', 0)), float(t['differential_peak_v']), rel_tol=0, abs_tol=1e-12)):
        raise ValueError('Cases 3 and 4 must use the same coherent input record')
    if not 0 < t['dc_resolution_lsb'] <= .01 or not 0 < t['differential_peak_v'] < 1:
        raise ValueError('Invalid DC refinement or dynamic amplitude')
    if t['window'] != 'rectangular':
        raise ValueError('Only coherent rectangular-window analysis is supported')
    # Analytical self-checks are separate from model simulation.
    if min(3002, 4096-3002) != 1094 or not math.isclose((37.51-1.76)/6.02, 5.938538205980067, abs_tol=1e-12):
        raise RuntimeError('Formula self-check failed')
    for j in range(1, 64):
        static_peak = float(t['static_differential_peak_v'])
        v = static_peak*(-1+j/32)
        probability = .5+math.asin(v/static_peak)/math.pi
        if abs(-static_peak*math.cos(math.pi*probability)-v) > 1e-12:
            raise RuntimeError('Sine CDF inversion self-check failed')
    if importlib.util.find_spec('docx') is None:
        raise RuntimeError('Install python3-docx in the Ubuntu/WSL environment')
    if check_runtime:
        if platform.system() != 'Linux':
            raise RuntimeError('Use the Ubuntu/WSL terminal to compile and run SystemC')
        for command in ['cmake', os.environ.get('CXX', 'g++')]:
            if not shutil.which(command):
                raise RuntimeError('Missing command: '+command)
        for base, header, lib in [
            (Path(os.environ.get('SYSTEMC_HOME', str(args.systemc_root/'install/systemc'))), 'systemc', 'systemc'),
            (Path(os.environ.get('SYSTEMC_AMS_HOME', str(args.systemc_root/'install/systemc-ams'))), 'systemc-ams', 'systemc-ams')]:
            if not (base/'include'/header).exists():
                raise RuntimeError('Missing header: '+str(base/'include'/header))
            if not any(list((base/folder).glob('lib'+lib+extension)) for folder in ['lib', 'lib64', 'lib-linux64'] for extension in ['.so', '.a']):
                raise RuntimeError('Missing library under '+str(base))
    return model, t


def run_command(command, log, cwd=PROJECT, show_output=True):
    with Path(log).open('w', encoding='utf-8') as stream:
        stream.write('$ '+shlex.join([str(x) for x in command])+'\n')
        stream.flush()
        process = subprocess.Popen([str(x) for x in command], cwd=cwd, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
        for line in process.stdout:
            if show_output:
                print(line, end='', flush=True)
            stream.write(line)
        code = process.wait()
        stream.write('\nEXIT_CODE='+str(code)+'\n')
    return code


def snapshot(run, args, cases):
    saved = run/FOLDERS[4]/'S28'
    saved.mkdir()
    for name in ['include', 'source', 'json', 'test']:
        shutil.copytree(PROJECT/name, saved/name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.legacy', '_permission_probe*'))
    for name in ['CMakeLists.txt', 'README.md']:
        shutil.copyfile(PROJECT/name, saved/name)
    shutil.copyfile(args.model_config, saved/'json/active_model.json')
    shutil.copyfile(args.test_config, saved/'json/active_test.json')
    scripts = run/FOLDERS[5]
    shutil.copytree(saved/'test', scripts/'test')
    json_write(scripts/'invocation.json', dict(cases=cases, project=str(PROJECT), build_dir=str(args.build_dir),
        systemc_root=str(args.systemc_root), model_config=str(args.model_config), test_config=str(args.test_config),
        no_vcd=bool(args.no_vcd)))
    replay = '''#!/usr/bin/env bash
set -euo pipefail
S28_RUN_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
S28_SNAPSHOT="${S28_RUN_DIR}/05_模型与配置/S28"
S28_REPLAY_BUILD=${S28_BUILD_DIR:-"${TMPDIR:-/tmp}/s28_replay_$(date +%Y%m%d_%H%M%S)_$$"}
exec "${S28_PYTHON:-python3}" "${S28_SNAPSHOT}/test/run_s28.py" CASE_SELECTION \\
  --build-dir "${S28_REPLAY_BUILD}" --result-root "${S28_RUN_DIR}/.." \\
  --model-config "${S28_SNAPSHOT}/json/active_model.json" \\
  --test-config "${S28_SNAPSHOT}/json/active_test.json" --systemc-root INSTALL_ROOT NO_VCD
'''
    if len(cases)==4:
        case_selection='--all'
    elif len(cases)==1:
        case_selection='--case '+str(cases[0])
    else:
        case_selection='--cases '+','.join(str(case) for case in cases)
    replay = replay.replace('CASE_SELECTION', case_selection)
    replay = replay.replace('INSTALL_ROOT', shlex.quote(str(args.systemc_root)))
    replay = replay.replace('NO_VCD', '--no-vcd' if args.no_vcd else '')
    # Path.write_text(newline=) 需要 Python 3.10，改用 open() 以兼容 Python 3.8
    with (scripts/'reproduce.sh').open('w', encoding='utf-8', newline='\n') as stream:
        stream.write(replay)
    json_write(run/FOLDERS[6]/'source_snapshot_sha256.json', {
        p.relative_to(saved).as_posix(): sha256(p) for p in sorted(saved.rglob('*')) if p.is_file()})
    return saved


def classify(run, raw, case):
    if not raw.exists():
        return
    for path in list(raw.iterdir()):
        if not path.is_file():
            continue
        if path.name in ['model_input.json', 'test_input.json', 'effective_profile.json']:
            target = run/FOLDERS[4]/CASE_NAMES[case]
            target.mkdir(exist_ok=True)
            shutil.move(str(path), target/path.name)
        elif path.suffix.lower() in ['.svg', '.png', '.jpg', '.pdf']:
            shutil.move(str(path), run/FOLDERS[2]/(CASE_NAMES[case]+'_'+path.name))
        elif path.name in ['result.json', 'transitions.csv', 'linearity.csv', 'spectrum.csv', 'harmonics.csv']:
            shutil.move(str(path), run/FOLDERS[1]/(CASE_NAMES[case]+'_'+path.name))


def terminal_case_summary(case, result, raw):
    """Print a concise per-case result for terminal capture; full details remain in the run folder."""
    print('\n测试用例：'+CASE_IDS[case])
    labels = {
        1: [('resolution_bits', '分辨率', ' bit'), ('average_lsb_v', '平均量化步长', ' V'), ('codes_seen', '出现码数', '')],
        2: [('actual_range_v', '实际量程', ' V'), ('saturation_verified', '饱和检查', '')],
        3: [('dnl_min_lsb', 'DNL最小值', ' LSB'), ('dnl_max_lsb', 'DNL最大值', ' LSB'), ('inl_min_lsb', 'INL最小值', ' LSB'), ('inl_max_lsb', 'INL最大值', ' LSB')],
        4: [('snr_db', 'SNR', ' dB'), ('thd_db', 'THD', ' dB'), ('enob_bit', 'ENOB', ' bit')]
    }
    measured = []
    for key, label, unit in labels[case]:
        if key in result:
            measured.append(f'{label}={result[key]}{unit}')
    print('实测结果：'+('；'.join(measured) if measured else '无'))


def report(run, summary):
    lines = ['# S28 SystemC 自测试报告', '', '场景编号：'+SCENARIO_ID, '运行目录：'+run.name, '状态：'+summary['status'], '',
             '本报告只读取本次平台产生的结果；参考值不作为实测值。', '']
    if summary.get('error'):
        lines += ['运行异常：'+summary['error'], '']
    rows = []
    for item in summary['cases']:
        lines += ['## '+CASE_IDS[item['case']]+'（'+CASE_NAMES[item['case']]+'）', '', '进程退出码：'+str(item['returncode']), '']
        result = item.get('result') or {}
        if not result:
            lines += ['本用例未产生结果文件，原因见运行日志。', '']
        for key, value in result.items():
            if isinstance(value, (dict, list)):
                value = json.dumps(value, ensure_ascii=False)
            rows.append([SCENARIO_ID, item['case'], key, METRIC_UNITS.get(key,''), METRIC_REFERENCES.get(key,''), value])
            lines.append(f'- {key}: {value}')
        lines.append('')
    lines += ['## 数据位置', '']+[f'- {name}' for name in FOLDERS[1:]]
    report_dir = run/FOLDERS[0]
    (report_dir/'S28_自测试报告.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    with (run/FOLDERS[1]/'metrics.csv').open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(['scenario_id', 'case', 'metric', 'unit', 'reference', 'measured'])
        writer.writerows(rows)
    from docx import Document
    from docx.shared import Cm, Pt, RGBColor
    from docx.oxml.ns import qn
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    doc = Document()
    for style_name in ['Normal', 'Title', 'Heading 1']:
        style = doc.styles[style_name]
        style.font.color.rgb = RGBColor(0,0,0)
        style.paragraph_format.space_before = Pt(6 if style_name=='Heading 1' else 0)
        style.paragraph_format.space_after = Pt(3)
    for border in list(doc.styles.element.iter(qn('w:pBdr'))):
        border.getparent().remove(border)
    for section in doc.sections:
        section.page_width = Cm(21); section.page_height = Cm(29.7)
        section.top_margin = section.bottom_margin = Cm(2.5)
        section.left_margin = Cm(2.8); section.right_margin = Cm(2.2)
    for line in lines:
        if not line:
            continue
        if line.startswith('# '):
            p = doc.add_paragraph(line[2:], 'Title')
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            size = 16
        elif line.startswith('## '):
            p = doc.add_paragraph(line[3:], 'Heading 1')
            p.paragraph_format.keep_with_next = True
            size = 14
        else:
            p = doc.add_paragraph(line[2:] if line.startswith('- ') else line)
            size = 10.5
        p.paragraph_format.line_spacing = 1.15
        p.paragraph_format.space_after = Pt(3)
        for r in p.runs:
            r.font.name = 'Times New Roman'; r.font.size = Pt(size); r.font.color.rgb = RGBColor(0,0,0)
            r._element.get_or_add_rPr().get_or_add_rFonts().set(qn('w:eastAsia'), '黑体' if size>12 else '宋体')
    table = doc.add_table(rows=1, cols=3)
    table.style = 'Table Grid'
    hdr = table.rows[0].cells
    hdr[0].text = '测试项目'; hdr[1].text = '预期结果'; hdr[2].text = '实际结果'
    expected = {
        1: '−1.10 V～+1.10 V 差分斜坡提取码0→1和码62→63跃迁，电压差除以62，范围为 28.125～34.375 mV/LSB。',
        2: '−1.10 V～+1.10 V 差分斜坡提取 V1、V63，实际量程覆盖 −0.9VRP～+0.9VRP。',
        3: '低频正弦码密度统计得到 DNL、INL，曲线及极值可追溯。',
        4: '相干正弦 FFT 得到频谱、SNR、THD 和 ENOB，功率关系一致。',
    }
    for item in summary['cases']:
        row = table.add_row().cells
        case_no = item['case']; result = item.get('result') or {}
        actual = '通过' if item.get('returncode') == 0 and result.get('passed') is True else '未通过'
        details = []
        for key in ('nominal_lsb_v','transition_code_1_v','transition_code_63_v','average_lsb_v','v1_code1_v','v63_code63_v','actual_range_v','codes_seen','code_range_min','code_range_max','saturation_verified','dnl_min_lsb','dnl_max_lsb','inl_min_lsb','inl_max_lsb','snr_db','thd_db','enob_bit'):
            if key in result: details.append(f'{key}={result[key]}')
        row[0].text = CASE_IDS[case_no] + '（' + CASE_NAMES[case_no] + '）'
        row[1].text = expected.get(case_no, '')
        row[2].text = actual + ('；' + '，'.join(details) if details else '')
    for row in table.rows:
        for cell in row.cells:
            for para in cell.paragraphs:
                para.paragraph_format.line_spacing = 1.0
                for run in para.runs:
                    run.font.name = '宋体'; run.font.size = Pt(9); run.font.color.rgb = RGBColor(0,0,0)
    doc.save(report_dir/'S28_自测试报告.docx')


def execute(args):
    if args.cases:
        try:
            cases = [int(item.strip()) for item in args.cases.split(',') if item.strip()]
        except ValueError as exc:
            raise ValueError('--cases must be a comma-separated list of integers') from exc
        if not cases or len(set(cases)) != len(cases) or any(case not in CASE_NAMES for case in cases):
            raise ValueError('--cases must contain unique values from 1 to 4')
    else:
        cases = [args.case] if args.case else list(CASE_NAMES)
    args.result_root.mkdir(parents=True, exist_ok=True)
    run = args.result_root/('run_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(exist_ok=False)
    for name in FOLDERS:
        (run/name).mkdir()
    summary = dict(model='S28', status='running', cases=[], error=None)
    logs = run/FOLDERS[6]
    try:
        preflight(args)
        args.build_dir = compatible_build_dir(args.build_dir)
        saved = snapshot(run, args, cases)
        if args.no_vcd:
            os.environ['S28_DISABLE_VCD'] = '1'
        else:
            os.environ.pop('S28_DISABLE_VCD', None)
        json_write(logs/'environment.json', dict(system=platform.platform(), python=sys.version, project=str(PROJECT),
            systemc_root=str(args.systemc_root), SYSTEMC_HOME=os.environ.get('SYSTEMC_HOME'),
            SYSTEMC_AMS_HOME=os.environ.get('SYSTEMC_AMS_HOME')))
        configure = ['cmake', '-S', PROJECT, '-B', args.build_dir, '-DCMAKE_BUILD_TYPE=Release', '-DROOT='+str(args.systemc_root)]
        if not (args.build_dir/'CMakeCache.txt').exists() and shutil.which('ninja'):
            configure += ['-G', 'Ninja']
        commands = [('configure.log', configure), ('build.log', ['cmake', '--build', args.build_dir, '--parallel', '4'])]
        for name, command in commands:
            code = run_command(command, logs/name, show_output=False)
            if code:
                raise RuntimeError(f'{name}: process exited with {code}')
        print('模型编译：通过')
        binary = args.build_dir/'s28_tb'
        linkage_path = args.build_dir/'s28_linkage.json'
        linkage = json.loads(linkage_path.read_text(encoding='utf-8')) if linkage_path.is_file() else {}
        for name in ['s28_tb', linkage.get('model_library', 'libs28.a'), 's28_linkage.json', 'CMakeCache.txt', 'compile_commands.json']:
            path = args.build_dir/name
            if path.is_file():
                shutil.copyfile(path, logs/name)
        for case in cases:
            raw = run/FOLDERS[3]/CASE_NAMES[case]
            command = [binary, str(case), saved/'json/active_model.json', saved/'json/active_test.json', raw, saved]
            case3_samples = run/FOLDERS[3]/CASE_NAMES[3]/'samples.csv'
            use_case3_records = case == 4 and 3 in cases and case3_samples.is_file()
            if use_case3_records:
                command.append(case3_samples)
            code = run_command(command, logs/(CASE_NAMES[case]+'.log'), show_output=False)
            if use_case3_records:
                case3_waveform = run/FOLDERS[3]/CASE_NAMES[3]/'waveforms.vcd'
                if case3_waveform.is_file():
                    shutil.copy2(case3_waveform, raw/'waveforms_shared_case3.vcd')
            if not args.no_vcd and not any(raw.glob('*.vcd')):
                raise RuntimeError(f'Scenario-10 case {case} did not produce a VCD waveform file')
            path = raw/'result.json'
            result = json.loads(path.read_text(encoding='utf-8')) if path.is_file() else None
            terminal_case_summary(case, result or {}, raw)
            print('状态：'+('通过' if code == 0 and (result or {}).get('passed') is True else '失败'))
            summary['cases'].append(dict(case=case, returncode=code, result=result))
            classify(run, raw, case)
        summary['status'] = 'passed' if all(x['returncode']==0 and (x['result'] or {}).get('passed') is True for x in summary['cases']) else 'failed'
    except Exception as exc:
        summary['status'] = 'failed'
        summary['error'] = str(exc)
        (logs/'error.log').write_text(traceback.format_exc(), encoding='utf-8')
    finally:
        json_write(run/FOLDERS[1]/'metrics.json', summary)
        try:
            report(run, summary)
        except Exception as exc:
            summary['status'] = 'failed'
            summary['report_error'] = str(exc)
            (logs/'report_error.log').write_text(traceback.format_exc(), encoding='utf-8')
            (run/FOLDERS[0]/'S28_自测试报告.md').write_text(
                '# S28 SystemC 自测试报告\n\n状态：failed\n\n报告生成异常：'+str(exc)+
                '\n\n本次已保存的指标及进程退出码见 02_性能指标/metrics.json；日志见 07_日志与校验。\n',
                encoding='utf-8')
            json_write(run/FOLDERS[1]/'metrics.json', summary)
        json_write(logs/'status.json', summary)
        problem = summary.get('error') or summary.get('report_error')
        if not problem and summary['status']=='failed':
            problem = 'One or more processes or result checks failed; see per-case logs.'
        (logs/'问题记录.md').write_text('# S28 运行记录\n\n'+(problem or '本次未记录进程或结果检查失败。')+'\n', encoding='utf-8')
        if problem:
            print('运行异常：'+problem+'（详见 07_日志与校验）', flush=True)
        entries = [p for p in run.rglob('*') if p.is_file() and p.name!='SHA256SUMS.txt']
        (logs/'SHA256SUMS.txt').write_text(''.join(sha256(p)+'  '+p.relative_to(run).as_posix()+'\n' for p in sorted(entries)), encoding='utf-8')
        print('RESULT_FOLDER='+str(run.resolve()), flush=True)
    return 0 if summary['status']=='passed' else 1


def main(argv=None):
    args = parse_args(argv)
    if args.check_only:
        preflight(args, check_runtime=platform.system()=='Linux')
        print('PASS project files, configuration, formula checks and report dependency.')
        print('No SystemC build or simulation executed.' if platform.system()=='Linux' else
              'Windows source checks only. Build and simulation must run in Ubuntu/WSL.')
        return 0
    return execute(args)


if __name__=='__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print('FAIL: '+str(exc), file=sys.stderr)
        sys.exit(1)
