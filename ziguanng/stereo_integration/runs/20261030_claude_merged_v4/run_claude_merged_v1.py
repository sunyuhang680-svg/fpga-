"""claude merged_v1 runner, derived from run_testing60_photo_mono_v7.py (GPT).
Differences: --replace ORIGINAL=CANDIDATE swaps a project source for an owned candidate
(originals stay untouched); RAW8_STEREO_CALIBRATION_VALID=1 is allowed only with a
stereo_binding (file path + sha256 list) in the source selection, all of which must match."""
import argparse,hashlib,json,os,re,shutil,subprocess,time
from pathlib import Path
import sys
sys.path.insert(0,'D:\\虚拟C盘\\ziguanng\\stereo_integration\\scripts')
from project_common import ROOT,WORKSPACE,configure,native_tool_path
from audit_board_timing import audit

def native_tool_path(path):
    # Same authorized workspace, short DOS alias avoids the native MAX_PATH limit.
    rel=Path(path).resolve().relative_to(WORKSPACE.resolve())
    alias=Path('S:/')/rel
    if not Path('S:/stereo_integration/AGENTS.md').is_file():
        raise RuntimeError('Required S: workspace alias is absent; do not change build identity')
    return alias



def verdict(commands_pass, timing_pass, pnr_requested):
    if not commands_pass:return 'failed'
    if pnr_requested:return 'passed' if timing_pass is True else 'timing_failed'
    return 'passed'


def actual_parameters(log,names=('ENABLE_ALGORITHM','ENABLE_AUX_READ','IMU_DEBUG_DISPLAY','ENABLE_CMP_CONTROLS','ENABLE_LIVE_STREAM')):
    pattern='('+'|'.join(re.escape(name) for name in names)+r')\s*=\s*\d+\x27b([01]+)'
    return {name:int(bits,2) for name,bits in re.findall(pattern,log)}

def main():
    configure()
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--aux',action='store_true')
    p.add_argument('--algorithm',action='store_true')
    p.add_argument('--cmp-controls',action='store_true',help='enable CMP UI controls in the actual Top compile')
    p.add_argument('--live-stream',type=int,choices=[0,1,2],help='ENABLE_LIVE_STREAM: 1 live video path, 2 plus camera HTS/VTS retime; omit to retain original value')
    p.add_argument('--controller',type=Path,help='temporary own-controller candidate, same actual Top')
    p.add_argument('--renderer',type=Path,help='temporary owned retimed renderer, same original Top')
    p.add_argument('--engine',type=Path,help='temporary owned core engine, same original Top')
    p.add_argument('--extra-source',action='append',type=Path,default=[],help='candidate dependency under integration only, without editing project list')
    p.add_argument('--replace',action='append',default=[],metavar='ORIGINAL=CANDIDATE',help='swap a project source for an owned candidate (both under the integration root)')
    p.add_argument('--extra-constraint',action='append',type=Path,default=[],help='additional FDC/SDC/RCF; original board FDC always retained')
    p.add_argument('--source-selection',type=Path,default=Path(__file__).resolve().parent/'source_selection_v7.json',help='source identity manifest; defaults to current run source_selection_v7.json')
    p.add_argument('--imu-debug-display',type=int,choices=[0,1],help='explicit existing Top parameter; omit to retain original value')
    p.add_argument('--top-param',action='append',default=[],metavar='NAME=VALUE',help='another existing Top parameter, non-negative integer; checked in the compile log like the others')
    p.add_argument('--map',action='store_true');p.add_argument('--pnr',action='store_true')
    p.add_argument('--bitgen',action='store_true',help='gen_bit_stream after PNR/report_timing, regardless of timing')
    p.add_argument('--scbv',choices=['High Voltage','Low Voltage'],help='explicit configuration strap selection; use documented board evidence');a=p.parse_args()
    if a.bitgen and not a.pnr:raise ValueError('--bitgen requires --pnr')
    out=a.output.resolve();out.relative_to(ROOT.resolve());out.mkdir(parents=True,exist_ok=False)
    project=Path('D:\\虚拟C盘\\ziguanng\\stereo_integration\\runs\\20261028_225421_164a1a65\\testing_import_v1\\testing\\200k\\prj\\ov5640.pds')
    constraint=Path('D:\\虚拟C盘\\ziguanng\\stereo_integration\\runs\\20261028_225421_164a1a65\\testing_import_v1\\testing\\200k\\ov5640.fdc')
    constraints=[constraint]
    for item in a.extra_constraint:
        item=item.resolve();item.relative_to(ROOT.resolve())
        if not item.is_file() or item.suffix.lower() not in ('.fdc','.sdc','.rcf'):
            raise ValueError('supplemental constraint must be an integration FDC/SDC/RCF')
        if item not in constraints:constraints.append(item)
    shutil.copyfile(Path('D:\\虚拟C盘\\ziguanng\\stereo_integration\\runs\\20261028_225421_164a1a65\\testing_import_v1\\testing\\200k\\prj\\build_cfg.vh'),out/'build_cfg.vh')
    paths=[]
    for name in re.findall(r'\(_(?:file|ip_source_item)\s+"([^"]+)"',project.read_text(encoding='utf-8-sig').split('(_task tsk_compile')[0]):
        if Path(name).suffix.lower() in ('.v','.sv','.vp'):
            f=(project.parent/name).resolve();f.relative_to(WORKSPACE.resolve())
            if not f.is_file():raise FileNotFoundError(f)
            if f not in paths:paths.append(f)
    if a.controller:
        candidate=a.controller.resolve();candidate.relative_to(ROOT.resolve())
        if not candidate.is_file():raise FileNotFoundError(candidate)
        original=ROOT/'rtl/stereo_algorithm_view.v'
        if original not in paths:raise RuntimeError('original controller absent from project list')
        paths[paths.index(original)]=candidate
        guard=ROOT/'rtl/pose_ray_domain_guard.v'
        if guard not in paths:paths.append(guard)
    for candidate,original in ((a.renderer,ROOT/'rtl/frame_view_render_retimed.v'),
                               (a.engine,ROOT/'rtl/core_v2/undistort_closure_engine_v2.sv')):
        if candidate:
            candidate=candidate.resolve();candidate.relative_to(ROOT.resolve())
            if not candidate.is_file() or candidate.suffix.lower() not in ('.v','.sv'):raise ValueError('owned candidate HDL required')
            if original not in paths:raise RuntimeError('original candidate source absent from project')
            paths[paths.index(original)]=candidate
    for item in a.replace:
        original,_,candidate=item.partition('=')
        original=Path(original).resolve();candidate=Path(candidate).resolve()
        original.relative_to(ROOT.resolve());candidate.relative_to(ROOT.resolve())
        if original not in paths:raise RuntimeError('replace: original not in project list: '+str(original))
        if not candidate.is_file() or candidate.suffix.lower() not in ('.v','.sv'):raise ValueError('replace: candidate HDL required: '+str(candidate))
        paths[paths.index(original)]=candidate
    for f in a.extra_source:
        f=f.resolve();f.relative_to(ROOT.resolve())
        if f.suffix.lower() not in ('.v','.sv') or not f.is_file():raise ValueError('candidate dependency must be a local HDL source')
        if f not in paths:paths.append(f)
    manifest={str(f.relative_to(WORKSPACE)).replace('\\','/'):hashlib.sha256(f.read_bytes()).hexdigest() for f in paths}
    selection_path=a.source_selection.resolve();selection_path.relative_to(ROOT.resolve())
    selection=json.loads(selection_path.read_text(encoding='utf-8-sig'))
    selected=selection.get('sources')
    if not isinstance(selected,list) or not selected:raise ValueError('source selection must contain a nonempty sources list')
    selected_hashes={}
    for row in selected:
        key=row['path'].replace('\\','/')
        if key in selected_hashes:raise ValueError('duplicate selected source: '+key)
        selected_hashes[key]=row['sha256']
    if selected_hashes != manifest:raise ValueError('source selection does not match actual project source paths and hashes')
    if selection.get('pds_sha256') and selection['pds_sha256'] != hashlib.sha256(project.read_bytes()).hexdigest():raise ValueError('source selection PDS hash mismatch')
    top=next((f for f in paths if f.name.lower()=='top.v'),None)
    if top is None or (selection.get('top_sha256') and selection['top_sha256'] != hashlib.sha256(top.read_bytes()).hexdigest()):raise ValueError('source selection Top hash mismatch')
    tcl=[]
    for f in paths:
        tcl.append(f'add_design {{{native_tool_path(f).as_posix()}}}')
    if a.map or a.pnr or a.extra_constraint:
        tcl += [f'add_constraint {{{native_tool_path(item).as_posix()}}}' for item in constraints]
    expected={'ENABLE_ALGORITHM':int(a.algorithm),'ENABLE_AUX_READ':int(a.aux)}
    expected['ENABLE_CMP_CONTROLS']=int(a.cmp_controls)
    if a.imu_debug_display is not None:expected['IMU_DEBUG_DISPLAY']=a.imu_debug_display
    if a.live_stream is not None:expected['ENABLE_LIVE_STREAM']=a.live_stream
    for item in a.top_param:
        name,_,value=item.partition('=')
        if not re.fullmatch(r'[A-Z][A-Z0-9_]*',name) or not value.isdigit():raise ValueError('--top-param needs NAME=VALUE: '+item)
        if name in expected:raise ValueError('parameter given twice: '+name)
        expected[name]=int(value)
    # Independent new mono captures authorize lens correction only. No new R/T.
    calibration_inputs=[]
    binding=selection.get('calibration_binding',{})
    stereo_binding=selection.get('stereo_binding',{})
    if expected.get('RAW8_STEREO_CALIBRATION_VALID',0)!=0:
        rows=stereo_binding.get('files',[])
        if len(rows)<3:raise ValueError('RAW8_STEREO_CALIBRATION_VALID=1 needs a stereo_binding with the R/T result and generated geometry')
        for row in rows:
            f=(WORKSPACE/row['path']).resolve();f.relative_to(ROOT.resolve())
            if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=row['sha256']:
                raise ValueError('Stereo calibration binding changed: '+str(f))
            calibration_inputs.append(f)
    if expected.get('RAW8_CALIBRATION_VALID',0)!=0:
        if not (binding.get('mono_valid') is True and binding.get('stereo_valid') is False):
            raise ValueError('New mono lens/crop binding required')
        for row in binding.get('files',[]):
            f=(WORKSPACE/row['path']).resolve();f.relative_to(ROOT.resolve())
            if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=row['sha256']:
                raise ValueError('Mono calibration binding changed: '+str(f))
            calibration_inputs.append(f)
        if len([f for f in calibration_inputs if f not in [(WORKSPACE/r['path']).resolve() for r in stereo_binding.get('files',[])]])<10:raise ValueError('Incomplete mono calibration evidence binding')
        reviewed=json.loads((WORKSPACE/binding['root_acceptance']).read_text(encoding='utf-8'))
        if reviewed.get('mono_integration_authorized') is not True or reviewed.get('stereo_valid') is not False:
            raise ValueError('Root mono review does not authorize this scope')
        top_text=top.read_text(encoding='utf-8-sig')
        if '.entry_allowed(RAW8_CALIBRATION_VALID && RAW8_STEREO_CALIBRATION_VALID &&' not in top_text:
            raise ValueError('Top does not independently gate metric entry')
    if a.pnr:
        required={'ENABLE_ALGORITHM':1,'ENABLE_AUX_READ':1,'ENABLE_CMP_CONTROLS':1,'IMU_DEBUG_DISPLAY':1,'ENABLE_MEASUREMENT':1,'DIRECT_VIDEO':1,'DIRECT_CAMERA':0,'CAM0_SAMPLE_NEGEDGE':0,'RAW8_CALIBRATION_VALID':1,'RAW8_STEREO_CALIBRATION_VALID':1 if stereo_binding.get('files') else 0,'CAM_RATE_OVERLAY':1,'ENABLE_DDR_VIEW_RENDER':1,'ENABLE_DDR_TEST_BARS':0}
        if any(expected.get(k)!=v for k,v in required.items()):raise ValueError('PNR requires the photo/mono candidate actual Top parameters')
        if expected.get('ENABLE_LIVE_STREAM',0)!=0:raise ValueError('PNR requires ENABLE_LIVE_STREAM=0')
        expected['ENABLE_LIVE_STREAM']=0
        if a.bitgen and a.scbv!='High Voltage':raise ValueError('PNR bitgen requires High Voltage')
        key_file=(ROOT/'runs/20261029_010642_fc25afd7/testing60_ui_keys.fdc').resolve()
        if key_file not in constraints:raise ValueError('PNR requires the existing C23/B22 button constraint')
    parameters=' '.join(f'{name} {value}' for name,value in expected.items())
    tcl += ['set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB676',
        'compile -system_verilog -top_module Top -defined_parameters {'+parameters+'}',
        'set stereo_compile_handle [open {compile/run.log} r]',
        'set stereo_compile_text [read $stereo_compile_handle]',
        'close $stereo_compile_handle']
    for name,value in expected.items():
        pattern=name+"[[:space:]]*=[[:space:]]*[0-9]+'b0*"+format(value,'b')+'[[:space:]]'
        tcl += ['if {![regexp {'+pattern+'} $stereo_compile_text]} {error "STEREO_PARAMETER_MISMATCH '+name+'"}',
            f'puts "STEREO_PARAMETER_PASS {name}={value}"']
    tcl += ['if {[regexp {Net key_(mode|menu)_n.*has no driver} $stereo_compile_text]} {error "STEREO_BUTTON_TOP_UNDRIVEN"}', 'puts "STEREO_BUTTON_INPUT_GUARD_PASS"']
    if a.cmp_controls:tcl += ['puts "STEREO_CMP_CONTROLS_PASS"']
    tcl += ['puts "STEREO_BOARD_COMPILE_PASS"']
    if a.map or a.pnr:tcl += ['synthesize -ads -selected_syn_tool_opt 2','dev_map','puts "STEREO_BOARD_MAP_PASS"']
    if a.pnr:tcl += ['pnr','report_timing -report_max_skew','puts "STEREO_BOARD_TIMING_REPORTED"']
    if a.bitgen:tcl += ['gen_bit_stream'+(' -scbv {'+a.scbv+'}' if a.scbv else ''),'puts "STEREO_BOARD_BITSTREAM_DONE"']
    tcl += ['exit']
    shutil.copyfile(ROOT/'rtl/core_v2/gamma_q8_v2.hex',out/'gamma_q8_v2.hex')
    (out/'build.tcl').write_text('\n'.join(tcl)+'\n',encoding='ascii')
    inputs=[project,Path('D:\\虚拟C盘\\ziguanng\\stereo_integration\\runs\\20261028_225421_164a1a65\\testing_import_v1\\testing\\200k\\prj\\build_cfg.vh'),selection_path,ROOT/'rtl/core_v2/gamma_q8_v2.hex']+(constraints if a.map or a.pnr or a.extra_constraint else [])+calibration_inputs
    input_hashes={str(f.relative_to(WORKSPACE)).replace('\\','/'):hashlib.sha256(f.read_bytes()).hexdigest() for f in inputs}
    execution_hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),ROOT/'scripts/audit_board_timing.py',ROOT/'scripts/project_common.py')}
    status={'state':'running','runner_pid':os.getpid(),'started_at':time.time(),'aux_enabled':a.aux,'algorithm_enabled':a.algorithm,'cmp_controls_enabled':a.cmp_controls,'pnr_requested':a.pnr,'map_requested':a.map,'source_sha256':manifest,'requested_top_parameters':expected,
        'candidate_controller':str(a.controller.resolve()) if a.controller else None,'requested_scbv':a.scbv,
        'supplemental_constraints':[str(item) for item in constraints[1:]],'build_inputs_sha256':input_hashes,'execution_sha256':execution_hashes,'tcl_sha256':hashlib.sha256((out/'build.tcl').read_bytes()).hexdigest(),
        'calibration_scope':binding,'stereo_scope':stereo_binding,'replacements':a.replace,'physical_constraint_acceptance':{'status':'NOT_CHECKED','compile_constraint_check':'inspect native diagnostics and exact 3 cell / 2 net matches; exit 0 alone is insufficient','map_pnr':'inspect final actual CPD and gate locations and priority routing applicability','timing':'fresh setup, hold, recovery, removal and IO reports require separate audit'},
        'scope':'actual PG2L200H Top; command execution, input identity, timing and physical constraint acceptance are distinct; IO-budget and CDC structural signoff require separate review'}
    def save(): (out/'status.json').write_text(json.dumps(status,indent=2),encoding='utf8')
    save();env=os.environ.copy();env.setdefault('PANGO_LICENSE_FILE','D:/PDS/licenses/pds_node_locked.lic')
    try:
        with (out/'build.log').open('wb') as log:
            tool=subprocess.Popen(['D:/PDS/PDS_2022.2-SP6.4/bin/pds_shell.exe','-file',native_tool_path(out/'build.tcl').as_posix(),'-work_dir',native_tool_path(out).as_posix(),'-project_name','stereo_board'],
                cwd=native_tool_path(out),env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            status['tool_pid']=tool.pid;status['tool_started_at']=time.time();save();status['returncode']=tool.wait()
        content=(out/'build.log').read_text(encoding='utf8',errors='replace')
        status['compile_pass']='STEREO_BOARD_COMPILE_PASS' in content
        status['actual_parameters']=actual_parameters(content,expected)
        status['parameter_check_pass']=status['actual_parameters']==expected and all(f'STEREO_PARAMETER_PASS {k}={v}' in content for k,v in expected.items())
        status['map_pass']='STEREO_BOARD_MAP_PASS' in content
        status['timing_reported']='STEREO_BOARD_TIMING_REPORTED' in content
        if a.bitgen:
            bits=sorted(out.rglob('*.sbit'))
            status['bitstream_done']='STEREO_BOARD_BITSTREAM_DONE' in content and bool(bits)
            status['bitstreams']={str(b):hashlib.sha256(b.read_bytes()).hexdigest() for b in bits}
        status['source_unchanged']=all(hashlib.sha256((WORKSPACE/k).read_bytes()).hexdigest()==v for k,v in manifest.items())
        status['build_inputs_unchanged']=all(hashlib.sha256((WORKSPACE/k).read_bytes()).hexdigest()==v for k,v in input_hashes.items())
        status['commands_pass']=status['returncode']==0 and status['compile_pass'] and status['parameter_check_pass'] and status['source_unchanged'] and status['build_inputs_unchanged'] and (not (a.map or a.pnr) or status['map_pass']) and (not a.pnr or status['timing_reported']) and (not a.bitgen or status['bitstream_done'])
        status['warnings']=[s for s in content.splitlines() if 'WARNING' in s.upper()]
        # timing_reported alone is never promoted to timing_pass.
        status['timing_pass']=None
        if a.pnr:
            report=out/'report_timing/Top.rtr'
            if report.is_file():
                data=report.read_bytes();timing=audit(data.decode('utf8',errors='replace'),content)
                timing.update(report=str(report),report_sha256=hashlib.sha256(data).hexdigest())
                (out/'timing_audit.json').write_text(json.dumps(timing,ensure_ascii=False,indent=2),encoding='utf8')
                status['timing_pass']=timing['timing_pass']
        status['state']=verdict(status['commands_pass'],status['timing_pass'],a.pnr)
    except Exception as e:
        status['state']='failed';status['exception']=str(e);raise
    finally:save()
    print(json.dumps({k:v for k,v in status.items() if k not in ('source_sha256','warnings')},ensure_ascii=False),flush=True)
    if status['state']!='passed':
        print(content[-4000:]);raise SystemExit(1)

if __name__=='__main__':main()
