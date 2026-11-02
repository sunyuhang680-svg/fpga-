"""Conservative multi-clock report audit. No inference from tool completion alone."""
import argparse,hashlib,json,re
from pathlib import Path

def audit(text,diagnostics=''):
    sections={m[1]:m[2] for m in re.finditer(r'^((?:Setup|Hold|Recovery|Removal|Minimum Pulse Width) Summary\((?:Slow|Fast) Corner\)):\s*\n(.*?)(?=^\w[^\n]* Summary\(|^Slow Corner\s*$|\Z)',text,re.M|re.S)}
    rows={};missing=[]
    for corner in ('Slow','Fast'):
        for kind in ('Setup','Hold','Recovery','Removal','Minimum Pulse Width'):
            key=f'{kind} Summary({corner} Corner)';section=sections.get(key,'')
            parsed=[]
            for m in re.finditer(r'^\s*(\S[^\n]*?)\s+(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)\s+(\d+)\s+(\d+)\s*$',section,re.M):
                parsed.append({'clocks':m[1].strip(),'slack_ns':float(m[2]),'total_slack_ns':float(m[3]),'failing_endpoints':int(m[4]),'endpoints':int(m[5])})
            rows[key]=parsed
            if not parsed:missing.append(key)
    verdict=re.search(r'^Design Summary\s*:\s*(.+)$',text,re.M)
    summary=verdict[1].strip() if verdict else None
    warnings=list(dict.fromkeys(line.strip() for line in (text+'\n'+diagnostics).splitlines() if re.search(r'unconstrained|not constrained|no clock|not covered',line,re.I)
        and not re.search(r'\b(?:0|no) unconstrained\b',line,re.I)))
    passed=not missing and summary=='All Constraints Met.' and not warnings and all(r['slack_ns']>=0 and r['failing_endpoints']==0 and r['endpoints']>0 for rs in rows.values() for r in rs)
    return {'timing_pass':passed,'summary':summary,'rows':rows,'missing':missing,'review_unconstrained':warnings,
        'scope':'actual PDS timing report only; not CDC structural signoff, board FPS or latency measurement'}

def main():
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--tool-log',type=Path);a=p.parse_args()
    if a.output.exists():raise FileExistsError('preserve prior audit')
    data=a.report.read_bytes();result=audit(data.decode('utf8',errors='replace'),a.tool_log.read_text(encoding='utf8',errors='replace') if a.tool_log else '')
    result['tool_log_reviewed']=a.tool_log is not None
    result.update(report=str(a.report.resolve()),report_sha256=hashlib.sha256(data).hexdigest())
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8');print(json.dumps(result,ensure_ascii=False))
    if not result['timing_pass']:raise SystemExit(1)

if __name__=='__main__':main()
