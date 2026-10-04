"""Offline data-integrity checks; no serial or network access."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from c6_floor_model_report import extract


class FloorModelReportTests(unittest.TestCase):
    def fixture(self):
        frames=[];history=[]
        for n in range(81):
            t=1000+n*10;u=[90,98,82,98,82][min(4,n//15)]
            history.append([t,1000,1000+20*(u-90),-1800,900,0,0,0,u,-90,80])
            if n%5==0:
                motors=[dict(count=n*10,edges=n*10,pwm=p,speed=v) for p,v in zip([u,-90,80],[1000,-1800,900])]
                frames.append(dict(ms=t,motors=motors,control_samples=copy.deepcopy(history[-8:])))
        return dict(capture_complete=True,final_status=dict(armed=False,motors=[dict(pwm=0,speed=0)]*3),
                    plan=dict(motor=1,direction='left',role='train'),baseline_pwm=[90,-90,80],
                    phases=[dict(accepted_ms=100),dict(accepted_ms=1000)],frames=frames)

    def test_deduplicated_trace_has_fixed_other_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'run.json';p.write_text(json.dumps(self.fixture()))
            _,t,u,y,q,_=extract(p)
            self.assertEqual(len(t),79);self.assertEqual(q['quadrature_efficiency'],1)
            self.assertEqual(q['pwm_span'],16)

    def test_invalid_data_cannot_become_a_model(self):
        for error in ('incomplete','moving','other_input','nonfinite','trace_gap'):
            r=self.fixture()
            if error=='incomplete':r['capture_complete']=False
            if error=='moving':r['final_status']['armed']=True
            for f in r['frames']:
                if error=='trace_gap':f['control_samples']=[s for s in f['control_samples'] if not 1300<=s[0]<=1400]
                for row in f['control_samples']:
                    if row[0]>=1500:
                        if error=='other_input':row[9]=-91
                        if error=='nonfinite':row[2]=float('nan')
            with tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp)/'run.json';p.write_text(json.dumps(r))
                with self.assertRaises(ValueError):extract(p)


if __name__=='__main__':unittest.main()
