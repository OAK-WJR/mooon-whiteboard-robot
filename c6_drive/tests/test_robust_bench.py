import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from c6_robust_bench import analyze, signed_delta


class ResponseMetricsTests(unittest.TestCase):
    def test_board_clock_response_and_reverse_encoder_wrap(self):
        test={'name':'synthetic','values':[0,-3000,0],'seconds':2.2}
        rows=[]
        for n in range(221):
            speed=-3000*(1-math.exp(-n*.01/.2))
            rows.append([500000+n*10,500000,0,speed,0,0,-3000,0,0,-100,0])
        motors=lambda count,edges:[{'count':0,'edges':0},{'count':count,'edges':edges},{'count':0,'edges':0}]
        frames=[{'armed':True,'motors':motors(100,10)},{'armed':True,'motors':motors(-1900,2010)}]
        result=analyze(test,rows+rows[20:40],frames,500000)
        m=result['motors'][0]
        self.assertEqual(result['sample_count'],221)
        self.assertAlmostEqual(m['t90_s'],.47,places=8)
        self.assertAlmostEqual(m['quadrature_efficiency'],1)
        self.assertTrue(m['tracking_pass'])
        self.assertEqual(signed_delta(0xfffffff0,16),-32)
        # Retain poor tracking and reject missing trace intervals.
        for row in rows:row[3]*=.5
        self.assertFalse(analyze(test,rows,frames,500000)['tracking_pass'])
        with self.assertRaisesRegex(RuntimeError,'gap'):
            analyze(test,rows[:30]+rows[40:],frames,500000)


if __name__=='__main__':unittest.main(verbosity=2)
