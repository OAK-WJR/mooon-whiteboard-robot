import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from c6_floor_excitation import plan,base_duties,perturbed,check_raw


class FloorExcitationTests(unittest.TestCase):
    def test_moving_range_plan_and_limits_are_explicit(self):
        for direction in ['left','right']:
            p=plan(direction,2,'train',True)
            self.assertEqual(p['body_request_mps'],.08)
            self.assertAlmostEqual(p['maximum_total_command_seconds'],1.4)
            self.assertEqual(p['pwm_perturbations'],[8,-8,8,-8])
            self.assertLess(p['body_request_mps']*p['maximum_total_command_seconds'],p['encoder_excursion_limit_m'])
            self.assertEqual(p['expected_revision'],'robust-wheels-14-coordinated-stop')
            check_raw({'motors':[dict(speed=3000,pwm=100)]},[100],p['maximum_raw_speed_cps'])
            with self.assertRaises(RuntimeError):check_raw({'motors':[dict(speed=3501,pwm=100)]},[100],p['maximum_raw_speed_cps'])
    def test_inputs_bounded_balanced_and_validation_distinct(self):
        for d in ['left','right']:
            for m in [1,2,3]:
                a=plan(d,m);b=plan(d,m,'validation')
                self.assertNotEqual(a['pwm_perturbations'],b['pwm_perturbations'])
                self.assertEqual(sum(a['pwm_perturbations']),0)
                self.assertLessEqual(a['pi_seconds']+a['fixed_baseline_seconds']+4*a['step_seconds'],2)
                base=[100,-80,85]
                for v in a['pwm_perturbations']:
                    u=perturbed(base,m,v)
                    self.assertEqual([u[i] for i in range(3) if i!=m-1],[base[i] for i in range(3) if i!=m-1])

    def test_duty_and_feedback_fail_closed(self):
        with self.assertRaises(RuntimeError):perturbed([128,-80,85],1,4)
        for speed,pwm in [(2300,100),(-300,100),(500,131),(float('nan'),100)]:
            with self.assertRaises(RuntimeError):check_raw({'motors':[dict(speed=speed,pwm=pwm)]},[100])

    def test_launch_not_settled_cannot_enter_raw_phase(self):
        targets=[500,-1000,500]
        rows=[dict(ms=t,motors=[dict(speed=s,pwm=p) for s,p in zip(targets,[100,-80,85])]) for t in range(1000,1250,50)]
        self.assertEqual(base_duties(rows,targets,1200),[100,-80,85])
        rows[-1]['motors'][0]['speed']=0
        with self.assertRaises(RuntimeError):base_duties(rows,targets,1200)


if __name__=='__main__':unittest.main()
