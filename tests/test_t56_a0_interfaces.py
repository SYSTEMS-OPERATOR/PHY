"""Independent assembly regressions for the A0-R2 manufacturing pass."""
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('a0_r2_export', ROOT / 'bin/export_t56_a0_article.py')
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)

@unittest.skipUnless(importlib.util.find_spec('cadquery'), 'optional CadQuery unavailable')
class A0AssemblyInterfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.design = exporter.load_design()
        cls.parts = exporter.build_parts(cls.design)
        cls.instances = {name: shape.val() for name, _, shape in exporter.located_instances(cls.parts, cls.design)}

    def test_yokes_seat_on_carriage_without_penetration(self):
        for name in ('yoke_1', 'yoke_2'):
            self.assertLess(self.instances[name].intersect(self.instances['carriage']).Volume(), 1e-5)
            self.assertAlmostEqual(self.instances[name].distance(self.instances['carriage']), 0, places=6)

    def test_caps_have_positive_outer_ring_overlap(self):
        cap = self.parts['A0-310_bearing_cap'].val()
        # Distinct circular inner edge: R1's opening was exactly the bearing OD.
        radii = sorted({round(e.radius(), 6) for e in cap.Edges() if e.geomType() == 'CIRCLE'})
        bearing_r = self.design['fabrication_geometry']['cartridge']['bearing_id_od_width'][1] / 2
        retaining_opening = next(r for r in radii if r > 10)
        self.assertGreater(bearing_r - retaining_opening, 1)


    def test_neutral_and_sampled_interfaces_are_exactly_checked(self):
        report=exporter.assembly_interface_report(self.parts,self.design)
        self.assertEqual(report['result'],'pass_revised_interfaces_at_samples_only')
        self.assertEqual(report['revised_neutral_unintended_penetrations'],[])
        self.assertEqual(report['fixture_penetrations'],[])
        self.assertFalse(report['whole_article_clearance_approved'])
        for row in report['pitch_pose_checks']:
            self.assertEqual(row['unintended_penetrations'],[])
            if row['pitch_deg']==-32:self.assertAlmostEqual(row['hub_gaps_mm']['pitch_stop_pad_1'],0,places=5)
            if row['pitch_deg']==92:self.assertAlmostEqual(row['hub_gaps_mm']['pitch_stop_pad_2'],0,places=5)
        self.assertLess(self.instances['dummy_member'].intersect(self.instances['hub']).Volume(),1e-5)

    def test_geometry_and_projected_dimensions_follow_stock_change(self):
        import copy
        changed=copy.deepcopy(self.design);changed['fabrication_geometry']['dummy_member']['cut_length']=328
        part=exporter.build_parts(changed)['A0-108_dummy_member'].val()
        self.assertAlmostEqual(part.BoundingBox().xlen,328)
        render=exporter.drawings(changed)
        drawing=next(value for name,value in render.items() if name.startswith('A0-D003'))
        self.assertIn('328.000 X',drawing)
        self.assertIn('<path',drawing);self.assertIn('marker-end',drawing)

class A0FreeBodies(unittest.TestCase):
    def test_independent_balances_and_axial_torque_separation(self):
        from PROJECTS.T56_CARBON.tools.a0_interfaces import bearing_reactions
        r=bearing_reactions([0,0,-1000],[0,0,0],[-30,30])
        self.assertEqual(r['A_N'],[0,0,500]);self.assertEqual(r['B_N'],[0,0,500])
        r=bearing_reactions([0,0,0],[0,10000,0],[-30,30])
        self.assertEqual(r['radial_N'],[0,0]);self.assertEqual(r['torque_requiring_separate_reaction_Nmm'],10000)
        r=bearing_reactions([0,0,-100],[-9000,0,0],[0,60])
        self.assertEqual(r['A_N'][2],-50);self.assertEqual(r['B_N'][2],150)
        self.assertEqual(r['force_residual_N'],[0,0,0]);self.assertEqual(r['moment_residual_Nmm'],[0,0,0])

    def test_hand_worked_impact_includes_mass_resultant(self):
        from PROJECTS.T56_CARBON.tools.a0_interfaces import impact_free_body
        # m=2 at L/2=0.5 gives J=0.5, first moment=1; F=3000,
        # alpha=3000*.06/.5=360; total force=3360, supports=1680.
        r=impact_free_body(exporter.load_design(),2,0,1000,6,4)
        self.assertAlmostEqual(r['stop_force_N'],3000)
        self.assertAlmostEqual(r['inertia_resultant_N'],360)
        self.assertAlmostEqual(r['maximum_bearing_radial_N'],1680)

    def test_invalid_units_and_degenerate_inputs_fail(self):
        from PROJECTS.T56_CARBON.tools.a0_interfaces import bearing_reactions, impact_free_body, equivalent_static_load
        with self.assertRaises(ValueError):bearing_reactions([0,0,0],[0,0,0],[30,30])
        with self.assertRaises(ValueError):bearing_reactions([0,0,float('nan')],[0,0,0],[-30,30])
        with self.assertRaises(ValueError):impact_free_body(exporter.load_design(),2,0,317,6,0)
        with self.assertRaises(ValueError):equivalent_static_load(-1,0)

    def test_supplier_abutments_and_interval_stack(self):
        from PROJECTS.T56_CARBON.tools.a0_interfaces import interface_stacks
        r=interface_stacks(exporter.load_design())
        self.assertAlmostEqual(r['cap_radial_overlap_mm'][0],2.475)
        self.assertAlmostEqual(r['left_outer_ring_axial_clearance_mm'][0],.01)
        self.assertGreater(r['floating_seat_reserve_mm'],.15)
        self.assertGreater(r['floating_seat_inward_available_mm'],r['floating_seat_inward_demand_mm'])
        self.assertGreater(r['floating_seat_outward_available_mm'],r['floating_seat_outward_demand_mm'])
        self.assertGreater(r['shaft_diametral_clearance_mm'][1],0)
        self.assertLess(r['shaft_diametral_clearance_mm'][0],0)
        self.assertFalse(r['fabrication_released'])

    def test_radius_propagates_into_contact_load_and_sensitivity(self):
        import copy
        from PROJECTS.T56_CARBON.tools.a0_interfaces import stop_geometry, impact_free_body
        d=exporter.load_design();changed=copy.deepcopy(d);changed['fabrication_geometry']['cartridge']['stop_lug_radius']=65
        a=impact_free_body(d,2.5,.5,317,6,4);b=impact_free_body(changed,2.5,.5,317,6,4)
        self.assertAlmostEqual(stop_geometry(changed)[0]['effective_arm_mm'],65)
        self.assertAlmostEqual(b['stop_torque_Nm']/a['stop_torque_Nm'],65/60)
        self.assertGreater(b['maximum_bearing_radial_N'],a['maximum_bearing_radial_N'])
        harder=impact_free_body(d,2.5,.5,317,6,2)
        self.assertAlmostEqual(harder['maximum_bearing_radial_N'],2*a['maximum_bearing_radial_N'])


class A0SelectiveShims(unittest.TestCase):
    def test_short_and_long_measured_stacks_require_different_shims(self):
        from PROJECTS.T56_CARBON.tools.a0_interfaces import select_end_shims
        short=select_end_shims(163.6,164)
        self.assertAlmostEqual(short['annular_stack_shim_mm'],.3)
        self.assertEqual(short['shaft_face_standoff_mm'],0)
        long=select_end_shims(164.2,164)
        self.assertAlmostEqual(long['shaft_face_standoff_mm'],.3)
        self.assertEqual(long['annular_stack_shim_mm'],0)
        with self.assertRaises(ValueError):select_end_shims(165,164)

class A0AttachmentArithmetic(unittest.TestCase):
    def test_crossbar_flat_section_converges_to_known_full_circle(self):
        from PROJECTS.T56_CARBON.tools.a0_interfaces import attachment_screens
        d=exporter.load_design();d['fabrication_geometry']['cartridge']['stop_bar_flat_offset']=9.999999
        r=attachment_screens(d,1000)
        self.assertAlmostEqual(r['crossbar_flat_section_area_mm2'],math.pi*100,places=5)
        self.assertAlmostEqual(r['crossbar_flat_section_modulus_mm3'],math.pi*1000/4,places=4)
        self.assertAlmostEqual(r['crossbar_unnotched_bending_MPa'],45000/(math.pi*1000/4),places=4)
