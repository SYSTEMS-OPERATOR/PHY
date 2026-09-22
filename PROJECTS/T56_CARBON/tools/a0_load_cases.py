#!/usr/bin/env python3
"""Deterministic closed-form checks for the bounded T56 A0 shoulder article."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
REQ = PROJECT / "requirements"


def read(name):
    return json.loads((REQ / name).read_text(encoding="utf-8"))


def value(scope, record_id):
    return scope["mission_inputs"][record_id]["value"]


def calculate():
    scope = read("a0_article_scope.json")
    design = read("a0_shoulder_mechanism.json")
    basis = read("a0_analysis_basis.json")
    constants, section = basis["constants"], basis["section"]
    material = basis["material_minima"]

    g = constants["gravity_m_s2"]
    mass = value(scope, "ME-MASS-002")
    payload = value(scope, "ME-PAYLOAD-001")
    speed = math.radians(value(scope, "ME-MOTION-001"))
    acceleration = math.radians(value(scope, "ME-MOTION-002"))
    external_force = value(scope, "ME-EXT-001")
    impact_energy = value(scope, "ME-IMPACT-001")
    life = value(scope, "ME-LIFE-001")
    service_force = value(scope, "ME-SERVICE-001")
    length_mm = design["dummy_member"]["effective_length_mm"]
    length_m = length_mm / 1000

    outer = section["dummy_tube_outer_mm"]
    inner = outer - 2 * section["dummy_tube_wall_mm"]
    area = outer**2 - inner**2
    inertia = (outer**4 - inner**4) / 12
    section_modulus = inertia / (outer / 2)
    mass_inertia = mass * (length_m / 2)**2 + payload * length_m**2
    gravity_moment = mass * g * length_m / 2 + payload * g * length_m
    total_weight = (mass + payload) * g
    gravity_tip_deflection = total_weight * length_mm**3 / (
        3 * material["6061_T6"]["elastic_modulus_MPa"] * inertia
    )

    sf_yield, sf_ultimate = 2, 3
    dynamic_torque = mass_inertia * acceleration
    ordinary_stop_energy = 0.5 * mass_inertia * speed**2
    push_moment = external_force * length_m
    push_retention_moment = push_moment * sf_ultimate
    push_tube_stress = push_retention_moment * 1000 / section_modulus

    design_impact_energy = impact_energy * sf_ultimate
    stop_travel_m = constants["stop_pad_usable_travel_mm"] / 1000
    stop_radius_m = constants["pitch_stop_radius_mm"] / 1000
    impact_peak_force = 2 * design_impact_energy / stop_travel_m
    impact_peak_moment = impact_peak_force * stop_radius_m
    impact_tube_stress = impact_peak_moment * 1000 / section_modulus

    shaft_d = section["pitch_shaft_diameter_mm"]
    shaft_tau = 16 * impact_peak_moment * 1000 / (math.pi * shaft_d**3)
    shaft_vm = math.sqrt(3) * shaft_tau
    shaft_yield_sf = material["4140_PREHARD"]["yield_MPa"] / shaft_vm

    rocker_i = section["rocker_width_mm"] * section["rocker_thickness_mm"]**3 / 12
    rocker_buckling = math.pi**2 * material["6061_T6"]["elastic_modulus_MPa"] * rocker_i / section["rocker_pin_center_mm"]**2
    index_energy = gravity_moment * math.sin(math.radians(constants["degraded_index_interval_deg"]))
    degraded_speed = math.sqrt(2 * index_energy / mass_inertia)
    service_moment = service_force * constants["service_handle_lever_mm"] / 1000

    common = {
        "status": "analysis_complete_physical_validation_open",
        "measured_evidence": False,
        "method": "closed_form",
    }
    cases = {
        "LC-SHO-001": {**common,
            "results": {
                "service_gravity_moment_Nm": gravity_moment,
                "yield_design_moment_Nm": gravity_moment * sf_yield,
                "ultimate_design_moment_Nm": gravity_moment * sf_ultimate,
                "conservative_tip_deflection_mm": gravity_tip_deflection,
            },
            "acceptance_criteria": {
                "proof_multiplier_x_service": 2.0,
                "maximum_elastic_deflection_mm": 2.0,
                "maximum_permanent_set_mm": 0.25,
                "minimum_yield_safety_factor": 2.0,
                "minimum_ultimate_safety_factor": 3.0,
            }},
        "LC-SHO-002": {**common,
            "results": {
                "rotational_inertia_kg_m2": mass_inertia,
                "commanded_acceleration_torque_Nm": dynamic_torque,
                "gravity_plus_acceleration_moment_Nm": gravity_moment + dynamic_torque,
                "ordinary_stop_energy_J": ordinary_stop_energy,
            },
            "acceptance_criteria": {
                "maximum_speed_deg_s": value(scope, "ME-MOTION-001"),
                "maximum_acceleration_deg_s2": value(scope, "ME-MOTION-002"),
                "cycle_count": life,
                "maximum_hysteresis_deg": 1.0,
                "maximum_ordinary_stop_energy_J": 0.25,
                "maximum_temperature_rise_C": 20,
                "maximum_sound_dBA_at_1m": value(scope, "ME-ACOUSTIC-001"),
            }},
        "LC-SHO-003": {**common,
            "results": {
                "service_force_N": external_force,
                "service_root_moment_Nm": push_moment,
                "retention_design_moment_Nm": push_retention_moment,
                "tube_stress_at_retention_design_MPa": push_tube_stress,
                "tube_yield_margin_x": material["6061_T6"]["yield_MPa"] / push_tube_stress,
            },
            "acceptance_criteria": {
                "service_force_each_direction_N": external_force,
                "proof_force_each_direction_N": external_force * sf_yield,
                "retention_force_each_direction_N": external_force * sf_ultimate,
                "maximum_permanent_set_mm": 0.25,
                "maximum_datum_shift_mm": 0.25,
            }},
        "LC-SHO-004": {**common,
            "results": {
                "service_energy_J": impact_energy,
                "design_energy_J": design_impact_energy,
                "linear_spring_peak_stop_force_N": impact_peak_force,
                "peak_stop_moment_Nm": impact_peak_moment,
                "tube_stress_at_design_energy_MPa": impact_tube_stress,
                "shaft_von_mises_stress_MPa": shaft_vm,
                "shaft_yield_margin_x": shaft_yield_sf,
                "rocker_weak_axis_euler_buckling_N": rocker_buckling,
            },
            "acceptance_criteria": {
                "impact_energy_J": impact_energy,
                "repetitions_per_direction": 3,
                "minimum_usable_stop_travel_mm": constants["stop_pad_usable_travel_mm"],
                "maximum_post_impact_alignment_shift_mm": 0.5,
                "maximum_post_impact_zero_shift_deg": 0.5,
                "maximum_rebound_deg": 10,
                "retainers_remaining_engaged": 1,
            }},
        "LC-SHO-005": {**common,
            "results": {
                "maximum_gravity_moment_Nm": gravity_moment,
                "specified_friction_breakaway_Nm": constants["pitch_friction_breakaway_min_Nm"],
                "friction_to_gravity_ratio": constants["pitch_friction_breakaway_min_Nm"] / gravity_moment,
                "degraded_single_fault_index_interval_deg": constants["degraded_index_interval_deg"],
                "degraded_single_fault_catch_energy_J": index_energy,
                "frictionless_degraded_peak_speed_deg_s": math.degrees(degraded_speed),
            },
            "acceptance_criteria": {
                "minimum_friction_breakaway_Nm": constants["pitch_friction_breakaway_min_Nm"],
                "normal_power_loss_maximum_drop_deg": 2,
                "single_fault_maximum_drop_deg": constants["degraded_index_interval_deg"],
                "single_fault_maximum_catch_energy_J": impact_energy,
                "single_fault_maximum_drop_speed_deg_s": 300,
                "maximum_safe_state_detection_ms": 100,
                "secondary_tether_minimum_rating_N": 1000,
            }},
        "LC-MNT-001": {**common,
            "results": {
                "service_handling_force_N": service_force,
                "service_handle_moment_Nm": service_moment,
                "proof_handling_force_N": service_force * sf_yield,
                "retention_handling_force_N": service_force * sf_ultimate,
            },
            "acceptance_criteria": {
                "service_handling_force_N": service_force,
                "maximum_root_datum_shift_mm": 0.25,
                "maximum_reassembled_output_position_error_mm": 0.5,
                "maximum_reassembled_zero_error_deg": 0.5,
                "maximum_fastener_preload_change_percent": 10,
                "minimum_tool_clearance_mm": 25,
                "maximum_service_time_min": 30,
            }},
    }
    return {
        "packet_id": design["packet_id"],
        "status": "closed_form_analysis_complete_physical_validation_open",
        "fabrication_released": False,
        "physical_evidence_complete": False,
        "inputs": {
            "mass_kg": mass, "payload_kg": payload, "effective_length_mm": length_mm,
            "tube_area_mm2": area, "tube_inertia_mm4": inertia,
            "tube_section_modulus_mm3": section_modulus,
        },
        "cases": cases,
        "method_limits": basis["method_limits"],
    }


def markdown(result):
    lines = ["# T56 A0 load-case calculation report", "",
             "Status: **CLOSED-FORM ANALYSIS COMPLETE — PHYSICAL VALIDATION OPEN**", "",
             "These are deterministic design checks, not measured capability, FEA, or test evidence.", "",
             "| Case | Key calculated result | Physical disposition |", "| --- | --- | --- |"]
    keys = {
        "LC-SHO-001": "service_gravity_moment_Nm",
        "LC-SHO-002": "ordinary_stop_energy_J",
        "LC-SHO-003": "retention_design_moment_Nm",
        "LC-SHO-004": "shaft_yield_margin_x",
        "LC-SHO-005": "degraded_single_fault_catch_energy_J",
        "LC-MNT-001": "retention_handling_force_N",
    }
    for case_id, key in keys.items():
        value = result["cases"][case_id]["results"][key]
        lines.append(f"| {case_id} | {key} = {value:.6g} | test record open |")
    lines += ["", "## Method limits", ""] + [f"- {item}" for item in result["method_limits"]]
    lines += ["", "Numeric acceptance criteria are recorded in `load_case_register.json`. Passing the arithmetic does not approve a load case; approval requires the corresponding signed bench record."]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result = calculate()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "A0_LOAD_CASES.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (args.output_dir / "A0_LOAD_CASES.md").write_text(markdown(result), encoding="utf-8")
    print(result["status"])


if __name__ == "__main__":
    main()
