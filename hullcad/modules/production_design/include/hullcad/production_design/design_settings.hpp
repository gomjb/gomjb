#pragma once
// Production-design settings (생산설계): the yard's facilities plus the rules and takeoff
// assumptions used to divide the hull into blocks.
#include <vector>

#include "hullcad/yard/facility.hpp"

namespace hullcad {

struct ProductionDesignSettings : FacilityProfile {
    struct Weight {
        double steel_margin = 0.03;    // welding, mill tolerance
        double outfit_ratio = 0.05;    // pre-outfitting weight as a fraction of steel
        double rigging_weight = 8.0;   // lifting beams and slings, added to every crane lift
    } weight;

    struct Division {
        double min_ring_length = 6.0;
        double max_ring_length = 30.0;
        double min_butt_clearance = 0.3;  // butt distance from transverse bulkheads and webs
        double seam_offset = 0.3;         // seam distance from longitudinal members
        double deck_zone_depth = 3.0;     // depth of the deck blocks below the upper deck
        double end_split_z = 10.0;        // aft/fore bodies: lower/upper block split height
        double side_split_z = 0.0;        // cargo side blocks: lower/upper split height, 0 = none
        std::vector<double> long_seams_y; // extra longitudinal seams (port offsets, mirrored) that
                                          // split DB/BHD/DECK and end-body blocks across the breadth
    } division;

    // Geometric takeoff assumptions for details the structure model does not carry.
    struct Takeoff {
        double hole_area = 0.5;              // m^2 per web opening (manhole / lightening hole)
        double hole_perimeter = 2.7;         // m of cutting per opening
        double slot_length = 0.9;            // m of cutting per longitudinal slot in a web
        double stiffener_paint_girth = 0.7;  // m^2 of coated surface per metre of stiffener
        double heating_line_density = 1.5;   // m of line heating per m^2 of curved shell
    } takeoff;
};

}  // namespace hullcad
