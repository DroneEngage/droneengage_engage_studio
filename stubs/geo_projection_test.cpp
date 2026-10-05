/**
 * geo_projection_test.cpp - the C++ side of the P5-08 projection
 * round-trip test. Prints the forward geoProjectToGround() result for a
 * JSON request on argv[1] so stubs/test_geo_projection.py can check the
 * Python inverse against the real shared header.
 *
 *   geo_projection_test '{"u":..,"v":..,"bw":..,"bh":..,"cam":{..},
 *     "mount":{..},"att":{..},"pose":{..},"sigma_att_deg":..,"sigma_alt_m":..}'
 *   -> {"ok":true,"lat":..,"lng":..,"ground_range_m":..,"unc_m":..}
 *
 * Compiled by test_geo_projection.py against the vendored
 * de_common/helpers/geo_projection.hpp (no other dependencies).
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "geo_projection.hpp"

using namespace de::geo;


// minimal field extraction: every request member is a flat
// "key": number pair inside one of the named objects
static double field (const std::string& text, const std::string& key,
                     const double fallback)
{
    const std::string pat = "\"" + key + "\"";
    const size_t i = text.find(pat);
    if (i == std::string::npos) return fallback;
    const size_t c = text.find(':', i + pat.size());
    if (c == std::string::npos) return fallback;
    return std::strtod(text.c_str() + c + 1, nullptr);
}


int main (int argc, char** argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s '<json request>'\n", argv[0]);
        return 2;
    }
    const std::string j(argv[1]);

    const GeoCameraParams cam  = {field(j, "hfov_deg", 62.2),
                                  field(j, "vfov_deg", 48.8),
                                  field(j, "k1", 0.0),
                                  field(j, "k2", 0.0)};
    const GeoMountParams mount = {field(j, "mount_pitch_deg", -90.0),
                                  field(j, "mount_yaw_deg", 0.0),
                                  field(j, "mount_roll_deg", 0.0)};
    const GeoAttitude att      = {field(j, "roll_rad", 0.0),
                                  field(j, "pitch_rad", 0.0),
                                  field(j, "yaw_rad", 0.0)};
    const GeoPose pose         = {field(j, "lat_deg", 0.0),
                                  field(j, "lng_deg", 0.0),
                                  field(j, "rel_alt_m", 0.0)};

    const GeoProjectionResult r = geoProjectToGround(
        field(j, "u", 0.0), field(j, "v", 0.0),
        field(j, "bw", 0.0), field(j, "bh", 0.0),
        cam, mount, att, pose,
        field(j, "sigma_att_deg", 0.0), field(j, "sigma_alt_m", 0.0),
        field(j, "rangefinder_m", -1.0));

    std::printf("{\"ok\":%s,\"lat\":%.10f,\"lng\":%.10f,"
                "\"ground_range_m\":%.6f,\"unc_m\":%.6f}\n",
                r.ok ? "true" : "false", r.lat, r.lng,
                r.ground_range_m, r.unc_m);
    return r.ok ? 0 : 1;
}
