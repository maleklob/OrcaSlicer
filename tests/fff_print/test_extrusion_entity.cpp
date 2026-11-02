#include <algorithm>
#include <catch2/catch_all.hpp>

#include <cmath>
#include <cstdlib>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"


using namespace Slic3r;

static inline Slic3r::Point3 random_point3(float LO=-50, float HI=50)
{
    Vec3f pt = Vec3f(LO, LO, LO) + (Vec3d(rand(), rand(), rand()) * (HI-LO) / RAND_MAX).cast<float>();
	return Point3(pt.cast<coord_t>());
}


// build a sample extrusion entity collection with random start and end points.
static Slic3r::ExtrusionPath random_path(size_t length = 20, float LO = -50, float HI = 50)
{
    ExtrusionPath t {erPerimeter, 1.0, 1.0, 1.0};
    for (size_t j = 0; j < length; ++ j)
        t.polyline.append(random_point3(LO, HI));
    return t;
}

static Slic3r::ExtrusionPaths random_paths(size_t count = 10, size_t length = 20, float LO = -50, float HI = 50)
{
    Slic3r::ExtrusionPaths p;
    for (size_t i = 0; i < count; ++ i)
        p.push_back(random_path(length, LO, HI));
    return p;
}

SCENARIO("Polygon flattening", "[ExtrusionEntity]") {
    srand(0xDEADBEEF); // consistent seed for test reproducibility.

    // Generate one specific random path set and save it for later comparison
    Slic3r::ExtrusionPaths nosort_path_set = random_paths();

    Slic3r::ExtrusionEntityCollection sub_nosort;
    sub_nosort.append(nosort_path_set);
    sub_nosort.no_sort = true;

    Slic3r::ExtrusionEntityCollection sub_sort;
    sub_sort.no_sort = false;
    sub_sort.append(random_paths());

    GIVEN("A Extrusion Entity Collection with a child that has one child that is marked as no-sort") {
        Slic3r::ExtrusionEntityCollection sample;
        Slic3r::ExtrusionEntityCollection output;

        sample.append(sub_sort);
        sample.append(sub_nosort);
        sample.append(sub_sort);

        WHEN("The EEC is flattened with default options (preserve_order=false)") {
			output = sample.flatten();
            THEN("The output EEC contains no Extrusion Entity Collections") {
                CHECK(std::count_if(output.entities.cbegin(), output.entities.cend(), [=](const ExtrusionEntity* e) {return e->is_collection();}) == 0);
            }
        }
        WHEN("The EEC is flattened with preservation (preserve_order=true)") {
			output = sample.flatten(true);
            THEN("The output EECs contains one EEC.") {
                CHECK(std::count_if(output.entities.cbegin(), output.entities.cend(), [=](const ExtrusionEntity* e) {return e->is_collection();}) == 1);
            }
            AND_THEN("The ordered EEC contains the same order of elements than the original") {
                // find the entity in the collection
                for (auto e : output.entities)
                    if (e->is_collection()) {
                        ExtrusionEntityCollection *temp = dynamic_cast<ExtrusionEntityCollection*>(e);
                        // check each Extrusion path against nosort_path_set to see if the first and last match the same
                        CHECK(nosort_path_set.size() == temp->entities.size());
                        for (size_t i = 0; i < nosort_path_set.size(); ++ i) {
                            CHECK(temp->entities[i]->first_point() == nosort_path_set[i].first_point());
                            CHECK(temp->entities[i]->last_point() == nosort_path_set[i].last_point());
                        }
                    }
            }
        }
    }
}

static ExtrusionPaths straight_path(const std::vector<double> &xs)
{
    ExtrusionPath path{erExternalPerimeter, 1.0, 0.45f, 0.2f};
    for (double x : xs)
        path.polyline.append(Point3::new_scale(x, 0., 0.));
    return {path};
}

TEST_CASE("Scarf ramp ends on the next loop vertex instead of leaving a short stub", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    // A 20 mm scarf in 10 steps: a remainder shorter than half a 2 mm step is snapped forward.
    const double slope_length = 20.;
    const double max_segment  = scale_(slope_length / 10);

    SECTION("a 0.09 mm remainder extends the ramp to the vertex") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 20.09, 25., 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        REQUIRE(loop.ends.size() == 1);
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20.09, 1e-3));
        CHECK_THAT(unscale_(loop.ends.front().polyline.last_point().x()), WithinAbs(20.09, 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.first_point().x()), WithinAbs(20.09, 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.lines().front().length()), WithinAbs(4.91, 1e-3));
    }

    SECTION("a remainder longer than half a step keeps the exact scarf length") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 21.5, 25., 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20., 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.first_point().x()), WithinAbs(20., 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.lines().front().length()), WithinAbs(1.5, 1e-3));
    }

    SECTION("the ramp never grows by more than a millimetre, whatever the step size") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 21.5, 25., 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, scale_(slope_length), 0.); // a single 20 mm step
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20., 1e-3));
    }

    SECTION("snapping onto the path's last vertex leaves no single-point flat path") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 20.5});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        CHECK(loop.paths.empty());
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20.5, 1e-3));
    }
}

TEST_CASE("Scarf loop drops the micro segments the seam insertion leaves at both ends", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    const double slope_length = 20.;
    const double max_segment  = scale_(slope_length / 10);

    SECTION("a 3 um segment at each end of a single path is removed, the seam point stays") {
        ExtrusionPaths     paths = straight_path({0., 0.003, 5., 10., 15., 21.5, 25., 29.997, 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        REQUIRE(loop.paths.size() == 1);
        const Polyline3 &start = loop.starts.front().polyline;
        CHECK_THAT(unscale_(start.first_point().x()), WithinAbs(0., 1e-4));
        CHECK_THAT(unscale_(start.lines().front().length()), WithinAbs(1.25, 1e-3)); // 5 mm halved twice
        const Polyline3 &flat = loop.paths.front().polyline;
        CHECK_THAT(unscale_(flat.last_point().x()), WithinAbs(30., 1e-4));
        CHECK_THAT(unscale_(flat.lines().back().length()), WithinAbs(5., 1e-3));
    }

    SECTION("a micro path of its own is dropped and the neighbour ends at the seam point") {
        ExtrusionPaths paths = straight_path({0., 0.003});
        ExtrusionPaths rest  = straight_path({0.003, 5., 10., 15., 21.5, 25., 30.});
        paths.push_back(rest.front());
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.first_point().x()), WithinAbs(0., 1e-4));
        CHECK_THAT(unscale_(loop.starts.front().polyline.lines().front().length()), WithinAbs(1.25, 1e-3));
    }

    SECTION("a scarf covering the whole loop still ends at full flow after a trim") {
        // The caller sizes the scarf from the untrimmed loop: 10.003 mm here, 10 mm after the trim.
        ExtrusionPaths     paths = straight_path({0., 0.003, 5., 10.});
        ExtrusionLoopSloped loop(paths, 0., 10.003, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        CHECK(loop.paths.empty());
        CHECK_THAT(loop.starts.back().slope_end.e_ratio, WithinAbs(1., 1e-9));
        CHECK_THAT(unscale_(loop.starts.back().polyline.last_point().x()), WithinAbs(10., 1e-4));
    }

    SECTION("segments longer than the tolerance are kept") {
        ExtrusionPaths     paths = straight_path({0., 0.3, 5., 10., 15., 21.5, 25., 29.7, 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.lines().front().length()), WithinAbs(0.3, 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.lines().back().length()), WithinAbs(0.3, 1e-3));
    }
}

// A teardrop with radius 20: a 270 degree arc in 5 degree steps, closed by two straight 20 mm edges
// that meet in a single 90 degree corner at index 0. Every other vertex is close to 180 degrees.
static std::vector<Vec2d> teardrop_outline()
{
    const double radius = 20.;
    std::vector<Vec2d> outline{Vec2d(radius * std::sqrt(2.), 0.)};
    for (int deg = 45; deg <= 315; deg += 5)
        outline.emplace_back(radius * std::cos(deg * PI / 180.), radius * std::sin(deg * PI / 180.));
    return outline;
}

// A closed loop whose first point, the seam, is outline[seam_idx].
static ExtrusionLoop loop_from_seam(std::vector<Vec2d> outline, size_t seam_idx)
{
    std::rotate(outline.begin(), outline.begin() + seam_idx, outline.end());
    ExtrusionPath path{erExternalPerimeter, 1.0, 0.45f, 0.2f};
    for (const Vec2d &p : outline)
        path.polyline.append(Point3::new_scale(p.x(), p.y(), 0.));
    path.polyline.append(Point3::new_scale(outline.front().x(), outline.front().y(), 0.));
    return ExtrusionLoop(path);
}

TEST_CASE("Loop smoothness only considers the corners within the requested length of the seam", "[ExtrusionEntity][Regression]")
{
    const double threshold = 155. * PI / 180.;
    const double arm       = 0.4;
    const std::vector<Vec2d> outline = teardrop_outline();

    SECTION("a corner beyond the length is ignored, one within it is not") {
        // Index 49 is the arc at 285 degrees: about 10.5 mm of arc and a 20 mm edge lead to the corner, about 30.5 mm away.
        const ExtrusionLoop loop = loop_from_seam(outline, 49);
        CHECK_FALSE(loop.is_smooth(threshold, arm));
        CHECK(loop.is_smooth(threshold, arm, 20.));
        CHECK_FALSE(loop.is_smooth(threshold, arm, 40.));
    }

    SECTION("a seam on the corner is never smooth") {
        CHECK_FALSE(loop_from_seam(outline, 0).is_smooth(threshold, arm, 20.));
    }

    SECTION("a corner just behind the seam is caught through the backward arm") {
        // The seam is 0.2 mm past the corner, within the 0.4 mm arm.
        std::vector<Vec2d> with_seam = outline;
        const Vec2d seam = outline[0] + 0.2 * (outline[1] - outline[0]).normalized();
        with_seam.insert(with_seam.begin() + 1, seam);
        CHECK_FALSE(loop_from_seam(with_seam, 1).is_smooth(threshold, arm, 20.));
    }
}

TEST_CASE("Every extrusion role name converts back to the same role and an unknown name to no role", "[ExtrusionEntity]")
{
    // erCount is not a role: role_to_string() asserts on it.
    const auto role = static_cast<ExtrusionRole>(GENERATE(range(int(erNone), int(erCount))));
    CAPTURE(int(role));
    CHECK(ExtrusionEntity::string_to_role(ExtrusionEntity::role_to_string(role)) == role);
    CHECK(ExtrusionEntity::string_to_role("no such role") == erNone);
}

// Hand-written test double. As a stub it returns a fixed role, length and min_mm3_per_mm; as a spy it counts
// reverse() calls. The counter is a pointer, so the clones ExtrusionEntityCollection makes report to the same int.
class FakeExtrusion : public ExtrusionEntity
{
public:
    FakeExtrusion(int id, ExtrusionRole role, double mm3_per_mm, bool loop = false, int *reverse_calls = nullptr)
        : id(id), m_role(role), m_mm3_per_mm(mm3_per_mm), m_loop(loop), m_reverse_calls(reverse_calls) {}

    int id;

    ExtrusionRole    role() const override { return m_role; }
    bool             is_loop() const override { return m_loop; }
    ExtrusionEntity *clone() const override { return new FakeExtrusion(*this); }
    ExtrusionEntity *clone_move() override { return new FakeExtrusion(std::move(*this)); }
    void             reverse() override { if (m_reverse_calls) ++*m_reverse_calls; }
    double           min_mm3_per_mm() const override { return m_mm3_per_mm; }
    double           length() const override { return m_length; }

    Point         first_point() const override { return m_point.to_point(); }
    Point         last_point() const override { return m_point.to_point(); }
    const Point3 &first_point3() const override { return m_point; }
    const Point3 &last_point3() const override { return m_point; }
    void          polygons_covered_by_width(Polygons &, const float) const override {}
    void          polygons_covered_by_spacing(Polygons &, const float) const override {}
    Polyline      as_polyline() const override { return Polyline(); }
    void          collect_polylines(Polylines &) const override {}
    void          collect_points(Points &) const override {}
    double        total_volume() const override { return 0.; }

private:
    ExtrusionRole m_role;
    double        m_mm3_per_mm;
    double        m_length = 1.;
    bool          m_loop;
    int          *m_reverse_calls;
    Point3        m_point;
};

static int fake_id(const ExtrusionEntity *entity)
{
    const auto *fake = dynamic_cast<const FakeExtrusion *>(entity);
    REQUIRE(fake != nullptr);
    return fake->id;
}

TEST_CASE("Reversing a collection reverses its order and its open children but leaves loops alone", "[ExtrusionEntity]")
{
    // The counters are declared first so they outlive the clones held by the collection.
    int first_reversed = 0, loop_reversed = 0, last_reversed = 0;
    ExtrusionEntityCollection collection;
    collection.append(FakeExtrusion(1, erSolidInfill, 1., false, &first_reversed));
    collection.append(FakeExtrusion(2, erPerimeter, 1., true, &loop_reversed));
    collection.append(FakeExtrusion(3, erSolidInfill, 1., false, &last_reversed));

    collection.reverse();

    REQUIRE(collection.size() == 3);
    CHECK(fake_id(collection.entities[0]) == 3);
    CHECK(fake_id(collection.entities[1]) == 2);
    CHECK(fake_id(collection.entities[2]) == 1);
    CHECK(first_reversed == 1);
    CHECK(last_reversed == 1);
    CHECK(loop_reversed == 0);
}

TEST_CASE("A collection reports the smallest flow of its children and a mixed role when roles differ", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    ExtrusionEntityCollection collection;
    collection.append(FakeExtrusion(1, erSolidInfill, 0.4));
    collection.append(FakeExtrusion(2, erSolidInfill, 0.15));
    collection.append(FakeExtrusion(3, erSolidInfill, 0.3));

    CHECK_THAT(collection.min_mm3_per_mm(), WithinAbs(0.15, 1e-9));
    CHECK(collection.role() == erSolidInfill);

    collection.append(FakeExtrusion(4, erPerimeter, 0.5));
    CHECK(collection.role() == erMixed);
}

TEST_CASE("Removing and replacing children keeps the nested item count consistent", "[ExtrusionEntity]")
{
    const ExtrusionPath wall = straight_path({0., 1.}).front();
    ExtrusionEntityCollection nested;
    nested.append(ExtrusionPaths{wall, wall, wall});

    ExtrusionEntityCollection collection;
    collection.append(wall);
    collection.append(nested);
    collection.append(wall);
    REQUIRE(collection.size() == 3);
    CHECK(collection.items_count() == 5); // 2 paths + the 3 paths inside the nested collection

    collection.remove(0);
    REQUIRE(collection.size() == 2);
    CHECK(collection.items_count() == 4);
    CHECK(collection.entities.front()->is_collection());

    ExtrusionPath gap_fill{erGapFill, 0.5, 0.4f, 0.2f};
    gap_fill.polyline = wall.polyline;
    collection.replace(1, gap_fill);
    REQUIRE(collection.size() == 2);
    CHECK(collection.entities[1]->role() == erGapFill);
    CHECK(collection.entities[1] != &gap_fill); // the collection stores its own clone
    CHECK(collection.items_count() == 4);
}

static ExtrusionPath path_through(ExtrusionRole role, const std::vector<Vec2d> &points)
{
    ExtrusionPath path{role, 1.0, 0.45f, 0.2f};
    for (const Vec2d &p : points)
        path.polyline.append(Point3::new_scale(p.x(), p.y(), 0.));
    return path;
}

TEST_CASE("Making a loop clockwise reverses it only when it runs the other way", "[ExtrusionEntity]")
{
    // A 10 mm square traced counter-clockwise.
    ExtrusionLoop loop(path_through(erExternalPerimeter,
        {Vec2d(0., 0.), Vec2d(10., 0.), Vec2d(10., 10.), Vec2d(0., 10.), Vec2d(0., 0.)}));
    REQUIRE(loop.is_counter_clockwise());

    CHECK(loop.make_clockwise());
    CHECK(loop.is_clockwise());
    CHECK_FALSE(loop.make_clockwise());
    CHECK(loop.is_clockwise());

    CHECK(loop.make_counter_clockwise());
    CHECK(loop.is_counter_clockwise());
}

TEST_CASE("A two-path loop splits, clips and reports overhangs at the requested points", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    // A 10 mm counter-clockwise square, 40 mm long: a regular wall on the bottom and right edges,
    // an overhang wall on the top and left edges.
    ExtrusionLoop loop(ExtrusionPaths{
        path_through(erExternalPerimeter, {Vec2d(0., 0.), Vec2d(10., 0.), Vec2d(10., 10.)}),
        path_through(erOverhangPerimeter, {Vec2d(10., 10.), Vec2d(0., 10.), Vec2d(0., 0.)})});
    REQUIRE_THAT(unscale<double>(loop.length()), WithinAbs(40., 1e-3));

    SECTION("splitting at a vertex starts the loop there and keeps its length") {
        CHECK(loop.split_at_vertex(Point::new_scale(10., 10.)));
        CHECK(loop.first_point() == Point::new_scale(10., 10.));
        CHECK_THAT(unscale<double>(loop.length()), WithinAbs(40., 1e-3));
    }

    SECTION("a point that is not a vertex is not split at") {
        CHECK_FALSE(loop.split_at_vertex(Point::new_scale(5., 5.)));
        CHECK(loop.first_point() == Point::new_scale(0., 0.));
    }

    SECTION("splitting inside a segment starts the loop at that point and keeps its length") {
        loop.split_at(Point::new_scale(5., 0.), false);
        CHECK(loop.first_point() == Point::new_scale(5., 0.));
        CHECK_THAT(unscale<double>(loop.length()), WithinAbs(40., 1e-3));
    }

    SECTION("preferring a non-overhang moves the seam off the overhang wall") {
        // (5, 10) lies on the overhang edge; the closest point of the regular wall is its corner (10, 10).
        loop.split_at(Point::new_scale(5., 10.), true);
        CHECK(loop.first_point() == Point::new_scale(10., 10.));
        CHECK_THAT(unscale<double>(loop.length()), WithinAbs(40., 1e-3));
    }

    SECTION("only an inner vertex of a bridging path is an overhang point") {
        CHECK(loop.has_overhang_point(Point::new_scale(0., 10.)));
        CHECK_FALSE(loop.has_overhang_point(Point::new_scale(10., 0.)));  // inner vertex of the regular wall
        CHECK_FALSE(loop.has_overhang_point(Point::new_scale(10., 10.))); // endpoint shared by both paths
    }

    SECTION("clipping shortens the last path") {
        ExtrusionPaths clipped;
        loop.clip_end(scale_(3.), &clipped);
        REQUIRE(clipped.size() == 2);
        CHECK_THAT(unscale_(clipped.back().polyline.last_point().x()), WithinAbs(0., 1e-3));
        CHECK_THAT(unscale_(clipped.back().polyline.last_point().y()), WithinAbs(3., 1e-3));
    }

    SECTION("clipping more than the last path drops it and shortens the one before") {
        ExtrusionPaths clipped;
        loop.clip_end(scale_(25.), &clipped); // the 20 mm overhang path, then 5 mm of the wall
        REQUIRE(clipped.size() == 1);
        CHECK_THAT(unscale_(clipped.back().polyline.last_point().x()), WithinAbs(10., 1e-3));
        CHECK_THAT(unscale_(clipped.back().polyline.last_point().y()), WithinAbs(5., 1e-3));
    }
}
