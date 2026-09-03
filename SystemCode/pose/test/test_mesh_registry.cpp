#include "the_eyes/pose/mesh_registry.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace the_eyes::pose {
namespace {

class TempYaml {
 public:
  explicit TempYaml(const std::string& body) {
    char tmpl[] = "/tmp/the_eyes_meshes_XXXXXX";
    const int fd = ::mkstemp(tmpl);
    if (fd >= 0) ::close(fd);
    path_ = tmpl;
    std::ofstream(path_) << body;
  }
  ~TempYaml() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

std::string ConfigPath() { return std::string(THE_EYES_CONFIG_DIR) + "/meshes.yaml"; }

TEST(MeshRegistry, LoadsTheShippedConfig) {
  // require_mesh_files=false: the registry must be exercisable before any CAD
  // asset exists, which is the whole point of building this module early. The
  // meshes now exist (2026-09-03), so the true/false paths both matter -- see
  // MeshFilesNamedByTheConfigExist below.
  const MeshRegistry registry = LoadMeshRegistry(ConfigPath(), /*require_mesh_files=*/false);
  EXPECT_GE(registry.size(), 3u);

  // MEASURED from the real meshes, not nominal. The mustard bottle is YCB
  // 006 via the FoundationPose demo data, and 0.19646 is the diameter
  // FoundationPose itself reports for it -- so this doubles as a check that
  // the mesh in assets/ is the one the pose estimator was validated against.
  const MeshEntry* mustard = registry.Find("mustard_bottle");
  ASSERT_NE(mustard, nullptr);
  EXPECT_EQ(mustard->cls, "mustard_bottle");
  EXPECT_FALSE(mustard->symmetric);
  EXPECT_NEAR(mustard->diameter_m, 0.196463, 1e-6);
  EXPECT_GT(mustard->extent.minCoeff(), 0.0);

  const MeshEntry* cube = registry.Find("red_cube");
  ASSERT_NE(cube, nullptr);
  EXPECT_TRUE(cube->symmetric) << "a cube has a discrete symmetry group";
  // Authored rather than scanned, so these are exact rather than measured.
  EXPECT_NEAR(cube->diameter_m, 0.05 * std::sqrt(3.0), 1e-5);
  EXPECT_NEAR(cube->extent.x(), 0.025, 1e-12);

  const MeshEntry* bowl = registry.Find("bowl");
  ASSERT_NE(bowl, nullptr);
  EXPECT_NEAR(bowl->diameter_m, 0.161952, 1e-6);
  // A bowl is wider than it is tall; the placeholder had this backwards enough
  // to matter (z was 7.5 mm too generous, which shifts every containment test).
  EXPECT_GT(bowl->extent.x(), 2.0 * bowl->extent.z());
}

// Re-derives the config's numbers from the OBJ files themselves. Without this,
// config/meshes.yaml is an unchecked assertion about geometry it does not
// contain, and a swapped mesh changes every ADD-S verdict silently.
//
// Parsing OBJs is deliberately kept OUT of the runtime loader (see
// mesh_registry.hpp) -- but a test is not the runtime, and this is the only
// place the two can be compared.
namespace {

struct MeshMeasurement {
  Eigen::Vector3d center_offset = Eigen::Vector3d::Zero();
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
  int vertices = 0;
};

MeshMeasurement MeasureObj(const std::string& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << "cannot open " << path;
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d hi = -lo;
  MeshMeasurement m;
  std::string line;
  while (std::getline(file, line)) {
    if (line.rfind("v ", 0) != 0) continue;
    std::istringstream stream(line.substr(2));
    Eigen::Vector3d v;
    if (!(stream >> v.x() >> v.y() >> v.z())) continue;
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
    ++m.vertices;
  }
  m.center_offset = (hi + lo) * 0.5;
  m.extent = (hi - lo) * 0.5;
  return m;
}

}  // namespace

TEST(MeshRegistry, MeshesAreCentredOnTheirAabb) {
  // The object-frame contract (config/meshes.yaml): pose_w.translation() is the
  // AABB centre. scenegraph's AabbOf() builds `translation() ± extent` and is
  // wrong by exactly this offset if it does not hold -- 43.7 mm on the raw YCB
  // bowl, which is 27% of its diameter. It also puts the origin off the bowl's
  // symmetry axis, so `symmetric: true` stops being true.
  //
  // If this fails, someone installed a raw scanned mesh. Run
  // `python3 training/prepare_meshes.py`.
  const std::string asset_root = std::string(THE_EYES_CONFIG_DIR) + "/..";
  const MeshRegistry registry = LoadMeshRegistry(ConfigPath(), true, asset_root);

  for (const std::string& cls : registry.classes()) {
    const MeshEntry* entry = registry.Find(cls);
    ASSERT_NE(entry, nullptr);
    const MeshMeasurement m = MeasureObj(asset_root + "/" + entry->mesh_path);
    EXPECT_GT(m.vertices, 0) << cls;
    // 1 um: the OBJ files carry 6 decimal places, so this is at the writer's
    // resolution rather than an arbitrary tolerance.
    EXPECT_LT(m.center_offset.cwiseAbs().maxCoeff(), 1e-6)
        << cls << " is not centred on its AABB -- offset "
        << m.center_offset.transpose() * 1000.0 << " mm";
  }
}

TEST(MeshRegistry, ConfiguredExtentsMatchTheActualGeometry) {
  // extent flows into ObjectState::extent and from there into every spatial
  // predicate. A config that disagrees with the mesh is undetectable at
  // runtime: the numbers stay self-consistent and are simply wrong.
  const std::string asset_root = std::string(THE_EYES_CONFIG_DIR) + "/..";
  const MeshRegistry registry = LoadMeshRegistry(ConfigPath(), true, asset_root);

  for (const std::string& cls : registry.classes()) {
    const MeshEntry* entry = registry.Find(cls);
    ASSERT_NE(entry, nullptr);
    const MeshMeasurement m = MeasureObj(asset_root + "/" + entry->mesh_path);
    EXPECT_LT((entry->extent - m.extent).cwiseAbs().maxCoeff(), 1e-5)
        << cls << ": config extent " << entry->extent.transpose() << " but the mesh measures "
        << m.extent.transpose();
    // A half-extent can never exceed the diameter -- catches a metres/millimetres
    // mix-up, which is the error that would otherwise look plausible.
    EXPECT_LT(m.extent.maxCoeff(), entry->diameter_m) << cls;
  }
}

TEST(MeshRegistry, MeshFilesNamedByTheConfigExist) {
  // The meshes landed 2026-09-03. Now that they do, the config must not drift
  // from what is on disk: a stale path here drops the class silently at
  // registry load, and the pipeline then reports "no detections" rather than
  // "your mesh moved".
  // asset_root is the repo root, which is what config/meshes.yaml's relative
  // paths are written against -- the parent of the config directory.
  const std::string asset_root = std::string(THE_EYES_CONFIG_DIR) + "/..";
  const MeshRegistry registry =
      LoadMeshRegistry(ConfigPath(), /*require_mesh_files=*/true, asset_root);
  EXPECT_GE(registry.size(), 3u);
}

TEST(MeshRegistry, UnregisteredClassReturnsNullNotADefault) {
  // The null IS the drop signal for INTERFACES.md §2. Returning a default entry
  // would let an unknown class reach the estimator with a wrong mesh.
  const MeshRegistry registry = LoadMeshRegistry(ConfigPath(), false);
  EXPECT_EQ(registry.Find("banana"), nullptr);
  EXPECT_FALSE(registry.Contains("banana"));
}

TEST(MeshRegistry, ClassesAreReturnedInDeterministicOrder) {
  const MeshRegistry registry = LoadMeshRegistry(ConfigPath(), false);
  const std::vector<std::string> a = registry.classes();
  const std::vector<std::string> b = registry.classes();
  EXPECT_EQ(a, b);
  EXPECT_TRUE(std::is_sorted(a.begin(), a.end()));
}

TEST(MeshRegistry, RejectsMissingDiameter) {
  // ADD-S normalises by diameter. A default here would make every reported
  // pose metric wrong without anything failing.
  const TempYaml f("objects:\n  cube:\n    mesh: a.obj\n    symmetric: false\n"
                   "    extent: [0.02, 0.02, 0.02]\n");
  EXPECT_THROW(LoadMeshRegistry(f.path(), false), std::runtime_error);
}

TEST(MeshRegistry, RejectsNonPositiveDiameter) {
  const TempYaml f("objects:\n  cube:\n    mesh: a.obj\n    symmetric: false\n"
                   "    diameter_m: 0.0\n    extent: [0.02, 0.02, 0.02]\n");
  EXPECT_THROW(LoadMeshRegistry(f.path(), false), std::runtime_error);
}

TEST(MeshRegistry, RejectsMissingSymmetryFlag) {
  // symmetric decides whether fusion may average orientation at all, so its
  // absence is a real ambiguity rather than something to default.
  const TempYaml f("objects:\n  cube:\n    mesh: a.obj\n    diameter_m: 0.08\n"
                   "    extent: [0.02, 0.02, 0.02]\n");
  EXPECT_THROW(LoadMeshRegistry(f.path(), false), std::runtime_error);
}

TEST(MeshRegistry, RejectsMalformedOrNonPositiveExtent) {
  const TempYaml bad_len("objects:\n  cube:\n    mesh: a.obj\n    symmetric: false\n"
                         "    diameter_m: 0.08\n    extent: [0.02, 0.02]\n");
  EXPECT_THROW(LoadMeshRegistry(bad_len.path(), false), std::runtime_error);

  const TempYaml negative("objects:\n  cube:\n    mesh: a.obj\n    symmetric: false\n"
                          "    diameter_m: 0.08\n    extent: [0.02, -0.02, 0.02]\n");
  EXPECT_THROW(LoadMeshRegistry(negative.path(), false), std::runtime_error);
}

TEST(MeshRegistry, RejectsEmptyOrMissingObjectsSection) {
  const TempYaml empty("objects: {}\n");
  EXPECT_THROW(LoadMeshRegistry(empty.path(), false), std::runtime_error);
  const TempYaml none("something_else: 1\n");
  EXPECT_THROW(LoadMeshRegistry(none.path(), false), std::runtime_error);
}

TEST(MeshRegistry, MeshFileCheckIsEnforcedWhenRequested) {
  const TempYaml f("objects:\n  cube:\n    mesh: definitely/not/here.obj\n"
                   "    symmetric: false\n    diameter_m: 0.08\n"
                   "    extent: [0.02, 0.02, 0.02]\n");
  EXPECT_NO_THROW(LoadMeshRegistry(f.path(), /*require_mesh_files=*/false));
  EXPECT_THROW(LoadMeshRegistry(f.path(), /*require_mesh_files=*/true), std::runtime_error);
}

TEST(MeshRegistry, MissingFileThrows) {
  EXPECT_THROW(LoadMeshRegistry("/nonexistent/meshes.yaml", false), std::runtime_error);
}

}  // namespace
}  // namespace the_eyes::pose
