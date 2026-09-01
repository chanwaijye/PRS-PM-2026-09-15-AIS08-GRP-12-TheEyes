#include "the_eyes/pose/mesh_registry.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

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
  // asset exists, which is the whole point of building this module early.
  const MeshRegistry registry = LoadMeshRegistry(ConfigPath(), /*require_mesh_files=*/false);
  EXPECT_GE(registry.size(), 3u);

  const MeshEntry* mustard = registry.Find("mustard_bottle");
  ASSERT_NE(mustard, nullptr);
  EXPECT_EQ(mustard->cls, "mustard_bottle");
  EXPECT_FALSE(mustard->symmetric);
  EXPECT_NEAR(mustard->diameter_m, 0.196, 1e-9);
  EXPECT_GT(mustard->extent.minCoeff(), 0.0);

  const MeshEntry* cube = registry.Find("red_cube");
  ASSERT_NE(cube, nullptr);
  EXPECT_TRUE(cube->symmetric) << "a cube has a discrete symmetry group";
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
