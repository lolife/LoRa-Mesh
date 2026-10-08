#include <mesh_protocol/esp32/mesh.h>
#include <mesh_protocol/esp32/build_role.h>

const MeshOptions& meshApplicationOptions() {
    static const MeshOptions options = {mesh::standardBuildRole(), nullptr, nullptr, nullptr};
    return options;
}
#include <mesh_protocol/esp32/mesh_impl.h>
