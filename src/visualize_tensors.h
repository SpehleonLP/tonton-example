#ifndef GLTF_VISUALIZE_TENSORS_H
#define GLTF_VISUALIZE_TENSORS_H

namespace fx { namespace gltf { struct Document; struct Primitive; }}
class RintintinCommand;

void VisualizeTensors(RintintinCommand const& cmd, fx::gltf::Document& dst);

#endif
