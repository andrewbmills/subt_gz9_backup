/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef SUBT_VISIBILITYTYPES_HH_
#define SUBT_VISIBILITYTYPES_HH_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <ignition/math/graph/Graph.hh>

namespace subt
{

/// \def VisibilityGraph
/// \brief An undirected graph to represent communication visibility between
/// different areas of the world.
using VisibilityGraph =
    ignition::math::graph::UndirectedGraph<std::string, uint8_t>;

// This is the lookup table where we store the visibility cost from any pair
// of vertices. The key is a pair containing the IDs of the two vertices.
// The value is the visibility cost between the two vertices.
using VisibilityInfo =
    std::map<std::pair<ignition::math::graph::VertexId,
                       ignition::math::graph::VertexId>,
             double>;

}
#endif
