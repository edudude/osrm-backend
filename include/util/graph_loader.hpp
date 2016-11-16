#ifndef GRAPH_LOADER_HPP
#define GRAPH_LOADER_HPP

#include "extractor/external_memory_node.hpp"
#include "extractor/node_based_edge.hpp"
#include "extractor/query_node.hpp"
#include "extractor/restriction.hpp"
#include "util/exception.hpp"
#include "util/fingerprint.hpp"
#include "util/simple_logger.hpp"
#include "util/typedefs.hpp"
#include "storage/io.hpp"

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <tbb/parallel_sort.h>

#include <cmath>

#include <fstream>
#include <ios>
#include <vector>

namespace osrm
{
namespace util
{

/**
 * Reads the .restrictions file and loads it to a vector.
 * The since the restrictions reference nodes using their external node id,
 * we need to renumber it to the new internal id.
*/
inline unsigned loadRestrictionsFromFile(std::string &filename,
                                         std::vector<extractor::TurnRestriction> &restriction_list)
{
    storage::io::FileReader file(filename, storage::io::FileReader::VerifyFingerprint);
    unsigned number_of_usable_restrictions = file.ReadElementCount32();
    
    restriction_list.resize(number_of_usable_restrictions);
    if (number_of_usable_restrictions > 0) {
        file.ReadInto(restriction_list.data(), number_of_usable_restrictions);
    }
    
    return number_of_usable_restrictions;
}

/**
 * Reads the beginning of an .osrm file and produces:
 *  - list of barrier nodes
 *  - list of traffic lights
 *  - nodes indexed by their internal (non-osm) id
 */
inline NodeID loadNodesFromFile(std::string &filename,
                                std::vector<NodeID> &barrier_node_list,
                                std::vector<NodeID> &traffic_light_node_list,
                                std::vector<extractor::QueryNode> &node_array)
{
    storage::io::FileReader file(filename, storage::io::FileReader::VerifyFingerprint);
    NodeID number_of_nodes = file.ReadElementCount32();
    SimpleLogger().Write() << "Importing number_of_nodes new = " << number_of_nodes << " nodes ";

    node_array.reserve(number_of_nodes);

    extractor::ExternalMemoryNode current_node;
    for (NodeID i = 0; i < number_of_nodes; ++i)
    {
        file.ReadInto(&current_node, 1);
        node_array.emplace_back(current_node.lon, current_node.lat, current_node.node_id);
        if (current_node.barrier)
        {
            barrier_node_list.emplace_back(i);
        }
        if (current_node.traffic_lights)
        {
            traffic_light_node_list.emplace_back(i);
        }
    }

    // tighten vector sizes
    barrier_node_list.shrink_to_fit();
    traffic_light_node_list.shrink_to_fit();

    return number_of_nodes;
}

/**
 * Reads a .osrm file and produces the edges.
 */
inline NodeID loadEdgesFromFile(std::string &filename,
                                std::vector<extractor::NodeBasedEdge> &edge_list)
{

    storage::io::FileReader file(filename, storage::io::FileReader::VerifyFingerprint);
    NodeID number_of_nodes = file.ReadElementCount32();
    file.Skip<extractor::ExternalMemoryNode>(number_of_nodes);
    EdgeID number_of_edges_filereader = file.ReadElementCount32();
    edge_list.resize(number_of_edges_filereader);
    SimpleLogger().Write() << " and " << number_of_edges_filereader << " edges ";

    file.ReadInto(edge_list.data(), number_of_edges_filereader);

    BOOST_ASSERT(edge_list.size() > 0);

#ifndef NDEBUG
    SimpleLogger().Write() << "Validating loaded edges...";
    tbb::parallel_sort(
        edge_list.begin(),
        edge_list.end(),
        [](const extractor::NodeBasedEdge &lhs, const extractor::NodeBasedEdge &rhs) {
            return (lhs.source < rhs.source) ||
                   (lhs.source == rhs.source && lhs.target < rhs.target);
        });
    for (auto i = 1u; i < edge_list.size(); ++i)
    {
        const auto &edge = edge_list[i];
        const auto &prev_edge = edge_list[i - 1];

        BOOST_ASSERT_MSG(edge.weight > 0, "loaded null weight");
        BOOST_ASSERT_MSG(edge.forward, "edge must be oriented in forward direction");
        BOOST_ASSERT_MSG(edge.travel_mode != TRAVEL_MODE_INACCESSIBLE, "loaded non-accessible");

        BOOST_ASSERT_MSG(edge.source != edge.target, "loaded edges contain a loop");
        BOOST_ASSERT_MSG(edge.source != prev_edge.source || edge.target != prev_edge.target,
                         "loaded edges contain a multi edge");
    }
#endif

    SimpleLogger().Write() << "Graph loaded ok and has " << edge_list.size() << " edges";

    return number_of_edges_filereader;
}
}
}

#endif // GRAPH_LOADER_HPP
