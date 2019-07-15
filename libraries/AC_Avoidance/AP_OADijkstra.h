#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_OAVisGraph.h"

/*
 * Dijkstra's algorithm for path planning around polygon fence
 */

class AP_OADijkstra {
public:

    AP_OADijkstra();

    /* Do not allow copies */
    AP_OADijkstra(const AP_OADijkstra &other) = delete;
    AP_OADijkstra &operator=(const AP_OADijkstra&) = delete;

    // set fence margin (in meters) used when creating "safe positions" within the polygon fence
    void set_fence_margin(float margin) { _polyfence_margin = MAX(margin, 0.0f); }

    // update return status enum
    enum AP_OADijkstra_State : uint8_t {
        DIJKSTRA_STATE_NOT_REQUIRED = 0,
        DIJKSTRA_STATE_ERROR,
        DIJKSTRA_STATE_SUCCESS
    };

    // calculate a destination to avoid the polygon fence
    // returns DIJKSTRA_STATE_SUCCESS and populates origin_new and destination_new if avoidance is required
    AP_OADijkstra_State update(const Location &current_loc, const Location &destination, Location& origin_new, Location& destination_new);

private:

    // returns true if at least one inclusion or exclusion zone is enabled
    bool some_fences_enabled() const;

    //
    // polygon fence related methods
    //

    // check if polygon fence has been updated since we created the inner fence. returns true if changed
    bool check_polygon_fence_updated() const;

    // create a smaller polygon fence within the existing polygon fence
    // returns true on success
    bool create_polygon_fence_with_margin(float margin_cm);

    //
    // exclusion polygon methods
    //

    // check if exclusion polygons have been updated since create_exclusion_polygon_with_margin was run
    // returns true if changed
    bool check_exclusion_polygon_updated() const;

    // create polygons around existing exclusion polygons
    // returns true on success
    bool create_exclusion_polygon_with_margin(float margin_cm);

    //
    // exclusion circle methods
    //

    // check if exclusion circles have been updated since create_exclusion_circle_with_margin was run
    // returns true if changed
    bool check_exclusion_circle_updated() const;

    // create polygons around existing exclusion circles
    // returns true on success
    bool create_exclusion_circle_with_margin(float margin_cm);

    //
    // other methods
    //

    // returns total number of points across all fence types
    uint16_t total_numpoints() const;

    // get a single point across the total list of points from all fence types
    // also returns the type of point
    bool get_point(uint16_t index, Vector2f& point) const;

    // returns true if line segment intersects polygon or circular fence
    bool intersects_fence(const Vector2f &seg_start, const Vector2f &seg_end) const;

    // create visibility graph for all fence (with margin) points
    // returns true on success
    bool create_fence_visgraph();

    // calculate shortest path from origin to destination
    // returns true on success
    // requires create_polygon_fence_with_margin and create_polygon_fence_visgraph to have been run
    // resulting path is stored in _shortest_path array as vector offsets from EKF origin
    bool calc_shortest_path(const Location &origin, const Location &destination);

    // shortest path state variables
    bool _polyfence_with_margin_ok;
    bool _exclusion_polygon_with_margin_ok;
    bool _exclusion_circle_with_margin_ok;
    bool _polyfence_visgraph_ok;
    bool _shortest_path_ok;

    Location _destination_prev;     // destination of previous iterations (used to determine if path should be re-calculated)
    uint8_t _path_idx_returned;     // index into _path array which gives location vehicle should be currently moving towards

    // polygon fence (with margin) related variables
    float _polyfence_margin = 10;
    AP_ExpandingArray<Vector2f> _polyfence_pts;
    uint8_t _polyfence_numpoints;
    uint32_t _polyfence_update_ms;  // system time of boundary update from AC_Fence (used to detect changes to polygon fence)

    // exclusion polygon related variables
    AP_ExpandingArray<Vector2f> _exclusion_polygon_pts; // array of nodes corresponding to exclusion polygon points plus a margin
    uint8_t _exclusion_polygon_numpoints;   // number of points held in above array
    uint32_t _exclusion_polygon_update_ms;  // system time exclusion polygon was updated (used to detect changes)

    // exclusion circle related variables
    AP_ExpandingArray<Vector2f> _exclusion_circle_pts; // array of nodes surrounding exclusion circles plus a margin
    uint8_t _exclusion_circle_numpoints;    // number of points held in above array
    uint32_t _exclusion_circle_update_ms;   // system time exclusion circles were updated (used to detect changes)

    // visibility graphs
    AP_OAVisGraph _fence_visgraph;          // holds distances between all inclusion/exclusion fence points (with margin)
    AP_OAVisGraph _source_visgraph;         // holds distances from source point to all other nodes
    AP_OAVisGraph _destination_visgraph;    // holds distances from the destination to all other nodes

    // updates visibility graph for a given position which is an offset (in cm) from the ekf origin
    // to add an additional position (i.e. the destination) set add_extra_position = true and provide the position in the extra_position argument
    // requires create_polygon_fence_with_margin to have been run
    // returns true on success
    bool update_visgraph(AP_OAVisGraph& visgraph, const AP_OAVisGraph::OAItemID& oaid, const Vector2f &position, bool add_extra_position = false, Vector2f extra_position = Vector2f(0,0));

    typedef uint8_t node_index;         // indices into short path data
    struct ShortPathNode {
        AP_OAVisGraph::OAItemID id;     // unique id for node (combination of type and id number)
        bool visited;                   // true if all this node's neighbour's distances have been updated
        node_index distance_from_idx;   // index into _short_path_data from where distance was updated (or 255 if not set)
        float distance_cm;              // distance from source (number is tentative until this node is the current node and/or visited = true)
    };
    AP_ExpandingArray<ShortPathNode> _short_path_data;
    node_index _short_path_data_numpoints;  // number of elements in _short_path_data array

    // update total distance for all nodes visible from current node
    // curr_node_idx is an index into the _short_path_data array
    void update_visible_node_distances(node_index curr_node_idx);

    // find a node's index into _short_path_data array from it's id (i.e. id type and id number)
    // returns true if successful and node_idx is updated
    bool find_node_from_id(const AP_OAVisGraph::OAItemID &id, node_index &node_idx) const;

    // find index of node with lowest tentative distance (ignore visited nodes)
    // returns true if successful and node_idx argument is updated
    bool find_closest_node_idx(node_index &node_idx) const;

    // final path variables and functions
    AP_ExpandingArray<AP_OAVisGraph::OAItemID> _path;   // ids of points on return path in reverse order (i.e. destination is first element)
    uint8_t _path_numpoints;                            // number of points on return path
    Vector2f _path_source;                              // source point used in shortest path calculations (offset in cm from EKF origin)
    Vector2f _path_destination;                         // destination position used in shortest path calculations (offset in cm from EKF origin)

    // return point from final path as an offset (in cm) from the ekf origin
    bool get_shortest_path_point(uint8_t point_num, Vector2f& pos);
};
