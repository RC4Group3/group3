#!/usr/bin/env python

import roslib; roslib.load_manifest('graph_planner')
import brics_msgs
import rospy



# actual map class ...
class mapGraph():
    def __init__(self):
        self.num_edges = 0
        self.num_nodes = 0
        pass

    # need to calculate shortest path from input point to goal 
    # first step will be adding start/goal nodes and edges to the graph
    # (can be in any node format...?)
    def calc_path(self):
        pass

    # load static graph from file
    def load_graph(self):
        pass


# node for graph ... *almost* all will be associated with a marker and a map location
# marker nodes need a markerID
# map nodes need a frame and a location w/in that frame
class graphNode():
    def __init__(self, node_id):
        self.node_id = node_id
        
class mapNode():
    pass

class markerNode():
    pass


# we have a number of different edge types, but they all need to have 
# start & end nodes, as well as a cost and an ID 
# NB - these edges are DIRECTED
# hmm ... may not actually need an ID after all ... if the graph is just a vector of nodes/edges, their index can be the ID
class graphEdge():
    def __init__(self, orig_node, dest_node, cost, edge_id):
        self.orig_node = orig_node
        self.dest_node = dest_node
        self.cost = cost
        self.edge_id = edge_id

# don't need any extra data in order to naviate on a mapEdge
class mapEdge(graphEdge):
    def __init__(self, orig_node, dest_node, cost, edge_id):
        pass

# this type of edge needs a path of markers between the two endpoints
class markerEdge(graphEdge):
    def __init__(self, orig_node, dest_node, cost, marker_ids):
        self.marker_ids = marker_ids



# huh. seems like nodes should be typed, and for a single node that's a location tagged to a tf location as well as a tag, add a transition node
