package brics;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import brics.types.Pose;

public class Planner {

	public class Edge {
		public final SubPlanner source;
		public final Pose goal;
		public final Pose start;

		public Edge(Pose start, Pose goal, SubPlanner source) {
			this.start = start;
			this.goal = goal;
			this.source = source;
		}
		
		@Override
		public String toString() {
			return start + " -> " + goal;
		}
	}

	private class Graph {
		private Map<Pose, List<Edge>> graph = new HashMap<Pose, List<Edge>>();
		Set<Pose> poses = new HashSet<Pose>();
		private List<Edge> initials = new ArrayList<Edge>();

		public void addEdge(Pose start, Edge edge) {
			if (!graph.containsKey(start))
				graph.put(start, new ArrayList<Planner.Edge>());
			
			
			if(start.toString().equals("Marker 4") && edge.goal.toString().equals("Marker 3"))
				System.out.println("wrong edge");
			graph.get(start).add(edge);
			poses.add(start);
			poses.add(edge.goal);
		}

		public void addInitial(Edge edge) {
			initials.add(edge);
			poses.add(edge.goal);
		}

		public Set<Pose> getPoses() {
			return poses;
		}

		public List<Edge> search(Pose start, Pose goal) {
			Map<Pose, Edge> ancestors = new HashMap<Pose, Edge>();

			List<Edge> edges = new ArrayList<Edge>();
			if (start == null)
				edges.addAll(initials);
			else
				edges.addAll(graph.get(start));

			while (edges.size() > 0) {
				Edge edge = edges.remove(0);
				Pose edgeGoal = edge.goal;
				if (ancestors.containsKey(edgeGoal))
					continue;
				ancestors.put(edgeGoal, edge);
				if (graph.containsKey(edgeGoal))
					edges.addAll(graph.get(edgeGoal));
				if (edgeGoal.equals(goal))
					break;
			}

			List<Edge> ret = new ArrayList<Edge>();
			Pose current = goal;
			while (ancestors.containsKey(current)) {
				Edge ancestor = ancestors.get(current);
				ret.add(0, ancestor);
				current = ancestor.start;
				if (current == start)
					return ret;
			}
			return null;
		}
	}

	private List<SubPlanner> planners = new ArrayList<SubPlanner>();

	public void addPlanner(SubPlanner planner) {
		planners.add(planner);
	}

	public boolean executePath(List<Edge> path) {
		if(path == null) return false;
		for (Edge edge : path) {
			if(!edge.source.moveTo(edge.goal)) return false;
		}
		return true;
	}

	public List<Edge> findPath(Pose goal) {
		Graph graph = new Graph();
		for (SubPlanner planner : planners) {
			Map<Pose, Double> reachablePoses = planner.getReachablePoses(null);
			for (Entry<Pose, Double> entry : reachablePoses.entrySet()) {
				graph.addInitial(new Edge(null, entry.getKey(), planner));
			}
		}

		for (SubPlanner planner : planners) {
			Map<Pose, Double> reachingPoses = planner.getReachingPoses(goal);
			for (Entry<Pose, Double> entry : reachingPoses.entrySet()) {
				graph.addEdge(entry.getKey(), new Edge(entry.getKey(), goal,
						planner));
			}
		}

		Set<Pose> completedPoses = new HashSet<Pose>();
		while (graph.getPoses().size() > completedPoses.size()) {
			for (Pose pose : new ArrayList<Pose>(graph.getPoses())) {
				if (completedPoses.contains(pose))
					continue;

				for (SubPlanner planner : planners) {
					Map<Pose, Double> reachablePoses = planner
							.getReachablePoses(pose);
					for (Entry<Pose, Double> entry : reachablePoses.entrySet()) {
						graph.addEdge(pose, new Edge(pose, entry.getKey(),
								planner));
					}
				}
				completedPoses.add(pose);
			}
		}

		List<Edge> result = graph.search(null, goal);
		return result;
	}

}
