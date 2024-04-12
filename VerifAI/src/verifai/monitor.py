from abc import ABC
import networkx as nx
import mtl

class specification_monitor(ABC):
    def __init__(self, specification):
        self.specification = specification

    def evaluate(self, traj):
        return self.specification(traj)


class mtl_specification(specification_monitor):
    def __init__(self, specification):
        mtl_specs = [mtl.parse(spec) for spec in specification]
        mtl_spec = mtl_specs[0]
        if len(mtl_specs) > 1:
            for spec in mtl_specs[1:]:
                mtl_spec = (mtl_spec & spec)
        super().__init__(mtl_spec)

    def evaluate(self, traj):
        return self.specification(traj)

class multi_objective_monitor(specification_monitor):
    def __init__(self, specification, priority_graph=None, linearize=False):
        super().__init__(specification)
        self.linearize = linearize
        if priority_graph is None:
            self.graph = nx.DiGraph()
            self.graph.add_nodes_from(range(self.num_objectives))
        else:
            self.graph = priority_graph
        if linearize: # "linearize" the prioritize graph by topologically sorting and connecting nodes
            self._linearize()
    
    def _linearize(self):
        new_graph = nx.DiGraph()
        S = set([node for node, degree in self.graph.in_degree() if degree == 0])
        nodes = []
        while len(S) > 0:
            n = random.choice(S)
            S.remove(n)
            nodes.append(n)
            neighbors = self.graph.neighbors(n)
            random.shuffle(neighbors)
            for m in neighbors:
                self.graph.remove_edge(n, m)
                if self.graph.in_degree(m) == 0:
                    S.add(m)
        new_graph.add_nodes_from(nodes)
        for i in range(1, len(nodes)):
            new_graph.add_edge(nodes[i - 1], nodes[i])
        self.graph = new_graph