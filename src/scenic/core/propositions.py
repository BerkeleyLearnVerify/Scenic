"""Objects representing propositions that can be used to specify conditions"""

from functools import reduce
import operator
import rv_ltl
from scenic.core.errors import InvalidScenarioError

from scenic.core.lazy_eval import needsLazyEvaluation

class PropositionMonitor:
	def __init__(self, proposition: "PropositionNode") -> None:
		self._proposition = proposition
		self._monitor = proposition.ltl_node.create_monitor()
		
	def update(self):
		atomic_propositions = self._proposition.atomics()
		state = {}
		for ap in atomic_propositions:
			b = ap.closure()
			if needsLazyEvaluation(b):
				raise InvalidScenarioError(f'value undefined outside of object definition')
			state[str(ap.syntax_id)] = b
		self._monitor.update(state)
		return self._monitor.evaluate()


class PropositionNode:
	"""Base class for temporal and non-temporal propositions"""
	def __init__(self, syntax_id, ltl_node) -> None:
		self.syntax_id = syntax_id
		self.ltl_node = ltl_node

		self.is_temporal = False
		"""tells if the proposition is temporal"""

	def check_constrains_sampling(self):
		"""checks if the proposition can be used for pruning

		A requirement can be used for pruning if it is evaluated on the scene generation phase before simulation, and
		violation in that phase immediately results in discarding the scene and regenerating a new one.

		For simplicity, we currently check two special cases:
		1. requirements with no temporal requirement
		2. requirements with only one `always` operator on top-level

		Returns:
			None|int: returns syntax id that can be used for pruning, None if proposition cannot be used for pruning
		"""
		node = self

		# if `always` is on top-level, check what's inside
		if isinstance(node, Always):
			node = node.req

		eligible = True
		for n in node.flatten():
			# turns false if any one of the node is temporal
			eligible = eligible and (not n.is_temporal)
		return node.syntax_id if eligible else None
	
	@property
	def children(self) -> list["PropositionNode"]:
		"""returns all children of proposition tree

		Returns:
			list: proposition nodes that are directly under this node
		"""
		return []

	def flatten(self) -> list["PropositionNode"]:
		"""flattens the tree and return the list of nodes

		Returns:
			list: list of all children nodes
		"""
		return [self] + reduce(
			operator.concat, [node.flatten() for node in self.children], []
		)
	
	def atomics(self) -> list["Atomic"]:
		return list(filter(lambda n: isinstance(n, Atomic), self.flatten()))

	def create_monitor(self) -> rv_ltl.Monitor:
		return PropositionMonitor(self)

	def evaluate(self):
		raise RuntimeError("This proposition contains temporal operators and can only be evaluated using monitor")

	@property
	def has_temporal_operator(self):
		node = self
		has_temporal_op = False
		for n in node.flatten():
			# turns false if any one of the node is temporal
			has_temporal_op = has_temporal_op or n.is_temporal
		return has_temporal_op

class Atomic(PropositionNode):
	def __init__(self, closure, syntax_id = None):
		ap = rv_ltl.Atomic(identifier=str(syntax_id))
		super().__init__(syntax_id, ap)
		self.closure = closure
	def __str__(self):
		return f"(AP)"
	def evaluate(self):
		return self.closure()

class UnaryProposition(PropositionNode):
	"""Base class for temporal unary operators"""
	@property
	def children(self):
		return [self.req]

class Always(UnaryProposition):
	def __init__(self, req: PropositionNode, syntax_id = None):
		ltl_node = rv_ltl.Always(req.ltl_node)
		super().__init__(syntax_id, ltl_node)
		self.req = req
		self.is_temporal = True
	def __str__(self):
		return f"(Always {str(self.req)})"

class Eventually(UnaryProposition):
	def __init__(self, req: PropositionNode, syntax_id = None):
		ltl_node = rv_ltl.Eventually(req.ltl_node)
		super().__init__(syntax_id, ltl_node)
		self.req = req
		self.is_temporal = True
	def __str__(self):
		return f"(Eventually {str(self.req)})"

class Next(UnaryProposition):
	def __init__(self, req: PropositionNode, syntax_id = None):
		ltl_node = rv_ltl.Next(req.ltl_node)
		super().__init__(syntax_id, ltl_node)
		self.req = req
		self.is_temporal = True
	def __str__(self):
		return f"(Next {str(self.req)})"

class Not(UnaryProposition):
	def __init__(self, req: PropositionNode, syntax_id = None):
		ltl_node = rv_ltl.Not(req.ltl_node)
		super().__init__(syntax_id, ltl_node)
		self.req = req
	def __str__(self):
		return f"(Not {str(self.req)})"
	def evaluate(self):
		return not self.req.evaluate()

class And(PropositionNode):
	def __init__(self, reqs, syntax_id):
		ltl_node = rv_ltl.And(reqs)
		super().__init__(syntax_id, ltl_node)
		self.reqs = reqs
	def __str__(self):
		return " and ".join([f"{str(req)}" for req in self.reqs])
	@property
	def children(self):
		return self.reqs
	def evaluate(self):
		return reduce(operator.and_, [node.evaluate() for node in self.reqs], True)

class Or(PropositionNode):
	def __init__(self, reqs, syntax_id):
		ltl_node = rv_ltl.Or(reqs)
		super().__init__(syntax_id, ltl_node)
		self.reqs = reqs
	def __str__(self):
		return " or ".join([f"{str(req)}" for req in self.reqs])
	@property
	def children(self):
		return self.reqs
	def evaluate(self):
		return reduce(operator.or_, [node.evaluate() for node in self.reqs], False)

class Until(PropositionNode):
	def __init__(self, lhs, rhs, syntax_id) -> None:
		self.lhs = lhs
		self.rhs = rhs
		ltl_node = rv_ltl.Until(lhs.ltl_node, rhs.ltl_node)
		super().__init__(syntax_id, ltl_node)
	def __str__(self):
		return f"({self.lhs} until {self.rhs})"
	@property
	def children(self):
		return [self.lhs, self.rhs]
