from typing import *

from ortools.sat.python import cp_model
from ortools.sat.python.cp_model import BoundedLinearExpression


class ConstraintsBlock:
    def __init__(
        self, model_manager, name: str, relaxable: bool = False, relaxation_weight: int = None, assumption=False
    ):
        self.model_manager = model_manager
        self.name = name
        self.relaxable = relaxable
        self.relaxation_weight = relaxation_weight

        self.begin_index = None
        self.end_index = None
        self.equations = []
        self.rev_equations = []

        self.objectif_value = None
        self.assumption = assumption
        self.assumption_bool = None

    def __enter__(self):
        self.begin_index = self.model_manager.current_index()
        return self

    def encode_with_assumption(self):
        self.begin_index = self.model_manager.current_index()

        self.assumption_bool = self.model_manager.model_or_tools.NewBoolVar("assumption_" + self.name)
        for equation in self.equations:
            self.model_manager.model_or_tools.Add(equation).OnlyEnforceIf(self.assumption_bool)

        self.end_index = self.model_manager.current_index() - 1

        return self.assumption_bool

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.relaxable and self.assumption:
            self.assumption_bool = self.model_manager.model_or_tools.NewBoolVar("assumption_" + self.name)
            for equation in self.equations:
                self.model_manager.model_or_tools.Add(equation).OnlyEnforceIf(self.assumption_bool)
            self.model_manager.model_or_tools.Proto().assumptions.extend([self.assumption_bool.Index()])

        if not self.relaxable and not self.assumption:
            for equation in self.equations:
                self.model_manager.model_or_tools.Add(equation)

        else:
            objective_value = self.relaxation_value()
            self.model_manager.store_objective(self.name, objective_value, self.relaxation_weight)

        self.end_index = self.model_manager.current_index() - 1

    def relaxation_value(self):
        """
        Lorsque l'on relaxe un bloc de contraintes, on calcule le nombre de fois que les contraintes unitaires
        associées à ce bloc ne sont pas respectées.

        Pour cela on définit pour chaque contrainte unitair une variable booléenne irrespect(c) tq :
            irrespect(c) = True <=> la contrainte c n'est pas respecté

        Le model_manager se chargera par la suite de minimiser la somme de ces variables
        """
        fail_list = []

        for index, (equation, rev_equation) in enumerate(zip(self.equations, self.rev_equations)):
            fail = self.model_manager.model_or_tools.NewBoolVar(self.name + "_" + str(index))
            self.model_manager.model_or_tools.Add(equation).OnlyEnforceIf(fail.Not())
            self.model_manager.model_or_tools.Add(rev_equation).OnlyEnforceIf(fail)
            fail_list.append(fail)

        return sum(fail_list)

    def registrer_unit_equation(self, equation: BoundedLinearExpression, rev_equation: BoundedLinearExpression = None):
        self.equations.append(equation)

        if self.relaxable:
            self.rev_equations.append(rev_equation)

    def shift_index(self, shift):
        self.begin_index -= shift
        self.end_index -= shift

    def remove_constraints_from_model(self):
        """
        Pour supprimer un bloc de contraintes, on récupère les index de début, fin des contraintes unitaires
        dans le modèle OR-TOOLS et on les supprime via le Proto()
        """
        for index in range(self.begin_index, self.end_index + 1):
            self.model_manager.model_or_tools.Proto().constraints[index].Clear()

        shift = self.end_index - self.begin_index + 1

        self.begin_index = None
        self.end_index = None

        return shift

    def __str__(self):
        return self.name


class ModelManager:
    """
    Wrapper d'un modèle OR TOOLS permettant de facilement supprimer / relaxer des contraintes

    Exemple d'utilisation :
    with model_manager.add_constraints_block("respect_position", relaxable=False) as bloc:
        for i,j,k in ... :
            bloc.registrer_unit_equation(x[i,j,k] = ...)

    (...)

    si le modèle renvoie infaillible, le modele manager permet de facilement relaxer un sous-ensemble de contraintes
    model_manager.relaxe_constraint_block("respect_position") permet de relaxer l'ensemble des positions

    """

    def __init__(self):
        self.model_or_tools = cp_model.CpModel()
        self._constraints_block_dict = dict()
        self._objective_values = dict()
        self._objective_weights = dict()
        self.assumption_dict = dict()

    def add_constraints_block(self, name, relaxable: bool = False, relaxation_weight=None, assumption=False):
        """
        Renvoie le context manager permettant d'indexer toutes les contraintes unitaires qui vont être associées
        par bloc

        """
        new_bloc = ConstraintsBlock(self, name, relaxable, relaxation_weight, assumption=assumption)
        self._constraints_block_dict[new_bloc.name] = new_bloc
        return new_bloc

    def current_index(self):
        return len(self.model_or_tools.Proto().constraints)

    def remove_constraint_block(self, block_name):
        """
        Pour supprimer un bloc de contraintes,
        1/ on détruit l'ensemble des contraintes unitaires associées dans le modèle OR TOOLS
        2/ on remet à jour l'ensemble des index
        """
        target_block = self._constraints_block_dict[block_name]
        threshold_index = target_block.end_index
        shift = target_block.remove_constraints_from_model()

        for block in self._constraints_block_dict.values():
            if block.begin_index and block.begin_index > threshold_index:
                block.shift_index(shift)

    def relaxe_constraint_block(self, block_name: str):
        """
        Supprime un bloc de contraintes du modèle et rajoute dans les objectifs le fait de minimiser
        le nombre de contraintes relaxées
        """
        self.remove_constraint_block(block_name)
        block = self._constraints_block_dict[block_name]
        self.store_objective(block.name, block.relaxation_value(), block.relaxation_weight)

    def add_assumption_to_blocks(self, block_name_list: List[str]):
        for block_name in block_name_list:
            self.add_assumption_to_block(block_name)

    def add_assumption_to_block(self, block_name: str):
        self.remove_constraint_block(block_name)
        target_block = self._constraints_block_dict[block_name]
        assumption_var = target_block.encode_with_assumption()
        self.assumption_dict[assumption_var.Index()] = block_name
        self.model_or_tools.Proto().assumptions.append(assumption_var.Index())

    def get_sufficient_assumptions_for_infeasibility(self, solveur):
        index_var_list = solveur.ResponseProto().sufficient_assumptions_for_infeasibility
        return [
            block.name
            for block in self._constraints_block_dict.values()
            if block.assumption and block.assumption_bool.Index() in index_var_list
        ]

    def store_objective(self, name, value, weight):
        self._objective_values[name] = value
        self._objective_weights[name] = weight

    def add_global_weighted_objective_to_model(self):
        global_weighted_objective = sum(
            self._objective_values[name] * self._objective_weights[name] for name in self._objective_values.keys()
        )
        self.model_or_tools.Minimize(global_weighted_objective)
