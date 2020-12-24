from typing import *

from ortools.sat.python import cp_model
from ortools.sat.python.cp_model import BoundedLinearExpression


class ConstraintsBlock:
    def __init__(
        self, model_manager, name: str, relaxable: bool = False, relaxation_weight: int = None, assumption=False
    ):
        """
        La classe a été conçue pour être initiée via ModelManager.add_constraints_block
        et comprends les attributs suivants.

        L'utilisateur n'a pas à toucher / utiliser aux méthodes de cette classe qui sert juste au Modele Manager

        - model_manager : référence vers le modèle manager
        - name : nom du bloc de contraintes afin de pouvoir le retrouver/supprimer/relaxer par la suite
        - relaxable : si le bloc peut être relaxé
        - relaxation_weight : dans le cas où le bloc peut être relaxé, son cout de relaxation sera égale au nombre de
        contraintes unitaires composant non respecté * relaxation weight

        - begin_index, end_index : l'ensemble des contraintes unitaires seront encodés de manière contingue dans le
        modèle OR-TOOLS, begin_index = l'index de la première contrainte, end_index = l'index de la dernière contrainte

        - equations, rev_equations : liste des équations (et de leur inverse), une par contrainte unitaire composant
        le bloc. Les contraintes rev_equations ne doivent être fournis que si on souhaite relaxer la contrainte

        Si les contraintes ne sont pas relaxables -> model.Add(equation)
        Si les contraintes sont relaxables, les contraintes rev_equations servent à la double réification
            Pour chaque équation i on définit b[i] tq
             model.Add(equation[i]).OnlyEnforceIf(b[i]), model.Add(equation_inv[i]).OnlyEnforceIf(b[i].not)
            et par la suite Minimize(sum(b[i]) for i ...)

        - objectif_value = sum(b)
        - assumption, si True : liera une unique variable assumption_bool à l'ensemble des variables.
            utile pour analyser un modèle qui n'a pas de solution
        """
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
        self.begin_index = len(self.model_manager.model_or_tools.Proto().constraints)
        return self

    def registrer_unit_equation(self, equation: BoundedLinearExpression, rev_equation: BoundedLinearExpression = None):
        """
        Enregistre une equation unitaire qui sera ajouté au modèle
        Si on est dans le cas non relaxable, on ajoute directement l'equation positive.
        Dans le cas relaxable, l'équation inverse sera utilisée pour encoder la double réification
        """
        self.equations.append(equation)

        if self.relaxable:
            self.rev_equations.append(rev_equation)

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        En sortie du contexte manager, on ajoute au modèle l'ensemble des contraintes unitaires
        """
        # Cas typique : on encode une contrainte en dur dans le modèle
        if not self.relaxable and not self.assumption:
            for equation in self.equations:
                self.model_manager.model_or_tools.Add(equation)

        # Cas où le modèle n'a pas de solution et on veut étudier la raison
        # Dans ce cas, l'intégralité du bloc est reliée à une variable booléenne qui est passé dans le
        # paramètre assumption du Proto
        if not self.relaxable and self.assumption:
            self.assumption_bool = self.model_manager.model_or_tools.NewBoolVar("assumption_" + self.name)
            for equation in self.equations:
                self.model_manager.model_or_tools.Add(equation).OnlyEnforceIf(self.assumption_bool)
            self.model_manager.model_or_tools.Proto().assumptions.extend([self.assumption_bool.Index()])

        # Cas où on ne veut pas encoder les contraintes en dur
        # Dans ce cas, on crée une variable bool par contrainte unitaire, et on cherchera à minimiser la somme
        # de ces variables
        else:
            objective_value = self.relaxation_value()
            self.model_manager.store_objective(self.name, objective_value, self.relaxation_weight)

        self.end_index = len(self.model_manager.model_or_tools.Proto().constraints) - 1

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

    def encode_with_assumption(self):
        """
        Utilisé pour relier à une variable booléenne unique des contraintes déjà encodées dans le modèle
        Renvoie la variable booléenne associée
        """
        # Si les contraintes unitaires relatives au bloc sont déjà encodées en dur dans le modèle, il faut les enlever
        if self.begin_index:
            self.remove_constraints_from_model()

        self.begin_index = self.model_manager.current_index()

        self.assumption_bool = self.model_manager.model_or_tools.NewBoolVar("assumption_" + self.name)
        for equation in self.equations:
            self.model_manager.model_or_tools.Add(equation).OnlyEnforceIf(self.assumption_bool)
        self.model_manager.model_or_tools.Proto().assumptions.extend([self.assumption_bool.Index()])
        self.end_index = len(self.model_manager.model_or_tools.Proto().constraints) - 1

    def shift_index(self, shift, threshold):
        """
        Lorsqu'un autre bloc de contraintes est supprimé ailleurs dans le modèle, si il est situé avant le bloc courant
        les index du bloc courant vont être modifiés dans le modèle OR-TOOLS et il est nécessaire de les mettre à jour
        """
        if self.begin_index is None:
            return  # Aucune contrainte n'a été encodée dans le modèle

        if self.begin_index <= threshold:
            return  # Non concerné car le bloc courant se situe avant le bloc supprimé dans le modèle

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
        threshold_index = self.end_index

        self.begin_index = None
        self.end_index = None

        # On propage l'information à tous les blocs "frères"
        for block in self.model_manager.block_list():
            block.shift_index(shift, threshold_index)

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

    Si le modèle renvoie infaillible, le modele manager permet de facilement relaxer un sous-ensemble de contraintes
    model_manager.relaxe_constraint_block("respect_position") permet de relaxer l'ensemble des positions

    """

    def __init__(self):
        self.model_or_tools = cp_model.CpModel()
        self._constraints_block_dict = dict()
        self._objective_values = dict()
        self._objective_weights = dict()
        self._assumption_dict = dict()

    def block_list(self):
        return self._constraints_block_dict.values()

    def add_constraints_block(self, name, relaxable: bool = False, relaxation_weight=None, assumption=False):
        """
        Renvoie le context manager permettant d'indexer toutes les contraintes unitaires qui vont
        être regroupés dans un seul bloc
        """
        new_bloc = ConstraintsBlock(self, name, relaxable, relaxation_weight, assumption=assumption)
        self._constraints_block_dict[new_bloc.name] = new_bloc

        if assumption:
            # WARNING : dans solver.ResponseProto().sufficient_assumptions_for_infeasibility, les variables booléennes
            # "assumptions" sont représentées par un index qui correspondant à leur ordre de création dans le modèle
            # parmi toutes les variables de type "assumption".
            new_index = len(self._assumption_dict)
            self._assumption_dict[new_index] = new_bloc.name

        return new_bloc

    def remove_hard_constraints_from_block(self, block_name):
        """
        Détruit l'ensemble des contraintes unitaires composant un bloc dans le modèle OR TOOLS
        lorsqu'elles ont été encodé dans le modèle
        """
        self._constraints_block_dict[block_name].remove_constraints_from_model()

    def relaxe_constraint_block(self, block_name: str):
        """
        Supprime un bloc de contraintes du modèle et rajoute dans les objectifs le fait de minimiser
        le nombre de contraintes relaxées
        """
        block = self._constraints_block_dict[block_name]
        block.remove_constraints_from_model()
        self.store_objective(block.name, block.relaxation_value(), block.relaxation_weight)

    def add_assumption_to_blocks(self, block_name_list: List[str]):
        for block_name in block_name_list:
            self.add_assumption_to_block(block_name)

    def add_assumption_to_block(self, block_name: str):
        block = self._constraints_block_dict[block_name]
        block.remove_constraints_from_model()
        block.encode_with_assumption()

        new_index = len(self._assumption_dict)
        self._assumption_dict[new_index] = block.name

    def get_sufficient_assumptions_for_infeasibility(self, solveur):
        sufficient_assumptions = solveur.ResponseProto().sufficient_assumptions_for_infeasibility
        # TODO : chercher d'où vient ce décalage déduit par les expériences ...
        shift = 2 if len(sufficient_assumptions) > 1 else 0
        return [self._assumption_dict[idx - shift] for idx in sufficient_assumptions]

    def store_objective(self, name, value, weight):
        self._objective_values[name] = value
        self._objective_weights[name] = weight

    def add_global_weighted_objective_to_model(self):
        global_weighted_objective = sum(
            self._objective_values[name] * self._objective_weights[name] for name in self._objective_values.keys()
        )
        self.model_or_tools.Minimize(global_weighted_objective)
