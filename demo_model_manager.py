from model_manager import ModelManager
from ortools.sat.python import cp_model

# FROM DOCUMENTATION OF OR-TOOLS
class VarArraySolutionPrinter(cp_model.CpSolverSolutionCallback):
    """Print intermediate solutions."""

    def __init__(self, variables):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.__variables = variables
        self.__solution_count = 0

    def on_solution_callback(self):
        self.__solution_count += 1
        for v in self.__variables:
            print('%s=%i' % (v, self.Value(v)), end=' ')
        print()

    def solution_count(self):
        return self.__solution_count

modele_manageur = ModelManager()

x = modele_manageur.model_or_tools.NewIntVar(0,100, "x")
y = modele_manageur.model_or_tools.NewIntVar(0,100, "y")

solution_printer = VarArraySolutionPrinter([x, y])

with modele_manageur.add_constraints_block("Bloc_x", relaxable=False) as bloc:
    bloc.registrer_unit_equation(x >= 2)
    bloc.registrer_unit_equation(x < 10)

with modele_manageur.add_constraints_block("Bloc_y", relaxable=False) as bloc:
    bloc.registrer_unit_equation(y >= 5)
    bloc.registrer_unit_equation(y < 15)

with modele_manageur.add_constraints_block("Bloc_x_et_y", relaxable=False) as bloc:
    bloc.registrer_unit_equation(x + y == 5)

solver = cp_model.CpSolver()
status = solver.SearchForAllSolutions(modele_manageur.model_or_tools, solution_printer)
if status == cp_model.INFEASIBLE:
    print("infeasible")  # logique

modele_manageur.add_assumption_to_blocks(["Bloc_x", "Bloc_y", "Bloc_x_et_y"])
status = solver.Solve(modele_manageur.model_or_tools)
if status == cp_model.INFEASIBLE:
    print("infeasible")  # toujours logique mais maintenant on peut avoir des infos
    list_bloc_problematique = modele_manageur.get_sufficient_assumptions_for_infeasibility(solver)
    for bloc_name in list_bloc_problematique:
        print(bloc)

# modele_manageur.remove_constraint_block("Contrainte sur x")
# resolution(modele_manageur.model_or_tools, solution_printer)