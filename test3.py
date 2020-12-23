from model_manager import ModelManager
from ortools.sat.python import cp_model

modele_manageur = ModelManager()

x = modele_manageur.model_or_tools.NewIntVar(0, 5, "x")
y = modele_manageur.model_or_tools.NewIntVar(0, 5, "y")


with modele_manageur.add_constraints_block("Bloc_1", assumption=True) as bloc:
    bloc.registrer_unit_equation(x + y == 5)

with modele_manageur.add_constraints_block("Bloc_2", assumption=True) as bloc:
    bloc.registrer_unit_equation(x + y == 11)


solver = cp_model.CpSolver()

status = solver.Solve(modele_manageur.model_or_tools)
r= solver.ResponseProto().sufficient_assumptions_for_infeasibility
if status == cp_model.INFEASIBLE:
    print("infeasible")  # logique

    list_bloc_problematique = modele_manageur.get_sufficient_assumptions_for_infeasibility(solver)
    for bloc_name in list_bloc_problematique:
        print(bloc)
