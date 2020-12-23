from ortools.sat.python import cp_model

model_or_tools = cp_model.CpModel()

x = model_or_tools.NewIntVar(0, 5, "x")
y = model_or_tools.NewIntVar(0, 5, "y")


b_1 = model_or_tools.NewBoolVar("b_1")
b_2 = model_or_tools.NewBoolVar("b_2")


model_or_tools.Add(x + y == 5).OnlyEnforceIf(b_1)
model_or_tools.Proto().assumptions.extend([b_1.Index()])

model_or_tools.Add(x + y == 11).OnlyEnforceIf(b_2)
model_or_tools.Proto().assumptions.extend([b_2.Index()])

solver = cp_model.CpSolver()
status = solver.Solve(model_or_tools)
if status == cp_model.INFEASIBLE:
    print("infeasible")  # logique

list_problematique = solver.ResponseProto().sufficient_assumptions_for_infeasibility
for index in list_problematique:
    print(index)
