import multiprocessing
from time import perf_counter
import traceback

from ortools.sat.python import cp_model


class OptimisationWithTracking:
    """
    Context manager

    Prends en entrée :
    - solver : solveur OR-TOOLS
    - model : modèle OR-TOOLS
    - dict_variables : dictionnaire nom_variable -> variable OR-TOOLS utilisé pour récupérer
    les solutions intermédiaires


    Lance le solveur sur un second processus
    et renvoie une queue via laquelle le solveur enverra progressivement les messages suivants

    Lors que l'optimisation est terminée :
        En cas d'échec :  {"state": "end", "message": "no_solution"}

        En cas de résolution : {"state": "end", "message": "solution_found"}

    Pendant l'optimisation, pour chaque solution trouvée:
        {
            "state": "processing",
            "message": "intermediate_solution",
            "solution": dict[index_variable -> valeur_variable],
            "objective_value" : int
        }
    """

    def __init__(self, solver: cp_model.CpSolver, model: cp_model, dict_variables: dict):

        self.output_queue = multiprocessing.Queue()
        self.solving_process = ParallelSolver(solver, model, dict_variables, self.output_queue)

    def __enter__(self):
        self.solving_process.start()
        return self.output_queue

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.solving_process.terminate()


class Tracker(cp_model.CpSolverSolutionCallback):
    def __init__(self, dict_variables: dict, output_queue: multiprocessing.Queue):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.dict_variables = dict_variables
        self.initial_time = None
        self.output_queue = output_queue

    def init_timer(self):
        self.initial_time = perf_counter()

    def on_solution_callback(self):
        try:
            new_solution = {name: self.Value(variable) for name, variable in self.dict_variables.items()}

            self.output_queue.put(
                {
                    "state": "processing",
                    "message": "intermediate_solution",
                    "time": perf_counter() - self.initial_time,
                    "solution": new_solution,
                    "objective_value": self.ObjectiveValue(),
                }
            )

        except Exception as exception:
            traceback.print_exc()
            raise exception


class ParallelSolver(multiprocessing.Process):
    def __init__(
        self,
        solver: cp_model.CpSolver,
        model: cp_model,
        dict_variables: dict,
        output_queue: multiprocessing.Queue,
    ):
        multiprocessing.Process.__init__(self)
        self.solver = solver
        self.model = model
        self.dict_variables = dict_variables
        self.output_queue = output_queue
        self.tracker = Tracker(self.dict_variables, self.output_queue)

    def run(self):
        self.tracker.init_timer()
        status = self.solver.SolveWithSolutionCallback(self.model, self.tracker)
        if status == cp_model.INFEASIBLE:
            self.output_queue.put({"state": "end", "message": "no_solution"})

        else:
            self.output_queue.put({"state": "end", "message": "solution_found"})
