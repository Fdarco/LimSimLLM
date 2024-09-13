from Replay import ReplayModel
from evaluate import Decision_Evaluator
if __name__ == "__main__":
    database = './results/2024-09-11_15-21-48.db'
    model = ReplayModel(database)
    evaluator = Decision_Evaluator(database, model.timeStep)
    
    while not model.tpEnd:
        model.moveStep()
        evaluator.Evaluate(model)
    