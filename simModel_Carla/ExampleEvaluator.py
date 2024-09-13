# This script is used for offline evaluation of decisions and trajectories 

from simInfo.Evaluation import Decision_Evaluation
from simModel.Replay import ReplayModel

if __name__ == "__main__":
    database = './results/2024-09-09_14-55-32.db'
    model = ReplayModel(database)
    evaluator = Decision_Evaluation(database, model.timeStep)
    while not model.tpEnd:
        model.runStep()
        if model.timeStep % 10 ==0:
            evaluator.Evaluate(model)