from Replay import ReplayModel
from evaluate import Decision_Evaluator
if __name__ == "__main__":
    # database = '/data/limsim-o/LimSimLLM/results/pdm_1/pdm_1.db'
    # database = '/data/limsim-o/LimSimLLM/results/interfuser_1/interfuser_1.db'
    database='/data/limsim-o/LimSimLLM/results/pdm_2/pdm_2.db'
    
    
    model = ReplayModel(database)
    evaluator = Decision_Evaluator(database, model.timeStep,'pdm2_eval_result.log')
    
    while not model.tpEnd:
        model.moveStep()
        evaluator.Evaluate(model)
    