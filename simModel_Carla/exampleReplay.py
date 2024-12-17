import sys
import os
import argparse
from Replay import ReplayModel
from evaluate import Decision_Evaluator

def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='Replay simulation from database')
    
    parser.add_argument('--database', '-d',
                        type=str,
                        required=True,
                        help='Path to database file')
    
    return parser.parse_args()

def main():
    # 解析命令行参数
    args = parse_args()
    
    # 如果输入的是目录名,构造完整的数据库文件路径
    if not os.path.isabs(args.database):
        db_name = os.path.basename(args.database)
        db_path = os.path.join('results', db_name, f'{db_name}.db')
    else:
        db_path = args.database

    # 检查数据库文件是否存在
    if not os.path.exists(db_path):
        print(f"错误: 数据库文件 {db_path} 不存在")
        sys.exit(1)
    
    # 更新数据库路径
    args.database = db_path
    eval_output = os.path.join(os.path.dirname(db_path), 'eval_result.log')
    try:
        # 初始化回放模型和评估器
        model = ReplayModel(args.database)
        evaluator = Decision_Evaluator(args.database, model.timeStep, eval_output)
        
        # 主循环
        while not model.tpEnd:
            model.moveStep()
            # 评估结果
            evaluator.Evaluate(model)
            
    except KeyboardInterrupt:
        print("\n用户中断执行")
    except Exception as e:
        print(f"执行过程中出现错误: {e}")
        raise

if __name__ == '__main__':
    main()
