import argparse
import sqlite3
import pandas as pd
import numpy as np
from pathlib import Path
from collections import defaultdict

def analyze_experiments(timestamp):
    # 获取所有相关的数据库文件
    results = []
    fail_reasons = defaultdict(list)  # 用于收集失败原因
    
    # 模型和场景列表
    models = ['interfuser', 'pdm']
    scenarios = ['ramp', 'intersection', 'roundabout', 'straight', 'curve']
    
    for model in models:
        for scenario in scenarios:
            # 查找匹配的数据库文件
            pattern = f"{model}_{scenario}_{timestamp}_run*"
            db_files = list(Path('.').glob(pattern))
            
            metrics = []
            for db_file in db_files:
                conn = sqlite3.connect(db_file)
                # 从resultINFO表中读取评估结果
                df = pd.read_sql_query("""
                    SELECT total_score, complete_percentage, drive_score, 
                           use_time, fail_reason 
                    FROM resultINFO
                """, conn)
                metrics.append(df)
                
                # 收集失败原因
                if not pd.isna(df['fail_reason'].iloc[0]):
                    fail_reasons[f"{model}_{scenario}"].append(
                        f"Run {db_file}: {df['fail_reason'].iloc[0]}"
                    )
                
                conn.close()
            
            if metrics:
                # 计算平均值和标准差
                metrics_df = pd.concat(metrics)
                mean_metrics = metrics_df.mean()
                std_metrics = metrics_df.std()
                
                results.append({
                    'model': model,
                    'scenario': scenario,
                    'mean_total_score': mean_metrics['total_score'],
                    'std_total_score': std_metrics['total_score'],
                    'mean_complete_percentage': mean_metrics['complete_percentage'],
                    'std_complete_percentage': std_metrics['complete_percentage'], 
                    'mean_drive_score': mean_metrics['drive_score'],
                    'std_drive_score': std_metrics['drive_score'],
                    'mean_use_time': mean_metrics['use_time'],
                    'std_use_time': std_metrics['use_time']
                })
    
    # 保存结果
    output_file = f'experiment_results_{timestamp}.csv'
    results_df = pd.DataFrame(results)
    results_df.to_csv(output_file, index=False)
    print(f"结果已保存到 {output_file}")
    
    # 打印汇总结果
    print("\n实验结果汇总:")
    for result in results:
        model_scenario = f"{result['model']}_{result['scenario']}"
        print(f"\n{result['model'].upper()} - {result['scenario'].upper()}")
        print(f"总分: {result['mean_total_score']:.3f} (±{result['std_total_score']:.3f})")
        print(f"完成度: {result['mean_complete_percentage']:.3f} (±{result['std_complete_percentage']:.3f})")
        print(f"驾驶得分: {result['mean_drive_score']:.3f} (±{result['std_drive_score']:.3f})")
        print(f"用时(秒): {result['mean_use_time']:.3f} (±{result['std_use_time']:.3f})")
        
        # 打印该场景的失败原因
        if model_scenario in fail_reasons:
            print("\n失败原因:")
            for reason in fail_reasons[model_scenario]:
                print(f"  - {reason}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--timestamp', type=str, required=True,
                      help='用于识别实验运行的时间戳')
    args = parser.parse_args()
    
    analyze_experiments(args.timestamp) 