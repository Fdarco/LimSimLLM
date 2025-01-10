import os
import re
import pandas as pd
import numpy as np
from pathlib import Path

def parse_log_file(log_path):
    """解析单个日志文件，提取关键指标"""
    try:
        with open(log_path, 'r') as f:
            content = f.read()
            
        # 提取关键信息
        result = {
            'success': 'success' in content.lower(),
            'final_score': float(re.search(r'final score is (\d+\.?\d*)', content, re.I).group(1)),
            'driving_mile': float(re.search(r'driving mile is (\d+\.?\d*)', content).group(1)),
            'complete_percentage': float(re.search(r'complete percentage is (\d+\.?\d*)', content).group(1)),
            'driving_time': float(re.search(r'driving time is (\d+\.?\d*)', content).group(1)),
            'decision_time': float(re.search(r'decision time is (\d+\.?\d*)', content).group(1)),
            'comfort_score': float(re.search(r'Comfort Score: (\d+\.?\d*)', content).group(1)),
            'efficiency_score': float(re.search(r'Efficiency Score: (\d+\.?\d*)', content).group(1)),
            'safety_score': float(re.search(r'Safety Score: (\d+\.?\d*)', content).group(1)),
            'red_light_penalty': float(re.search(r'Red Light Penalty: (\d+\.?\d*)', content).group(1)),
            'speed_limit_penalty': float(re.search(r'Speed Limit Penalty: (\d+\.?\d*)', content).group(1)),
            'fail_penalty': float(re.search(r'Fail Penalty: (\d+\.?\d*)', content).group(1))
        }
        return result
    except Exception as e:
        print(f"Error processing {log_path}: {str(e)}")
        return None

def is_valid_exp_dir(dir_name):
    """检查目录名是否符合实验命名规则"""
    pattern = r'^(interfuser|VLMagentCloseLoop|PDM|Limsimtm)_(straight|curve|roundabout|intersection|ramp|long_term)_\d{8}_\d{6}_gpu\d+_exp\d+$'
    return bool(re.match(pattern, dir_name))

def format_mean_std(values):
    """格式化均值和标准差"""
    mean = np.mean(values)
    std = np.std(values)
    return f"{mean:.3f}±{std:.3f}"

def process_results(results_dir='results'):
    """处理所有实验结果"""
    results = []
    
    # 遍历results目录
    for exp_dir in os.listdir(results_dir):
        if not is_valid_exp_dir(exp_dir):
            continue
            
        exp_path = os.path.join(results_dir, exp_dir)
        if not os.path.isdir(exp_path):
            continue
            
        log_path = os.path.join(exp_path, 'eval_result.log')
        if not os.path.exists(log_path):
            continue
            
        # 解析实验目录名称
        parts = exp_dir.split('_')
        model_name = parts[0]
        scenario = parts[1]
        timestamp = '_'.join(parts[2:4])
        gpu_exp = '_'.join(parts[4:])
            
        # 解析日志文件
        log_results = parse_log_file(log_path)
        if log_results:
            log_results.update({
                'model': model_name,
                'scenario': scenario,
                'timestamp': timestamp,
                'gpu_exp': gpu_exp,
                'exp_dir': exp_dir
            })
            results.append(log_results)
    
    # 转换为DataFrame
    df = pd.DataFrame(results)
    
    # 重新排列列顺序
    columns = [
        'model', 'scenario', 'timestamp', 'gpu_exp', 'success', 'final_score',
        'driving_mile', 'complete_percentage', 'driving_time', 'decision_time',
        'comfort_score', 'efficiency_score', 'safety_score',
        'red_light_penalty', 'speed_limit_penalty', 'fail_penalty',
        'exp_dir'
    ]
    df = df[columns]
    
    return df

def calculate_statistics(df):
    """计算统计信息"""
    # 按模型和场景分组计算统计信息
    metrics = ['final_score', 'driving_mile', 'complete_percentage', 'driving_time', 
              'decision_time', 'comfort_score', 'efficiency_score', 'safety_score']
    
    stats = {}
    for metric in metrics:
        stats[metric] = df.groupby(['model', 'scenario'])[metric].agg(
            lambda x: format_mean_std(x)
        ).unstack()
    
    # 计算成功率
    success_rate = df.groupby(['model', 'scenario'])['success'].agg(
        lambda x: format_mean_std(x.astype(float))
    ).unstack()
    
    return {'success_rate': success_rate, **stats}

if __name__ == '__main__':
    # 处理结果
    df = process_results()
    
    # 保存原始数据到CSV文件
    output_file = 'experiment_results.csv'
    df.to_csv(output_file, index=False)
    print(f"Raw results have been saved to {output_file}")
    
    # 计算并打印统计信息
    stats = calculate_statistics(df)
    
    print("\n=== Statistical Results ===")
    print(f"\nTotal experiments: {len(df)}")
    
    print("\nSuccess Rate by Model and Scenario:")
    print(stats['success_rate'])
    
    print("\nFinal Score by Model and Scenario:")
    print(stats['final_score'])
    
    print("\nDriving Time by Model and Scenario:")
    print(stats['driving_time'])
    
    print("\nDecision Time by Model and Scenario:")
    print(stats['decision_time'])
    
    # 保存统计结果到Excel文件
    with pd.ExcelWriter('experiment_statistics.xlsx') as writer:
        for metric, data in stats.items():
            data.to_excel(writer, sheet_name=metric[:31])  # Excel sheet名称最大31字符
    print("\nDetailed statistics have been saved to experiment_statistics.xlsx") 