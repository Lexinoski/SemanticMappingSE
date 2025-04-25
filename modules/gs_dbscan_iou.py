import numpy as np
from sklearn.cluster import DBSCAN
import pandas as pd
from itertools import product

# Sua função existente
def evaluate_IoU(xi, yi, xf, yf, classe, equip_name):
    df = pd.read_excel("Positions.xlsx", sheet_name="Plan1")
    inter = 0
    union = 0
    for i in range(0, len(df)):
        if classe == df.iloc[i, 5]:
            xi_gt = df.iloc[i, 1]
            yi_gt = df.iloc[i, 2]
            xf_gt = df.iloc[i, 3]
            yf_gt = df.iloc[i, 4]
            if xi < xf_gt and xf > xi_gt and yi < yf_gt and yf > yi_gt:
                inter += abs((min(xf, xf_gt) - max(xi, xi_gt)) * (min(yf, yf_gt) - max(yi, yi_gt)))
                union += (abs((xf - xi) * (yf - yi)) + abs((xf_gt - xi_gt) * (yf_gt - yi_gt)) - abs((min(xf, xf_gt) - max(xi, xi_gt)) * (min(yf, yf_gt) - max(yi, yi_gt))))
    
       
    if inter > 0:
        iou = inter/union
        # print(f"{equip_name:<15} - IoU = {iou:.4f}")
        return iou 
    return 0

# Função modificada de DBSCAN com parâmetros variáveis
def run_dbscan(positions, eps, min_samples):
    
    # Input data
    X = np.array(positions)

    # Create dbscan object with variable variable
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)

    # Fit the model to data
    dbscan.fit(X)

    # Get cluster lables to each point
    labels = dbscan.labels_


    types = [list(X[i, 3:]).index(max(list(X[i, 3:]))) for i in range(len(labels))]
    cores = []
    stds = []

    # Computing the stad deviation
    for i in range(len(np.unique(labels))):
        if np.unique(labels)[i] == -1: continue  # ignora ruído
        
        cluster_points = X[np.where(labels == i)]
        x_axis = [p[0] for p in cluster_points]
        y_axis = [p[1] for p in cluster_points]
        z_axis = [p[2] for p in cluster_points]
        
        class_axes = [ [p[j] for p in cluster_points] for j in range(3, 9) ]
        class_means = [np.mean(ax) for ax in class_axes]
        
        cores.append([np.mean(x_axis), np.mean(y_axis), np.mean(z_axis), class_means.index(max(class_means))])
        stds.append([np.std(x_axis), np.std(y_axis), np.std(z_axis)])
    return cores, stds

# Carregar o arquivo CSV
from_file = pd.read_csv('2025-04-05-21-50-52.csv')
# Extrair as características relevantes do DataFrame (ajuste de acordo com a sua necessidade)
positions = from_file.values

# Parâmetros a testar (total 300000)
eps_values = [x/100 for x in range(80, 180)] # 100 values
min_samples_values = [x for x in range(2, 22)] # 20 values
scale_factors = [x/100 for x in range(50, 350, 2)]  # 150 values

# Parâmetros a testar
eps_values = [x/100 for x in range(90, 140)] # 50 values
min_samples_values = [x for x in range(4, 16)] # 12 values
scale_factors = [x/100 for x in range(150, 300, 2)]  # 75' values

# Parâmetros a testar 
eps_values = [1.29] # pretest
min_samples_values = [4]
scale_factors = [2.1]

# Grid Search
best_iou = 0
best_params = ()

best_iou_acum = 0 
best_params_acum = ()

results = []

ind = 0

for eps, min_samples in product(eps_values, min_samples_values):
    cores, stds = run_dbscan(positions, eps, min_samples)

    for scale in scale_factors:
        total_iou = 0
        count = 0
        print(f'N Cores: {len(cores)}')
        for i in range(len(cores)):
            core_x, core_y, _, classe = cores[i]
            scale_x = scale * stds[i][0]
            scale_y = scale * stds[i][1]

            xi = core_x - scale_x/2 - 0.87606
            xf = core_x + scale_x/2 - 0.87606
            yi = core_y - scale_y/2 - 5.4753
            yf = core_y + scale_y/2 - 5.4753

            iou = evaluate_IoU(yi, xi, yf, xf, classe, f"cluster_{i}")
            total_iou += iou
            if iou > 0:
                count += 1
        avg_iou = total_iou / count if count > 0 else 0
        results.append((eps, min_samples, scale, avg_iou))
        print(f"{ind} - Params: eps={eps}, min_samples={min_samples}, scale={scale} => mIoU: {avg_iou:.4f} and total IoU: {total_iou :.4f}\n")
        if avg_iou > best_iou:
            best_iou = avg_iou
            best_params = (eps, min_samples, scale)
        if total_iou > best_iou_acum:
            best_iou_acum = total_iou
            best_params_acum = (eps, min_samples, scale)
        ind += 1

print("\n🔍 Melhor combinação according mean IoU:")
print(f"  - eps: {best_params[0]}")
print(f"  - min_samples: {best_params[1]}")
print(f"  - scale_factor: {best_params[2]}")
print(f"  - Melhor mIoU: {best_iou:.4f}")

print("\n🔍 Melhor combinação according sum total IoU:")
print(f"  - eps: {best_params_acum[0]}")
print(f"  - min_samples: {best_params_acum[1]}")
print(f"  - scale_factor: {best_params_acum[2]}")
print(f"  - Melhor total IoU: {best_iou_acum:.4f}")