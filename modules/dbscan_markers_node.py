# -*- coding: utf-8 -*-
#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

import numpy as np
from std_msgs.msg import Float32MultiArray


from sklearn.cluster import KMeans,  DBSCAN
from sklearn.datasets import make_blobs
from sklearn.metrics import silhouette_score
import matplotlib.pyplot as plt
from matplotlib import cm

from sklearn.preprocessing import OneHotEncoder

# import random
import threading

import csv

import pandas as pd



# Define the function to run the DBScan algorithm.
def run_dbscan(positions):

    # Input data
    X = np.array(positions)

    # Criar o objeto DBSCAN
    # dbscan = DBSCAN(eps=1.15, min_samples=5)
    # dbscan = DBSCAN(eps=1.4, min_samples=12)
    # dbscan = DBSCAN(eps=1.2, min_samples=14)

    # According silhouette
    dbscan = DBSCAN(eps=1.16, min_samples=4)


    # Ajustar o modelo aos dados
    dbscan.fit(X)   

    # Obter rótulos de cluster atribuídos a cada ponto de dados
    labels = dbscan.labels_

    # Visualizar os resultados
    #print(labels)
    types = []
    for i in range(len(labels)):
        types.append(list(X[i, 3:]).index(max(list(X[i, 3:]))))

    #print(types)

    #plt.scatter(X[:, 0], X[:, 1], c=labels, cmap='jet')#'viridis', jet', 'coolwarm', 'inferno'
    # plt.scatter(X[:, 0], X[:, 1], c=types, cmap='jet')#'viridis', jet', 'coolwarm', 'inferno'
    # plt.title('DBSCAN Clustering')
    # plt.pause(0.1)  # Adiciona uma pausa curta para permitir a atualização da janela


    # Get cores (position) and standard desviation (size)
    cores = list()
    stds = list()

    # Calculando o desvio padrão dos pontos dentro de cada cluster
    for i in range(len(np.unique(labels))):
        cluster_points = X[np.where(labels == i)]  # seleciona os pontos pertencentes ao cluster i

        x_axis = [point[0] for point in cluster_points]
        y_axis = [point[1] for point in cluster_points]
        z_axis = [point[2] for point in cluster_points]

        core_x, core_y, core_z = np.mean(x_axis), np.mean(y_axis), np.mean(z_axis)
        std_dev_x, std_dev_y, std_dev_z = np.std(x_axis), np.std(y_axis), np.std(z_axis)  # calcula o desvio padrão ao longo do eixo 0 (por dimensão)


        class0_axis = [point[3] for point in cluster_points]
        class1_axis = [point[4] for point in cluster_points]
        class2_axis = [point[5] for point in cluster_points]
        class3_axis = [point[6] for point in cluster_points]
        class4_axis = [point[7] for point in cluster_points]
        class5_axis = [point[8] for point in cluster_points]

        class_types = [np.mean(class0_axis), np.mean(class1_axis), np.mean(class2_axis), np.mean(class3_axis), np.mean(class4_axis), np.mean(class5_axis)]
        class_type = class_types.index(max(class_types))

        cores.append([core_x, core_y, core_z, class_type])
        stds.append([std_dev_x, std_dev_y, std_dev_z])
        # print(cores)
    
    return cores, stds


# Define a fuction for publish the markers according the cores of the algorith descan
def publish_markers(cores, stds):

    marker_id = 0
    scale = 2.10


    # Create a Marker message
    marker_msg = Marker()
    marker_msg.header.frame_id = "odom"  # Frame de referência dos marcadores
    # marker_msg.header.stamp = rospy.Time.now()


    cont=[0,0,0,0,0,0]

    for i in range(0, len(cores)):

        marker_msg.action = Marker.ADD  # Ação a ser realizada (pode ser ADD, DELETE, etc.)

        # # Solve the orientation em Doctor degree
        marker_msg.type = Marker.CUBE#SPHERE  # Tipo de marcador (pode ser SPHERE, CYLINDER, etc.)
        # marker_msg.type = Marker.SPHERE  # Tipo de marcador (pode ser SPHERE, CYLINDER, etc.)
        # marker_msg.type = Marker.MESH_RESOURCE  # Type of marker (using a mesh file)

        # Specify the path to the STL file you want to use
        # marker_msg.mesh_resource = "/home/gilberto/Desktop/calco_armario_parafiso.STL"  # Replace with correct path


        marker_msg.scale.x = scale*np.array(stds)[i,0]  # Escala dos marcadores no eixo x
        marker_msg.scale.y = scale*np.array(stds)[i,1]  # Escala dos marcadores no eixo y
        marker_msg.scale.z = 1  # Escala dos marcadores no eixo z

        
        colors_map = {
            0: (255, 0, 0, 0.5),      # Vermelho
            1: (255, 165, 0, 0.5),    # Laranja
            2: (255, 255, 0, 0.5),    # Amarelo
            3: (0, 255, 0, 0.5),      # Verde
            4: (0, 0, 255, 0.5),      # Azul
            5: (75, 0, 130, 0.5)      # Índigo
            }    


        # Obtenha as cores correspondentes aos labels

        marker_msg.color.r = colors_map.get(cores[i][3])[0] # Componente vermelha da cor
        marker_msg.color.g = colors_map.get(cores[i][3])[1]   # Componente verde da cor
        marker_msg.color.b = colors_map.get(cores[i][3])[2]  # Componente azul da cor
        marker_msg.color.a = colors_map.get(cores[i][3])[3]   # Transparência dos marcadores


        marker_msg.pose.position.x = cores[i][0]
        marker_msg.pose.position.y = cores[i][1]
        marker_msg.pose.position.z = cores[i][2]
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 1.0
        marker_msg.pose.orientation.w = 0.0

        marker_msg.id = marker_id  # Assign a unique ID to marker
        marker_pub.publish(marker_msg)


        description_map = {
            0: "Disjuntor",      
            1: "Nível de Óleo",
            2: "Rampa de Acesso",    
            3: "Seccionador",  
            4: "Transformador",
            5: "Trincheira", 
            }    

        # Criar um marcador de texto
        text_marker = Marker()
        text_marker.header.frame_id = "odom"
        text_marker.action = Marker.ADD
        text_marker.type = Marker.TEXT_VIEW_FACING  # Tipo de marcador para texto

        # Definindo a posição do texto
        text_marker.pose.position = marker_msg.pose.position
        text_marker.pose.position.z += 1.0  # Levanta o texto um pouco acima do marcador

        # Definindo o texto com base no valor de cores[i][3]
        text_marker.text = description_map.get(cores[i][3]) + str(cont[cores[i][3]])  # Adiciona o valor como texto

        # Definindo as propriedades de cor e escala do texto
        text_marker.scale.z = 0.5  # Tamanho do texto
        text_marker.color.r = 0  # Cor vermelha
        text_marker.color.g = 0  # Cor verde
        text_marker.color.b = 0  # Cor azul
        text_marker.color.a = 1.0  # Opacidade do texto

        text_marker.id = marker_id + 1000  # Atribui um ID único diferente para o texto
        marker_pub.publish(text_marker)

        marker_id += 1  # Incrementa o ID do marcador
        cont[cores[i][3]] += 1 # Increment equipment type;


        xi = cores[i][0] - scale * np.array(stds)[i,0]/2
        yi = cores[i][1] - scale * np.array(stds)[i,1]/2
        xf = cores[i][0] + scale * np.array(stds)[i,0]/2
        yf = cores[i][1] + scale * np.array(stds)[i,1]/2
        evaluate_IoU(yi-5.4753, xi-0.87606, yf-5.4753, xf-0.87606, cores[i][3], text_marker.text)
        

# Function to evaluate the position using mAP for the position and size
def evaluate_IoU(xi, yi, xf, yf, classe, equip_name):
    
    # Carregar a planilha
    df = pd.read_excel("Positions.xlsx", sheet_name="Plan1")

    inter = 0
    union = 0

    for i in range(0, len(df)):

        if classe == df.iloc[i, 5]:

            xi_gt = df.iloc[i, 1]
            yi_gt = df.iloc[i, 2]
            xf_gt = df.iloc[i, 3]
            yf_gt = df.iloc[i, 4]

            
            # All cases are defined by
            if xi < xf_gt and xf > xi_gt and yi < yf_gt and yf > yi_gt:
                inter = inter + abs((min(xf, xf_gt) - max(xi, xi_gt)) * (min(yf, yf_gt) - max(yi, yi_gt)))
                union = union + (abs((xf - xi) * (yf - yi)) + abs((xf_gt - xi_gt) * (yf_gt - yi_gt)) - abs((min(xf, xf_gt) - max(xi, xi_gt)) * (min(yf, yf_gt) - max(yi, yi_gt))))


    if inter > 0:
        iou = inter/union
        print(f"{equip_name:<15} - IoU = {iou:.4f}")





# Define a function for the get bb positions
def make_bb_positions_list():
    

    # Set the confidence threshold
    conf_thresold = 0.7

    # Class values
    class_values = [0, 1, 2, 3, 4, 5]

    # Reshape the values into a 2D array
    class_values = np.array(class_values).reshape(-1, 1)

    # Initialize the OneHotEncoder
    encoder = OneHotEncoder(sparse=False)

    # Fit the encoder to the data
    encoder.fit(class_values)


    global positions_list

    while not rospy.is_shutdown():
        
        # Wait for message from YOLOv7 detections
        positions = rospy.wait_for_message("/positions", Float32MultiArray)#, timeout=5)
        # Deserialize data
        positions_tabled = [[float(positions.data[5*i+j]) for j in range(0, 5)] for i in range(0,int(len(positions.data)/5))]
        # Turn to array
        positions = np.array(object=positions)
        

        # Process Positions
        for i in range(0, len(positions_tabled)):
            if positions_tabled[i][3] > conf_thresold:
                                
                # Get a octo box
                position = positions_tabled[i]

                # Compute the center of octobox
                center_position = [position[0], position[1], position[2]]
                pos_x, pos_y, pos_z = center_position

                # Turn class a high scale
                classs = positions_tabled[i][4]

                # Convert the value to one-hot encoded
                classs_onehot = 1000*encoder.transform(np.array(classs).reshape(1, -1))

                positions_list.append([pos_x, pos_y, pos_z] + classs_onehot.tolist()[0])
        
                #print(positions_list)

        #print(positions_list)
        if len(np.unique(np.array([positions_list[i][3] for i in range(0, len(positions_list))]))) > 0:
            # run the dbscan algorith
            cores, stds = run_dbscan(positions_list)
            print('N Cores: {}'. format(len(cores)))
            publish_markers(cores, stds)


# # Function for run db scan algorith cotinuouss
# def dbscan_loop():
#     global positions_list
#     while not rospy.is_shutdown():
#         # print('N bb positions: {}'. format(len(positions_list)))
#         if len(np.unique(np.array([positions_list[i][3] for i in range(0, len(positions_list))]))) > 0:
#             # run the dbscan algorith
#             labels, cores, stds = run_dbscan(positions_list)
#             print('N Cores: {}'. format(len(cores)))

#             # publish one marker for each equipment                 
#             #publish_markers(labels, cores, stds)

#             # publish all detections for test
#             # publish_markers(labels, positions_list, stds)

#         rate.sleep()



# Define a function to save positions list in a file
def save_list_to_csv(data_list, filename):
    # Abre o arquivo CSV em modo de escrita
    with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        
        # Escreve os dados da lista no arquivo CSV
        for row in data_list:
            writer.writerow(row)

    print("The BBs positions list has been saved in %s file\n\n" % filename)


positions_list = list()

if __name__ == '__main__':
    
    print('dbscan_markers_node was initialized')

    try:
        rospy.init_node('markers_pub', anonymous=True)    
        marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
        

        # global marker_pub
        rate = rospy.Rate(10)  # Frequência de atualização em Hz
        

        # # publish markers according dbscan algorith
        # thread = threading.Thread(target=dbscan_loop)
        # # Set the thread as daemon True
        # thread.daemon = True
        # # Starting thread
        # thread.start()
        

        make_bb_positions_list()

        # # Carregar o arquivo CSV
        # positions_list = pd.read_csv('2025-04-05-21-50-52.csv')
        # X = positions_list.values
        # cores, stds = run_dbscan(X)
        # print('N Cores: {}'. format(len(cores)))
        # publish_markers(cores, stds)
        # # print(cores)

    except KeyboardInterrupt:

        # Exemplo de uso
        print('\n\nN bb positions: {}'. format(len(positions_list)))
        save_list_to_csv(positions_list, 'bb_positions_list.csv')

        
    except rospy.ROSInterruptException:
        pass

