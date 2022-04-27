# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 17:17:52 2022

@author: pmayaduque
"""
import opt as opt
from utilities import read_data

# data paths
# data_filepath = r"https://raw.githubusercontent.com/mariampena/model_canal_acceso/main/data/data.json"
data_filepath = "../data/data.json"

# read data from file
data = read_data(data_filepath)

Buques = data["Buques"]
Nodos = data["Nodos"]
Puertos = data["Puertos"]
Arcos = data["Arcos"]
Rutas = data["Rutas"]
t_viaje = data["t_viaje"]
puerto = data["puerto"]
tiempo_programado = data["tiempo_programado"]
tiempo_seguridad = data["tiempo_seguridad"]
tiempodescarga =  data["tiempodescarga"]
tiempoliberacion =  data["tiempoliberacion"] 

M=1000 #Valor muy grande que despues calcularemos bien con base en los datos del problema

model=opt.create_model(Buques, 
                 Nodos, 
                 Puertos, 
                 Arcos,
                 Rutas,
                 t_viaje,
                 puerto,
                 tiempo_programado,
                 tiempo_seguridad,
                 tiempodescarga,
                 tiempoliberacion,
                 M)

results, termination = opt.solve_model(model,
                optimizer='gurobi',
                mipgap=0.02,
                tee=False)

#print termination condition
print(termination)

x_values = opt.get_results(model)
#print results
for key, value in x_values.items():
  if value != None and value > 0:
    print(key, value)
    
# get values used
x_used= {(key[0], key[1], key[2]) : int(value)  for key, value in x_values.items() if  value != None }
# get maximum time (makespan)
max_value = max(x_used.values())+t_viaje[(1,2)]+1
# get solution for each ship
sol_dict= dict()
for i in Buques:
    arcs_dict = {(key[0], key[1]) : value  for key, value in x_used.items() if (key[2]==i and value != None) }
    sol_dict[i] = sorted(arcs_dict.items(), key=lambda x: x[1])

# get space-temporal dictionary
space_time = {k : [0]*max_value for k in Buques}
for key, value in sol_dict.items():
    for i in range(len(value) -1):
        for j in range(value[i][1], value[i+1][1]):
            if j <= value[i][1] + t_viaje[value[i][0]]:
                space_time[key][j]= value[i][0][0] + (j - value[i][1])*(value[i][0][1]-value[i][0][0])/t_viaje[value[i][0]]
            else:
                space_time[key][j]= value[i][0][1]
    for j in range(value[len(value) -1][1], value[len(value) -1][1] + t_viaje[value[len(value) -1][0]] +1):
        space_time[key][j]= value[len(value) -1][0][0] - (j - value[len(value) -1][1]) *(1/t_viaje[value[len(value) -1][0]])   