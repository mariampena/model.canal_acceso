# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 17:17:52 2022

@author: pmayaduque
"""
import opt as opt
from utilities import read_data
import pandas as pd
import plotly.express as px
import plotly.io as pio
from plotly.offline import plot
pio.renderers.default='browser'

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
ventana_inicio = data['ventana_inicio']
ventana_fin = data['ventana_fin']
a = 24
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
                 ventana_inicio,
                 ventana_fin,
                 a,
                 M)

results, termination = opt.solve_model(model,
                optimizer='gurobi',
                mipgap=0.02,
                tee=False)

#print termination condition
print(termination)

if termination['Temination Condition'] == 'optimal': 
   model_results = opt.Results(model) 
   figure = model_results.create_graph(model)
   plot(figure)
   
