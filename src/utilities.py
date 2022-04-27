# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 

@author: pmayaduque
"""
import pandas as pd
import requests
import json 
import  ast

def read_data(data_path):
    
    try:
        data =  requests.get(data_path)
        data = json.loads(data.text)
    except:
        f = open(data_path)
        data = json.load(f)
    
    data_read = {}    
    data_read['Buques'] = set(data["Buques"])
    data_read['Nodos'] = set(data["Nodos"])
    data_read['Puertos'] = set(data["Puertos"])
    data_read['Arcos'] =set([tuple(i) for i in data["Arcos"]])
    data_read['Rutas'] = {int(k): v for k, v in data["Rutas"].items()}
    data_read['t_viaje'] = {tuple(ast.literal_eval(k)): v for k, v in data["t_viaje"].items()}
    data_read['puerto'] = {int(k): v for k, v in data["puerto"].items()}
    data_read['tiempo_programado'] = {int(k): v for k, v in data["tiempo_programado"].items()}
    data_read['tiempo_seguridad'] = {tuple(ast.literal_eval(k)): v for k, v in data["tiempo_seguridad"].items()}
    data_read['tiempodescarga'] = {int(k): v for k, v in data["tiempodescarga"].items()}
    data_read['tiempoliberacion'] = {int(k): v for k, v in data["tiempoliberacion"].items()} 
        
    return data_read