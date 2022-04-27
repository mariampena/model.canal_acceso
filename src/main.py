# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 17:17:52 2022

@author: pmayaduque
"""
from opt import create_model, solve_model

Buques={1,2,3,4,5}
Nodos={1, 2, 3, 4, 5, 6, 7}
Puertos={5, 7}
Arcos={(1,2), (2,3), (3,4), (4,5), (3,6), (6,7), (7,6), (6,3), (5,4), (4,3), (3,2), (2,1)}

Rutas={1:[1, 2, 3, 4, 5, 4, 3, 2, 1],
    2:[1, 2, 3, 4, 5, 4, 3, 2, 1],
    3:[1, 2, 3, 4, 5, 4, 3, 2, 1],
    4:[1, 2, 3, 6, 7, 6, 3, 2, 1],
    5:[1, 2, 3, 6, 7, 6, 3, 2, 1],}

t_viaje={(1,2) : 10, 
   (2,3) : 30,
   (3,4) : 13,
   (4,5) : 30,
   (3,6) : 15,
   (6,7) : 20, 
   (7,6) : 20, 
   (6,3) : 15,
   (5,4) : 30, 
   (4,3) : 13, 
   (3,2) : 30, 
   (2,1) : 10}

puerto={1:5, # numero del nodo donde esta ubicado el puerto de cada buque
        2:5,
        3:5,
        4:7,
        5:7}


tiempo_programado={1:16,
                   2:20,
                   3:42,
                   4:63,
                   5:85}

tiempo_seguridad={(1,2) : 2, 
   (2,3) : 2,
   (3,4) : 2,
   (4,5) : 2,
   (3,6) : 2,
   (6,7) : 2, 
   (7,6) : 2, 
   (6,3) : 2,
   (5,4) : 2, 
   (4,3) : 2, 
   (3,2) : 2, 
   (2,1) : 2}
  
tiempodescarga={1:30,
                2:19,
                3:19,
                4:19,
                5:50}


tiempoliberacion={1:10,
                 2:10,
                 3:43,
                 4:60,
                 5:80}



M=1000 #Valor muy grande que despues calcularemos bien con base en los datos del problema

model=create_model(Buques, 
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

results, termination =solve_model(model,
                optimizer='gurobi',
                mipgap=0.02,
                tee=False)

print(termination)
