# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 17:17:59 2022

@author: pmayaduque
"""

from pyomo.environ import *
from pyomo.opt import *
from pyomo.core.base import initializer
# Crear el modelo - abstracto/concreto

def create_model(Buques, 
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
                 M):
    
    model = ConcreteModel(name="AsignacionPrioridad")
    
    # Declaración de conjuntos
    model.Buques = Set(ordered = False, initialize=Buques)                          #Conjunto de Buques
    model.Nodos = Set(ordered = False, initialize=Nodos)                            #Conjunto de Nodos
    model.Puertos = Set(ordered = False, initialize=Puertos)                        #Conjunto de Puerts
    model.Arcos = Set(ordered = False, initialize=Arcos)                            #Conjunto de Arcos
    
    # Declaración parámetros
    model.Rutas = Param(model.Buques, initialize=Rutas, within=Any)
    model.t_viaje = Param(model.Arcos, initialize=t_viaje, within=PositiveReals)    #Tiempo de tránsito en el tramo arco 
    model.t_prog = Param(model.Buques, initialize=tiempo_programado)                #Tiempo programado de llegada de los buques
    model.t_seg = Param(model.Arcos, initialize=tiempo_seguridad)                   #Tiempo de serguridad entre buques en el arco
    model.t_desc = Param(model.Buques, initialize=tiempodescarga)                   #Tiempo de descarga del buque
    model.t_lib = Param(model.Buques, initialize=tiempoliberacion)                  #Tiempo de liberación de los buques (cuando está disponible para transitar)
    model.puerto = Param(model.Buques, initialize=puerto) 
    
    # Declarión variables de decisión}
    model.x = Var(model.Nodos, model.Nodos, model.Buques, domain=NonNegativeReals)  # tiempo de inicio del transito en el arco (i,j)
                                                                                    # TODO: Para mejorar la eficiencia, crear solo las variables de los arcos que existan
    model.wkr = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque k atravieza antes que el buque r el arco (i,j)
    model.wrk = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque r atravieza antes que el buque k el arco (i,j)
    model.ykr = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque k inicia el arco (i,j) antes que el buque r el arco (j,i)
    model.yrk = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque r inicia el arco (i,j) antes que el buque k el arco (j,i)
    model.z = Var(model.Arcos, model.Buques, domain=NonNegativeReals)                           # TODO Para mejorar la eficiencia, crear solo las variables de los arcos que existan
    
    # Función objetivo
    def obj_rule(model):
        return sum(model.z[a[0], a[1], k] for a in model.Arcos for k in model.Buques)
    model.objetivo = Objective(sense=minimize, rule=obj_rule)
    
    # Función que determina si un nodo j es el puerto del buque k
    def funPuerto(k, j):
      es_puerto = 0
      if model.puerto[k]==j:
        es_puerto = 1
      return es_puerto
    
    # Restricción de linealización
    def linealizacion(model,k,i,j):
      return model.z[i,j,k] >= funPuerto(k,j)*(model.x[i,j,k] - model.t_prog[k])
      model.z[a[0], a[1], k] >= (model.x[a[0], a[1], k] - model.t_prog[k])#*funPuerto(k, a[1])
    model.linealizacion = Constraint(model.Buques, model.Arcos, rule=linealizacion)
    #model.linealizacion.pprint()
    
    # Restricción de liberación
    def liberacion(model,k):
        return model.x[model.Rutas[k][0], model.Rutas[k][1], k] >= model.t_lib[k]
    model.libe = Constraint(model.Buques,rule=liberacion)
    #model.libe.pprint()
    
    # Restricción de continuidad de la ruta del buque k
    len_max = max([len(x) for x in Rutas.values()]) # obtiene la longitud máxima de las rutas
    pos_ruta = set(range(len_max)) # Crea conjunto de las posciones de la ruta
    def continuidad(model,k,pos):
      if pos<len(model.Rutas[k])-2:    
        return model.x[model.Rutas[k][pos+1], model.Rutas[k][pos+2], k] >= model.x[model.Rutas[k][pos], model.Rutas[k][pos+1], k] + model.t_viaje[(model.Rutas[k][pos], model.Rutas[k][pos+1])]
      else:
        return Constraint.Skip
    #model.continuidad = Constraint(model.Buques, pos_ruta, rule=continuidad)
    
    # caso especial de cuando se arriba al puerto (hay que sumarle el tiempo de descarga)
    #función que retorna la posición del puerto en la ruta del buque k  
    def pos_puerto(buque, id_puerto):
      return Rutas[buque].index(id_puerto)
    def continuidad_p(model,k): 
      pos = pos_puerto(k, model.puerto[k])
      return model.x[model.Rutas[k][pos], model.Rutas[k][pos+1], k] >= model.x[model.Rutas[k][pos-1], model.Rutas[k][pos], k] + model.t_viaje[(model.Rutas[k][pos-1], model.Rutas[k][pos])] + model.t_desc[k]
    model.continuidad_p = Constraint(model.Buques, rule=continuidad_p)
    model.continuidad_p.pprint() 
    
    # Restricción no superposición del flujo 
    def superposicion(model,i,j,k,r):
      if (i,j) in model.Arcos and k!=r:
        return model.x[i,j,k]-model.x[i,j,r]>=model.t_seg[i,j]-M*model.wkr[i,j,k,r]
      else:
        return Constraint.Skip
    model.sup = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superposicion)
    
    # Restricción no superposición del flujo2
    def superposicion2(model,i,j,k,r):
      if (i,j) in model.Arcos and k!=r:
        return model.x[i,j,r]-model.x[i,j,k]>=model.t_seg[i,j]-M*(1-model.wkr[i,j,k,r])
      else:
        return Constraint.Skip
    model.sup2 = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superposicion2)
    
    # Restriccion no simultaneidad
    def simultaneidad(model,i,j,k,r):
      return model.wkr[i,j,k,r]+model.wrk[i,j,r,k]<=1
    model.simultaneidad = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=simultaneidad)
    
    
    #Restriccion superposicion en sentidos opuestos
    def superpopuestos(model,i,j,k,r):
     if (i,j) in model.Arcos and k!=r:
        return model.x[i,j,k]-model.x[j,i,r]>=model.t_viaje[j,i]-M*model.ykr[i,j,k,r]
     else:
        return Constraint.Skip
    #model.supop = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superpopuestos)
    
    
    
    #Segunda restriccion superposicion en sentidos opuestos
    def superpopuestos2(model,i,j,k,r):
     if (i,j) in model.Arcos and k!=r:
        return model.x[j,i,r]-model.x[i,j,k]>=model.t_viaje[i,j]-M*model.ykr[i,j,r,k]
     else:
        return Constraint.Skip
    #model.supop2 = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superpopuestos2)
    
    return model


def solve_model(model,
                optimizer='gurobi',
                mipgap=0.02,
                tee=True):
    solver = pyo.SolverFactory(optimizer)
    solver.options['MIPGap'] = mipgap
    results = solver.solve(model, tee = tee)