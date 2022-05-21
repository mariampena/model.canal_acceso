# -*- coding: utf-8 -*-
"""
Created on Tue Apr 26 17:17:59 2022

@author: pmayaduque
"""

from pyomo.environ import *
from pyomo.opt import *
from pyomo.core import value
import pandas as pd
import time
import plotly.express as px
import plotly.io as pio
from plotly.offline import plot
pio.renderers.default='browser'

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
                 ventana_inicio,
                 ventana_fin,
                 a, # número de unidades de tiempo en un día
                 M):
    
    # Crear el modelo - abstracto/concreto
    model = ConcreteModel(name="AsignacionPrioridad")
    
    # Declaración de conjuntos
    model.Buques = Set(ordered = False, initialize=Buques)                          #Conjunto de Buques
    model.Nodos = Set(ordered = False, initialize=Nodos)                            #Conjunto de Nodos
    model.Puertos = Set(ordered = False, initialize=Puertos)                        #Conjunto de Puertos
    model.Arcos = Set(ordered = False, initialize=Arcos)                            #Conjunto de Arcos
    
    # Declaración parámetros
    model.Rutas = Param(model.Buques, initialize=Rutas, within=Any)
    model.t_viaje = Param(model.Arcos, initialize=t_viaje, within=PositiveReals)    #Tiempo de tránsito en el tramo arco 
    model.t_prog = Param(model.Buques, initialize=tiempo_programado)                #Tiempo programado de llegada de los buques
    model.t_seg = Param(model.Arcos, initialize=tiempo_seguridad)                   #Tiempo de serguridad entre buques en el arco
    model.t_desc = Param(model.Buques, initialize=tiempodescarga)                   #Tiempo de descarga del buque
    model.t_lib = Param(model.Buques, initialize=tiempoliberacion)                  #Tiempo de liberación de los buques (cuando está disponible para transitar)
    model.puerto = Param(model.Buques, initialize=puerto) 
    # TODO mejorar las llaves del diccionario
    def v_initialization(model, i, j, k):
        return ventana_inicio[(i, (j, k))]
    model.v_inicio = Param(model.Buques, model.Arcos, initialize=v_initialization) 
    def v_initialization(model, i, j, k):
        return ventana_fin[(i, (j, k))]
    model.v_fin = Param(model.Buques, model.Arcos, initialize=v_initialization) 
    model.a = Param(initialize = a)
    
    # Declarión variables de decisión}
    # TODO: Para mejorar la eficiencia, crear solo las variables de los arcos que existan
    model.x = Var(model.Nodos, model.Nodos, model.Buques, domain=NonNegativeReals)  # tiempo de inicio del transito en el arco (i,j)
    model.y = Var(model.Nodos, model.Nodos, model.Buques, domain=NonNegativeReals, bounds=(0,model.a-0.01)) # Modulo de la variable x sobre el número de periodos a
    model.k = Var(model.Nodos, model.Nodos, model.Buques, domain=Integers)
    model.wkr = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque k atravieza antes que el buque r el arco (i,j)
    model.wrk = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque r atravieza antes que el buque k el arco (i,j)
    model.ykr = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque k inicia el arco (i,j) antes que el buque r el arco (j,i)
    model.yrk = Var(model.Nodos, model.Nodos, model.Buques, model.Buques, domain=Binary)        # 1 si el buque r inicia el arco (i,j) antes que el buque k el arco (j,i)
    model.z = Var(model.Arcos, model.Buques, domain=NonNegativeReals)                           # TODO Para mejorar la eficiencia, crear solo las variables de los arcos que existan
    
    # Función objetivo
    def obj_rule(model):
        return sum(model.z[a[0], a[1], k] for a in model.Arcos for k in model.Buques )
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
    model.linealizacion = Constraint(model.Buques, model.Arcos, rule=linealizacion)
    
    # Restricción de liberación
    def liberacion(model,k):
        return model.x[model.Rutas[k][0], model.Rutas[k][1], k] >= model.t_lib[k]
    model.libe = Constraint(model.Buques,rule=liberacion)
    
    #función que retorna la posición del puerto en la ruta del buque k  
    def pos_puerto(buque, id_puerto):
      return Rutas[buque].index(id_puerto)
      
    # Restricción de continuidad de la ruta del buque k
    len_max = max([len(x) for x in Rutas.values()]) # obtiene la longitud máxima de las rutas
    pos_ruta = set(range(len_max))
    def continuidad(model,k,pos):
      if pos==pos_puerto(k, model.puerto[k]):
        return Constraint.Skip
      elif pos==0:
        return model.x[model.Rutas[k][pos], model.Rutas[k][pos+1], k] >= model.t_lib[k] # Esta es redundante con liberación
      elif pos<len(model.Rutas[k])-1:    
        return model.x[model.Rutas[k][pos], model.Rutas[k][pos+1], k] == model.x[model.Rutas[k][pos-1], model.Rutas[k][pos], k] + model.t_viaje[(model.Rutas[k][pos-1], model.Rutas[k][pos])]
      else:
        return Constraint.Skip#model.continuidad = Constraint(model.Buques, pos_ruta, rule=continuidad)
    model.continuidad = Constraint(model.Buques, pos_ruta, rule=continuidad)
    
    
    # caso especial de cuando se arriba al puerto (hay que sumarle el tiempo de descarga)
    def continuidad_p(model,k): 
      pos = pos_puerto(k, model.puerto[k])
      return model.x[model.Rutas[k][pos], model.Rutas[k][pos+1], k] == model.x[model.Rutas[k][pos-1], model.Rutas[k][pos], k] + model.t_viaje[(model.Rutas[k][pos-1], model.Rutas[k][pos])] + model.t_desc[k]
    model.continuidad_p = Constraint(model.Buques, rule=continuidad_p)
      
    # Función que determina si un arco esta en la ruta del buque
    def arcoEnRuta(buque, arco):
      arcos_buque = [(Rutas[buque][i],Rutas[buque][i+1]) for i in range(len(Rutas[buque])-1)]
      if arco in arcos_buque:
        return True
      else: 
        return False
    
    # Restricción no superposición del flujo 
    def superposicion(model,i,j,k,r):
      if (i,j) in model.Arcos and k!=r and arcoEnRuta(k, (i,j)) and arcoEnRuta(r, (i,j)):
        return model.x[i,j,k]-model.x[i,j,r]>=model.t_seg[i,j]-M*model.wkr[i,j,k,r]
      else:
        return Constraint.Skip
    model.sup = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superposicion)
    
    # Restricción no superposición del flujo2
    def superposicion2(model,i,j,k,r):
      if (i,j) in model.Arcos and k!=r and arcoEnRuta(k, (i,j)) and arcoEnRuta(r, (i,j)):
        return model.x[i,j,r]-model.x[i,j,k]>=model.t_seg[i,j]-M*(1-model.wkr[i,j,k,r])
      else:
        return Constraint.Skip
    model.sup2 = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superposicion2)
    
    
    #Restriccion superposicion en muelles
    def supmuelle(model,i,j,k,r):
      if j in model.Puertos and (i,j) in model.Arcos and k!=r and arcoEnRuta(k, (i,j)) and arcoEnRuta(r, (i,j)):
        #return model.x[i,j,r]+model.t_viaje[i,j]+model.t_desc[r]>=model.x[i,j,k]-M*(1-model.wkr[i,j,k,r])
        return model.x[i,j,r]+model.t_viaje[i,j]>=model.x[j,i,k]-M*(1-model.wkr[i,j,k,r])
      else:
        return Constraint.Skip
    model.supmuelle = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=supmuelle)
    
    
    #Restriccion superposicion en muelles
    def supmuelle2(model,i,j,k,r):
      if j in model.Puertos and (i,j) in model.Arcos and k!=r and arcoEnRuta(k, (i,j)) and arcoEnRuta(r, (i,j)):
        return model.x[i,j,k]+model.t_viaje[i,j]+model.t_desc[k]>=model.x[i,j,r]-M*(model.wkr[i,j,k,r])
        #return model.x[i,j,k]+model.t_viaje[i,j]>=model.x[j,i,r]-M*(model.wkr[i,j,k,r])
      else:
        return Constraint.Skip
    model.supmuelle2 = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=supmuelle2)
    
    
    #Restriccion superposicion en sentidos opuestos
    def superpopuestos(model,i,j,k,r):
     #if (i,j) in model.Arcos and k!=r:
     if (i,j) in model.Arcos and i > j and k!=r and arcoEnRuta(k, (i,j)) and arcoEnRuta(r, (j,i)):
        return model.x[i,j,k]-model.x[j,i,r]>=model.t_viaje[j,i]-M*(1 - model.ykr[i,j,k,r])
     else:
        return Constraint.Skip
    model.supop = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superpopuestos)
    
    #Segunda restriccion superposicion en sentidos opuestos
    def superpopuestos2(model,i,j,k,r):
     #if (i,j) in model.Arcos and k!=r:
     if (i,j) in model.Arcos and i > j and k!=r and arcoEnRuta(k, (i,j)) and arcoEnRuta(r, (j,i)):
        #return model.x[j,i,r]-model.x[i,j,k]>=model.t_viaje[i,j]-M*model.ykr[i,j,r,k]
        return model.x[j,i,r]-model.x[i,j,k]>=model.t_viaje[i,j]-M*model.ykr[i,j,k,r]
     else:
        return Constraint.Skip
    model.supop2 = Constraint(model.Nodos, model.Nodos,model.Buques,model.Buques, rule=superpopuestos2)
   
    
    # Mareas, ventanas de tiempo
    def ventanaInicio(model, k, i, j):
        if (i,j) in model.Arcos and arcoEnRuta(k, (i,j)):
            return model.y[i, j, k] >= model.v_inicio[k, i, j]
        else:
            return Constraint.Skip
    model.v_inicio1 = Constraint(model.Buques, model.Arcos,  rule=ventanaInicio)
    
    def ventanaFin(model, k, i, j):
        if (i,j) in model.Arcos and arcoEnRuta(k, (i,j)):
            return model.y[i, j, k] <= model.v_fin[k, i, j]
        else:
            return Constraint.Skip
    #model.v_fin1 = Constraint(model.Buques, model.Arcos,  rule=ventanaFin)
    
    # modulo tiempo de inicio
    def modulo(model, k, i, j):
        if (i,j) in model.Arcos and arcoEnRuta(k, (i,j)):
            return model.x[i,j,k] == a*model.k[i,j,k] + model.y[i,j,k]
        else:
            return Constraint.Skip        
    model.modulo = Constraint(model.Buques, model.Arcos,  rule=modulo)    
        
    return model


def solve_model(model,
                optimizer='gurobi',
                mipgap=0.02,
                tee=True):
    solver = SolverFactory(optimizer)
    solver.options['MIPGap'] = mipgap
    timea = time.time()
    results = solver.solve(model, tee = tee)
    term_cond = results.solver.termination_condition
    term = {}
    if term_cond != TerminationCondition.optimal:
          term['Temination Condition'] = format(term_cond)
          execution_time = time.time() - timea
          term['Execution time'] = execution_time
          raise RuntimeError("Optimization failed.")

    else: 
          term['Temination Condition'] = format(term_cond)
          execution_time = time.time() - timea
          term['Execution time'] = execution_time    
    return results, term


class Results():
    def __init__(self, model):
        
        # general descriptives of the solution
        self.descriptive = {}
        self.descriptive['OF'] = model.objetivo.expr()
        x_values = model.x.get_values()
        # save only the arcs that are being used
        self.descriptive['x_values'] = {(key[0], key[1], key[2]) : int(value)  for key, value in x_values.items() if  value != None }
        # port arrival and lateness for each boat 
        lateness = dict()
        port_arrival = dict()
        programmed = dict()
        for k in model.Buques:
            pos_port = model.Rutas[k].index(model.puerto[k])
            arrival = value(model.x[model.Rutas[k][pos_port-1],model.puerto[k],k])
            port_arrival['ship'+ str(k)] = arrival
            programmed['ship'+ str(k)] = model.t_prog[k]
            late = arrival - model.t_prog[k]
            lateness['ship'+ str(k)] = late*(late>0) # gets value 0 if it is negative
        self.descriptive['programmed'] = programmed
        self.descriptive['port_arrival'] = port_arrival
        self.descriptive['lateness'] = lateness
    
    def create_graph2(self, model):
        x_values = model.x.get_values()
            
        # get values used
        x_used= {(key[0], key[1], key[2]) : int(value)  for key, value in x_values.items() if  value != None }
        # get maximum time (makespan)
        max_value = max(x_used.values())+model.t_viaje[(1,2)]+5
        # get solution for each ship
        sol_dict= dict()
        for i in model.Buques:
            arcs_dict = {(key[0], key[1]) : value  for key, value in x_used.items() if (key[2]==i and value != None) }
            sol_dict[i] = sorted(arcs_dict.items(), key=lambda x: x[1])

        # get space-temporal dictionary
        space_time = {k : [0]*max_value for k in model.Buques}
        for key, value in sol_dict.items():
            for i in range(len(value) -1):
                for j in range(value[i][1], value[i+1][1]):
                    if j <= value[i][1] + model.t_viaje[value[i][0]]:
                        space_time[key][j]= value[i][0][0] + (j - value[i][1])*(value[i][0][1]-value[i][0][0])/model.t_viaje[value[i][0]]
                    else:
                        space_time[key][j]= value[i][0][1]
            for j in range(value[len(value) -1][1], value[len(value) -1][1] + model.t_viaje[value[len(value) -1][0]] +1):
                space_time[key][j]= value[len(value) -1][0][0] - (j - value[len(value) -1][1]) *(1/model.t_viaje[value[len(value) -1][0]])   


        df_space_time = pd.DataFrame(space_time, columns=[*space_time.keys()])
        df_space_time.reset_index(inplace = True)
        columns_names={i: ('ship'+ str(i)) for i in [*space_time.keys()]}
        columns_names['index'] ='time'
        df_space_time.rename(columns=columns_names, inplace = True)

        # graficamos usando el tipo de grafico de linea
        ship_names=[('ship'+ str(i)) for i in model.Buques]
        fig = px.line(df_space_time, x='time', y=ship_names, 
                      title='Spatial-temporal graph for the ship scheduling problem ',
                      labels={'value': 'node',
                              'variable': ' '})
        
        return fig
    
    def create_graph(self, model):
        x_values = model.x.get_values()
            
        # get values used
        x_used= {(key[0], key[1], key[2]) : int(value)  for key, value in x_values.items() if  value != None }
        # get maximum time (makespan)
        max_value = max(x_used.values())+model.t_viaje[(1,2)]+5
        # get solution for each ship
        sol_dict= dict()
        for i in model.Buques:
            arcs_dict = {(key[0], key[1]) : value  for key, value in x_used.items() if (key[2]==i and value != None) }
            sol_dict[i] = sorted(arcs_dict.items(), key=lambda x: x[1])

        # get space-temporal dictionary
        space_time = {k : [0]*max_value for k in model.Buques}
        for key, value in sol_dict.items():
            for i in range(len(value) -1):
                for j in range(value[i][1], value[i+1][1]):
                    if j <= value[i][1] + model.t_viaje[value[i][0]]:
                        space_time[key][j]= value[i][0][0] + (j - value[i][1])*(value[i][0][1]-value[i][0][0])/model.t_viaje[value[i][0]]
                    else:
                        space_time[key][j]= value[i][0][1]
            for j in range(value[len(value) -1][1], value[len(value) -1][1] + model.t_viaje[value[len(value) -1][0]] +1):
                space_time[key][j]= value[len(value) -1][0][0] - (j - value[len(value) -1][1]) *(1/model.t_viaje[value[len(value) -1][0]])   


        df_space_time = pd.DataFrame(space_time, columns=[*space_time.keys()])
        df_space_time.reset_index(inplace = True)
        columns_names={i: ('ship'+ str(i)) for i in [*space_time.keys()]}
        columns_names['index'] ='time'
        df_space_time.rename(columns=columns_names, inplace = True)

        # graficamos usando el tipo de grafico de linea
        import plotly.graph_objects as go
        pallete = px.colors.qualitative.Dark24
        n_colors = len(pallete)
        fig = go.Figure()
        trace = 0
        for col in ['ship'+ str(i) for i in [*space_time.keys()]]:            
            fig.add_trace(go.Scatter(x=df_space_time['time'], y=df_space_time[col],
                                     name = col,
                                     line = dict(color=pallete[trace])))
            fig.add_trace(go.Scatter(x=[self.descriptive['programmed'][col], self.descriptive['programmed'][col]], y=[-1, len(model.Nodos)+1],
                                     name = "sched_"+col,
                                     line = dict(dash='dot', color=pallete[trace])))
            '''
            fig.add_shape(type="line",
                x0=self.descriptive['programmed'][col], y0=-1, x1=self.descriptive['programmed'][col], y1=len(model.Nodos)+1,
                name = "sched_"+col,
                line=dict(
                    color=pallete[trace],
                    dash="dot",
                )
            )  '''          
            trace += 1
            trace = trace%n_colors
        return fig
        
        
def get_results(model):
    x_values = model.x.get_values()
    return x_values