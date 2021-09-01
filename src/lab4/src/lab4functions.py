#!/usr/bin/env python
# encoding: utf-8

import numpy as np
from copy import copy

#cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):
	"""
	Calcular la matriz de transformacion homogenea asociada con los parametros
	de Denavit-Hartenberg.
	Los valores d, theta, a, alpha son escalares.


	kinetic (ver informe!)

	joint   tetha   dj          a       alpha
	j1      q1     l1          0       pi/2
	j2      q2     0           -l2      0
	j3      q3     0           -l3      0
	j4      q4     m2          0       pi/2
	j5      q5     l4          0       -pi/2
	j6      q6     m1          0       0
	
	distancias en metros, de abajo hacia arriba, y de izquierda a derecha
	l1 = 0.089159
	l2 = 0.425
	l3 = 0.39225
	l4 = 0.09465
	m1 = 0.0823
	m2 = 0.10915
	
	"""
	T = np.array([ 	[np.cos(theta),    	-np.cos(alpha)*np.sin(theta),  	np.sin(alpha)*np.sin(theta),	a*np.cos(theta)],
					[np.sin(theta),    	np.cos(alpha)*np.cos(theta),  	-np.sin(alpha)*np.cos(theta), 	a*np.sin(theta)],
					[0,             	np.sin(alpha),             		np.cos(alpha),              	d],
					[0,             	0,                      		0,                      		1]])
	return T
	
	

def fkine_ur5(q):
	"""
	Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
	q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]

	"""
	# Longitudes (en metros)
	l1 = 0.089159
	l2 = 0.425
	l3 = 0.39225
	l4 = 0.09465
	m1 = 0.0823
	m2 = 0.10915
	alpha1 = np.pi/2
	alpha4 = np.pi/2
	alpha5 = np.pi/2
	# Matrices DH (completar)
	T1 = dh(l1, q[0],   0,  	alpha1 )
	T2 = dh(0,  q[1]-np.pi/2,   -l2,	0      )
	T3 = dh(0,  q[2],   -l3,	0      )
	T4 = dh(m2, q[3] - np.pi/2,   0,  	alpha4 )
	T5 = dh(l4, q[4] + np.pi,   0,  	alpha5 )
	T6 = dh(m1, q[5],   0,  	0      )
	'''
	alpha1 = np.pi/2
    alpha4 = np.pi/2
    alpha5 = np.pi/2
    # Matrices DH (completar)
    T1 = dh( l1, q[0],             0,    alpha1   )
    T2 = dh( 0,  q[1]-np.pi/2, 	 -l2,     0       ) 
    T3 = dh( 0,  q[2],           -l3,     0       )
    T4 = dh( m2, q[3]-np.pi/2, 	   0,    alpha4   ) 
    T5 = dh( l4, q[4]+ np.pi,      0,    alpha5   )
    T6 = dh( m1, q[5],             0,     0       )'''


	# Efector final con respecto a la base
	T12 = np.dot(T1, T2)
	T34 = np.dot(T3, T4)
	T56 = np.dot(T5, T6)

	T14 = np.dot(T12, T34)
	T = np.dot(T14, T56)
	return T



def jacobian_ur5(q, delta=0.0001):
	#Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6] # Alocacion de memoria
	J = np.zeros((3,6))
	# Transformacion homogenea inicial (usando q)
	TH = fkine_ur5(q)
	TH_aux = TH
	# Iteracion para la derivada de cada columna
	for i in xrange(6):
		# Copiar la configuracion articular inicial
		dq = copy(q);
		# Incrementar la articulacion i-esima usando un delta
		dq[i] = dq[i]+delta
		# Transformacion homogenea luego del incremento (q+dq)
		TH_inc = fkine_ur5(dq)
		TH_delta2 = np.array(TH_inc)
		# Aproximacion del Jacobiano de posicion usando diferencias finitas
		for k in range(3):
			J[k,i] = (TH_delta2[k,3]-TH_aux[k,3])/delta
	return J


def ikine_ur5(xdes, q0):
	#Calcular la cinematica inversa de UR5 numericamente a partir de la configuración articular inicial de q0.
	epsilon  = 0.1
	max_iter = 1000
	delta    = 0.001

	q  = copy(q0)
	for i in range(max_iter):
		F_ini = np.array(fkine_ur5(q))
		dif = jacobian_ur5(q,delta)

		error = xdes-F_ini[0:3,3]
		
		q = q+np.dot(np.linalg.pinv(dif),error)
		if (np.linalg.norm(error) < epsilon):
			break
	
	return q
