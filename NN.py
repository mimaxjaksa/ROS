from __future__ import division
import numpy as np
import random
import math as m


pop_size = 100
mutation_happens = 0.2
mutation_prob = 0.15
mutation_amount = 2 #1
swap_chance = 0.8

class NeuralNetwork():
	def __init__(self, nn = False):
		if not nn:
			# setting the number of nodes in layer 2 and layer 3
			# more nodes --> more confidence in predictions (?)
#			l2 = 4
#			l3 = 4			
			l2 = 5
			l3 = 5

			# assign random weights to matrices in network
			# format is (no. of nodes in previous layer) x (no. of nodes in following layer)
#			self.synaptic_weights1 = 2 * np.random.rand(7, l2) - 1
			self.synaptic_weights1 = 2 * np.random.rand(4, l2) - 1
			self.synaptic_weights2 = 2 * np.random.rand(l2, l3) - 1
			self.synaptic_weights3 = 2 * np.random.rand(l3, 3) - 1
		else:
#			l2 = 4
#			l3 = 4
			l2 = 5
			l3 = 5

			self.synaptic_weights1 = nn.synaptic_weights1
			self.synaptic_weights2 = nn.synaptic_weights2
			self.synaptic_weights3 = nn.synaptic_weights3

	def __sigmoid(self, x):
		return 1/(1+np.exp(-x))

	def forward_pass(self, inputs):
		# pass our inputs through our neural network
		a2 = self.__sigmoid(np.dot(inputs, self.synaptic_weights1))
		a3 = self.__sigmoid(np.dot(a2, self.synaptic_weights2))
		output = self.__sigmoid(np.dot(a3, self.synaptic_weights3)) 
		return output

	def mutate(self):
		for i in range(self.synaptic_weights1.shape[0]):
			for j in range(self.synaptic_weights1.shape[1]):
				if (np.random.rand() < mutation_prob):
					self.synaptic_weights1[i, j] += 2*mutation_amount*np.random.rand() - mutation_amount

		for i in range(self.synaptic_weights2.shape[0]):
			for j in range(self.synaptic_weights2.shape[1]):
				if (np.random.rand() < mutation_prob):
					self.synaptic_weights2[i, j] += 2*mutation_amount*np.random.rand() - mutation_amount

		for i in range(self.synaptic_weights3.shape[0]):
			for j in range(self.synaptic_weights3.shape[1]):
				if (np.random.rand() < mutation_prob):
					self.synaptic_weights3[i, j] += 2*mutation_amount*np.random.rand() - mutation_amount

def save1(nn):
    path = "home/student/catkin_ws/src/projekat/src/scripts"
    w1 = open(path + "/" + "w1.txt", "w+")
    w2 = open(path + "/" + "w2.txt", "w+")
    w3 = open(path + "/" + "w3.txt", "w+")
    
    np.savetxt(w1, nn.synaptic_weights1, delimiter=',')
    np.savetxt(w2, nn.synaptic_weights2)
    np.savetxt(w3, nn.synaptic_weights3)
    
    w1.close()
    w2.close()
    w3.close()



def crossover(nn1, nn2):
	off1 = NeuralNetwork()
	off2 = NeuralNetwork()

	for i in range(off1.synaptic_weights1.shape[0]):
		for j in range(off1.synaptic_weights1.shape[1]):
			if (np.random.rand() < swap_chance):
				off1.synaptic_weights1[i, j] = nn2.synaptic_weights1[i, j]
				off2.synaptic_weights1[i, j] = nn1.synaptic_weights1[i, j]
			else:
				off1.synaptic_weights1[i, j] = nn1.synaptic_weights1[i, j]
				off2.synaptic_weights1[i, j] = nn2.synaptic_weights1[i, j]
				
	for i in range(off1.synaptic_weights2.shape[0]):
		for j in range(off1.synaptic_weights2.shape[1]):
			if (np.random.rand() < swap_chance):
				off1.synaptic_weights2[i, j] = nn2.synaptic_weights2[i, j]
				off2.synaptic_weights2[i, j] = nn1.synaptic_weights2[i, j]
			else:
				off1.synaptic_weights2[i, j] = nn1.synaptic_weights2[i, j]
				off2.synaptic_weights2[i, j] = nn2.synaptic_weights2[i, j]
				

	for i in range(off1.synaptic_weights3.shape[0]):
		for j in range(off1.synaptic_weights3.shape[1]):
			if (np.random.rand() < swap_chance):
				off1.synaptic_weights3[i, j] = nn2.synaptic_weights3[i, j]
				off2.synaptic_weights3[i, j] = nn1.synaptic_weights3[i, j]
			else:
				off1.synaptic_weights3[i, j] = nn1.synaptic_weights3[i, j]
				off2.synaptic_weights3[i, j] = nn2.synaptic_weights3[i, j]
				
	return off1, off2

def createPopulation(n):
	result = []
	for i in range(n):
		result.append(NeuralNetwork())
	return result


def population_change(population, fitness):
	# new_population = createPopulation(100)

	ind = list(range(len(population)))
	sorted_ind = [x for _,x in sorted(zip(fitness, ind))]
#	sorted_fitness = fitness.

#	print(sorted_ind)
	#init
	new_population = []

	#10% random
	for i in range(int(m.ceil(len(population)/10))):
		new_population.append(NeuralNetwork())

	#10% best
#	print("PRINFTTTT0", m.ceil(len(population)/10))
	for i in range(int(m.ceil(len(population)/10))):
		new_population.append(NeuralNetwork(population[sorted_ind[len(population) - 1 - i]]))


#	parent_ind = random.choices(sorted_ind, ind, int(len(population)/2.5))







#	print(ind, np.sum(ind))	
	p = ind/np.sum(ind)
#	print(p)

	ind1 = list(range(int(len(population)/2)))
	p = ind1/np.sum(ind1)
	parent_ind = np.random.choice(sorted_ind[int(len(population)/2):], size = int(len(population)/2.5), p = p)







#	ind1 = list(range(int(len(population)/2)))
#	p = ind1/np.sum(ind1)
#	print(fitness)
#	p = fitness + np.abs(np.min(fitness))
#	print(p)	
#	p = p/np.sum(p)
#	print(p)
#	parent_ind = np.random.choice(ind, size = int(len(population)/2.5), p = p)
#	print(parent_ind)






#	print(ind)
#	print()
#	print(sorted_ind)
#	print()
#	print(fitness)
#	print()
#	fitness.sort()
##	print(fitness)
#	p = fitness[int(len(fitness)/2):] + np.abs(np.min(fitness[int(len(fitness)/2):]))
##	print(p)	
#	p = p/np.sum(p)
##	print(p)
#	parent_ind = np.random.choice(sorted_ind[int(len(population)/2):], size = int(len(population)/2.5), p = p)

	print(parent_ind)	

	while(len(new_population) < len(population)):
		parents = np.random.choice(parent_ind, size = 2)
		off1, off2 = crossover(population[parents[0]], population[parents[1]])
		if np.random.rand() < mutation_happens:		
			off1.mutate()
		if np.random.rand() < mutation_happens:
			off2.mutate()	
		new_population.append(off1)
		if len(new_population) == len(population):
			break
		new_population.append(off2)


#	for nn in new_population:
#		if np.random.rand() < mutation_happens:
#			nn.mutate()
	
#	print(len(new_population))

	return new_population




				

	

