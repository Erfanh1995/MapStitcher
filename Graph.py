# Last edited: 2024-02-06
# Authors: Erfan Hosseini Sereshgi - Tulane University
#          Will Rodman - Tulane University

from geojson import LineString, Feature, FeatureCollection
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math
import geojson
import random


class Graph:
	def __init__(self, filename=None, name="Untitled", id=None):
		self.filename = filename
		self.name = name
		self.id = id
		self.nodes = {}  # id -> [lon,lat]
		self.edges = {}  # id -> [nid1, nid2]
		self.nodeLink = {}  # id -> list of next nodes
		self.numberOfNodes = 0  # total number of nodes
		self.numberOfEdges = 0  # total number of edges
		self.largestEdgeID = 0  # largest edge id
		#self.largestNodeID = 0  # largest node id
		self.edgeLink = {} # node id -> [edge id 1, edge id 2, ...]
		self.nodeHash = {} # (lon,lat) -> nid
		self.edgeHash = {}  # (nid1, nid2) -> edge id
		self.edgeWeight = {} # edge id -> weight of the edge
		self.nodeWeight = {} # node id -> weight of the node
		self.nodeDegree = {} # node id -> degree of the node (int)
		self.edgeInt = {}
		self.deletedNodes = {}
		self.breadcrumbs = {}  # edge id -> [(lon,lat),(lon,lat), ...]
		self.breadcrumbHash = {} # (lon,lat) -> [edge id 1, edge id 2, ...]
		self.samples = {} # edge id -> [(lon,lat),(lon,lat), ...]
		self.sampleHash = {} # (lon,lat) -> [edge id 1, edge id 2, ...]
		self.duplicateNodePointer = {}
		if filename is not None:
			with open(filename + "_vertices.txt", 'r') as vf:
				for line in vf:
					if line != "\n" and line != "" and line != "\r\n" and line != "\r" and line != "\n\r":  # No empty lines. Compatible with all OSs
						vertex = line.strip('\n').split(',')
						self.addNode(int(vertex[0]), float(
							vertex[1]), float(vertex[2]))
			with open(filename + "_edges.txt", 'r') as ve:
				for line in ve:
					if line != "\n" and line != "" and line != "\r\n" and line != "\r" and line != "\n\r":  # No empty lines. Compatible with all OSs
						edge = line.strip('\n').split(',')
						self.connectTwoNodes(int(edge[0]), int(edge[1]), int(edge[2]))

	def addNode(self, nid, lon, lat, nodeweight=0):
		if [lon,lat] not in self.nodes.values():
			if nid not in self.nodes.keys():
				self.nodes[nid] = [lon, lat]
				self.nodeHash[(lon,lat)] = nid
				self.nodeLink[nid] = []
				self.edgeLink[nid] = []
				self.nodeWeight[nid] = nodeweight
				self.nodeDegree[nid] = 0
				self.numberOfNodes += 1
				return nid
			else:
				self.numberOfNodes += 1
				print(nid, "already exists. The new node ID is:", self.numberOfNodes)
				self.nodes[self.numberOfNodes] = [lon, lat]
				self.nodeHash[(lon,lat)] = self.numberOfNodes
				self.nodeLink[self.numberOfNodes] = []
				self.edgeLink[self.numberOfNodes] = []
				self.nodeWeight[self.numberOfNodes] = nodeweight
				self.nodeDegree[self.numberOfNodes] = 0
				return self.numberOfNodes
		else:
			self.duplicateNodePointer[nid] = list(self.nodes.keys())[list(self.nodes.values()).index([lon,lat])]
			#print("Duplicated Node:", list(self.nodes.keys())[list(self.nodes.values()).index([lon,lat])])
			return list(self.nodes.keys())[list(self.nodes.values()).index([lon,lat])]


	def addEdge(self, nid1, lon1, lat1, nid2, lon2, lat2, eid, nodeweight1=0, nodeweight2=0, edgeweight=0):
		"""
		if nid1 not in self.nodes.keys():
			self.nodes[nid1] = [lon1, lat1]
			self.nodeLink[nid1] = []
			self.nodeWeight[nid1] = nodeweight1
			self.numberOfNodes += 1

		if nid2 not in self.nodes.keys():
			self.nodes[nid2] = [lon2, lat2]
			self.nodeLink[nid2] = []
			self.nodeWeight[nid2] = nodeweight2
			self.numberOfNodes += 1
		"""
		nid1 = self.addNode(nid1,lon1,lat1,nodeweight1)
		nid2 = self.addNode(nid2,lon2,lat2,nodeweight2)

		if [nid1,nid2] in self.edges.values():
			print("Duplicated Edge:", list(self.edges.keys())[list(self.edges.values()).index([nid1,nid2])])
			return list(self.edges.keys())[list(self.edges.values()).index([nid1,nid2])]

		if eid in self.edges.keys():
			self.numberOfEdges += 1
			print(eid, "already exists. The new edge ID is:", self.numberOfEdges)
			self.edges[self.numberOfEdges] = [nid1, nid2]
			self.edgeLink[nid1].append(self.numberOfEdges)
			self.nodeDegree[nid1] += 1
			self.edgeLink[nid2].append(self.numberOfEdges)
			self.nodeDegree[nid2] += 1
			self.edgeHash[(nid1, nid2)] = self.numberOfEdges
			self.edgeHash[(nid2, nid1)] = self.numberOfEdges
			self.edgeWeight[self.numberOfEdges] = edgeweight
			return self.numberOfEdges

		self.edges[eid] = [nid1, nid2]
		self.edgeLink[nid1].append(eid)
		self.nodeDegree[nid1] += 1
		self.edgeLink[nid2].append(eid)
		self.nodeDegree[nid1] += 1
		self.edgeHash[(nid1, nid2)] = eid
		self.edgeHash[(nid2, nid1)] = eid
		self.edgeWeight[eid] = edgeweight
		self.numberOfEdges += 1

		if eid > self.largestEdgeID:
			self.largestEdgeID = eid

		if nid2 not in self.nodeLink[nid1]:
			self.nodeLink[nid1].append(nid2)
		
		if nid1 not in self.nodeLink[nid2]:
			self.nodeLink[nid2].append(nid1)

		return eid

	def connectTwoNodes(self, eid, n1, n2, edgeweight=0):
		if n1 not in self.nodes.keys(): #it's not already in nodes? maybe it just has a different nodeID
			n1 = self.duplicateNodePointer[n1]
		if n2 not in self.nodes.keys():
			n2 = self.duplicateNodePointer[n2]

		lon1 = self.nodes[n1][0]
		lat1 = self.nodes[n1][1]

		lon2 = self.nodes[n2][0]
		lat2 = self.nodes[n2][1]

		#if lon1 == lon2 and lat1 == lat2:
		#	print("Not an edge. The edge length is zero. Skipping...")
		#	return

		return self.addEdge(n1, lon1, lat1, n2, lon2, lat2, eid, edgeweight=edgeweight)

	def removeNode(self, nodeid):
		for next_node in self.nodeLink[nodeid]:
			edgeid = self.edgeHash[(nodeid, next_node)]

			del self.edges[edgeid]
			del self.edgeWeight[edgeid]
			del self.edgeHash[(nodeid, next_node)]
			del self.edgeHash[(next_node, nodeid)]
			self.numberOfEdges -= 1

		self.deletedNodes[nodeid] = self.nodes[nodeid]
		del self.nodeHash[self.nodes[nodeid]]
		del self.nodes[nodeid]
		del self.nodeWeight[nodeid]
		del self.nodeLink[nodeid]
		self.numberOfNodes -= 1
	
	def removeEdge(self, edgeid):
		nid1 = self.edges[edgeid][0]
		nid2 = self.edges[edgeid][1]
		self.nodeDegree[nid1] -= 1
		self.nodeDegree[nid2] -= 1
		self.nodeLink[nid1].remove(nid2)
		self.nodeLink[nid2].remove(nid1)
		self.edgeLink[nid1].remove(edgeid)
		self.edgeLink[nid2].remove(edgeid)
		del self.edges[edgeid]
		del self.edgeHash[tuple([nid1,nid2])]
		del self.edgeHash[tuple([nid2,nid1])]
		if edgeid in self.samples.keys():
			for i in self.samples[edgeid]:
				if self.sampleHash[i] == [edgeid]:
					del self.sampleHash[i]
				else:
					if edgeid in self.sampleHash[i]:
						self.sampleHash[i].remove(edgeid)
			del self.samples[edgeid]
		#self.numberOfEdges -= 1

	def removeDuplicateEdges(self):
		edges = {}
		c = 0
		for edgeID in self.edges.keys():
			if (self.edges[edgeID][0], self.edges[edgeID][1]) in edges.keys():
				del self.edges[edgeID]
				c += 1
			else:
				edges[(self.edges[edgeID][0], self.edges[edgeID][1])] = edgeID

		for edgeid, edge in edges.items():
			self.edgeHash[(edge[0], edge[1])] = edgeid
			self.edgeHash[(edge[1], edge[0])] = edgeid

		self.largestEdgeID = max(self.edges.keys())
		self.numberOfEdges -= c
		print("Remove", c, "Duplicated Edges")

	def getNeighbors(self, nodeid):
		neighbor = {}

		for next_node in self.nodeLink[nodeid]:
			if next_node != nodeid:
				neighbor[next_node] = 1

		return neighbor.keys()
	
	def findIntersections(self,bearing_limit=45.0):
		intersections = {}

		for nodeid in self.nodes:
			if len(self.nodeLink[nodeid]) > 2 or len(self.nodeLink[nodeid]) == 1: # Add intersections and dead ends
				intersections[nodeid]=(self.nodes[nodeid][0],self.nodes[nodeid][1])
			elif len(self.nodeLink[nodeid]) == 2:  # Add turns with more than 45 degrees
				bearing_i = self.path_bearing_meters(self.nodes[self.nodeLink[nodeid][0]][0],self.nodes[self.nodeLink[nodeid][0]][1],self.nodes[nodeid][0],self.nodes[nodeid][1])
				bearing_j = self.path_bearing_meters(self.nodes[self.nodeLink[nodeid][1]][0],self.nodes[self.nodeLink[nodeid][1]][1],self.nodes[nodeid][0],self.nodes[nodeid][1])
				#bearing_i = self.path_bearing_meters(self.nodes[self.edges[self.edgeLink[nodeid][0]][0]][0], self.nodes[self.edges[self.edgeLink[nodeid][0]][0]][1], self.nodes[self.edges[self.edgeLink[nodeid][0]][1]][0], self.nodes[self.edges[self.edgeLink[nodeid][0]][1]][1])
				#bearing_j = self.path_bearing_meters(self.nodes[self.edges[self.edgeLink[nodeid][1]][0]][0], self.nodes[self.edges[self.edgeLink[nodeid][1]][0]][1], self.nodes[self.edges[self.edgeLink[nodeid][1]][1]][0], self.nodes[self.edges[self.edgeLink[nodeid][1]][1]][1])
				if self.bearing_difference(bearing_i,bearing_j) > bearing_limit:
					intersections[nodeid]=(self.nodes[nodeid][0],self.nodes[nodeid][1])

		return intersections

	def getConnectedComponent(self, nodeID):
		node_list = []

		queue = [nodeID]

		while len(queue) > 0:
			n0 = queue.pop(0)
			node_list.append(n0)
			for n in self.nodeLink[n0]:
				if (n not in queue) and (n not in node_list):
					queue.append(n)

		return node_list

	def removeConnectedComponent(self, size=0):
		node_list = []

		for n in self.nodes.keys():
			if n not in node_list:
				cclist = self.getConnectedComponent(n)

				if len(cclist) < size or size == 0:
					for nn in cclist:
						self.removeNode(nn)

				node_list = node_list + cclist
		self.largestEdgeID = max(self.edges.keys())

	def findRootLocations(self):
		rootlocations = []
		bfs_queue = []
		list_of_edges = list(self.edges.keys())
		connected_component_nodes = []
		connected_component = []
		list_of_nodes = list(self.nodes.keys())
		while len(list_of_nodes) > 0:
			source_node = list_of_nodes[0]
			if len(self.nodeLink[source_node]) == 0:
				list_of_nodes.remove(source_node)
				continue
			bfs_queue.append(source_node)
			rootlocations.append(source_node)
			while (len(bfs_queue) > 0):
				curr_node = bfs_queue.pop(0)
				connected_component_nodes.append(curr_node)
				list_of_nodes.remove(curr_node)
				for out_node in self.nodeLink(curr_node):
					if out_node not in bfs_queue and out_node in list_of_nodes:
						bfs_queue.append(out_node)
			for node in connected_component_nodes:
				for edge in list_of_edges:
					if self.edges[edge][0] == node or self.edges[edge][1] == node:
						connected_component.append(edge)
						list_of_edges.remove(edge)
			connected_component = []
			connected_component_nodes = []
		return rootlocations

	
	def putBreadcrumbs(self, interval):
		if self.breadcrumbs == {}:
			for edge in self.edges:
				self.breadcrumbs[edge] = [(self.nodes[self.edges[edge][0]][0],self.nodes[self.edges[edge][0]][1])]
				if (self.nodes[self.edges[edge][0]][0],self.nodes[self.edges[edge][0]][1]) not in self.breadcrumbHash:
					self.breadcrumbHash[(self.nodes[self.edges[edge][0]][0],self.nodes[self.edges[edge][0]][1])] = [edge]
				else:
					self.breadcrumbHash[(self.nodes[self.edges[edge][0]][0],self.nodes[self.edges[edge][0]][1])].append(edge)
				edgelength = self.length(self.vector(self.nodes[self.edges[edge][0]],self.nodes[self.edges[edge][1]]))
				u = [self.vector(self.nodes[self.edges[edge][0]],self.nodes[self.edges[edge][1]])[0]/self.length(self.vector(self.nodes[self.edges[edge][0]],self.nodes[self.edges[edge][1]])), self.vector(self.nodes[self.edges[edge][0]],self.nodes[self.edges[edge][1]])[1]/self.length(self.vector(self.nodes[self.edges[edge][0]],self.nodes[self.edges[edge][1]]))]
				num_of_breadcrumbs = int(edgelength/interval)+1
				interval_length = edgelength/num_of_breadcrumbs
				for i in range(num_of_breadcrumbs):
					if i == num_of_breadcrumbs-1:
						new_bc = (self.nodes[self.edges[edge][1]][0],self.nodes[self.edges[edge][1]][1])
					else:
						new_bc = (self.nodes[self.edges[edge][0]][0] + i * interval_length * u[0], self.nodes[self.edges[edge][0]][1] + i * interval_length * u[1])
					if new_bc not in self.breadcrumbs[edge]:
						self.breadcrumbs[edge].append(new_bc)
					if new_bc not in self.breadcrumbHash:
						self.breadcrumbHash[new_bc] = [edge]
					else:
						self.breadcrumbHash[new_bc].append(edge)
			#print(self.breadcrumbs)
			#print(self.breadcrumbHash)
		else:
			print ("Already has breadcrumbs")
	
	def putSamples(self,interval):
		if self.samples == {}:
			bfs_queue = []
			surplus_queue = []
			list_of_edges = list(self.edges.keys())
			connected_component_nodes = []
			connected_component = []
			list_of_nodes = list(self.nodes.keys())
			while len(list_of_edges) > 0:
				source_node = list_of_nodes[0]
				if len(self.nodeLink[source_node]) == 0:
					list_of_nodes.remove(source_node)
					continue
				bfs_queue.append(source_node)
				surplus_queue.append(0)
				for edge in self.edgeLink[source_node]:
					self.samples[edge] = [(self.nodes[source_node][0],self.nodes[source_node][1])]
					self.sampleHash[(self.nodes[source_node][0],self.nodes[source_node][1])] = [edge]
				while (len(bfs_queue) > 0):
					curr_node = bfs_queue.pop(0)
					curr_surplus = surplus_queue.pop(0)
					connected_component_nodes.append(curr_node)
					#list_of_nodes.remove(curr_node)
					for out_node in self.nodeLink[curr_node]:
						# finding the edge
						if (curr_node,out_node) in self.edgeHash.keys():
							x = (curr_node,out_node)
						else:
							x = (out_node,curr_node) 
						# finding the edge id
						edgeid = self.edgeHash[x]
						if out_node not in bfs_queue and edgeid in list_of_edges:
							# finding the length of the current edge and number of samples needed
							edgelength = self.length(self.vector(self.nodes[x[0]],self.nodes[x[1]]))
							
							# initialization
							self.samples[edgeid] = []

							if edgelength - curr_surplus > 0:
								number_of_samples = int((edgelength - curr_surplus)/interval)+1
								surplus = interval - ((edgelength - curr_surplus)%interval)
								# making the vector of the edge
								u = [self.vector(self.nodes[x[0]],self.nodes[x[1]])[0]/self.length(self.vector(self.nodes[x[0]],self.nodes[x[1]])), self.vector(self.nodes[x[0]],self.nodes[x[1]])[1]/self.length(self.vector(self.nodes[x[0]],self.nodes[x[1]]))]
								for i in range(number_of_samples):
									new_s = (self.nodes[x[0]][0] + (i * interval + curr_surplus) * u[0], self.nodes[x[0]][1] + (i * interval + curr_surplus) * u[1])
									# making the sample
									if new_s not in self.samples[edgeid]: 
										self.samples[edgeid].append(new_s) 
									if new_s not in self.sampleHash:
										self.sampleHash[new_s] = [edgeid] 
									else:
										self.sampleHash[new_s].append(edgeid)  

							elif edgelength - curr_surplus == 0:
								surplus = 0
								new_s = (self.nodes[x[1]][0], self.nodes[x[1]][1])
								# making the sample
								if new_s not in self.samples[edgeid]: 
									self.samples[edgeid].append(new_s) 
								if new_s not in self.sampleHash:
									self.sampleHash[new_s] = [edgeid] 
								else:
									self.sampleHash[new_s].append(edgeid)

							else: 
								surplus = interval - ((curr_surplus + edgelength)%interval)
							
							connected_component.append(edgeid)
							list_of_edges.remove(edgeid)
							bfs_queue.append(out_node)
							surplus_queue.append(surplus)

				for n in connected_component_nodes:
					removalflag = 0
					for e in self.edgeLink[n]:
						if e not in list_of_edges:
							continue
						else:
							removalflag = 1
							break
					if removalflag == 0 and n in list_of_nodes:
						list_of_nodes.remove(n)
					#for edge in list_of_edges:
						#if self.edges[edge][0] == node or self.edges[edge][1] == node:
							#connected_component.append(edge)
							#list_of_edges.remove(edge)
				connected_component = []
				connected_component_nodes = []

		else:
			print ("Already has samples")
		
	def vector(self, node1,node2):  # Find vector given two points
		return (node2[0] - node1[0], node2[1] - node1[1])

	def length(self, v): # Find length given a vector
		return math.sqrt(v[0]**2 + v[1]**2)	

	def path_bearing_meters(self, a_x, a_y, b_x, b_y):
		ydiff = float(b_y)-float(a_y)
		xdiff = float(b_x)-float(a_x) 
		bearing = math.atan2(ydiff, xdiff)
		return math.fmod(math.degrees(bearing) + 360.0, 360.0)

	def bearing_difference(self, a_bearing, b_bearing):
		max_bearing = max(a_bearing, b_bearing)
		min_bearing = min(a_bearing, b_bearing)
		return min(abs(max_bearing - min_bearing), abs((360.0 - max_bearing) + min_bearing), abs(180.0 - (max_bearing - min_bearing)))

	def _get_breadcrumbs(self):
		return self.breadcrumbs
	
	def _get_breadcrumbHash(self):
		return self.breadcrumbHash
	
	def _get_samples(self):
		return self.samples
	
	def _get_sampleHash(self):
		return self.sampleHash

	def Dump2GeoJson(self, filename):

		print("Dump geojson to " + filename)

		myfeature = []

		for edgeId, edge in self.edges.items():
			n1, n2 = edge[0], edge[1]

			lon1 = self.nodes[n1][0]
			lat1 = self.nodes[n1][1]

			lon2 = self.nodes[n2][0]
			lat2 = self.nodes[n2][1]

			myfeature.append(Feature(properties={
				"id": edgeId, "type": "residential"}, geometry=LineString([(lon1, lat1), (lon2, lat2)])))

		feature_collection = FeatureCollection(myfeature)

		with open(filename, "w") as fout:
			geojson.dump(feature_collection, fout, indent=2)

		print("Done.")

	def Dump2txt(self, filename):

		print("Dump text files to " + filename)
		file1 = open(filename + '_vertices.txt', 'w+')
		file2 = open(filename + '_edges.txt', 'w+')
		vertices = []

		for edgeId, edge in self.edges.items():
			n1, n2 = edge[0], edge[1]

			lon1 = self.nodes[n1][0]
			lat1 = self.nodes[n1][1]

			lon2 = self.nodes[n2][0]
			lat2 = self.nodes[n2][1]

			if self.nodes[n1] not in vertices:
				vertices.append(self.nodes[n1])
				file1.write(str(n1) + "," + str(lon1) + "," + str(lat1) + "\n")
			if self.nodes[n2] not in vertices:
				vertices.append(self.nodes[n2])
				file1.write(str(n2) + "," + str(lon2) + "," + str(lat2) + "\n")
			file2.write(str(edgeId) + "," + str(n1) + "," + str(n2) + "\n")

		file1.close()
		file2.close()
		print("Done.")

	def Plot(self):
		n = list()
		for id, edge in self.edges.items():
			n1_id, n2_id = edge[0], edge[1]
			n1, n2 = self.nodes[n1_id], self.nodes[n2_id]

			if n1 not in n:
				n.append(n1)
			if n2 not in n:
				n.append(n2)

			plt.plot([n1[0], n2[0]], [n1[1], n2[1]],
			         color='dimgray', linewidth=3)

		lons, lats = map(list, zip(*n))

		plt.scatter(lons, lats, s=200, c='dimgray')
		plt.show()
	

	def randomizeEdges(self):
		for key, value in self.edges.items():
			if isinstance(value, list) and len(value) == 2:
				if random.choice([True, False]):
					self.edges[key] = value[::-1]
		

	def Plot2MatPlotLib(self, plot=True):
		fig, ax = plt.subplots()
		n = []
		for id, edge in self.edges.items():
			n1_id, n2_id = edge[0], edge[1]
			n1, n2 = self.nodes[n1_id], self.nodes[n2_id]
			if n1 not in n:
				n.append(n1)
			if n2 not in n:
				n.append(n2)

			ax.plot([n1[0], n2[0]], [n1[1], n2[1]], color='black', linewidth=1.5)

		lons, lats = zip(*n)  # Unpack nodes for plotting

		ax.scatter(lons, lats, s=15, c='black')
		g1_label = mpatches.Patch(color='black', label=self.name)
		ax.legend(handles=[g1_label], loc='upper left')

		samples = []
		if self.sampleHash != {}:
			for key in self.sampleHash.keys():
				samples.append(key)
			x,y =zip(*samples)
			ax.scatter(x, y, s=15, c='blue')		

		if plot:
			plt.plot()
			return None
		else:   
			return fig, ax
        

	def __eq__(self, other):
		if (self.nodes == other.nodes) and \
			(self.edges == other.edges):
			return True 
		return False
	

	"""
	def BiDirection(self):
		edgeList = list(self.edges.values())

		for edge in edgeList:
			node1 = edge[1]
			node2 = edge[0]

			self.edges[self.largestEdgeID + 1] = [node1, node2]
			self.edgeHash[(node1, node2)] = self.largestEdgeID + 1
			self.edgeWeight[self.largestEdgeID + 1] = self.edgeWeight[self.edgeHash[(
				node2, node1)]]
			self.largestEdgeID += 1
			self.numberOfEdges += 1

			if node2 not in self.nodeLink[node1]:
				self.nodeLink[node1].append(node2)
"""
